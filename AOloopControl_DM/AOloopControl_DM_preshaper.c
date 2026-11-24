// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_DM_preshaper.c
 * @brief   High-rate asynchronous look-ahead DM command pre-shaper
 *
 * Transparent real-time layer inserted immediately before DM hardware (or simulation):
 *   upstream DM command stream (.streamin)  --->  [ dmpreshaper ]  --->  DM output (.streamout)
 *
 * Runs asynchronously at the DM hardware frame rate (.rate_hz or .trigstream) using a
 * look-ahead ring buffer holding future scheduled counter-dynamics corrections generated
 * from each upstream command increment du = u_new - u_prev:
 *
 *   ring[(head + k) % M_ring] += gain * C[k] * du   (k = 0 .. M_hw - 1)
 *   u_out = clip(u_base + ring[head])               (slot M_hw drains cleanly to 0)
 *
 * Supports all five kernel layouts produced by cacao-fpsexec-cacaocc-compdmpreshaper:
 *   - KMODE 0: MIMO modal/subspace (ring stored in modal space [M_ring x N_in])
 *   - KMODE 1: Decoupled SISO modal (diagonal C in modal space [M_ring x N_in])
 *   - KMODE 2: Uniform broadcast SISO (ring stored directly in actuator space [M_ring x N_dm])
 *   - KMODE 3: Per-actuator diagonal SISO ([dmx, dmy, M_buf], actuator-space ring [M_ring x N_dm])
 *   - KMODE 4: Per-actuator spatially-varying local-stencil MIMO ([N_nb, N_dm, M_buf], actuator-space ring)
 *
 * When idle (look-ahead buffer drained and .freerun=OFF), waits directly on .streamin's
 * semaphore for < 5 us reaction latency; while draining scheduled taps, paces strictly on
 * CLOCK_MONOTONIC TIMER_ABSTIME with tightened kernel timer slack + final spin-wait.
 */

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#    include <sys/prctl.h>
#endif
#include <time.h>
#include <unistd.h>
#ifdef _OPENMP
#    include <omp.h>
#endif

#include "milk_runnable.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "ImageStreamIO/ImageStruct.h"
#include "processinfo.h"
#include "milk_compiler.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

#if defined(__x86_64__) || defined(__i386__)
#    include <immintrin.h>
#    define DMPS_CPU_PAUSE() _mm_pause()
#else
#    define DMPS_CPU_PAUSE() ((void) 0)
#endif

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     p_streamin[FUNCTION_PARAMETER_STRMAXLEN]   = "dm01cmd";
static char     p_streamout[FUNCTION_PARAMETER_STRMAXLEN]  = "dm01disp";
static char     p_kernelfile[FUNCTION_PARAMETER_STRMAXLEN] = "dm_preshaper.fits";
static char     p_trigstream[FUNCTION_PARAMETER_STRMAXLEN] = "";
static float    p_rate_hz                                  = 0.0f;
static int32_t  p_freerun                                  = 0;
static uint32_t p_spinus                                   = 60;
static int32_t  p_enable                                   = 1;
static float    p_gain                                     = 1.0f;
static int32_t  p_clip_enable                              = 0;
static float    p_clip_min                                 = -10.0f;
static float    p_clip_max                                 = 10.0f;

static float    o_rate_hz   = 0.0f;
static uint64_t o_in_cnt    = 0;
static uint64_t o_out_cnt   = 0;
static float    o_peak_corr = 0.0f;
static uint64_t o_clip_cnt  = 0;

#define FPS_PARAMS_DMPRESHAPER(X)                                                                  \
    X(".streamin", p_streamin, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "input unshaped DM command stream (float32)")                                                \
    X(".streamout", p_streamout, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "output high-rate shaped DM command stream (float32)")                                       \
    X(".kernelfile", p_kernelfile, FPTYPE_FITSFILENAME, 1,                                         \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "pre-shaper MEF kernel FITS file")                  \
    X(".trigstream", p_trigstream, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "optional hardware clock stream for lockstep ticking")                                       \
    X(".rate_hz", &p_rate_hz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "DM output rate [Hz] (<= 0 = inherit 1/DT_SEC from kernel)")                                 \
    X(".freerun", &p_freerun, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,            \
      "OFF: zero-latency idle-wake + high-rate drain | ON: strict free-running clock")             \
    X(".spinus", &p_spinus, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "final busy-wait threshold [us] after clock_nanosleep")                                      \
    X(".enable", &p_enable, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,               \
      "live enable (OFF = zero-overhead pass-through bypass)")                                     \
    X(".gain", &p_gain, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,                 \
      "live pre-shaping strength blend (0.0 = bypass, 1.0 = full)")                                \
    X(".clip_enable", &p_clip_enable, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,     \
      "enable hardware DAC stroke clamping on streamout")                                          \
    X(".clip_min", &p_clip_min, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,         \
      "minimum allowed output actuator command")                                                   \
    X(".clip_max", &p_clip_max, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,         \
      "maximum allowed output actuator command")                                                   \
    X(".out.rate_hz", &o_rate_hz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                        \
      "effective high-rate DM pre-shaper tick rate [Hz]")                                          \
    X(".out.in_cnt", &o_in_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "upstream command updates ingested")                                                         \
    X(".out.out_cnt", &o_out_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "high-rate shaped DM frames published")                                                      \
    X(".out.peak_corr", &o_peak_corr, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "peak |u_shaped - u_raw| correction seen")                                                   \
    X(".out.clip_cnt", &o_clip_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                       \
      "cumulative clipped actuator samples")

MILK_PARAMS_DEFINE(dmpreshaper, FPS_PARAMS_DMPRESHAPER);

/* ================================================================
 * 2.  TIMING & SEMAPHORE HELPERS
 * ============================================================= */

static inline uint64_t dmps_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static inline int dmps_exit_requested(PROCESSINFO *processinfo)
{
    return (processinfo != NULL && processinfo_exit_requested(processinfo)) ? 1 : 0;
}

static void dmps_sleep_until_ns(uint64_t target_ns, uint32_t spin_us, PROCESSINFO *processinfo)
{
    uint64_t now = dmps_now_ns();
    if (now >= target_ns)
    {
        return;
    }

    uint64_t spin_ns = (uint64_t) spin_us * 1000ULL;
    if (target_ns - now > spin_ns)
    {
        uint64_t        wake_ns = target_ns - spin_ns;
        struct timespec ts_wake;
        ts_wake.tv_sec  = (time_t) (wake_ns / 1000000000ULL);
        ts_wake.tv_nsec = (long) (wake_ns % 1000000000ULL);

        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_wake, NULL) == EINTR)
        {
            if (dmps_exit_requested(processinfo))
            {
                return;
            }
        }
    }

    while (dmps_now_ns() < target_ns)
    {
        if (dmps_exit_requested(processinfo))
        {
            return;
        }
        DMPS_CPU_PAUSE();
    }
}

static int dmps_semwait_ms(IMAGE *im, int semidx, uint64_t timeout_ms, PROCESSINFO *processinfo)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t add_ns = timeout_ms * 1000000ULL;
    uint64_t nsec   = (uint64_t) ts.tv_nsec + add_ns;
    ts.tv_sec      += (time_t) (nsec / 1000000000ULL);
    ts.tv_nsec      = (long) (nsec % 1000000000ULL);

    int rc = ImageStreamIO_semtimedwait(im, semidx, &ts);
    if (rc == 0)
    {
        return 0;
    }
    if (errno == EINTR && dmps_exit_requested(processinfo))
    {
        return -2;
    }
    return 1; /* timeout or interrupted */
}

/* ================================================================
 * 3.  MEF KERNEL LOADER & RATE RESAMPLER
 * ============================================================= */

typedef struct
{
    uint32_t kmode;
    double   dt_kern_s;
    uint32_t dmx;
    uint32_t dmy;
    uint32_t n_in;
    uint32_t m_buf;
    uint32_t stencil_rad;
    uint32_t stencil_nb;
    float   *c_kern; /* KMODE 0,1: [m_buf, n_in, n_in]; 2: [m_buf]; 3: [m_buf, n_dm]; 4: [m_buf, n_dm, n_nb] */
    float   *p_fwd;  /* [n_in, n_dm] (NULL for KMODE 2, 3, 4) */
    float   *p_pinv; /* [n_in, n_dm] (NULL for KMODE 2, 3, 4) */
} DMPS_KERNEL;

static void dmps_kernel_free(DMPS_KERNEL *k)
{
    free(k->c_kern);
    free(k->p_fwd);
    free(k->p_pinv);
    memset(k, 0, sizeof(*k));
}

static int dmps_load_kernel(const char *raw_path, DMPS_KERNEL *out)
{
    memset(out, 0, sizeof(*out));
    if (raw_path == NULL || raw_path[0] == '\0')
    {
        return -1;
    }
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;

#ifdef USE_CFITSIO
    fitsfile *fptr   = NULL;
    int       status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        fprintf(stderr, "dmpreshaper: cannot open kernel '%s' (status=%d)\n", path, status);
        return -1;
    }

    long   lval = 0;
    double dval = 0.0;
    int    kst  = 0;

    if (fits_read_key(fptr, TLONG, "KMODE", &lval, NULL, &kst) == 0)
    {
        out->kmode = (uint32_t) lval;
    }
    kst = 0;
    if (fits_read_key(fptr, TDOUBLE, "DT_SEC", &dval, NULL, &kst) == 0 && dval > 0.0)
    {
        out->dt_kern_s = dval;
    }
    else
    {
        out->dt_kern_s = 2.5e-4;
    }
    kst = 0;
    if (fits_read_key(fptr, TLONG, "DMX", &lval, NULL, &kst) == 0)
    {
        out->dmx = (uint32_t) lval;
    }
    kst = 0;
    if (fits_read_key(fptr, TLONG, "DMY", &lval, NULL, &kst) == 0)
    {
        out->dmy = (uint32_t) lval;
    }

    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    fits_get_img_size(fptr, (naxis <= 3) ? naxis : 3, naxes, &status);
    if (status != 0 || naxis < 1 || naxis > 3)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    if (out->kmode > 4u)
    {
        fprintf(stderr, "dmpreshaper: unsupported KMODE=%u in '%s'\n", out->kmode, path);
        fits_close_file(fptr, &status);
        return -1;
    }

    if (out->kmode == 2u)
    {
        out->n_in  = 1u;
        out->m_buf = (naxis == 3) ? (uint32_t) naxes[2] : (uint32_t) naxes[0];
    }
    else if (out->kmode == 3u)
    {
        if (naxis != 3)
        {
            fprintf(stderr, "dmpreshaper: expected 3D [dmx, dmy, M_buf] kernel for KMODE=3\n");
            fits_close_file(fptr, &status);
            return -1;
        }
        out->dmx   = (uint32_t) naxes[0];
        out->dmy   = (uint32_t) naxes[1];
        out->n_in  = 1u;
        out->m_buf = (uint32_t) naxes[2];
    }
    else if (out->kmode == 4u)
    {
        if (naxis != 3)
        {
            fprintf(stderr, "dmpreshaper: expected 3D [N_nb, N_dm, M_buf] kernel for KMODE=4\n");
            fits_close_file(fptr, &status);
            return -1;
        }
        kst = 0;
        if (fits_read_key(fptr, TLONG, "STNCLRAD", &lval, NULL, &kst) != 0 || lval < 1 || lval > 5)
        {
            fprintf(stderr, "dmpreshaper: missing/invalid STNCLRAD header for KMODE=4\n");
            fits_close_file(fptr, &status);
            return -1;
        }
        out->stencil_rad = (uint32_t) lval;
        uint32_t s_side  = 2u * out->stencil_rad + 1u;
        out->stencil_nb  = s_side * s_side;
        if ((uint32_t) naxes[0] != out->stencil_nb ||
            (uint64_t) naxes[1] != (uint64_t) out->dmx * out->dmy)
        {
            fprintf(stderr,
                    "dmpreshaper: KMODE=4 shape mismatch [%ld,%ld,%ld] vs nb=%u dmx=%u dmy=%u\n",
                    naxes[0], naxes[1], naxes[2], out->stencil_nb, out->dmx, out->dmy);
            fits_close_file(fptr, &status);
            return -1;
        }
        out->n_in  = 1u;
        out->m_buf = (uint32_t) naxes[2];
    }
    else
    {
        if (naxis != 3 || naxes[0] != naxes[1])
        {
            fprintf(stderr, "dmpreshaper: expected square 3D kernel for KMODE=%u\n", out->kmode);
            fits_close_file(fptr, &status);
            return -1;
        }
        out->n_in  = (uint32_t) naxes[0];
        out->m_buf = (uint32_t) naxes[2];
    }

    LONGLONG n_c = (LONGLONG) naxes[0] * naxes[1] * ((naxis == 3) ? naxes[2] : 1);
    out->c_kern  = (float *) malloc(sizeof(float) * (size_t) n_c);
    if (out->c_kern == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, n_c, NULL, out->c_kern, &anynul, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        dmps_kernel_free(out);
        return -1;
    }

    if (out->kmode <= 1u)
    {
        int  hdutype = 0;
        long pax[3]  = { 1, 1, 1 };
        int  pnax    = 0;

        /* Extension P_FWD */
        if (fits_movnam_hdu(fptr, IMAGE_HDU, (char *) "P_FWD", 0, &status) != 0)
        {
            fprintf(stderr, "dmpreshaper: missing P_FWD extension in '%s'\n", path);
            fits_close_file(fptr, &status);
            dmps_kernel_free(out);
            return -1;
        }
        fits_get_img_dim(fptr, &pnax, &status);
        fits_get_img_size(fptr, 3, pax, &status);
        out->dmx     = (uint32_t) pax[0];
        out->dmy     = (uint32_t) pax[1];
        LONGLONG n_p = (LONGLONG) pax[0] * pax[1] * ((pnax >= 3) ? pax[2] : 1);
        out->p_fwd   = (float *) malloc(sizeof(float) * (size_t) n_p);
        if (out->p_fwd == NULL)
        {
            fits_close_file(fptr, &status);
            dmps_kernel_free(out);
            return -1;
        }
        fits_read_img(fptr, TFLOAT, 1, n_p, NULL, out->p_fwd, &anynul, &status);

        /* Extension P_PINV */
        if (fits_movnam_hdu(fptr, IMAGE_HDU, (char *) "P_PINV", 0, &status) != 0)
        {
            fprintf(stderr, "dmpreshaper: missing P_PINV extension in '%s'\n", path);
            fits_close_file(fptr, &status);
            dmps_kernel_free(out);
            return -1;
        }
        (void) hdutype;
        out->p_pinv = (float *) malloc(sizeof(float) * (size_t) n_p);
        if (out->p_pinv == NULL)
        {
            fits_close_file(fptr, &status);
            dmps_kernel_free(out);
            return -1;
        }
        fits_read_img(fptr, TFLOAT, 1, n_p, NULL, out->p_pinv, &anynul, &status);
        if (status != 0)
        {
            fits_close_file(fptr, &status);
            dmps_kernel_free(out);
            return -1;
        }
    }

    fits_close_file(fptr, &status);
    return 0;
#else
    (void) path;
    fprintf(stderr, "dmpreshaper: built without CFITSIO support\n");
    return -1;
#endif
}

/**
 * Resample look-ahead kernel C(t) from dt_kern_s onto target hardware period dt_hw_s.
 * C(t) = 0 for t >= m_buf * dt_kern_s so unity DC gain is strictly preserved at any rate.
 */
static int dmps_resample_kernel(const DMPS_KERNEL *k,
                                double             dt_hw_s,
                                float            **c_hw_out,
                                uint32_t          *m_hw_out)
{
    size_t block_sz = 1u;
    if (k->kmode <= 1u)
    {
        block_sz = (size_t) k->n_in * k->n_in;
    }
    else if (k->kmode == 3u)
    {
        block_sz = (size_t) k->dmx * k->dmy;
    }
    else if (k->kmode == 4u)
    {
        block_sz = (size_t) k->dmx * k->dmy * k->stencil_nb;
    }
    const double rel_diff = fabs(dt_hw_s - k->dt_kern_s) / k->dt_kern_s;

    if (rel_diff < 1.0e-4)
    {
        float *copy = (float *) malloc(sizeof(float) * k->m_buf * block_sz);
        if (copy == NULL)
        {
            return -1;
        }
        memcpy(copy, k->c_kern, sizeof(float) * k->m_buf * block_sz);
        *c_hw_out = copy;
        *m_hw_out = k->m_buf;
        return 0;
    }

    const double dur_s = (double) k->m_buf * k->dt_kern_s;
    uint32_t     m_hw  = (uint32_t) ceil(dur_s / dt_hw_s);
    if (m_hw < 1u)
    {
        m_hw = 1u;
    }

    float *dst = (float *) calloc((size_t) m_hw * block_sz, sizeof(float));
    if (dst == NULL)
    {
        return -1;
    }

    for (uint32_t j = 0; j < m_hw; j++)
    {
        double u   = ((double) j * dt_hw_s) / k->dt_kern_s;
        size_t i0  = (size_t) floor(u);
        double f   = u - (double) i0;
        float *row = &dst[(size_t) j * block_sz];

        if (i0 >= k->m_buf)
        {
            continue; /* 0 */
        }
        const float *s0 = &k->c_kern[i0 * block_sz];
        const float *s1 = (i0 + 1 < k->m_buf) ? &k->c_kern[(i0 + 1) * block_sz] : NULL;
        const float  w0 = (float) (1.0 - f);
        const float  w1 = (float) f;
        for (size_t q = 0; q < block_sz; q++)
        {
            row[q] = w0 * s0[q] + ((s1 != NULL) ? (w1 * s1[q]) : 0.0f);
        }
    }

    *c_hw_out = dst;
    *m_hw_out = m_hw;
    return 0;
}

/* ================================================================
 * 4.  LOOK-AHEAD RING BUFFER ENGINE
 * ============================================================= */

typedef struct
{
    DMPS_KERNEL kern;
    float      *c_hw;
    uint32_t    m_hw;
    uint32_t    m_ring; /* m_hw + 1: final slot holds 0 so DM settles to exact u_base */
    uint32_t    head;
    uint32_t    active_rem;

    uint32_t dmx;
    uint32_t dmy;
    uint64_t n_dm;

    float   *u_base;
    float   *u_prev;
    float   *du;
    float   *u_out;
    float   *dc;
    float   *ring;   /* KMODE 0,1: [m_ring, n_in]; KMODE 2,3,4: [m_ring, n_dm] */
    uint8_t *row_nz; /* [dmy] non-zero input row mask for fast KMODE 4 gather */

    IMGID img_in;
    IMGID img_out;
    IMGID img_trig;
    int   sem_in;
    int   sem_trig;
    int   use_trig;

    uint64_t last_in_cnt0;
    uint64_t in_cnt;
    uint64_t out_cnt;
    uint64_t clip_cnt;
    float    peak_corr;
    int      prev_enable;
} DMPS_CTX;

static void dmps_clear_ring(DMPS_CTX *ctx)
{
    const size_t slot_len = (ctx->kern.kmode >= 2u) ? (size_t) ctx->n_dm : (size_t) ctx->kern.n_in;
    memset(ctx->ring, 0, sizeof(float) * ctx->m_ring * slot_len);
    ctx->head       = 0;
    ctx->active_rem = 0;
}

static void dmps_publish_out(DMPS_CTX *ctx, const float *src, int apply_clip)
{
    IMAGE   *im   = ctx->img_out.im;
    uint64_t n_dm = ctx->n_dm;

    im->md->write = 1;
    if (apply_clip)
    {
        const float vmin  = p_clip_min;
        const float vmax  = p_clip_max;
        uint64_t    clips = 0;
        for (uint64_t i = 0; i < n_dm; i++)
        {
            float v = src[i];
            if (v < vmin)
            {
                v = vmin;
                clips++;
            }
            else if (v > vmax)
            {
                v = vmax;
                clips++;
            }
            im->array.F[i] = v;
        }
        ctx->clip_cnt += clips;
    }
    else
    {
        memcpy(im->array.F, src, sizeof(float) * n_dm);
    }

    clock_gettime(CLOCK_REALTIME, &im->md->writetime);
    im->md->atime = im->md->writetime;
    im->md->cnt0++;
    im->md->write = 0;
    ImageStreamIO_sempost(im, -1);
    ctx->out_cnt++;
}

static int dmps_ingest_input(DMPS_CTX *ctx)
{
    const IMAGE *imin = ctx->img_in.im;
    if (imin->md->cnt0 == ctx->last_in_cnt0)
    {
        return 0;
    }
    ctx->last_in_cnt0 = imin->md->cnt0;
    ctx->in_cnt++;

    const uint64_t n_dm = ctx->n_dm;
    const float   *src  = imin->array.F;
    float         *base = ctx->u_base;
    float         *prev = ctx->u_prev;
    float         *du   = ctx->du;

    float max_du = 0.0f;
    for (uint64_t i = 0; i < n_dm; i++)
    {
        float v = src[i];
        float d = v - prev[i];
        base[i] = v;
        prev[i] = v;
        du[i]   = d;
        float a = fabsf(d);
        if (a > max_du)
        {
            max_du = a;
        }
    }

    if (!p_enable || fabsf(p_gain) < 1.0e-7f || max_du < 1.0e-12f)
    {
        return 1;
    }

    const float    gain   = p_gain;
    const uint32_t m_hw   = ctx->m_hw;
    const uint32_t m_ring = ctx->m_ring;
    const uint32_t head   = ctx->head;

    if (ctx->kern.kmode == 2u)
    {
        /* Uniform broadcast SISO: ring [m_ring x n_dm] */
        for (uint32_t k = 0; k < m_hw; k++)
        {
            const float w = gain * ctx->c_hw[k];
            if (fabsf(w) < 1.0e-12f)
            {
                continue;
            }
            const uint32_t slot = (head + k) % m_ring;
            float         *dst  = &ctx->ring[(size_t) slot * n_dm];
#pragma omp simd
            for (uint64_t i = 0; i < n_dm; i++)
            {
                dst[i] += w * du[i];
            }
        }
    }
    else if (ctx->kern.kmode == 3u)
    {
        /* Per-actuator diagonal SISO: c_hw [m_hw x n_dm], ring [m_ring x n_dm] */
        for (uint32_t k = 0; k < m_hw; k++)
        {
            const uint32_t slot = (head + k) % m_ring;
            float         *dst  = &ctx->ring[(size_t) slot * n_dm];
            const float   *ck   = &ctx->c_hw[(size_t) k * n_dm];
#pragma omp simd
            for (uint64_t i = 0; i < n_dm; i++)
            {
                dst[i] += gain * ck[i] * du[i];
            }
        }
    }
    else if (ctx->kern.kmode == 4u)
    {
        /* Per-actuator spatially-varying local-stencil MIMO gather:
         * c_hw [m_hw, n_dm, n_nb], ring [m_ring, n_dm].
         * Build row non-zero mask so sparse pokes skip empty rows cleanly. */
        const int      dmx_i = (int) ctx->dmx;
        const int      dmy_i = (int) ctx->dmy;
        const int      rad_i = (int) ctx->kern.stencil_rad;
        const int      side  = 2 * rad_i + 1;
        const uint32_t n_nb  = ctx->kern.stencil_nb;
        uint8_t       *rnz   = ctx->row_nz;
        for (int jy = 0; jy < dmy_i; jy++)
        {
            uint8_t      nz   = 0;
            const float *drow = &du[(size_t) jy * (size_t) dmx_i];
            for (int jx = 0; jx < dmx_i; jx++)
            {
                if (fabsf(drow[jx]) > 1.0e-12f)
                {
                    nz = 1;
                    break;
                }
            }
            rnz[jy] = nz;
        }

#ifdef _OPENMP
#    pragma omp parallel for schedule(static) if ((size_t) m_hw * (size_t) n_dm * n_nb > 32768u)
#endif
        for (uint32_t k = 0; k < m_hw; k++)
        {
            const uint32_t slot = (head + k) % m_ring;
            float         *dst  = &ctx->ring[(size_t) slot * n_dm];
            const float   *ck   = &ctx->c_hw[(size_t) k * n_dm * n_nb];

            for (int iy = 0; iy < dmy_i; iy++)
            {
                const int dy0 = (iy - rad_i < 0) ? -iy : -rad_i;
                const int dy1 = (iy + rad_i >= dmy_i) ? (dmy_i - 1 - iy) : rad_i;
                int       any = 0;
                for (int dy = dy0; dy <= dy1; dy++)
                {
                    if (rnz[iy + dy])
                    {
                        any = 1;
                        break;
                    }
                }
                if (!any)
                {
                    continue;
                }
                for (int ix = 0; ix < dmx_i; ix++)
                {
                    const size_t i_dm = (size_t) iy * (size_t) dmx_i + (size_t) ix;
                    const float *w    = &ck[i_dm * n_nb];
                    const int    dx0  = (ix - rad_i < 0) ? -ix : -rad_i;
                    const int    dx1  = (ix + rad_i >= dmx_i) ? (dmx_i - 1 - ix) : rad_i;
                    float        acc  = 0.0f;
                    for (int dy = dy0; dy <= dy1; dy++)
                    {
                        const int jy = iy + dy;
                        if (!rnz[jy])
                        {
                            continue;
                        }
                        const float *wrow = &w[(size_t) (dy + rad_i) * (size_t) side + (size_t) rad_i];
                        const float *drow = &du[(size_t) jy * (size_t) dmx_i + (size_t) ix];
                        for (int dx = dx0; dx <= dx1; dx++)
                        {
                            acc += wrow[dx] * drow[dx];
                        }
                    }
                    dst[i_dm] += gain * acc;
                }
            }
        }
    }
    else
    {
        /* Modal / Subspace MIMO (kmode=0) or Decoupled Modal SISO (kmode=1) */
        const uint32_t n_in   = ctx->kern.n_in;
        const float   *p_pinv = ctx->kern.p_pinv;
        float         *dc     = ctx->dc;

        for (uint32_t m = 0; m < n_in; m++)
        {
            const float *prow = &p_pinv[(size_t) m * n_dm];
            float        sum  = 0.0f;
#pragma omp simd reduction(+ : sum)
            for (uint64_t i = 0; i < n_dm; i++)
            {
                sum += prow[i] * du[i];
            }
            dc[m] = sum;
        }

        if (ctx->kern.kmode == 0u)
        {
            for (uint32_t k = 0; k < m_hw; k++)
            {
                const uint32_t slot = (head + k) % m_ring;
                float         *dst  = &ctx->ring[(size_t) slot * n_in];
                const float   *c_bl = &ctx->c_hw[(size_t) k * n_in * n_in];
                for (uint32_t r = 0; r < n_in; r++)
                {
                    const float *crow = &c_bl[(size_t) r * n_in];
                    float        acc  = 0.0f;
#pragma omp simd reduction(+ : acc)
                    for (uint32_t c = 0; c < n_in; c++)
                    {
                        acc += crow[c] * dc[c];
                    }
                    dst[r] += gain * acc;
                }
            }
        }
        else
        {
            for (uint32_t k = 0; k < m_hw; k++)
            {
                const uint32_t slot = (head + k) % m_ring;
                float         *dst  = &ctx->ring[(size_t) slot * n_in];
                const float   *c_bl = &ctx->c_hw[(size_t) k * n_in * n_in];
                for (uint32_t r = 0; r < n_in; r++)
                {
                    dst[r] += gain * c_bl[(size_t) r * n_in + r] * dc[r];
                }
            }
        }
    }

    /* Schedule m_hw correction ticks + 1 trailing zero-slot tick to settle cleanly to u_base */
    ctx->active_rem = m_ring;
    return 1;
}

static void dmps_emit_tick(DMPS_CTX *ctx)
{
    const uint64_t n_dm = ctx->n_dm;

    if (!p_enable)
    {
        if (ctx->prev_enable)
        {
            dmps_clear_ring(ctx);
            ctx->prev_enable = 0;
        }
        dmps_publish_out(ctx, ctx->u_base, p_clip_enable);
        return;
    }
    ctx->prev_enable = 1;

    if (ctx->active_rem == 0u)
    {
        dmps_publish_out(ctx, ctx->u_base, p_clip_enable);
        return;
    }

    const uint32_t slot = ctx->head;
    ctx->head           = (ctx->head + 1u) % ctx->m_ring;
    ctx->active_rem--;

    float *u_out = ctx->u_out;
    memcpy(u_out, ctx->u_base, sizeof(float) * n_dm);

    float pk = 0.0f;
    if (ctx->kern.kmode >= 2u)
    {
        float *rslot = &ctx->ring[(size_t) slot * n_dm];
        for (uint64_t i = 0; i < n_dm; i++)
        {
            float c = rslot[i];
            u_out[i] += c;
            rslot[i]  = 0.0f;
            float a   = fabsf(c);
            if (a > pk)
            {
                pk = a;
            }
        }
    }
    else
    {
        const uint32_t n_in  = ctx->kern.n_in;
        const float   *p_fwd = ctx->kern.p_fwd;
        float         *rslot = &ctx->ring[(size_t) slot * n_in];

        for (uint32_t m = 0; m < n_in; m++)
        {
            const float coeff = rslot[m];
            rslot[m]          = 0.0f;
            if (fabsf(coeff) < 1.0e-12f)
            {
                continue;
            }
            const float *prow = &p_fwd[(size_t) m * n_dm];
#pragma omp simd
            for (uint64_t i = 0; i < n_dm; i++)
            {
                u_out[i] += coeff * prow[i];
            }
        }
        for (uint64_t i = 0; i < n_dm; i++)
        {
            float a = fabsf(u_out[i] - ctx->u_base[i]);
            if (a > pk)
            {
                pk = a;
            }
        }
    }

    if (pk > ctx->peak_corr)
    {
        ctx->peak_corr = pk;
    }
    dmps_publish_out(ctx, u_out, p_clip_enable);
}

/* ================================================================
 * 5.  RUNNABLE ENTRY POINT
 * ============================================================= */

static MILK_HOT errno_t dmpreshaper_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

#if defined(__linux__) && defined(PR_SET_TIMERSLACK)
    (void) prctl(PR_SET_TIMERSLACK, 1000UL, 0, 0, 0);
#endif

    if (p_streamin[0] == '\0' || p_streamout[0] == '\0' || p_kernelfile[0] == '\0')
    {
        fprintf(stderr, "dmpreshaper: .streamin, .streamout, and .kernelfile are required\n");
        return RETURN_FAILURE;
    }

    DMPS_CTX ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.img_in.ID   = -1;
    ctx.img_out.ID  = -1;
    ctx.img_trig.ID = -1;
    ctx.sem_in      = -1;
    ctx.sem_trig    = -1;
    ctx.prev_enable = p_enable;

    /* 1. Load pre-shaper kernel MEF */
    if (dmps_load_kernel(p_kernelfile, &ctx.kern) != 0)
    {
        return RETURN_FAILURE;
    }

    /* 2. Connect to streamin */
    ctx.img_in = imgid_make_from_name(p_streamin);
    resolveIMGID(&ctx.img_in, ERRMODE_WARN, dcimg, dcnimg);
    if (ctx.img_in.ID == -1)
    {
        fprintf(stderr, "dmpreshaper: cannot resolve streamin '%s'\n", p_streamin);
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }
    if (ctx.img_in.md->datatype != _DATATYPE_FLOAT)
    {
        fprintf(stderr, "dmpreshaper: streamin '%s' must be float32\n", p_streamin);
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }

    ctx.dmx  = ctx.img_in.md->size[0];
    ctx.dmy  = (ctx.img_in.md->naxis >= 2) ? ctx.img_in.md->size[1] : 1u;
    ctx.n_dm = (uint64_t) ctx.dmx * ctx.dmy;

    if (ctx.kern.kmode != 2u && (ctx.dmx != ctx.kern.dmx || ctx.dmy != ctx.kern.dmy))
    {
        fprintf(stderr, "dmpreshaper: streamin (%ux%u) does not match kernel DM (%ux%u)\n", ctx.dmx,
                ctx.dmy, ctx.kern.dmx, ctx.kern.dmy);
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }

    /* 3. Connect to (or create) streamout in SHM */
    ctx.img_out = imgid_make_from_name(p_streamout);
    resolveIMGID(&ctx.img_out, ERRMODE_NULL, dcimg, dcnimg);
    if (ctx.img_out.ID == -1)
    {
        uint32_t out_size[2] = { ctx.dmx, ctx.dmy };
        create_image_ID(p_streamout, 2, out_size, _DATATYPE_FLOAT, 1, 10, 0, NULL);
        ctx.img_out = imgid_make_from_name(p_streamout);
        resolveIMGID(&ctx.img_out, ERRMODE_WARN, dcimg, dcnimg);
        if (ctx.img_out.ID == -1)
        {
            fprintf(stderr, "dmpreshaper: failed to create streamout '%s'\n", p_streamout);
            dmps_kernel_free(&ctx.kern);
            return RETURN_FAILURE;
        }
    }
    if (ctx.img_out.md->datatype != _DATATYPE_FLOAT ||
        (uint64_t) ctx.img_out.md->size[0] *
                ((ctx.img_out.md->naxis >= 2) ? ctx.img_out.md->size[1] : 1u) !=
            ctx.n_dm)
    {
        fprintf(stderr, "dmpreshaper: streamout '%s' shape/type mismatch\n", p_streamout);
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }

    /* 4. Optional hardware trigger stream */
    if (p_trigstream[0] != '\0' && strcmp(p_trigstream, "none") != 0 &&
        strcmp(p_trigstream, "null") != 0)
    {
        ctx.img_trig = imgid_make_from_name(p_trigstream);
        resolveIMGID(&ctx.img_trig, ERRMODE_WARN, dcimg, dcnimg);
        if (ctx.img_trig.ID == -1)
        {
            fprintf(stderr, "dmpreshaper: cannot resolve trigstream '%s'\n", p_trigstream);
            dmps_kernel_free(&ctx.kern);
            return RETURN_FAILURE;
        }
        ctx.use_trig = 1;
    }

    /* 5. Determine DM hardware tick rate and resample kernel if needed */
    const double dt_hw_s =
        (p_rate_hz > 0.1f) ? (1.0 / (double) p_rate_hz) : ctx.kern.dt_kern_s;
    const uint64_t dt_hw_ns = (uint64_t) llround(dt_hw_s * 1.0e9);
    if (dmps_resample_kernel(&ctx.kern, dt_hw_s, &ctx.c_hw, &ctx.m_hw) != 0)
    {
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }
    ctx.m_ring = ctx.m_hw + 1u;

    /* 6. Allocate real-time working buffers & look-ahead ring */
    const size_t slot_len = (ctx.kern.kmode >= 2u) ? (size_t) ctx.n_dm : (size_t) ctx.kern.n_in;
    ctx.u_base            = (float *) calloc(ctx.n_dm, sizeof(float));
    ctx.u_prev            = (float *) calloc(ctx.n_dm, sizeof(float));
    ctx.du                = (float *) calloc(ctx.n_dm, sizeof(float));
    ctx.u_out             = (float *) calloc(ctx.n_dm, sizeof(float));
    ctx.dc                = (float *) calloc(ctx.kern.n_in, sizeof(float));
    ctx.ring              = (float *) calloc((size_t) ctx.m_ring * slot_len, sizeof(float));
    ctx.row_nz            = (uint8_t *) calloc(ctx.dmy, sizeof(uint8_t));

    if (!ctx.u_base || !ctx.u_prev || !ctx.du || !ctx.u_out || !ctx.dc || !ctx.ring || !ctx.row_nz)
    {
        free(ctx.row_nz);
        free(ctx.ring);
        free(ctx.dc);
        free(ctx.u_out);
        free(ctx.du);
        free(ctx.u_prev);
        free(ctx.u_base);
        free(ctx.c_hw);
        dmps_kernel_free(&ctx.kern);
        return RETURN_FAILURE;
    }

    /* Sync initial state cleanly to current streamin without firing a startup transient */
    memcpy(ctx.u_base, ctx.img_in.im->array.F, sizeof(float) * ctx.n_dm);
    memcpy(ctx.u_prev, ctx.u_base, sizeof(float) * ctx.n_dm);
    ctx.last_in_cnt0 = ctx.img_in.md->cnt0;

    ctx.sem_in = ImageStreamIO_getsemwaitindex(ctx.img_in.im, -1);
    if (ctx.sem_in < 0)
    {
        ctx.sem_in = 0;
    }
    ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);

    if (ctx.use_trig)
    {
        ctx.sem_trig = ImageStreamIO_getsemwaitindex(ctx.img_trig.im, -1);
        if (ctx.sem_trig < 0)
        {
            ctx.sem_trig = 0;
        }
        ImageStreamIO_semflush(ctx.img_trig.im, ctx.sem_trig);
    }

    const float eff_rate_hz = (float) (1.0 / dt_hw_s);
    o_rate_hz               = eff_rate_hz;
    if (dcfpsptr != NULL)
    {
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    MILK_PROCINFO_INIT(self);
    if (processinfo != NULL)
    {
        processinfo_WriteMessage_fmt(processinfo,
                                     "KMODE=%u rate=%.0fHz M_hw=%u (%s -> %s)", ctx.kern.kmode,
                                     1.0 / dt_hw_s, ctx.m_hw, p_streamin, p_streamout);
    }

    /* Emit initial settled state so downstream consumers see streamout alive immediately */
    dmps_publish_out(&ctx, ctx.u_base, p_clip_enable);

    uint64_t next_tick_ns  = dmps_now_ns() + dt_hw_ns;
    uint64_t resync_ticker = 0;

    MILK_PROCINFO_LOOPSTART(self);
    {
        /* Resync live WRITERUN params (.enable, .gain, .clip_*) and publish telemetry */
        if (((resync_ticker++ & 15ULL) == 0ULL || ctx.active_rem == 0u) && dcfpsptr != NULL)
        {
            fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
            o_rate_hz   = eff_rate_hz;
            o_in_cnt    = ctx.in_cnt;
            o_out_cnt   = ctx.out_cnt;
            o_peak_corr = ctx.peak_corr;
            o_clip_cnt  = ctx.clip_cnt;
            fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
        }

        if (ctx.use_trig)
        {
            int wrc = dmps_semwait_ms(ctx.img_trig.im, ctx.sem_trig, 100ULL, processinfo);
            if (wrc == -2 || dmps_exit_requested(processinfo))
            {
                break;
            }
            if (wrc == 0)
            {
                (void) dmps_ingest_input(&ctx);
                dmps_emit_tick(&ctx);
            }
        }
        else if (ctx.active_rem == 0u && !p_freerun)
        {
            /* Idle: zero-latency semaphore wake on streamin */
            int got = dmps_ingest_input(&ctx);
            if (!got)
            {
                int wrc = dmps_semwait_ms(ctx.img_in.im, ctx.sem_in, 50ULL, processinfo);
                if (wrc == -2 || dmps_exit_requested(processinfo))
                {
                    break;
                }
                got = dmps_ingest_input(&ctx);
            }
            if (got)
            {
                ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
                uint64_t t0  = dmps_now_ns();
                dmps_emit_tick(&ctx);
                next_tick_ns = t0 + dt_hw_ns;
            }
        }
        else
        {
            /* Active look-ahead draining (or freerun): strict high-rate wall-clock pacing */
            dmps_sleep_until_ns(next_tick_ns, p_spinus, processinfo);
            if (dmps_exit_requested(processinfo))
            {
                break;
            }
            if (dmps_ingest_input(&ctx))
            {
                ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
            }
            dmps_emit_tick(&ctx);

            next_tick_ns += dt_hw_ns;
            uint64_t now  = dmps_now_ns();
            if (next_tick_ns + dt_hw_ns < now)
            {
                next_tick_ns = now + dt_hw_ns;
            }
        }
    }
    MILK_PROCINFO_END(self);

    /* Safe shutdown: flush look-ahead spikes and leave DM on exact settled unshaped u_base */
    dmps_clear_ring(&ctx);
    dmps_publish_out(&ctx, ctx.u_base, p_clip_enable);

    if (dcfpsptr != NULL)
    {
        o_rate_hz   = eff_rate_hz;
        o_in_cnt    = ctx.in_cnt;
        o_out_cnt   = ctx.out_cnt;
        o_peak_corr = ctx.peak_corr;
        o_clip_cnt  = ctx.clip_cnt;
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    free(ctx.row_nz);
    free(ctx.ring);
    free(ctx.dc);
    free(ctx.u_out);
    free(ctx.du);
    free(ctx.u_prev);
    free(ctx.u_base);
    free(ctx.c_hw);
    dmps_kernel_free(&ctx.kern);

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_DM_preshaper = {
    .kind = MILK_RUNNABLE_FPS,
    .app = {
        .fps_name         = "dmpreshaper",
        .cmdkey           = "dmpreshaper",
        .description      = "High-rate asynchronous look-ahead DM command pre-shaper",
        .description_long = "Transparent real-time look-ahead pre-shaper layer placed before DM "
                            "hardware. Runs asynchronously at DM hardware rate using a calibrated "
                            "3D RM kernel (cacaocc-compdmpreshaper) to shorten rise time and "
                            "suppress overshoot and mechanical plate waves.",
    },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(dmpreshaper),
    .compute = dmpreshaper_compute,
};

EXPORT_FPSEXEC(cacao, "dmpreshaper", runnable_AOloopControl_DM_preshaper);
