// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    measure_dyn_resp.c
 * @brief   Acquire raw 3D spatiotemporal linear response matrix (mode_out x mode_in x time)
 *
 * Acquires the exposure-integrated dynamic step response between an input perturbation stream
 * (e.g. DM command channel) and an output sensor stream (e.g. WFS pixels or modal stream) at
 * fixed camera frame rate and exposure time by varying the sub-frame time offset between DM
 * pokes and WFS acquisition (stroboscopic sampling, generalizing mlat to arbitrary mode cubes).
 *
 * Output layout follows CACAO's 3D selfRM convention:
 *   FITS axes: [NAXIS1 = N_out, NAXIS2 = N_in, NAXIS3 = N_t] (float32)
 *   C memory:  acc[((size_t) t * N_in + m_in) * N_out + m_out]
 */

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef __linux__
#    include <sys/prctl.h>
#endif
#include <time.h>
#include <unistd.h>
#ifdef _OPENMP
#    include <omp.h>
#endif

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "processinfo.h"
#include "timeutils.h"

#include "AOloopControl_acquireCalib.h"

#if defined(__x86_64__) || defined(__i386__)
#    include <immintrin.h>
#    define MDR_CPU_PAUSE() _mm_pause()
#else
#    define MDR_CPU_PAUSE() ((void) 0)
#endif

/* ================================================================
 * 1.  LOCAL PARAMETER VARIABLES
 * ============================================================= */

static char     p_streamin[FUNCTION_PARAMETER_STRMAXLEN]   = "";
static char     p_streamout[FUNCTION_PARAMETER_STRMAXLEN]  = "";
static char     p_inmodes[FUNCTION_PARAMETER_STRMAXLEN]    = "";
static char     p_outmask[FUNCTION_PARAMETER_STRMAXLEN]    = "";
static char     p_trigstream[FUNCTION_PARAMETER_STRMAXLEN] = "";
static char     p_outcube[FUNCTION_PARAMETER_STRMAXLEN]    = "resp3D.fits";
static char     p_outdir[FUNCTION_PARAMETER_STRMAXLEN]     = "";
static float    p_ampl                                     = 0.01f;
static int32_t  p_normalize                                = 0;
static float    p_wfsfrequ                                 = 0.0f;
static float    p_frameratewait                            = 1.0f;
static uint32_t p_nbsubfr                                  = 8;
static uint32_t p_nbrespfr                                 = 16;
static uint32_t p_nbprefr                                  = 2;
static uint32_t p_nbsettlefr                               = 2;
static uint32_t p_nbcycle                                  = 1;
static uint32_t p_spinus                                   = 60;
static uint32_t p_pokemode                                 = 0;
static uint32_t p_phaseorder                               = 0;
static int32_t  p_saveall                                  = 0;

static float    o_framerateHz    = 0.0f;
static float    o_dt_us          = 0.0f;
static float    o_jitter_us      = 0.0f;
static uint64_t o_dropped_frames = 0;
static float    o_progress_pct   = 0.0f;

#define FPS_PARAMS_MEASDYNRESP(X)                                                                  \
    X(".streamin", p_streamin, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "input (DM/perturbation) stream")                                                            \
    X(".streamout", p_streamout, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "output (WFS/sensed) stream")                                                                \
    X(".inmodes", p_inmodes, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "input poke modes 3D FITS [dmx, dmy, N_in]")                                                 \
    X(".outcube", p_outcube, FPTYPE_FITSFILENAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "output 3D response FITS [N_out, N_in, N_t]")                                                \
    X(".outmask", p_outmask, FPTYPE_FITSFILENAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "optional 2D FITS mask for streamout pixels")                                                \
    X(".trigstream", p_trigstream, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "optional fast clock stream for lockstep sub-frame stepping")                                \
    X(".outdir", p_outdir, FPTYPE_DIRNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "optional output dir for aux files (empty = beside outcube)")                                \
    X(".ampl", &p_ampl, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,                \
      "poke amplitude [streamin units]")                                                           \
    X(".normalize", &p_normalize, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,        \
      "normalize each streamout frame by its sum")                                                 \
    X(".timing.WFSfrequ", &p_wfsfrequ, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, \
      "known WFS frame rate [Hz] (0 = auto-measure)")                                              \
    X(".timing.frameratewait", &p_frameratewait, FPTYPE_FLOAT32, 0,                                \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "startup frame-rate measurement duration [s]")      \
    X(".timing.NBsubfr", &p_nbsubfr, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "sub-frame phase offsets per WFS frame")                                                     \
    X(".timing.NBrespfr", &p_nbrespfr, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "post-poke response window [WFS frames]")                                                    \
    X(".timing.NBprefr", &p_nbprefr, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "pre-poke baseline frames averaged")                                                         \
    X(".timing.NBsettlefr", &p_nbsettlefr, FPTYPE_UINT32, 0,                                       \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT, "extra settle frames per segment")                  \
    X(".timing.NBcycle", &p_nbcycle, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "outer averaging cycles per run")                                                            \
    X(".timing.spinus", &p_spinus, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "final busy-wait threshold [us] after clock_nanosleep")                                      \
    X(".pokemode", &p_pokemode, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,         \
      "0: 0->+a->0->-a->0 | 1: +a<->-a | 2: 0->+a->0")                                             \
    X(".phaseorder", &p_phaseorder, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,     \
      "0: bit-reversed | 1: sequential | 2: random")                                               \
    X(".saveALL", &p_saveall, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,            \
      "save asymmetry cube and detailed metadata")                                                 \
    X(".out.framerateHz", &o_framerateHz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                \
      "measured/effective WFS frame rate [Hz]")                                                    \
    X(".out.dt_us", &o_dt_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                            \
      "effective time-bin resolution [us]")                                                        \
    X(".out.jitter_us", &o_jitter_us, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "measured poke timing RMS jitter [us]")                                                      \
    X(".out.dropped_frames", &o_dropped_frames, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,           \
      "detected dropped WFS frames")                                                               \
    X(".out.progress_pct", &o_progress_pct, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,              \
      "acquisition progress [%]")

MILK_PARAMS_DEFINE(cacaoac_measdynresp, FPS_PARAMS_MEASDYNRESP);

/* ================================================================
 * 2.  UTILITY & TIMING HELPERS
 * ============================================================= */

static inline uint64_t mdr_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static inline int mdr_exit_requested(PROCESSINFO *processinfo)
{
    return (processinfo != NULL && processinfo_exit_requested(processinfo)) ? 1 : 0;
}

static void mdr_sleep_until_ns(uint64_t target_ns, uint32_t spin_us, PROCESSINFO *processinfo)
{
    uint64_t now = mdr_now_ns();
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
            if (mdr_exit_requested(processinfo))
            {
                return;
            }
        }
    }

    while (mdr_now_ns() < target_ns)
    {
        if (mdr_exit_requested(processinfo))
        {
            return;
        }
        MDR_CPU_PAUSE();
    }
}

static int mdr_semwait_timeout(IMAGE *im, int semidx, double timeout_s, PROCESSINFO *processinfo)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t add_ns = (uint64_t) (timeout_s * 1.0e9);
    uint64_t nsec   = (uint64_t) ts.tv_nsec + add_ns;
    ts.tv_sec      += (time_t) (nsec / 1000000000ULL);
    ts.tv_nsec      = (long) (nsec % 1000000000ULL);

    for (;;)
    {
        int rc = ImageStreamIO_semtimedwait(im, semidx, &ts);
        if (rc == 0)
        {
            return 0;
        }
        if (errno == EINTR)
        {
            if (mdr_exit_requested(processinfo))
            {
                return -2;
            }
            continue;
        }
        return -1;
    }
}

static int mdr_cmp_double_asc(const void *a, const void *b)
{
    double da = *(const double *) a;
    double db = *(const double *) b;
    return (da > db) - (da < db);
}

static uint32_t mdr_bitrev32(uint32_t v)
{
    v = ((v >> 1) & 0x55555555u) | ((v & 0x55555555u) << 1);
    v = ((v >> 2) & 0x33333333u) | ((v & 0x33333333u) << 2);
    v = ((v >> 4) & 0x0F0F0F0Fu) | ((v & 0x0F0F0F0Fu) << 4);
    v = ((v >> 8) & 0x00FF00FFu) | ((v & 0x00FF00FFu) << 8);
    return (v >> 16) | (v << 16);
}

typedef struct
{
    uint32_t idx;
    uint32_t key;
} MDR_PHASE_KEY;

static int mdr_cmp_phase_key(const void *a, const void *b)
{
    uint32_t ka = ((const MDR_PHASE_KEY *) a)->key;
    uint32_t kb = ((const MDR_PHASE_KEY *) b)->key;
    return (ka > kb) - (ka < kb);
}

static void mdr_build_phase_order(uint32_t *order, uint32_t n_subfr, uint32_t phaseorder)
{
    if (phaseorder == 0 && n_subfr > 1)
    {
        MDR_PHASE_KEY *keys = (MDR_PHASE_KEY *) malloc(sizeof(MDR_PHASE_KEY) * n_subfr);
        if (keys != NULL)
        {
            for (uint32_t i = 0; i < n_subfr; i++)
            {
                keys[i].idx = i;
                keys[i].key = mdr_bitrev32(i);
            }
            qsort(keys, n_subfr, sizeof(MDR_PHASE_KEY), mdr_cmp_phase_key);
            for (uint32_t i = 0; i < n_subfr; i++)
            {
                order[i] = keys[i].idx;
            }
            free(keys);
            return;
        }
    }

    for (uint32_t i = 0; i < n_subfr; i++)
    {
        order[i] = i;
    }
}

static uint64_t mdr_xorshift64(uint64_t *state)
{
    uint64_t x = *state;
    if (x == 0)
    {
        x = 0x9E3779B97F4A7C15ULL;
    }
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    return x;
}

static double mdr_rand_u01(uint64_t *state)
{
    uint64_t r = mdr_xorshift64(state) >> 11;
    return ((double) r + 0.5) * (1.0 / 9007199254740992.0);
}

static void mdr_shuffle_modes(uint32_t *seq, uint32_t n_modes, uint64_t seed)
{
    for (uint32_t i = 0; i < n_modes; i++)
    {
        seq[i] = i;
    }
    if (seed == 0 || n_modes < 2)
    {
        return;
    }
    uint64_t rng = seed * 0xBF58476D1CE4E5B9ULL + 0x94D049BB133111EBULL;
    for (uint32_t i = n_modes - 1; i > 0; i--)
    {
        uint32_t j = (uint32_t) (mdr_xorshift64(&rng) % (uint64_t) (i + 1));
        uint32_t t = seq[i];
        seq[i]     = seq[j];
        seq[j]     = t;
    }
}

static int mdr_mkdir_p(const char *path)
{
    if (path == NULL || path[0] == '\0')
    {
        return 0;
    }
    char tmp[STRINGMAXLEN_FULLFILENAME];
    snprintf(tmp, sizeof(tmp), "%s", path);
    size_t len = strlen(tmp);
    while (len > 1 && tmp[len - 1] == '/')
    {
        tmp[len - 1] = '\0';
        len--;
    }
    for (char *p = tmp + 1; *p != '\0'; p++)
    {
        if (*p == '/')
        {
            *p = '\0';
            if (mkdir(tmp, 0775) != 0 && errno != EEXIST)
            {
                return -1;
            }
            *p = '/';
        }
    }
    if (mkdir(tmp, 0775) != 0 && errno != EEXIST)
    {
        return -1;
    }
    return 0;
}

static void mdr_ensure_parent_dir(const char *filepath)
{
    const char *slash = strrchr(filepath, '/');
    if (slash == NULL || slash == filepath)
    {
        return;
    }
    char   dir[STRINGMAXLEN_FULLFILENAME];
    size_t dlen = (size_t) (slash - filepath);
    if (dlen >= sizeof(dir))
    {
        dlen = sizeof(dir) - 1;
    }
    memcpy(dir, filepath, dlen);
    dir[dlen] = '\0';
    mdr_mkdir_p(dir);
}

/* ================================================================
 * 3.  FITS I/O WITH METADATA KEYWORDS
 * ============================================================= */

typedef struct
{
    double   wfsfreq;
    double   wfsdt;
    uint32_t nbsubfr;
    double   dt_sec;
    uint32_t q0_bin;
    uint32_t nbrespfr;
    uint32_t nbprefr;
    double   ampl;
    uint32_t pokemode;
    uint64_t n_out;
    uint32_t n_in;
    uint32_t n_t;
    uint32_t wfsx;
    uint32_t wfsy;
    uint32_t dmx;
    uint32_t dmy;
    double   jitter_us;
    uint64_t dropped_frames;
    uint64_t late_pokes;
    int      lockstep;
    uint32_t completed_cycles;
} MDR_META;

static int mdr_load_fits_f32(const char *fname,
                             float     **out_data,
                             int        *out_naxis,
                             long        out_naxes[3])
{
    *out_data      = NULL;
    *out_naxis     = 0;
    out_naxes[0]   = 1;
    out_naxes[1]   = 1;
    out_naxes[2]   = 1;

#ifdef USE_CFITSIO
    fitsfile *fptr   = NULL;
    int       status = 0;

    if (fits_open_file(&fptr, fname, READONLY, &status) != 0)
    {
        fprintf(stderr, "measdynresp: cannot open FITS '%s' (cfitsio status=%d)\n", fname, status);
        return -1;
    }

    int  naxis    = 0;
    long naxes[4] = { 1, 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || naxis < 1 || naxis > 3)
    {
        fprintf(stderr, "measdynresp: unsupported FITS naxis=%d in '%s'\n", naxis, fname);
        fits_close_file(fptr, &status);
        return -1;
    }
    fits_get_img_size(fptr, naxis, naxes, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    size_t nelem = 1;
    for (int i = 0; i < naxis; i++)
    {
        out_naxes[i] = naxes[i];
        nelem       *= (size_t) naxes[i];
    }
    *out_naxis = naxis;

    float *buf = (float *) malloc(sizeof(float) * nelem);
    if (buf == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, (LONGLONG) nelem, NULL, buf, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(buf);
        fprintf(stderr, "measdynresp: error reading FITS '%s' (status=%d)\n", fname, status);
        return -1;
    }
    *out_data = buf;
    return 0;
#else
    (void) fname;
    fprintf(stderr, "measdynresp: built without CFITSIO support\n");
    return -1;
#endif
}

static int mdr_write_fits_atomic(const char     *raw_path,
                                 int             bitpix,
                                 int             datatype,
                                 int             naxis,
                                 const long     *naxes,
                                 const void     *data,
                                 const MDR_META *meta)
{
    if (raw_path == NULL || raw_path[0] == '\0')
    {
        return -1;
    }
    const char *path = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    mdr_ensure_parent_dir(path);

#ifdef USE_CFITSIO
    char tmp_path[STRINGMAXLEN_FULLFILENAME];
    char cfits_path[STRINGMAXLEN_FULLFILENAME];
    snprintf(tmp_path, sizeof(tmp_path), "%s.%d.tmp", path, (int) getpid());
    snprintf(cfits_path, sizeof(cfits_path), "!%s", tmp_path);

    fitsfile *fptr   = NULL;
    int       status = 0;

    if (fits_create_file(&fptr, cfits_path, &status) != 0)
    {
        fprintf(stderr, "measdynresp: fits_create_file '%s' failed (status=%d)\n", tmp_path,
                status);
        return -1;
    }

    LONGLONG nelem = 1;
    long     ax[3] = { 1, 1, 1 };
    for (int i = 0; i < naxis; i++)
    {
        ax[i]  = naxes[i];
        nelem *= (LONGLONG) naxes[i];
    }

    fits_create_img(fptr, bitpix, naxis, ax, &status);

    if (meta != NULL)
    {
        double dval;
        long   lval;

        dval = meta->wfsfreq;
        fits_update_key(fptr, TDOUBLE, "WFSFREQ", &dval, "WFS frame rate [Hz]", &status);
        dval = meta->wfsdt;
        fits_update_key(fptr, TDOUBLE, "WFSDT", &dval, "WFS frame period [s]", &status);
        lval = (long) meta->nbsubfr;
        fits_update_key(fptr, TLONG, "NBSUBFR", &lval, "Sub-frame phase bins per WFS frame",
                        &status);
        dval = meta->dt_sec;
        fits_update_key(fptr, TDOUBLE, "DT_SEC", &dval, "Time-bin width [s]", &status);
        lval = (long) meta->q0_bin;
        fits_update_key(fptr, TLONG, "Q0_BIN", &lval, "Zero-lag bin index", &status);
        lval = (long) meta->nbrespfr;
        fits_update_key(fptr, TLONG, "NBRESPFR", &lval, "Post-poke response window [frames]",
                        &status);
        lval = (long) meta->nbprefr;
        fits_update_key(fptr, TLONG, "NBPREFR", &lval, "Pre-poke baseline frames averaged",
                        &status);
        dval = meta->ampl;
        fits_update_key(fptr, TDOUBLE, "AMPL", &dval, "Nominal poke amplitude", &status);
        lval = (long) meta->pokemode;
        fits_update_key(fptr, TLONG, "POKEMODE", &lval, "0=4seg, 1=pushpull, 2=unipolar", &status);
        lval = (long) meta->n_out;
        fits_update_key(fptr, TLONG, "NBOUT", &lval, "Output axis size (sensed modes/pixels)",
                        &status);
        lval = (long) meta->n_in;
        fits_update_key(fptr, TLONG, "NBIN", &lval, "Input axis size (poked modes)", &status);
        lval = (long) meta->n_t;
        fits_update_key(fptr, TLONG, "NBT", &lval, "Time axis size (bins)", &status);
        lval = (long) meta->wfsx;
        fits_update_key(fptr, TLONG, "WFSSIZEX", &lval, "Original streamout X size", &status);
        lval = (long) meta->wfsy;
        fits_update_key(fptr, TLONG, "WFSSIZEY", &lval, "Original streamout Y size", &status);
        lval = (long) meta->dmx;
        fits_update_key(fptr, TLONG, "DMSIZEX", &lval, "Original streamin X size", &status);
        lval = (long) meta->dmy;
        fits_update_key(fptr, TLONG, "DMSIZEY", &lval, "Original streamin Y size", &status);
        dval = meta->jitter_us;
        fits_update_key(fptr, TDOUBLE, "JITTERUS", &dval, "Poke timing RMS jitter [us]", &status);
        lval = (long) meta->dropped_frames;
        fits_update_key(fptr, TLONG, "NDROP", &lval, "Detected dropped WFS frames", &status);
        lval = (long) meta->late_pokes;
        fits_update_key(fptr, TLONG, "NLATEPK", &lval, "Skipped late-poke overruns", &status);
        lval = (long) meta->lockstep;
        fits_update_key(fptr, TLONG, "LOCKSTEP", &lval, "1=trigstream lockstep, 0=wall-clock",
                        &status);
        lval = (long) meta->completed_cycles;
        fits_update_key(fptr, TLONG, "NCYCLE", &lval, "Completed outer cycles", &status);
    }

    fits_write_img(fptr, datatype, 1, nelem, (void *) data, &status);
    fits_close_file(fptr, &status);

    if (status != 0)
    {
        remove(tmp_path);
        fprintf(stderr, "measdynresp: FITS write failed for '%s' (status=%d)\n", path, status);
        return -1;
    }

    if (rename(tmp_path, path) != 0)
    {
        remove(tmp_path);
        fprintf(stderr, "measdynresp: rename '%s' -> '%s' failed: %s\n", tmp_path, path,
                strerror(errno));
        return -1;
    }
    return 0;
#else
    (void) bitpix;
    (void) datatype;
    (void) naxis;
    (void) naxes;
    (void) data;
    (void) meta;
    return -1;
#endif
}

/* ================================================================
 * 4.  TYPED STREAM READ / DM PUBLISH HELPERS
 * ============================================================= */

static int mdr_read_wfs_frame(const IMAGE    *im,
                              const uint64_t *out_idx,
                              uint64_t        n_out,
                              int             norm_flag,
                              float          *dst)
{
    switch (im->md->datatype)
    {
    case _DATATYPE_FLOAT:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = im->array.F[out_idx[i]];
        }
        break;
    case _DATATYPE_DOUBLE:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.D[out_idx[i]];
        }
        break;
    case _DATATYPE_UINT16:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.UI16[out_idx[i]];
        }
        break;
    case _DATATYPE_INT16:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.SI16[out_idx[i]];
        }
        break;
    case _DATATYPE_UINT32:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.UI32[out_idx[i]];
        }
        break;
    case _DATATYPE_INT32:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.SI32[out_idx[i]];
        }
        break;
    case _DATATYPE_UINT64:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.UI64[out_idx[i]];
        }
        break;
    case _DATATYPE_INT64:
        for (uint64_t i = 0; i < n_out; i++)
        {
            dst[i] = (float) im->array.SI64[out_idx[i]];
        }
        break;
    default:
        return -1;
    }

    if (norm_flag)
    {
        double sum = 0.0;
        for (uint64_t i = 0; i < n_out; i++)
        {
            sum += (double) dst[i];
        }
        if (fabs(sum) > 1.0e-20)
        {
            float inv = (float) (1.0 / sum);
            for (uint64_t i = 0; i < n_out; i++)
            {
                dst[i] *= inv;
            }
        }
    }
    return 0;
}

static void mdr_publish_dm(PROCESSINFO *processinfo,
                           IMGID       *imgin,
                           const float *pattern,
                           float        scale,
                           uint64_t     n_dm,
                           uint64_t     cnt1_tag)
{
    imgin->md->write = 1;
    float *dst       = imgin->im->array.F;
    if (pattern == NULL || scale == 0.0f)
    {
        memset(dst, 0, sizeof(float) * n_dm);
    }
    else
    {
        for (uint64_t i = 0; i < n_dm; i++)
        {
            dst[i] = scale * pattern[i];
        }
    }
    imgin->md->cnt1 = cnt1_tag;
    processinfo_update_output_stream(processinfo, imgin->im, NULL);
}

/* ================================================================
 * 5.  ACQUISITION STATE & CORE TRANSITION ENGINE
 * ============================================================= */

typedef struct
{
    IMGID        imgin;
    IMGID        imgout;
    IMGID        imgtrig;
    int          sem_out;
    int          sem_trig;
    int          use_lockstep;
    PROCESSINFO *processinfo;

    uint32_t dmx;
    uint32_t dmy;
    uint64_t n_dm;
    uint32_t wfsx;
    uint32_t wfsy;
    uint64_t n_out;
    uint32_t n_in;
    uint32_t n_subfr;
    uint32_t n_respfr;
    uint32_t n_prefr;
    uint32_t n_settlefr;
    uint32_t n_hold;
    uint32_t q0_bin;
    uint32_t n_t;

    double   t_grid_s;
    double   dt_bin_s;
    double   t_live_ns;
    float    ampl;
    int      normalize;
    uint32_t spinus;
    uint32_t pokemode;
    uint32_t phaseorder;
    int      saveall;

    float    *inmodes;  /* [n_in x n_dm] */
    uint64_t *out_idx;  /* [n_out] */
    float    *pre_ring; /* [n_prefr x n_out] */
    uint32_t  pre_head;
    float    *base_vec; /* [n_out] */
    float    *k0_vec;   /* [n_out] */
    float    *cur_vec;  /* [n_out] */

    /* Accumulators: FITS order [N_out, N_in, N_t] => C [t][m_in][out] */
    float    *acc;       /* [n_t * n_in * n_out] */
    float    *asym_acc;  /* [n_t * n_in * n_out] if saveall */
    double   *wsum;      /* [n_t * n_in] */
    double   *asym_wsum; /* [n_t * n_in] if saveall (w_+ - w_-) */
    double   *tsum;      /* [n_t * n_in] */
    uint32_t *cnt;       /* [n_t * n_in] */

    uint64_t last_wfs_t_ns;
    uint64_t last_cnt0;
    uint64_t dropped_frames;
    uint64_t late_pokes;
    double   jitter_sq_sum_ns2;
    uint64_t jitter_cnt;
    uint64_t rng_state;
} MDR_CTX;

static void mdr_push_pre_ring(MDR_CTX *ctx, const float *frame)
{
    memcpy(&ctx->pre_ring[(size_t) ctx->pre_head * ctx->n_out], frame, sizeof(float) * ctx->n_out);
    ctx->pre_head = (ctx->pre_head + 1u) % ctx->n_prefr;
}

static void mdr_compute_baseline(MDR_CTX *ctx)
{
    const uint64_t n_out = ctx->n_out;
    const uint32_t n_pre = ctx->n_prefr;
    const float    inv   = 1.0f / (float) n_pre;

    memset(ctx->base_vec, 0, sizeof(float) * n_out);
    for (uint32_t p = 0; p < n_pre; p++)
    {
        const float *src = &ctx->pre_ring[(size_t) p * n_out];
        for (uint64_t i = 0; i < n_out; i++)
        {
            ctx->base_vec[i] += src[i];
        }
    }
    for (uint64_t i = 0; i < n_out; i++)
    {
        ctx->base_vec[i] *= inv;
    }

    uint32_t latest = (ctx->pre_head + n_pre - 1u) % n_pre;
    memcpy(ctx->k0_vec, &ctx->pre_ring[(size_t) latest * n_out], sizeof(float) * n_out);
}

static int mdr_wait_next_wfs(MDR_CTX *ctx, int track_drops)
{
    int rc = mdr_semwait_timeout(ctx->imgout.im, ctx->sem_out, 2.0, ctx->processinfo);
    if (rc != 0)
    {
        return rc;
    }
    uint64_t t_now    = mdr_now_ns();
    uint64_t cnt0_now = ctx->imgout.md->cnt0;

    if (mdr_read_wfs_frame(ctx->imgout.im, ctx->out_idx, ctx->n_out, ctx->normalize,
                           ctx->cur_vec) != 0)
    {
        return -1;
    }

    if (track_drops && ctx->last_wfs_t_ns != 0)
    {
        uint64_t dcnt = (cnt0_now >= ctx->last_cnt0) ? (cnt0_now - ctx->last_cnt0) : 1ULL;
        double   dt   = (double) (t_now - ctx->last_wfs_t_ns);
        if (dcnt > 1ULL)
        {
            ctx->dropped_frames += (dcnt - 1ULL);
        }
        else if (!ctx->use_lockstep && ctx->t_live_ns > 0.0 && dt > 1.6 * ctx->t_live_ns)
        {
            ctx->dropped_frames++;
        }
        else if (!ctx->use_lockstep && dt > 0.90 * ctx->t_live_ns && dt < 1.10 * ctx->t_live_ns)
        {
            ctx->t_live_ns = 0.99 * ctx->t_live_ns + 0.01 * dt;
        }
    }

    ctx->last_wfs_t_ns = t_now;
    ctx->last_cnt0     = cnt0_now;
    mdr_push_pre_ring(ctx, ctx->cur_vec);
    return 0;
}

static void mdr_accum_sample(MDR_CTX     *ctx,
                             uint32_t     mode_idx,
                             int          q,
                             double       tau_s,
                             const float *frame,
                             float        step_ampl,
                             float        asym_sign)
{
    if (q < 0 || (uint32_t) q >= ctx->n_t)
    {
        return;
    }
    const uint64_t n_out = ctx->n_out;
    const size_t   bin   = (size_t) q * ctx->n_in + mode_idx;
    float         *dst   = &ctx->acc[bin * n_out];
    const float   *base  = ctx->base_vec;

#pragma omp simd
    for (uint64_t i = 0; i < n_out; i++)
    {
        dst[i] += step_ampl * (frame[i] - base[i]);
    }

    const double w_samp = (double) step_ampl * (double) step_ampl;

    if (ctx->asym_acc != NULL && asym_sign != 0.0f)
    {
        float       *adst = &ctx->asym_acc[bin * n_out];
        const float  w    = asym_sign * step_ampl;
#pragma omp simd
        for (uint64_t i = 0; i < n_out; i++)
        {
            adst[i] += w * (frame[i] - base[i]);
        }
        if (ctx->asym_wsum != NULL)
        {
            ctx->asym_wsum[bin] += (double) asym_sign * w_samp;
        }
    }

    ctx->wsum[bin] += w_samp;
    ctx->tsum[bin] += tau_s;
    ctx->cnt[bin]  += 1u;
}

static int mdr_settle_state(MDR_CTX     *ctx,
                            const float *pattern,
                            float        scale,
                            uint32_t     n_frames,
                            uint64_t     tag)
{
    mdr_publish_dm(ctx->processinfo, &ctx->imgin, pattern, scale, ctx->n_dm, tag);
    for (uint32_t r = 0; r < n_frames; r++)
    {
        int rc = mdr_wait_next_wfs(ctx, 0);
        if (rc != 0)
        {
            return rc;
        }
    }
    return 0;
}

static int mdr_run_transition(MDR_CTX     *ctx,
                              uint32_t     mode_idx,
                              uint32_t     phase_idx,
                              const float *target_pat,
                              float        target_scale,
                              float        step_ampl,
                              float        asym_sign,
                              uint64_t     tag)
{
    mdr_compute_baseline(ctx);
    const uint64_t t_k0    = ctx->last_wfs_t_ns;
    const uint64_t cnt0_k0 = ctx->last_cnt0;
    uint64_t       t_poke  = 0;
    int            skip_ac = 0;

    if (ctx->use_lockstep)
    {
        ImageStreamIO_semflush(ctx->imgtrig.im, ctx->sem_trig);
        for (uint32_t s = 0; s < phase_idx; s++)
        {
            int rc = mdr_semwait_timeout(ctx->imgtrig.im, ctx->sem_trig, 2.0, ctx->processinfo);
            if (rc != 0)
            {
                return rc;
            }
        }
        mdr_publish_dm(ctx->processinfo, &ctx->imgin, target_pat, target_scale, ctx->n_dm, tag);
        t_poke = mdr_now_ns();
    }
    else
    {
        double frac;
        if (ctx->phaseorder == 2)
        {
            frac = mdr_rand_u01(&ctx->rng_state);
        }
        else
        {
            frac = ((double) phase_idx + 0.5) / (double) ctx->n_subfr;
        }
        uint64_t off_ns    = (uint64_t) llround(frac * ctx->t_live_ns);
        uint64_t target_ns = t_k0 + off_ns;
        mdr_sleep_until_ns(target_ns, ctx->spinus, ctx->processinfo);
        if (mdr_exit_requested(ctx->processinfo))
        {
            return -2;
        }

        /* Guard against OS sleep overrun past the next WFS arrival */
        uint64_t cnt0_pre = ctx->imgout.md->cnt0;
        if (cnt0_pre > cnt0_k0)
        {
            ctx->late_pokes++;
            skip_ac = 1;
            /* Consume already-arrived pre-poke frame so post-poke frame count stays aligned */
            if (ImageStreamIO_semtrywait(ctx->imgout.im, ctx->sem_out) == 0)
            {
                ctx->last_wfs_t_ns = mdr_now_ns();
                ctx->last_cnt0     = ctx->imgout.md->cnt0;
                if (mdr_read_wfs_frame(ctx->imgout.im, ctx->out_idx, ctx->n_out, ctx->normalize,
                                       ctx->cur_vec) == 0)
                {
                    mdr_push_pre_ring(ctx, ctx->cur_vec);
                }
            }
        }

        uint64_t t_b = mdr_now_ns();
        mdr_publish_dm(ctx->processinfo, &ctx->imgin, target_pat, target_scale, ctx->n_dm, tag);
        uint64_t t_a = mdr_now_ns();
        t_poke       = t_b + ((t_a - t_b) / 2ULL);

        if (!skip_ac)
        {
            double err_ns           = (double) t_poke - (double) target_ns;
            ctx->jitter_sq_sum_ns2 += err_ns * err_ns;
            ctx->jitter_cnt++;
        }
    }

    /* Bin pre-poke reference frame k0 into negative-lag verification bins [0 .. q0_bin-1] */
    const double dt_bin_live_s =
        (!ctx->use_lockstep && ctx->t_live_ns > 0.0)
            ? ((ctx->t_live_ns * 1.0e-9) / (double) ctx->n_subfr)
            : ctx->dt_bin_s;
    const double tau_scale =
        (!ctx->use_lockstep && dt_bin_live_s > 1.0e-12) ? (ctx->dt_bin_s / dt_bin_live_s) : 1.0;

    if (!skip_ac)
    {
        int    q_pre;
        double tau_pre;
        if (ctx->use_lockstep)
        {
            q_pre   = (int) ctx->q0_bin - (int) phase_idx - 1;
            tau_pre = ((double) (q_pre - (int) ctx->q0_bin) + 0.5) * ctx->dt_bin_s;
        }
        else
        {
            double tau_wall = ((double) t_k0 - (double) t_poke) * 1.0e-9;
            q_pre           = (int) floor(tau_wall / dt_bin_live_s) + (int) ctx->q0_bin;
            tau_pre         = tau_wall * tau_scale;
        }
        mdr_accum_sample(ctx, mode_idx, q_pre, tau_pre, ctx->k0_vec, step_ampl, asym_sign);
    }

    /* Stream NBhold post-poke WFS frames */
    for (uint32_t r = 1; r <= ctx->n_hold; r++)
    {
        int rc = mdr_wait_next_wfs(ctx, 1);
        if (rc != 0)
        {
            return rc;
        }

        if (!skip_ac && r <= ctx->n_respfr)
        {
            int    q;
            double tau_s;
            if (ctx->use_lockstep)
            {
                q     = (int) ctx->q0_bin + (int) (r * ctx->n_subfr) - (int) phase_idx - 1;
                tau_s = ((double) (q - (int) ctx->q0_bin) + 0.5) * ctx->dt_bin_s;
            }
            else
            {
                double tau_wall = ((double) ctx->last_wfs_t_ns - (double) t_poke) * 1.0e-9;
                q               = (int) floor(tau_wall / dt_bin_live_s) + (int) ctx->q0_bin;
                tau_s           = tau_wall * tau_scale;
            }
            mdr_accum_sample(ctx, mode_idx, q, tau_s, ctx->cur_vec, step_ampl, asym_sign);
        }
    }

    return 0;
}

static int mdr_measure_wfs_rate(MDR_CTX *ctx, double wait_s, double *out_dt_s)
{
    const uint32_t max_samp = 2000;
    double        *deltas   = (double *) malloc(sizeof(double) * max_samp);
    if (deltas == NULL)
    {
        return -1;
    }

    ImageStreamIO_semflush(ctx->imgout.im, ctx->sem_out);

    /* Kickstart lockstep benches by publishing zero once */
    mdr_publish_dm(ctx->processinfo, &ctx->imgin, NULL, 0.0f, ctx->n_dm, 0);

    uint64_t t_start  = mdr_now_ns();
    uint64_t t_prev   = 0;
    uint32_t n_deltas = 0;
    uint32_t min_s    = 12;
    double   eff_wait = (wait_s > 0.05) ? wait_s : 0.25;

    while (n_deltas < max_samp)
    {
        int rc = mdr_semwait_timeout(ctx->imgout.im, ctx->sem_out, 2.0, ctx->processinfo);
        if (rc != 0)
        {
            free(deltas);
            return rc;
        }
        uint64_t t_now = mdr_now_ns();
        if (t_prev != 0)
        {
            double d = (double) (t_now - t_prev) * 1.0e-9;
            if (d > 1.0e-7)
            {
                deltas[n_deltas++] = d;
            }
        }
        t_prev = t_now;
        if (n_deltas >= min_s && ((double) (t_now - t_start) * 1.0e-9 >= eff_wait))
        {
            break;
        }
    }

    if (n_deltas < 3)
    {
        free(deltas);
        return -1;
    }

    qsort(deltas, n_deltas, sizeof(double), mdr_cmp_double_asc);
    *out_dt_s = deltas[n_deltas / 2];
    free(deltas);
    return 0;
}

/* ================================================================
 * 6.  OUTPUT COMPILATION & WRITING
 * ============================================================= */

static void mdr_build_aux_path(const char *outcube,
                               const char *outdir,
                               const char *suffix,
                               char       *dst,
                               size_t      dst_sz)
{
    const char *cube = (outcube[0] == '!') ? (outcube + 1) : outcube;
    if (outdir != NULL && outdir[0] != '\0')
    {
        snprintf(dst, dst_sz, "%s/resp3D%s", outdir, suffix);
        return;
    }

    char   base[STRINGMAXLEN_FULLFILENAME];
    size_t len = strlen(cube);
    if (len >= sizeof(base))
    {
        len = sizeof(base) - 1;
    }
    memcpy(base, cube, len);
    base[len] = '\0';
    if (len > 5 && strcmp(base + len - 5, ".fits") == 0)
    {
        base[len - 5] = '\0';
    }
    snprintf(dst, dst_sz, "%s%s", base, suffix);
}

static int mdr_save_outputs(const MDR_CTX *ctx, uint32_t completed_cycles, int write_aux)
{
    const uint64_t n_out   = ctx->n_out;
    const uint32_t n_in    = ctx->n_in;
    const uint32_t n_t     = ctx->n_t;
    const size_t   n_bins  = (size_t) n_t * n_in;
    const size_t   n_elems = n_bins * n_out;

    float *norm_cube = (float *) malloc(sizeof(float) * n_elems);
    if (norm_cube == NULL)
    {
        return -1;
    }

#ifdef _OPENMP
#    pragma omp parallel for schedule(static)
#endif
    for (size_t bin = 0; bin < n_bins; bin++)
    {
        const float *src = &ctx->acc[bin * n_out];
        float       *dst = &norm_cube[bin * n_out];
        if (ctx->wsum[bin] > 1.0e-30)
        {
            float inv = (float) (1.0 / ctx->wsum[bin]);
            for (uint64_t i = 0; i < n_out; i++)
            {
                dst[i] = src[i] * inv;
            }
        }
        else
        {
            memset(dst, 0, sizeof(float) * n_out);
        }
    }

    double jitter_us =
        (ctx->jitter_cnt > 0)
            ? (sqrt(ctx->jitter_sq_sum_ns2 / (double) ctx->jitter_cnt) * 1.0e-3)
            : 0.0;

    MDR_META meta = {
        .wfsfreq          = 1.0 / ctx->t_grid_s,
        .wfsdt            = ctx->t_grid_s,
        .nbsubfr          = ctx->n_subfr,
        .dt_sec           = ctx->dt_bin_s,
        .q0_bin           = ctx->q0_bin,
        .nbrespfr         = ctx->n_respfr,
        .nbprefr          = ctx->n_prefr,
        .ampl             = (double) ctx->ampl,
        .pokemode         = ctx->pokemode,
        .n_out            = n_out,
        .n_in             = n_in,
        .n_t              = n_t,
        .wfsx             = ctx->wfsx,
        .wfsy             = ctx->wfsy,
        .dmx              = ctx->dmx,
        .dmy              = ctx->dmy,
        .jitter_us        = jitter_us,
        .dropped_frames   = ctx->dropped_frames,
        .late_pokes       = ctx->late_pokes,
        .lockstep         = ctx->use_lockstep,
        .completed_cycles = completed_cycles,
    };

    long naxes3[3] = { (long) n_out, (long) n_in, (long) n_t };
    int  rc        = mdr_write_fits_atomic(p_outcube, FLOAT_IMG, TFLOAT, 3, naxes3, norm_cube,
                                           &meta);

    if (write_aux && rc == 0)
    {
        char auxpath[STRINGMAXLEN_FULLFILENAME];

        /* 1. Hit count map [N_in, N_t] (int32) */
        int32_t *cnt_i32 = (int32_t *) malloc(sizeof(int32_t) * n_bins);
        float   *tmean   = (float *) malloc(sizeof(float) * n_bins);
        if (cnt_i32 != NULL && tmean != NULL)
        {
            for (size_t bin = 0; bin < n_bins; bin++)
            {
                cnt_i32[bin] = (int32_t) ctx->cnt[bin];
                tmean[bin]   = (ctx->cnt[bin] > 0)
                                   ? (float) (ctx->tsum[bin] / (double) ctx->cnt[bin])
                                   : 0.0f;
            }
            long naxes2_bt[2] = { (long) n_in, (long) n_t };
            mdr_build_aux_path(p_outcube, p_outdir, "_cnt.fits", auxpath, sizeof(auxpath));
            mdr_write_fits_atomic(auxpath, LONG_IMG, TINT, 2, naxes2_bt, cnt_i32, &meta);

            mdr_build_aux_path(p_outcube, p_outdir, "_tmean.fits", auxpath, sizeof(auxpath));
            mdr_write_fits_atomic(auxpath, FLOAT_IMG, TFLOAT, 2, naxes2_bt, tmean, &meta);
        }
        free(cnt_i32);
        free(tmean);

        /* 2. Settled static 2D RM [N_out, N_in] from last tail bins */
        uint32_t tail_len   = ctx->n_subfr * 2u;
        uint32_t post_bins  = ctx->n_respfr * ctx->n_subfr;
        uint32_t tail_cap   = (post_bins >= 4u) ? (post_bins / 4u) : 1u;
        if (tail_len > tail_cap)
        {
            tail_len = tail_cap;
        }
        if (tail_len < 1u)
        {
            tail_len = 1u;
        }
        uint32_t q_tail0 = n_t - tail_len;

        float *static_rm = (float *) calloc((size_t) n_in * n_out, sizeof(float));
        if (static_rm != NULL)
        {
            for (uint32_t m = 0; m < n_in; m++)
            {
                uint32_t n_valid = 0;
                float   *dst     = &static_rm[(size_t) m * n_out];
                for (uint32_t q = q_tail0; q < n_t; q++)
                {
                    size_t bin = (size_t) q * n_in + m;
                    if (ctx->cnt[bin] > 0)
                    {
                        const float *src = &norm_cube[bin * n_out];
                        for (uint64_t i = 0; i < n_out; i++)
                        {
                            dst[i] += src[i];
                        }
                        n_valid++;
                    }
                }
                if (n_valid > 0)
                {
                    float inv = 1.0f / (float) n_valid;
                    for (uint64_t i = 0; i < n_out; i++)
                    {
                        dst[i] *= inv;
                    }
                }
            }
            long naxes2_rm[2] = { (long) n_out, (long) n_in };
            mdr_build_aux_path(p_outcube, p_outdir, "_static.fits", auxpath, sizeof(auxpath));
            mdr_write_fits_atomic(auxpath, FLOAT_IMG, TFLOAT, 2, naxes2_rm, static_rm, &meta);
        }
        free(static_rm);

        /* 3. Optional polarity-balanced asymmetry cube [N_out, N_in, N_t]: 0.5*(y_+ - y_-) */
        if (ctx->saveall && ctx->asym_acc != NULL && ctx->asym_wsum != NULL)
        {
            float *asym_cube = (float *) malloc(sizeof(float) * n_elems);
            if (asym_cube != NULL)
            {
#ifdef _OPENMP
#    pragma omp parallel for schedule(static)
#endif
                for (size_t bin = 0; bin < n_bins; bin++)
                {
                    const float *asrc  = &ctx->asym_acc[bin * n_out];
                    const float *ssrc  = &ctx->acc[bin * n_out];
                    float       *dst   = &asym_cube[bin * n_out];
                    double       w_pos = 0.5 * (ctx->wsum[bin] + ctx->asym_wsum[bin]);
                    double       w_neg = 0.5 * (ctx->wsum[bin] - ctx->asym_wsum[bin]);
                    if (w_pos > 1.0e-30 && w_neg > 1.0e-30)
                    {
                        float c_sum  = (float) (0.25 / w_pos - 0.25 / w_neg);
                        float c_diff = (float) (0.25 / w_pos + 0.25 / w_neg);
                        for (uint64_t i = 0; i < n_out; i++)
                        {
                            dst[i] = c_sum * ssrc[i] + c_diff * asrc[i];
                        }
                    }
                    else
                    {
                        memset(dst, 0, sizeof(float) * n_out);
                    }
                }
                mdr_build_aux_path(p_outcube, p_outdir, "_asym.fits", auxpath, sizeof(auxpath));
                mdr_write_fits_atomic(auxpath, FLOAT_IMG, TFLOAT, 3, naxes3, asym_cube, &meta);
                free(asym_cube);
            }
        }

        /* 4. Human-readable metadata text file */
        mdr_build_aux_path(p_outcube, p_outdir, "_meta.txt", auxpath, sizeof(auxpath));
        mdr_ensure_parent_dir(auxpath);
        FILE *fp = fopen(auxpath, "w");
        if (fp != NULL)
        {
            fprintf(fp, "streamin          %s\n", p_streamin);
            fprintf(fp, "streamout         %s\n", p_streamout);
            fprintf(fp, "inmodes           %s\n", p_inmodes);
            fprintf(fp, "outmask           %s\n", p_outmask[0] ? p_outmask : "none");
            fprintf(fp, "trigstream        %s\n", p_trigstream[0] ? p_trigstream : "none");
            fprintf(fp, "wfsfreq_hz        %.6f\n", meta.wfsfreq);
            fprintf(fp, "wfsdt_s           %.9f\n", meta.wfsdt);
            fprintf(fp, "nbsubfr           %u\n", meta.nbsubfr);
            fprintf(fp, "dt_bin_s          %.9f\n", meta.dt_sec);
            fprintf(fp, "q0_bin            %u\n", meta.q0_bin);
            fprintf(fp, "nbrespfr          %u\n", meta.nbrespfr);
            fprintf(fp, "nbprefr           %u\n", meta.nbprefr);
            fprintf(fp, "nbsettlefr        %u\n", ctx->n_settlefr);
            fprintf(fp, "ampl              %.6g\n", meta.ampl);
            fprintf(fp, "pokemode          %u\n", meta.pokemode);
            fprintf(fp, "phaseorder        %u\n", ctx->phaseorder);
            fprintf(fp, "n_out             %lu\n", (unsigned long) meta.n_out);
            fprintf(fp, "n_in              %u\n", meta.n_in);
            fprintf(fp, "n_t               %u\n", meta.n_t);
            fprintf(fp, "jitter_us         %.3f\n", meta.jitter_us);
            fprintf(fp, "dropped_frames    %lu\n", (unsigned long) meta.dropped_frames);
            fprintf(fp, "late_pokes        %lu\n", (unsigned long) meta.late_pokes);
            fprintf(fp, "lockstep          %d\n", meta.lockstep);
            fprintf(fp, "completed_cycles  %u\n", meta.completed_cycles);
            fclose(fp);
        }
    }

    free(norm_cube);
    return rc;
}

/* ================================================================
 * 7.  RUNNABLE ENTRY POINT
 * ============================================================= */

static MILK_HOT errno_t measdynresp_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

#if defined(__linux__) && defined(PR_SET_TIMERSLACK)
    /* Tighten kernel hrtimer slack to 1 us for accurate sub-frame poke scheduling */
    (void) prctl(PR_SET_TIMERSLACK, 1000UL, 0, 0, 0);
#endif

    if (p_streamin[0] == '\0' || p_streamout[0] == '\0' || p_inmodes[0] == '\0')
    {
        fprintf(stderr, "measdynresp: .streamin, .streamout, and .inmodes are required\n");
        return RETURN_FAILURE;
    }

    MDR_CTX ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.imgin.ID    = -1;
    ctx.imgout.ID   = -1;
    ctx.imgtrig.ID  = -1;
    ctx.sem_out     = -1;
    ctx.sem_trig    = -1;
    ctx.rng_state   = 0x243F6A8885A308D3ULL;

    /* 1. Connect to streamin (DM) and streamout (WFS) */
    ctx.imgin = imgid_make_from_name(p_streamin);
    resolveIMGID(&ctx.imgin, ERRMODE_WARN, dcimg, dcnimg);
    if (ctx.imgin.ID == -1)
    {
        fprintf(stderr, "measdynresp: cannot resolve streamin '%s'\n", p_streamin);
        return RETURN_FAILURE;
    }
    if (ctx.imgin.md->datatype != _DATATYPE_FLOAT)
    {
        fprintf(stderr, "measdynresp: streamin '%s' must be float32\n", p_streamin);
        return RETURN_FAILURE;
    }

    ctx.imgout = imgid_make_from_name(p_streamout);
    resolveIMGID(&ctx.imgout, ERRMODE_WARN, dcimg, dcnimg);
    if (ctx.imgout.ID == -1)
    {
        fprintf(stderr, "measdynresp: cannot resolve streamout '%s'\n", p_streamout);
        return RETURN_FAILURE;
    }

    if (p_trigstream[0] != '\0' && strcmp(p_trigstream, "none") != 0 &&
        strcmp(p_trigstream, "null") != 0)
    {
        ctx.imgtrig = imgid_make_from_name(p_trigstream);
        resolveIMGID(&ctx.imgtrig, ERRMODE_WARN, dcimg, dcnimg);
        if (ctx.imgtrig.ID == -1)
        {
            fprintf(stderr, "measdynresp: cannot resolve trigstream '%s'\n", p_trigstream);
            return RETURN_FAILURE;
        }
        ctx.use_lockstep = 1;
    }

    ctx.dmx  = ctx.imgin.md->size[0];
    ctx.dmy  = (ctx.imgin.md->naxis >= 2) ? ctx.imgin.md->size[1] : 1u;
    ctx.n_dm = (uint64_t) ctx.dmx * ctx.dmy;

    ctx.wfsx             = ctx.imgout.md->size[0];
    ctx.wfsy             = (ctx.imgout.md->naxis >= 2) ? ctx.imgout.md->size[1] : 1u;
    uint64_t wfs_raw_pix = (uint64_t) ctx.wfsx * ctx.wfsy;

    /* 2. Load input poke cube [dmx, dmy, N_in] (or 2D [dmx, dmy] => N_in=1) */
    int  in_naxis    = 0;
    long in_naxes[3] = { 1, 1, 1 };
    if (mdr_load_fits_f32(p_inmodes, &ctx.inmodes, &in_naxis, in_naxes) != 0)
    {
        return RETURN_FAILURE;
    }
    if ((uint32_t) in_naxes[0] != ctx.dmx ||
        (in_naxis >= 2 && (uint32_t) in_naxes[1] != ctx.dmy))
    {
        fprintf(stderr, "measdynresp: inmodes (%ldx%ld) does not match streamin (%ux%u)\n",
                in_naxes[0], in_naxes[1], ctx.dmx, ctx.dmy);
        free(ctx.inmodes);
        return RETURN_FAILURE;
    }
    ctx.n_in = (in_naxis >= 3) ? (uint32_t) in_naxes[2] : 1u;
    if (ctx.n_in == 0)
    {
        free(ctx.inmodes);
        return RETURN_FAILURE;
    }

    /* 3. Build output pixel index map (full streamout or masked subset) */
    ctx.out_idx = (uint64_t *) malloc(sizeof(uint64_t) * wfs_raw_pix);
    if (ctx.out_idx == NULL)
    {
        free(ctx.inmodes);
        return RETURN_FAILURE;
    }
    if (p_outmask[0] != '\0' && strcmp(p_outmask, "none") != 0 && strcmp(p_outmask, "null") != 0)
    {
        float *mask_buf    = NULL;
        int    m_naxis     = 0;
        long   m_naxes[3]  = { 1, 1, 1 };
        if (mdr_load_fits_f32(p_outmask, &mask_buf, &m_naxis, m_naxes) != 0)
        {
            free(ctx.out_idx);
            free(ctx.inmodes);
            return RETURN_FAILURE;
        }
        if ((uint64_t) m_naxes[0] * (uint64_t) m_naxes[1] != wfs_raw_pix)
        {
            fprintf(stderr, "measdynresp: outmask (%ldx%ld) does not match streamout (%ux%u)\n",
                    m_naxes[0], m_naxes[1], ctx.wfsx, ctx.wfsy);
            free(mask_buf);
            free(ctx.out_idx);
            free(ctx.inmodes);
            return RETURN_FAILURE;
        }
        uint64_t cnt_m = 0;
        for (uint64_t i = 0; i < wfs_raw_pix; i++)
        {
            if (mask_buf[i] > 0.5f)
            {
                ctx.out_idx[cnt_m++] = i;
            }
        }
        free(mask_buf);
        if (cnt_m == 0)
        {
            fprintf(stderr, "measdynresp: outmask has 0 active pixels\n");
            free(ctx.out_idx);
            free(ctx.inmodes);
            return RETURN_FAILURE;
        }
        ctx.n_out = cnt_m;
    }
    else
    {
        for (uint64_t i = 0; i < wfs_raw_pix; i++)
        {
            ctx.out_idx[i] = i;
        }
        ctx.n_out = wfs_raw_pix;
    }

    /* 4. Sanitize timing and excitation parameters */
    ctx.ampl       = (fabsf(p_ampl) > 1.0e-12f) ? p_ampl : 0.01f;
    ctx.normalize  = (p_normalize != 0) ? 1 : 0;
    ctx.n_subfr    = (p_nbsubfr >= 1u) ? p_nbsubfr : 1u;
    ctx.n_respfr   = (p_nbrespfr >= 1u) ? p_nbrespfr : 16u;
    ctx.n_prefr    = (p_nbprefr >= 1u) ? p_nbprefr : 2u;
    ctx.n_settlefr = p_nbsettlefr;
    ctx.n_hold     = ((ctx.n_respfr > ctx.n_prefr) ? ctx.n_respfr : ctx.n_prefr) + ctx.n_settlefr;
    ctx.q0_bin     = ctx.n_subfr;
    ctx.n_t        = ctx.q0_bin + ctx.n_respfr * ctx.n_subfr;
    ctx.spinus     = p_spinus;
    ctx.pokemode   = (p_pokemode <= 2u) ? p_pokemode : 0u;
    ctx.phaseorder = (p_phaseorder <= 2u) ? p_phaseorder : 0u;
    ctx.saveall    = (p_saveall != 0) ? 1 : 0;

    uint32_t eff_nbcycle = (p_nbcycle >= 1u) ? p_nbcycle : 1u;

    /* 5. Allocate working buffers and 3D accumulators */
    size_t n_bins  = (size_t) ctx.n_t * ctx.n_in;
    size_t n_elems = n_bins * ctx.n_out;

    ctx.pre_ring = (float *) calloc((size_t) ctx.n_prefr * ctx.n_out, sizeof(float));
    ctx.base_vec = (float *) calloc(ctx.n_out, sizeof(float));
    ctx.k0_vec   = (float *) calloc(ctx.n_out, sizeof(float));
    ctx.cur_vec  = (float *) calloc(ctx.n_out, sizeof(float));
    ctx.acc      = (float *) calloc(n_elems, sizeof(float));
    ctx.wsum     = (double *) calloc(n_bins, sizeof(double));
    ctx.tsum     = (double *) calloc(n_bins, sizeof(double));
    ctx.cnt      = (uint32_t *) calloc(n_bins, sizeof(uint32_t));
    if (ctx.saveall)
    {
        ctx.asym_acc  = (float *) calloc(n_elems, sizeof(float));
        ctx.asym_wsum = (double *) calloc(n_bins, sizeof(double));
    }

    uint32_t *phase_seq = (uint32_t *) malloc(sizeof(uint32_t) * ctx.n_subfr);
    uint32_t *mode_seq  = (uint32_t *) malloc(sizeof(uint32_t) * ctx.n_in);

    if (!ctx.pre_ring || !ctx.base_vec || !ctx.k0_vec || !ctx.cur_vec || !ctx.acc || !ctx.wsum ||
        !ctx.tsum || !ctx.cnt || (ctx.saveall && (!ctx.asym_acc || !ctx.asym_wsum)) ||
        !phase_seq || !mode_seq)
    {
        fprintf(stderr, "measdynresp: out of memory allocating 3D cube (%lu x %u x %u)\n",
                (unsigned long) ctx.n_out, ctx.n_in, ctx.n_t);
        free(mode_seq);
        free(phase_seq);
        free(ctx.asym_wsum);
        free(ctx.asym_acc);
        free(ctx.cnt);
        free(ctx.tsum);
        free(ctx.wsum);
        free(ctx.acc);
        free(ctx.cur_vec);
        free(ctx.k0_vec);
        free(ctx.base_vec);
        free(ctx.pre_ring);
        free(ctx.out_idx);
        free(ctx.inmodes);
        return RETURN_FAILURE;
    }

    mdr_build_phase_order(phase_seq, ctx.n_subfr, ctx.phaseorder);

    ctx.sem_out = ImageStreamIO_getsemwaitindex(ctx.imgout.im, -1);
    if (ctx.sem_out < 0)
    {
        ctx.sem_out = 0;
    }
    if (ctx.use_lockstep)
    {
        ctx.sem_trig = ImageStreamIO_getsemwaitindex(ctx.imgtrig.im, -1);
        if (ctx.sem_trig < 0)
        {
            ctx.sem_trig = 0;
        }
    }

    if (p_outdir[0] != '\0')
    {
        mdr_mkdir_p(p_outdir);
    }

    uint32_t completed_cycles = 0;
    int      rate_ready       = 0;
    int      aborted          = 0;
    errno_t  ret_status       = RETURN_SUCCESS;

    MILK_PROCINFO_INIT(self);
    ctx.processinfo = processinfo;

    MILK_PROCINFO_LOOPSTART(self);
    {
        /* Measure or lock WFS frame rate once on first iteration */
        if (!rate_ready)
        {
            double dt_meas = 0.0;
            if (p_wfsfrequ > 0.1f)
            {
                ctx.t_grid_s = 1.0 / (double) p_wfsfrequ;
                if (!ctx.use_lockstep && p_frameratewait > 0.05f)
                {
                    if (mdr_measure_wfs_rate(&ctx, (double) p_frameratewait, &dt_meas) == 0 &&
                        dt_meas > 1.0e-7)
                    {
                        ctx.t_live_ns = dt_meas * 1.0e9;
                    }
                    else
                    {
                        ctx.t_live_ns = ctx.t_grid_s * 1.0e9;
                    }
                }
                else
                {
                    ctx.t_live_ns = ctx.t_grid_s * 1.0e9;
                }
            }
            else
            {
                if (processinfo != NULL)
                {
                    processinfo_WriteMessage_fmt(processinfo, "measuring WFS rate (%.2fs)",
                                                 p_frameratewait);
                }
                int rrc = mdr_measure_wfs_rate(&ctx, (double) p_frameratewait, &dt_meas);
                if (rrc != 0 || dt_meas <= 1.0e-7)
                {
                    fprintf(stderr, "measdynresp: failed to measure WFS frame rate\n");
                    aborted    = 1;
                    ret_status = RETURN_FAILURE;
                    break;
                }
                ctx.t_grid_s  = dt_meas;
                ctx.t_live_ns = dt_meas * 1.0e9;
            }

            ctx.dt_bin_s  = ctx.t_grid_s / (double) ctx.n_subfr;
            o_framerateHz = (float) (1.0 / ctx.t_grid_s);
            o_dt_us       = (float) (ctx.dt_bin_s * 1.0e6);
            rate_ready    = 1;
        }

        uint64_t total_steps = (uint64_t) eff_nbcycle * ctx.n_subfr * ctx.n_in;
        uint64_t done_steps  = 0;

        for (uint32_t cyc = 0; cyc < eff_nbcycle && !aborted; cyc++)
        {
            mdr_shuffle_modes(mode_seq, ctx.n_in, completed_cycles);

            for (uint32_t pi = 0; pi < ctx.n_subfr && !aborted; pi++)
            {
                uint32_t phase_idx = phase_seq[pi];

                /* Re-anchor DM to settled zero at start of each sub-frame phase sweep */
                ImageStreamIO_semflush(ctx.imgout.im, ctx.sem_out);
                ctx.last_wfs_t_ns = 0;
                int src           = mdr_settle_state(&ctx, NULL, 0.0f, ctx.n_hold, 0);
                if (src != 0)
                {
                    aborted = 1;
                    if (src == -1)
                    {
                        ret_status = RETURN_FAILURE;
                    }
                    break;
                }

                for (uint32_t mi = 0; mi < ctx.n_in && !aborted; mi++)
                {
                    if (mdr_exit_requested(processinfo))
                    {
                        aborted = 1;
                        break;
                    }

                    uint32_t     mode_idx = mode_seq[mi];
                    const float *pat      = &ctx.inmodes[(size_t) mode_idx * ctx.n_dm];
                    int          trc      = 0;

                    if (processinfo != NULL && (mi == 0 || (mi % 8u) == 0))
                    {
                        processinfo_WriteMessage_fmt(processinfo,
                                                     "cyc %u/%u ph %u/%u mode %u/%u", cyc + 1u,
                                                     eff_nbcycle, pi + 1u, ctx.n_subfr, mi + 1u,
                                                     ctx.n_in);
                    }

                    if (ctx.pokemode == 0)
                    {
                        /* 4-segment drift-canceling push/release: 0 -> +a -> 0 -> -a -> 0 */
                        trc = mdr_run_transition(&ctx, mode_idx, phase_idx, pat, +ctx.ampl,
                                                 +ctx.ampl, +1.0f, (uint64_t) mode_idx * 4ULL + 1);
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, NULL, 0.0f,
                                                     -ctx.ampl, +1.0f,
                                                     (uint64_t) mode_idx * 4ULL + 2);
                        }
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, pat, -ctx.ampl,
                                                     -ctx.ampl, -1.0f,
                                                     (uint64_t) mode_idx * 4ULL + 3);
                        }
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, NULL, 0.0f,
                                                     +ctx.ampl, -1.0f,
                                                     (uint64_t) mode_idx * 4ULL + 4);
                        }
                    }
                    else if (ctx.pokemode == 1)
                    {
                        /* 2-segment push-pull: settle at -a, then -a -> +a and +a -> -a */
                        trc = mdr_settle_state(&ctx, pat, -ctx.ampl, ctx.n_hold,
                                               (uint64_t) mode_idx * 4ULL + 1);
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, pat, +ctx.ampl,
                                                     +2.0f * ctx.ampl, +1.0f,
                                                     (uint64_t) mode_idx * 4ULL + 2);
                        }
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, pat, -ctx.ampl,
                                                     -2.0f * ctx.ampl, -1.0f,
                                                     (uint64_t) mode_idx * 4ULL + 3);
                        }
                    }
                    else
                    {
                        /* Single-polarity: 0 -> +a -> 0 */
                        trc = mdr_run_transition(&ctx, mode_idx, phase_idx, pat, +ctx.ampl,
                                                 +ctx.ampl, +1.0f, (uint64_t) mode_idx * 2ULL + 1);
                        if (trc == 0)
                        {
                            trc = mdr_run_transition(&ctx, mode_idx, phase_idx, NULL, 0.0f,
                                                     -ctx.ampl, -1.0f,
                                                     (uint64_t) mode_idx * 2ULL + 2);
                        }
                    }

                    if (trc != 0)
                    {
                        aborted = 1;
                        if (trc == -1)
                        {
                            ret_status = RETURN_FAILURE;
                        }
                        break;
                    }

                    done_steps++;
                    o_progress_pct =
                        (total_steps > 0) ? (100.0f * (float) done_steps / (float) total_steps)
                                          : 100.0f;
                }

                /* Update live FPS telemetry and write intermediate checkpoint after each phase */
                o_dropped_frames = ctx.dropped_frames;
                o_jitter_us      = (ctx.jitter_cnt > 0)
                                       ? (float) (sqrt(ctx.jitter_sq_sum_ns2 /
                                                       (double) ctx.jitter_cnt) *
                                                  1.0e-3)
                                       : 0.0f;
                if (dcfpsptr != NULL)
                {
                    fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
                }
                if (!aborted && (pi + 1u) < ctx.n_subfr)
                {
                    mdr_save_outputs(&ctx, completed_cycles, 0);
                }
            }

            if (!aborted)
            {
                completed_cycles++;
            }
        }

        /* Always zero DM stream immediately upon finishing or aborting */
        mdr_publish_dm(processinfo, &ctx.imgin, NULL, 0.0f, ctx.n_dm, 0);

        /* Write final normalized 3D cube + auxiliary diagnostics */
        if (mdr_save_outputs(&ctx, completed_cycles, 1) != 0)
        {
            ret_status = RETURN_FAILURE;
        }

        if (aborted)
        {
            processloopOK = 0;
        }
    }
    MILK_PROCINFO_END(self);

    /* Ensure DM stream is zeroed even if exited prior to loop body */
    mdr_publish_dm(NULL, &ctx.imgin, NULL, 0.0f, ctx.n_dm, 0);

    free(mode_seq);
    free(phase_seq);
    free(ctx.asym_wsum);
    free(ctx.asym_acc);
    free(ctx.cnt);
    free(ctx.tsum);
    free(ctx.wsum);
    free(ctx.acc);
    free(ctx.cur_vec);
    free(ctx.k0_vec);
    free(ctx.base_vec);
    free(ctx.pre_ring);
    free(ctx.out_idx);
    free(ctx.inmodes);

    DEBUG_TRACE_FEXIT();
    return ret_status;
}

const MILK_RUNNABLE runnable_AOloopControl_acquireCalib_measdynresp = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name    = "measdynresp",
                 .cmdkey      = "measdynresp",
                 .description = "acquire raw 3D spatiotemporal response matrix",
                 .description_long =
                     "Acquire a raw exposure-integrated 3D linear response matrix "
                     "[N_out x N_in x N_t] between a DM/perturbation stream and a fixed-rate "
                     "WFS stream by stepping sub-frame poke offsets." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(cacaoac_measdynresp),
    .compute = measdynresp_compute,
};
EXPORT_FPSEXEC(cacao, "cacaoac-measdynresp", runnable_AOloopControl_acquireCalib_measdynresp);
