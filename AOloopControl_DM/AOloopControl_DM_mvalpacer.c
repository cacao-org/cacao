// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    AOloopControl_DM_mvalpacer.c
 * @brief   High-rate modal DM trajectory pacer for multi-rate predictive AO control
 *
 * Consumes a WFS-rate 2-row modal trajectory stream (.streamin, [NBmode x 2]:
 *   row 0 = u_mid[m],  row 1 = u_ramp[m])
 * produced by modalfilter + multi-rate mkEOFPF/applyEOFPF, and emits K_sub smooth
 * zero-mean centered linear ramp sub-commands at rate f_DM = K_sub * f_WFS into
 * .streamout (aolN_modevalDM [NBmode x 1]):
 *
 *   s_j      = (j + 0.5) / K_sub - 0.5        (j = 0 .. K_sub - 1, sum_j s_j == 0)
 *   u_out[m] = u_mid[m] + vel_gain * s_j * u_ramp[m]
 *
 * Placed strictly upstream of mvalC2dm -> DMch2disp -> dmpreshaper so the existing
 * 2-4 kHz DM anti-vibration / plate-wave pre-shaper layer (cacao-fpsexec-dmpreshaper)
 * automatically shapes every smooth sub-step without recalibration.
 *
 * Supports both lockstep clock-stream ticking (.trigstream + .trig_decim, for exact
 * cacaosim synchronization) and low-jitter CLOCK_MONOTONIC wall-clock pacing (.rate_hz
 * + .spinus, for on-sky hardware).
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

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "ImageStreamIO/ImageStruct.h"
#include "processinfo.h"
#include "milk_compiler.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

#if defined(__x86_64__) || defined(__i386__)
#    include <immintrin.h>
#    define MVP_CPU_PAUSE() _mm_pause()
#else
#    define MVP_CPU_PAUSE() ((void) 0)
#endif

static char     p_streamin[FUNCTION_PARAMETER_STRMAXLEN]   = "aol0_modevalDM_traj";
static char     p_streamout[FUNCTION_PARAMETER_STRMAXLEN]  = "aol0_modevalDM";
static char     p_trigstream[FUNCTION_PARAMETER_STRMAXLEN] = "";
static uint32_t p_trig_decim                               = 1;
static float    p_rate_hz                                  = 1000.0f;
static uint32_t p_k_sub                                    = 1;
static uint32_t p_spinus                                   = 60;
static int32_t  p_enable                                   = 1;
static float    p_vel_gain                                 = 1.0f;

static uint64_t o_in_cnt    = 0;
static uint64_t o_out_cnt   = 0;
static float    o_peak_ramp = 0.0f;

#define FPS_PARAMS_MVALPACER(X)                                                                    \
    X(".streamin", p_streamin, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,      \
      "input WFS-rate modal trajectory [NBmode x 2] (u_mid, u_ramp)")                              \
    X(".streamout", p_streamout, FPTYPE_STREAMNAME, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,    \
      "output fast DM modal command stream [NBmode x 1]")                                          \
    X(".trigstream", p_trigstream, FPTYPE_STREAMNAME, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,  \
      "optional fast clock stream for lockstep sub-step pacing")                                   \
    X(".trig_decim", &p_trig_decim, FPTYPE_UINT32, 0,                                              \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                   \
      "trigger clock decimation factor per DM sub-step (>=1)")                                     \
    X(".rate_hz", &p_rate_hz, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,          \
      "wall-clock DM sub-step rate [Hz] when trigstream is unset")                                 \
    X(".k_sub", &p_k_sub, FPTYPE_UINT32, 0,                                                        \
      FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT | FPFLAG_WRITERUN,                                   \
      "number of DM sub-steps emitted per WFS frame (1..32)")                                      \
    X(".spinus", &p_spinus, FPTYPE_UINT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT,             \
      "final busy-wait threshold [us] after clock_nanosleep")                                      \
    X(".enable", &p_enable, FPTYPE_ONOFF, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,               \
      "live enable (OFF = pass-through u_mid with zero ramp)")                                     \
    X(".vel_gain", &p_vel_gain, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,         \
      "live velocity ramp gain multiplier (0.0 = ZOH, 1.0 = full)")                                \
    X(".out.in_cnt", &o_in_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                           \
      "WFS-rate trajectory frames ingested")                                                       \
    X(".out.out_cnt", &o_out_cnt, FPTYPE_UINT64, 0, FPFLAG_DEFAULT_OUTPUT,                         \
      "fast DM modal frames emitted")                                                              \
    X(".out.peak_ramp", &o_peak_ramp, FPTYPE_FLOAT32, 0, FPFLAG_DEFAULT_OUTPUT,                    \
      "peak |u_ramp| modal velocity seen")

MILK_PARAMS_DEFINE(mvalpacer, FPS_PARAMS_MVALPACER);

static inline uint64_t mvp_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static inline int mvp_exit_requested(PROCESSINFO *processinfo)
{
    return (processinfo != NULL && processinfo_exit_requested(processinfo)) ? 1 : 0;
}

static void mvp_sleep_until_ns(uint64_t target_ns, uint32_t spin_us, PROCESSINFO *processinfo)
{
    uint64_t now = mvp_now_ns();
    if (now >= target_ns)
    {
        return;
    }
    uint64_t spin_ns = (uint64_t) spin_us * 1000ULL;
    if (target_ns - now > spin_ns)
    {
        uint64_t        wake_ns = target_ns - spin_ns;
        struct timespec ts;
        ts.tv_sec  = (time_t) (wake_ns / 1000000000ULL);
        ts.tv_nsec = (long) (wake_ns % 1000000000ULL);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
        {
            if (mvp_exit_requested(processinfo))
            {
                return;
            }
        }
    }
    while (mvp_now_ns() < target_ns)
    {
        if (mvp_exit_requested(processinfo))
        {
            return;
        }
        MVP_CPU_PAUSE();
    }
}

static int mvp_semwait_ms(IMAGE *im, int sem_idx, uint64_t timeout_ms, PROCESSINFO *processinfo)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    uint64_t add_ns = timeout_ms * 1000000ULL;
    ts.tv_sec      += (time_t) ((ts.tv_nsec + add_ns) / 1000000000ULL);
    ts.tv_nsec      = (long) ((ts.tv_nsec + add_ns) % 1000000000ULL);

    int rc = ImageStreamIO_semtimedwait(im, sem_idx, &ts);
    if (rc == 0)
    {
        return 0;
    }
    if (mvp_exit_requested(processinfo))
    {
        return -2;
    }
    return -1;
}

typedef struct
{
    uint32_t nbmode;
    int      has_vel_row;
    float   *u_mid;
    float   *u_ramp;
    float   *u_out;

    IMGID img_in;
    IMGID img_out;
    IMGID img_trig;
    int   sem_in;
    int   sem_trig;
    int   use_trig;

    uint64_t last_in_cnt0;
    uint32_t sub_idx;
    uint32_t k_sub_cur;
    uint32_t tick_acc;

    uint64_t in_cnt;
    uint64_t out_cnt;
    float    peak_ramp;
} MVP_CTX;

static void mvp_publish(MVP_CTX *ctx, uint32_t sub_j, uint32_t k_sub)
{
    const uint32_t M  = ctx->nbmode;
    float         *uo = ctx->u_out;
    float          sc = 0.0f;

    if (p_enable && k_sub > 1u && fabsf(p_vel_gain) > 1.0e-7f)
    {
        float sj = ((float) sub_j + 0.5f) / (float) k_sub - 0.5f;
        sc       = p_vel_gain * sj;
    }

    if (fabsf(sc) < 1.0e-7f)
    {
        memcpy(uo, ctx->u_mid, sizeof(float) * M);
    }
    else
    {
        const float *restrict um = ctx->u_mid;
        const float *restrict ur = ctx->u_ramp;
#pragma omp simd
        for (uint32_t m = 0; m < M; m++)
        {
            uo[m] = um[m] + sc * ur[m];
        }
    }

    IMAGE *im     = ctx->img_out.im;
    im->md->write = 1;
    memcpy(im->array.F, uo, sizeof(float) * M);
    clock_gettime(CLOCK_REALTIME, &im->md->writetime);
    im->md->atime = im->md->writetime;
    im->md->cnt0++;
    im->md->write = 0;
    ImageStreamIO_sempost(im, -1);
    ctx->out_cnt++;
}

static int mvp_ingest(MVP_CTX *ctx)
{
    const IMAGE *imin = ctx->img_in.im;
    if (imin->md->cnt0 == ctx->last_in_cnt0)
    {
        return 0;
    }
    ctx->last_in_cnt0 = imin->md->cnt0;
    ctx->in_cnt++;

    const uint32_t M = ctx->nbmode;
    memcpy(ctx->u_mid, imin->array.F, sizeof(float) * M);
    if (ctx->has_vel_row)
    {
        const float *vr = &imin->array.F[M];
        float        pk = ctx->peak_ramp;
        for (uint32_t m = 0; m < M; m++)
        {
            float v        = vr[m];
            ctx->u_ramp[m] = v;
            float a        = fabsf(v);
            if (a > pk)
            {
                pk = a;
            }
        }
        ctx->peak_ramp = pk;
    }
    else
    {
        memset(ctx->u_ramp, 0, sizeof(float) * M);
    }

    uint32_t ks = (p_k_sub >= 1u && p_k_sub <= 32u) ? p_k_sub : 1u;
    if (!p_enable)
    {
        ks = 1u;
    }
    ctx->k_sub_cur = ks;
    ctx->sub_idx   = 0u;
    ctx->tick_acc  = 0u;
    return 1;
}

static MILK_HOT errno_t mvalpacer_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    if (dcfpsptr != NULL)
    {
        fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
    }

#if defined(__linux__) && defined(PR_SET_TIMERSLACK)
    (void) prctl(PR_SET_TIMERSLACK, 1000UL, 0, 0, 0);
#endif

    if (p_streamin[0] == '\0' || p_streamout[0] == '\0')
    {
        fprintf(stderr, "mvalpacer: .streamin and .streamout are required\n");
        return RETURN_FAILURE;
    }

    MVP_CTX ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.img_in.ID   = -1;
    ctx.img_out.ID  = -1;
    ctx.img_trig.ID = -1;
    ctx.sem_in      = -1;
    ctx.sem_trig    = -1;

    ctx.img_in = imgid_make_from_name(p_streamin);
    resolveIMGID(&ctx.img_in, ERRMODE_WARN, dcimg, dcnimg);
    if (ctx.img_in.ID == -1)
    {
        fprintf(stderr, "mvalpacer: cannot resolve streamin '%s'\n", p_streamin);
        return RETURN_FAILURE;
    }
    if (ctx.img_in.md->datatype != _DATATYPE_FLOAT)
    {
        fprintf(stderr, "mvalpacer: streamin '%s' must be float32\n", p_streamin);
        return RETURN_FAILURE;
    }

    ctx.nbmode      = ctx.img_in.md->size[0];
    ctx.has_vel_row = (ctx.img_in.md->naxis >= 2 && ctx.img_in.md->size[1] >= 2u) ? 1 : 0;
    if (ctx.nbmode == 0u)
    {
        return RETURN_FAILURE;
    }

    ctx.img_out = stream_connect_create_2Df32(p_streamout, ctx.nbmode, 1);
    if (ctx.img_out.ID == -1)
    {
        fprintf(stderr, "mvalpacer: failed to connect/create streamout '%s'\n", p_streamout);
        return RETURN_FAILURE;
    }

    if (p_trigstream[0] != '\0' && strcmp(p_trigstream, "none") != 0 &&
        strcmp(p_trigstream, "null") != 0)
    {
        ctx.img_trig = imgid_make_from_name(p_trigstream);
        resolveIMGID(&ctx.img_trig, ERRMODE_WARN, dcimg, dcnimg);
        if (ctx.img_trig.ID == -1)
        {
            fprintf(stderr, "mvalpacer: cannot resolve trigstream '%s'\n", p_trigstream);
            return RETURN_FAILURE;
        }
        ctx.use_trig = 1;
    }

    ctx.u_mid  = (float *) calloc(ctx.nbmode, sizeof(float));
    ctx.u_ramp = (float *) calloc(ctx.nbmode, sizeof(float));
    ctx.u_out  = (float *) calloc(ctx.nbmode, sizeof(float));
    if (!ctx.u_mid || !ctx.u_ramp || !ctx.u_out)
    {
        free(ctx.u_out);
        free(ctx.u_ramp);
        free(ctx.u_mid);
        return RETURN_FAILURE;
    }

    memcpy(ctx.u_mid, ctx.img_in.im->array.F, sizeof(float) * ctx.nbmode);
    ctx.last_in_cnt0 = ctx.img_in.md->cnt0;
    ctx.k_sub_cur    = 1u;
    ctx.sub_idx      = 1u; /* idle until first new trajectory arrives */

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

    double   rate_use = (p_rate_hz > 1.0f) ? (double) p_rate_hz : 1000.0;
    uint64_t dt_ns    = (uint64_t) llround(1.0e9 / rate_use);

    MILK_PROCINFO_INIT(self);
    if (processinfo != NULL)
    {
        processinfo_WriteMessage_fmt(processinfo, "M=%u k_sub=%u %s (%s -> %s)", ctx.nbmode,
                                     p_k_sub, ctx.use_trig ? "lockstep" : "wallclock", p_streamin,
                                     p_streamout);
    }

    uint64_t next_tick_ns  = mvp_now_ns() + dt_ns;
    uint64_t resync_ticker = 0;

    MILK_PROCINFO_LOOPSTART(self);
    {
        if (((resync_ticker++ & 15ULL) == 0ULL || ctx.sub_idx + 1u >= ctx.k_sub_cur) &&
            dcfpsptr != NULL)
        {
            fpsresync_fps_to_modvar(dcfpsptr, dcfpsbindings, dcfpsnbindings, 1);
            o_in_cnt    = ctx.in_cnt;
            o_out_cnt   = ctx.out_cnt;
            o_peak_ramp = ctx.peak_ramp;
            fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
        }

        if (ctx.use_trig)
        {
            if (ctx.sub_idx + 1u >= ctx.k_sub_cur)
            {
                /* Zero-latency wake directly on streamin so sub-step j=0 adds ~0 us lag */
                int got = mvp_ingest(&ctx);
                if (!got)
                {
                    int wrc = mvp_semwait_ms(ctx.img_in.im, ctx.sem_in, 50ULL, processinfo);
                    if (wrc == -2 || mvp_exit_requested(processinfo))
                    {
                        break;
                    }
                    got = mvp_ingest(&ctx);
                }
                if (got)
                {
                    ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
                    ImageStreamIO_semflush(ctx.img_trig.im, ctx.sem_trig);
                    mvp_publish(&ctx, 0u, ctx.k_sub_cur);
                }
            }
            else
            {
                /* Sub-steps j = 1 .. K_sub - 1 paced strictly on lockstep trigger stream */
                int wrc = mvp_semwait_ms(ctx.img_trig.im, ctx.sem_trig, 100ULL, processinfo);
                if (wrc == -2 || mvp_exit_requested(processinfo))
                {
                    break;
                }
                if (wrc == 0)
                {
                    if (mvp_ingest(&ctx))
                    {
                        ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
                        ImageStreamIO_semflush(ctx.img_trig.im, ctx.sem_trig);
                        mvp_publish(&ctx, 0u, ctx.k_sub_cur);
                    }
                    else
                    {
                        uint32_t dec = (p_trig_decim >= 1u) ? p_trig_decim : 1u;
                        ctx.tick_acc++;
                        if (ctx.tick_acc >= dec)
                        {
                            ctx.tick_acc = 0u;
                            ctx.sub_idx++;
                            mvp_publish(&ctx, ctx.sub_idx, ctx.k_sub_cur);
                        }
                    }
                }
            }
        }
        else if (ctx.sub_idx + 1u >= ctx.k_sub_cur)
        {
            /* Idle: zero-latency semaphore wake on streamin */
            int got = mvp_ingest(&ctx);
            if (!got)
            {
                int wrc = mvp_semwait_ms(ctx.img_in.im, ctx.sem_in, 50ULL, processinfo);
                if (wrc == -2 || mvp_exit_requested(processinfo))
                {
                    break;
                }
                got = mvp_ingest(&ctx);
            }
            if (got)
            {
                ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
                uint64_t t0 = mvp_now_ns();
                mvp_publish(&ctx, 0u, ctx.k_sub_cur);
                next_tick_ns = t0 + dt_ns;
            }
        }
        else
        {
            /* Active sub-step ramp emission on monotonic wall clock */
            mvp_sleep_until_ns(next_tick_ns, p_spinus, processinfo);
            if (mvp_exit_requested(processinfo))
            {
                break;
            }
            if (mvp_ingest(&ctx))
            {
                ImageStreamIO_semflush(ctx.img_in.im, ctx.sem_in);
                uint64_t t0 = mvp_now_ns();
                mvp_publish(&ctx, 0u, ctx.k_sub_cur);
                next_tick_ns = t0 + dt_ns;
            }
            else
            {
                ctx.sub_idx++;
                mvp_publish(&ctx, ctx.sub_idx, ctx.k_sub_cur);
                next_tick_ns += dt_ns;
                uint64_t now  = mvp_now_ns();
                if (next_tick_ns + dt_ns < now)
                {
                    next_tick_ns = now + dt_ns;
                }
            }
        }
    }
    MILK_PROCINFO_END(self);

    if (dcfpsptr != NULL)
    {
        o_in_cnt    = ctx.in_cnt;
        o_out_cnt   = ctx.out_cnt;
        o_peak_ramp = ctx.peak_ramp;
        fpsresync_modvar_to_fps(dcfpsptr, dcfpsbindings, dcfpsnbindings);
    }

    free(ctx.u_out);
    free(ctx.u_ramp);
    free(ctx.u_mid);
    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}

const MILK_RUNNABLE runnable_AOloopControl_DM_mvalpacer = {
    .kind = MILK_RUNNABLE_FPS,
    .app  = {
        .fps_name         = "mvalpacer",
        .cmdkey           = "mvalpacer",
        .description      = "High-rate modal DM trajectory pacer for multi-rate predictive control",
        .description_long = "Emits K_sub smooth centered linear ramp DM modal commands per slow "
                            "WFS frame upstream of mvalC2dm -> DMch2disp -> dmpreshaper.",
    },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(mvalpacer),
    .compute = mvalpacer_compute,
};

EXPORT_FPSEXEC(cacao, "mvalpacer", runnable_AOloopControl_DM_mvalpacer);
