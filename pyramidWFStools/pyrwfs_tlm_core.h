// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    pyrwfs_tlm_core.h
 * @brief   Self-contained C core for Pyramid WFS telemetry, pupil registration, and 4-pupil gather
 *
 * Pure C (libm + optional OpenMP, zero milk dependency) shared by:
 *   - `pyWFStlm.c`           (Tier 3 MILK_RUNNABLE live 2D stream & offline 3D cube unit)
 *   - `calc/pyrwfs_calc_ext.c` (Tier 2 @calc extension operators in lib/milk/calc.d/)
 *   - `pyWFSgridmatch.c`     (DM-to-4-pupil affine geometry calibration)
 *
 * Packed zone-label encoding (`uint8_t`, also exact in float32 from `@calc`):
 *   Low nibble (`val & 0x0F`, mutually exclusive regions):
 *     0 = NONE       (ignored)
 *     1 = PUP0       (bottom-left pupil,  dx < 0, dy < 0)
 *     2 = PUP1       (top-left pupil,     dx < 0, dy > 0)
 *     3 = PUP2       (bottom-right pupil, dx > 0, dy < 0)
 *     4 = PUP3       (top-right pupil,    dx > 0, dy > 0)
 *     5 = DIAMOND    (central apex diffraction gap between all 4 pupils)
 *     6 = BRIDGE_H   (horizontal inter-pupil bridges along y ~ yc)
 *     7 = BRIDGE_V   (vertical inter-pupil bridges along x ~ xc)
 *     8 = OUTER      (outer diffraction halo & central obscuration diffraction shadows)
 *     9 = BG         (far-corner background / bias monitor)
 *   High nibble (`val & 0xF0`, overlay flags valid on pupil pixels):
 *     0x10 = INNER_PUP (eroded interior pupil pixels for curl/div/scintillation)
 *     0x20 = SPIDER    (bands flanking telescope spider arms for LWE / petaling)
 *     0x40 = INNER_RIM (pupil rim facing the pyramid center)
 */

#ifndef CACAO_PYRWFS_TLM_CORE_H
#define CACAO_PYRWFS_TLM_CORE_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef _OPENMP
#    include <omp.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define PYRWFS_ZONE_MASK      0x0Fu
#define PYRWFS_ZONE_NONE      0u
#define PYRWFS_ZONE_PUP0      1u
#define PYRWFS_ZONE_PUP1      2u
#define PYRWFS_ZONE_PUP2      3u
#define PYRWFS_ZONE_PUP3      4u
#define PYRWFS_ZONE_DIAMOND   5u
#define PYRWFS_ZONE_BRIDGE_H  6u
#define PYRWFS_ZONE_BRIDGE_V  7u
#define PYRWFS_ZONE_OUTER     8u
#define PYRWFS_ZONE_BG        9u

#define PYRWFS_FLAG_INNER_PUP 0x10u
#define PYRWFS_FLAG_SPIDER    0x20u
#define PYRWFS_FLAG_INNER_RIM 0x40u

#define PYRWFS_NPUP           4
#define PYRWFS_DOF_NCOL       8
#define PYRWFS_TLM_NCOL       29

/* Indices in packed telemetry vector `tlm_vec[PYRWFS_TLM_NCOL]` */
enum pyrwfs_tlm_idx
{
    PYRWFS_TLM_TS            = 0,
    PYRWFS_TLM_FTOT          = 1,
    PYRWFS_TLM_BG_MEAN       = 2,
    PYRWFS_TLM_BG_STD        = 3,
    PYRWFS_TLM_SAT_HI_FRAC   = 4,
    PYRWFS_TLM_SAT_DARK_FRAC = 5,
    PYRWFS_TLM_F_PUP         = 6,
    PYRWFS_TLM_F_DIAM        = 7,
    PYRWFS_TLM_F_BH          = 8,
    PYRWFS_TLM_F_BV          = 9,
    PYRWFS_TLM_F_OUT         = 10,
    PYRWFS_TLM_DF_PUP        = 11,
    PYRWFS_TLM_ANISO         = 12,
    PYRWFS_TLM_NL_RATIO      = 13,
    PYRWFS_TLM_QX_MEAN       = 14,
    PYRWFS_TLM_QY_MEAN       = 15,
    PYRWFS_TLM_QX_RMS        = 16,
    PYRWFS_TLM_QY_RMS        = 17,
    PYRWFS_TLM_QDIAG         = 18,
    PYRWFS_TLM_SCINT_IDX     = 19,
    PYRWFS_TLM_CURL_RATIO    = 20,
    PYRWFS_TLM_LWE_METRIC    = 21,
    PYRWFS_TLM_PUP_DX        = 22,
    PYRWFS_TLM_PUP_DY        = 23,
    PYRWFS_TLM_PUP_DS        = 24,
    PYRWFS_TLM_PUP_DAST      = 25,
    PYRWFS_TLM_PUP_ROT       = 26,
    PYRWFS_TLM_PUP_CORR      = 27,
    PYRWFS_TLM_FLUX_OK       = 28
};

/* Per-pupil geometry row (4 pupils: 0=BL, 1=TL, 2=BR, 3=TR) */
typedef struct
{
    float xc[PYRWFS_NPUP];
    float yc[PYRWFS_NPUP];
    float rout[PYRWFS_NPUP];
    float rin[PYRWFS_NPUP];
    float corr[PYRWFS_NPUP];
} pyrwfs_pupgeom_t;

/* 6-DOF + residual decomposition of the 4 measured pupil shifts */
typedef struct
{
    float dx;    /* Common horizontal shift [pix] */
    float dy;    /* Common vertical shift [pix] */
    float dsx;   /* Horizontal half-separation shift [pix] (+ = outward) */
    float dsy;   /* Vertical half-separation shift [pix] (+ = outward) */
    float rot;   /* Global rotation [rad] */
    float scale; /* Relative magnification 0.5*(dsx/sx0 + dsy/sy0) */
    float skew;  /* Shear / skew [rad] */
    float resid; /* Non-rigid RMS residual across 4 pupils [pix] */
} pyrwfs_pupdof_t;

/* Single-frame flux, quad-cell, and nonlinearity metrics */
typedef struct
{
    double ftot;
    double bg_mean;
    double bg_std;
    double sat_hi_frac;
    double sat_dark_frac;
    double f_pup;
    double f_diam;
    double f_bH;
    double f_bV;
    double f_out;
    double df_pup;
    double aniso;
    double c_even;
    double nl_ratio;
    double qx;
    double qy;
    double qdiag;
    double pup_flux[PYRWFS_NPUP];
    int    flux_ok;
} pyrwfs_frame_metrics_t;

/* Configuration for the telemetry core */
typedef struct
{
    uint32_t nx;
    uint32_t ny;
    uint32_t npix;       /* Registered pupil grid size (npix x npix) */
    int      sub_bg;     /* 1: subtract bg_mean before flux budget, 0: subtract bg_const */
    int      norm_mode;  /* 0: global mean Isum norm, 1: local Isum norm */
    int      edge_mode;  /* 0: full soft-Sobel, 1: downweight inner diffraction quadrant */
    int      maxshift;   /* Registration search half-width [pix] */
    double   bg_const;   /* Fixed camera dark/bias pedestal [ADU] used when sub_bg == 0 */
    double   sat_hi;     /* High saturation threshold [ADU] */
    double   dark_floor; /* Dark-floor / facet-dropout threshold [ADU] */
    double   flux_min;   /* Minimum valid total flux for flux_ok interlock */
    double   f_pup0;     /* Reference flat-WF in-pupil flux fraction */
    double   a_slow;     /* Slow EMA weight (centering, scintillation, local var) */
    double   a_fast;     /* Fast EMA weight (TT jitter RMS) */
    double   a_reg;      /* EMA weight for registration frame smoothing */
    double   corr_min;   /* Minimum xcorr peak to accept live pupil tracking update */
} pyrwfs_tlm_cfg_t;

/* Persistent state for edge-based pupil registration */
typedef struct
{
    uint32_t         nx;
    uint32_t         ny;
    int              edge_mode;
    pyrwfs_pupgeom_t nom;
    float           *ref_edge; /* [nx * ny] precomputed soft-clipped Sobel magnitude */
    float           *cur_edge; /* [nx * ny] scratch Sobel magnitude */
    float           *wgt_map;  /* [nx * ny] per-pixel registration weight (e.g. inner-arc taper) */
    int              ready;
} pyrwfs_pupreg_state_t;

/* Full persistent telemetry state */
typedef struct
{
    pyrwfs_tlm_cfg_t      cfg;
    pyrwfs_pupgeom_t      nom;
    pyrwfs_pupgeom_t      live;
    pyrwfs_pupdof_t       dof;
    pyrwfs_pupreg_state_t reg;

    uint8_t *zones_nom;      /* [nx * ny] nominal packed zone labels + flags */
    uint8_t *zones;          /* [nx * ny] live co-registered zone labels + flags */
    float   *ref_norm_nom;   /* [nx * ny] nominal flux-normalized flat reference */
    float   *ref_norm;       /* [nx * ny] live co-registered flat reference */
    float   *even_basis_nom; /* [nx * ny] nominal unit-norm even-mode basis */
    float   *even_basis;     /* [nx * ny] live co-registered even-mode basis */
    float   *im_avg;         /* [nx * ny] EMA image for registration */
    int      zones_sx;       /* current integer X shift applied to zones [pix] */
    int      zones_sy;       /* current integer Y shift applied to zones [pix] */

    /* Registered pupil-grid buffers [npix * npix] */
    uint8_t *p_in_mask;   /* interior pupil mask on npix x npix */
    uint8_t *p_sp_mask;   /* spider-flanking mask on npix x npix */
    float   *isum;        /* current 4-pupil sum */
    float   *sx;          /* current x-slope */
    float   *sy;          /* current y-slope */
    float   *isum_avg;         /* EMA 4-pupil sum */
    float   *isum0_norm;       /* reference normalized 4-pupil sum */
    float   *isum_rel_prev;    /* previous valid frame's relative Isum fluctuation (lag-1 cov) */
    float   *scint_map;        /* EMA normalized intensity variance map (raw, incl. noise) */
    float   *scint_debias_map; /* EMA lag-1 noise-debiased scintillation covariance map */
    float   *lvar_map;         /* local phase-variance / vignetting map */

    /* Running scalar EMAs */
    uint64_t nframe;
    double   qx_mean;
    double   qy_mean;
    double   qx_var;
    double   qy_var;
    double   scint_idx;
    double   scint_debias;
    double   curl_ratio;
    double   lwe_metric;
    double   pup_corr_min;
    int      has_ref;
    int      has_even;
    int      maps_seeded;
    int      has_prev_rel;
} pyrwfs_tlm_state_t;

/* =========================================================================
 * Helper: bilinear sampling with clamp-to-edge
 * ========================================================================= */
static inline float pyrwfs_sample_bilinear(const float *im,
                                           uint32_t     nx,
                                           uint32_t     ny,
                                           float        x,
                                           float        y)
{
    if (x < 0.0f || y < 0.0f || x > (float) (nx - 1u) || y > (float) (ny - 1u))
    {
        return 0.0f;
    }
    int   x0 = (int) floorf(x);
    int   y0 = (int) floorf(y);
    int   x1 = (x0 + 1 < (int) nx) ? (x0 + 1) : x0;
    int   y1 = (y0 + 1 < (int) ny) ? (y0 + 1) : y0;
    float fx = x - (float) x0;
    float fy = y - (float) y0;

    const float *r0 = im + (size_t) y0 * nx;
    const float *r1 = im + (size_t) y1 * nx;
    float        v0 = r0[x0] + fx * (r0[x1] - r0[x0]);
    float        v1 = r1[x0] + fx * (r1[x1] - r1[x0]);
    return v0 + fy * (v1 - v0);
}

/* Exact 3x3 linear solver for Kasa circle fit */
static inline int pyrwfs_solve_3x3(double A[3][3], double b[3], double x[3])
{
    double M[3][4];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            M[i][j] = A[i][j];
        }
        M[i][3] = b[i];
    }
    for (int col = 0; col < 3; col++)
    {
        int    piv = col;
        double am  = fabs(M[col][col]);
        for (int r = col + 1; r < 3; r++)
        {
            if (fabs(M[r][col]) > am)
            {
                am  = fabs(M[r][col]);
                piv = r;
            }
        }
        if (am < 1e-12)
        {
            return -1;
        }
        if (piv != col)
        {
            for (int j = col; j < 4; j++)
            {
                double t = M[col][j];
                M[col][j] = M[piv][j];
                M[piv][j] = t;
            }
        }
        double d = M[col][col];
        for (int r = col + 1; r < 3; r++)
        {
            double f = M[r][col] / d;
            for (int j = col; j < 4; j++)
            {
                M[r][j] -= f * M[col][j];
            }
        }
    }
    for (int i = 2; i >= 0; i--)
    {
        double s = M[i][3];
        for (int j = i + 1; j < 3; j++)
        {
            s -= M[i][j] * x[j];
        }
        x[i] = s / M[i][i];
    }
    return 0;
}

/* =========================================================================
 * Auto-detect 4 pupil centers and radii from a 2D intensity image.
 * Uses robust histogram median thresholding + 270-deg outer-rim radial-cliff
 * Kasa circle fit so intense non-modulated inner-rim / apex-diamond spikes
 * do NOT pull the detected pupil centers inward.
 * ========================================================================= */
static inline int pyrwfs_detect_pupils(const float      *im,
                                       uint32_t          nx,
                                       uint32_t          ny,
                                       float             thr_frac,
                                       pyrwfs_pupgeom_t *out)
{
    if (im == NULL || out == NULL || nx < 8u || ny < 8u)
    {
        return -1;
    }
    if (thr_frac <= 0.0f || thr_frac >= 1.0f)
    {
        thr_frac = 0.25f;
    }

    uint32_t hx = nx / 2u;
    uint32_t hy = ny / 2u;

    /* Corner background estimate (4 corner boxes of size ~nx/10) */
    uint32_t cb = nx / 10u;
    if (cb < 2u)
    {
        cb = 2u;
    }
    if (cb > ny / 4u)
    {
        cb = ny / 4u;
    }
    double bg_sum = 0.0;
    size_t bg_cnt = 0;
    for (uint32_t y = 0; y < cb; y++)
    {
        for (uint32_t x = 0; x < cb; x++)
        {
            bg_sum += (double) im[y * nx + x];
            bg_sum += (double) im[y * nx + (nx - 1u - x)];
            bg_sum += (double) im[(ny - 1u - y) * nx + x];
            bg_sum += (double) im[(ny - 1u - y) * nx + (nx - 1u - x)];
            bg_cnt += 4u;
        }
    }
    double bg = (bg_cnt > 0u) ? (bg_sum / (double) bg_cnt) : 0.0;

    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        uint32_t x0 = (q == 0 || q == 1) ? 0u : hx;
        uint32_t x1 = (q == 0 || q == 1) ? hx : nx;
        uint32_t y0 = (q == 0 || q == 2) ? 0u : hy;
        uint32_t y1 = (q == 0 || q == 2) ? hy : ny;

        double vmax = 0.0;
        for (uint32_t y = y0; y < y1; y++)
        {
            const float *row = im + (size_t) y * nx;
            for (uint32_t x = x0; x < x1; x++)
            {
                double v = (double) row[x] - bg;
                if (v > vmax)
                {
                    vmax = v;
                }
            }
        }
        if (vmax <= 0.0)
        {
            return -1;
        }

        /* Robust median of active pixels (> 2% of vmax) via 256-bin histogram so
         * intense flat-WF apex diffraction spikes cannot inflate the threshold. */
        size_t hist[256];
        memset(hist, 0, sizeof(hist));
        size_t nact = 0;
        double vmin = 0.02 * vmax;
        for (uint32_t y = y0; y < y1; y++)
        {
            const float *row = im + (size_t) y * nx;
            for (uint32_t x = x0; x < x1; x++)
            {
                double v = (double) row[x] - bg;
                if (v > vmin)
                {
                    int bin = (int) (255.0 * (v - vmin) / (vmax - vmin + 1e-12));
                    if (bin < 0)
                    {
                        bin = 0;
                    }
                    else if (bin > 255)
                    {
                        bin = 255;
                    }
                    hist[bin]++;
                    nact++;
                }
            }
        }
        double vmed = 0.25 * vmax;
        if (nact > 0u)
        {
            size_t acc = 0;
            for (int b = 0; b < 256; b++)
            {
                acc += hist[b];
                if (acc * 2u >= nact)
                {
                    vmed = vmin + ((double) b + 0.5) / 256.0 * (vmax - vmin);
                    break;
                }
            }
        }

        double thr = (double) thr_frac * vmed;
        if (thr < 0.75 * vmed)
        {
            /* Keep initial rough footprint above inter-pupil bridge halo */
            thr = 0.75 * vmed;
        }
        double sx  = 0.0;
        double sy  = 0.0;
        size_t np  = 0;
        for (uint32_t y = y0; y < y1; y++)
        {
            const float *row = im + (size_t) y * nx;
            for (uint32_t x = x0; x < x1; x++)
            {
                if ((double) row[x] - bg > thr)
                {
                    sx += (double) x;
                    sy += (double) y;
                    np++;
                }
            }
        }
        if (np < 9u)
        {
            /* Fallback to caller threshold if 0.75*vmed was too high */
            thr = (double) thr_frac * vmed;
            sx  = 0.0;
            sy  = 0.0;
            np  = 0;
            for (uint32_t y = y0; y < y1; y++)
            {
                const float *row = im + (size_t) y * nx;
                for (uint32_t x = x0; x < x1; x++)
                {
                    if ((double) row[x] - bg > thr)
                    {
                        sx += (double) x;
                        sy += (double) y;
                        np++;
                    }
                }
            }
            if (np < 9u)
            {
                return -1;
            }
        }
        double xc   = sx / (double) np;
        double yc   = sy / (double) np;
        double rout = sqrt((double) np / 3.1415926535897932);
        if (rout < 3.0)
        {
            rout = 3.0;
        }

        /* Refine (xc, yc, rout) via 3-pass outer-rim radial-cliff Kasa circle fit,
         * walking INWARD from the outside to grab the outermost rim (never central
         * obscuration or inner apex diffraction cusp). On iter==0 start near the
         * quadrant geometric center with wide rhi so faint non-modulated pupils
         * whose inner cusp is much brighter than the interior still hit the true rim. */
        double in_x = (q == 0 || q == 1) ? 1.0 : -1.0;
        double in_y = (q == 0 || q == 2) ? 1.0 : -1.0;
        double qcx  = 0.5 * (double) (x0 + x1);
        double qcy  = 0.5 * (double) (y0 + y1);
        double qmin = (double) ((x1 - x0 < y1 - y0) ? (x1 - x0) : (y1 - y0));

        /* If initial intensity centroid got pulled inward toward the central apex
         * or shrank (characteristic of non-modulated flat-WF diffraction), seed at (qcx, qcy). */
        if ((xc - qcx) * in_x > 0.04 * qmin || (yc - qcy) * in_y > 0.04 * qmin ||
            rout < 0.32 * qmin)
        {
            xc   = qcx;
            yc   = qcy;
            rout = 0.40 * qmin;
        }

        for (int iter = 0; iter < 3; iter++)
        {
            double px[96], py[96], pw[96];
            int    npts = 0;
            double rlo  = (iter == 0) ? (0.22 * qmin) : (0.72 * rout);
            double rhi  = (iter == 0) ? (0.50 * qmin) : (1.35 * rout);

            for (int k = 0; k < 72; k++)
            {
                double th = (2.0 * 3.1415926535897932 * (double) k) / 72.0;
                double ct = cos(th);
                double st = sin(th);
                /* Skip the inner quadrant pointing toward the central diamond / bridges */
                if (ct * in_x > 0.0 && st * in_y > 0.0)
                {
                    continue;
                }

                /* Clamp r_max strictly inside own quadrant so rays never cross into a neighbor */
                double r_max = rhi;
                if (ct > 1e-6)
                {
                    double lim = ((double) x1 - 2.0 - xc) / ct;
                    if (lim < r_max)
                    {
                        r_max = lim;
                    }
                }
                else if (ct < -1e-6)
                {
                    double lim = ((double) x0 + 1.0 - xc) / ct;
                    if (lim < r_max)
                    {
                        r_max = lim;
                    }
                }
                if (st > 1e-6)
                {
                    double lim = ((double) y1 - 2.0 - yc) / st;
                    if (lim < r_max)
                    {
                        r_max = lim;
                    }
                }
                else if (st < -1e-6)
                {
                    double lim = ((double) y0 + 1.0 - yc) / st;
                    if (lim < r_max)
                    {
                        r_max = lim;
                    }
                }
                if (r_max <= rlo + 2.0)
                {
                    continue;
                }

                double step  = 0.5;
                int    nstep = (int) floor((r_max - rlo) / step);
                if (nstep < 4)
                {
                    continue;
                }
                double drops[160];
                if (nstep > 159)
                {
                    nstep = 159;
                }
                double max_drop = 0.0;
                for (int s = 0; s <= nstep; s++)
                {
                    double r  = rlo + (double) s * step;
                    float  vi = pyrwfs_sample_bilinear(im, nx, ny, (float) (xc + (r - 0.5) * ct),
                                                       (float) (yc + (r - 0.5) * st));
                    float  vo = pyrwfs_sample_bilinear(im, nx, ny, (float) (xc + (r + 1.0) * ct),
                                                       (float) (yc + (r + 1.0) * st));
                    /* Positive drop = bright inside, dark outside */
                    double drop = (double) (vi - vo);
                    drops[s]    = drop;
                    if (drop > max_drop)
                    {
                        max_drop = drop;
                    }
                }
                if (max_drop <= 0.12 * vmed)
                {
                    continue;
                }

                /* Walk INWARD from rhi and pick the OUTERMOST local peak.
                 * Use 0.22 on coarse iter 0 (captures faint outer rim vs bright inner cusp),
                 * then 0.40 on refinement passes (rejects weak outer Airy sidelobes). */
                double frac_thr  = (iter == 0) ? 0.22 : 0.40;
                int    best_s    = -1;
                double best_drop = 0.0;
                for (int s = nstep - 1; s >= 1; s--)
                {
                    if (drops[s] >= frac_thr * max_drop && drops[s] >= drops[s - 1] &&
                        drops[s] >= drops[s + 1])
                    {
                        best_s    = s;
                        best_drop = drops[s];
                        break;
                    }
                }
                if (best_s < 0)
                {
                    continue;
                }

                double best_r = rlo + (double) best_s * step;
                /* Parabolic refinement along r */
                {
                    double dm  = drops[best_s - 1];
                    double d0  = drops[best_s];
                    double dp  = drops[best_s + 1];
                    double den = dm - 2.0 * d0 + dp;
                    if (den < -1e-12)
                    {
                        double off = 0.5 * (dm - dp) / den;
                        if (off > 0.5)
                        {
                            off = 0.5;
                        }
                        else if (off < -0.5)
                        {
                            off = -0.5;
                        }
                        best_r += off * step;
                    }
                }
                px[npts] = xc + best_r * ct;
                py[npts] = yc + best_r * st;
                pw[npts] = (best_drop < vmed) ? best_drop : vmed;
                npts++;
            }

            if (npts >= 8)
            {
                /* 2-step IRLS Kasa circle fit: x^2 + y^2 = 2*xc*x + 2*yc*y + c */
                double fx = xc;
                double fy = yc;
                double fr = rout;
                for (int rf = 0; rf < 2; rf++)
                {
                    double A[3][3] = { { 0 } };
                    double b[3]    = { 0 };
                    int    nkeep   = 0;
                    for (int i = 0; i < npts; i++)
                    {
                        if (rf == 1 && fabs(hypot(px[i] - fx, py[i] - fy) - fr) > 1.5)
                        {
                            continue;
                        }
                        double w   = pw[i];
                        double rhs = px[i] * px[i] + py[i] * py[i];
                        double row[3] = { 2.0 * px[i], 2.0 * py[i], 1.0 };
                        for (int a = 0; a < 3; a++)
                        {
                            b[a] += w * row[a] * rhs;
                            for (int c = 0; c < 3; c++)
                            {
                                A[a][c] += w * row[a] * row[c];
                            }
                        }
                        nkeep++;
                    }
                    double sol[3];
                    if (nkeep >= 6 && pyrwfs_solve_3x3(A, b, sol) == 0)
                    {
                        double r2 = sol[2] + sol[0] * sol[0] + sol[1] * sol[1];
                        if (r2 > 4.0)
                        {
                            fx = sol[0];
                            fy = sol[1];
                            fr = sqrt(r2);
                        }
                    }
                }
                xc   = fx;
                yc   = fy;
                rout = fr;
            }
        }

        /* Estimate central obscuration Rin inside < 0.55*rout around refined (xc, yc) */
        double int_sum = 0.0;
        size_t int_cnt = 0;
        for (uint32_t y = y0; y < y1; y++)
        {
            const float *row = im + (size_t) y * nx;
            for (uint32_t x = x0; x < x1; x++)
            {
                double r = hypot((double) x - xc, (double) y - yc);
                if (r > 0.55 * rout && r < 0.85 * rout)
                {
                    int_sum += (double) row[x] - bg;
                    int_cnt++;
                }
            }
        }
        double ring_lvl = (int_cnt > 0u) ? (int_sum / (double) int_cnt) : vmed;
        size_t hole_cnt = 0;
        for (uint32_t y = y0; y < y1; y++)
        {
            const float *row = im + (size_t) y * nx;
            for (uint32_t x = x0; x < x1; x++)
            {
                double r = hypot((double) x - xc, (double) y - yc);
                if (r < 0.55 * rout && ((double) row[x] - bg) < 0.35 * ring_lvl)
                {
                    hole_cnt++;
                }
            }
        }
        double rin = (hole_cnt >= 4u) ? sqrt((double) hole_cnt / 3.1415926535897932) : 0.0;

        out->xc[q]   = (float) xc;
        out->yc[q]   = (float) yc;
        out->rout[q] = (float) rout;
        out->rin[q]  = (float) rin;
        out->corr[q] = 1.0f;
    }

    /* 4-pupil symmetry consensus guard: all 4 pyramid pupils share the same aperture radius.
     * If a single quadrant locked onto an inner diffraction cusp instead of the outer rim,
     * replace it with the mirror-symmetric estimate from the 3 consistent quadrants. */
    float rs[PYRWFS_NPUP];
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        rs[q] = out->rout[q];
    }
    for (int a = 0; a < PYRWFS_NPUP; a++)
    {
        for (int b = a + 1; b < PYRWFS_NPUP; b++)
        {
            if (rs[b] < rs[a])
            {
                float t = rs[a];
                rs[a]   = rs[b];
                rs[b]   = t;
            }
        }
    }
    /* Upper-median radius favors the true outer rim over a shrunken inner-cusp outlier */
    float r_ref = 0.5f * (rs[1] + rs[2]);
    if (rs[3] - rs[1] < 0.08f * rs[3] && rs[1] - rs[0] > 0.12f * rs[3])
    {
        r_ref = rs[2];
    }
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        if (fabsf(out->rout[q] - r_ref) > 0.12f * r_ref)
        {
            /* Opposite in X: (0<->2, 1<->3); opposite in Y: (0<->1, 2<->3) */
            int q_same_x = q ^ 1;
            int q_same_y = q ^ 2;
            if (fabsf(out->rout[q_same_x] - r_ref) <= 0.12f * r_ref &&
                fabsf(out->rout[q_same_y] - r_ref) <= 0.12f * r_ref)
            {
                out->xc[q]   = out->xc[q_same_x];
                out->yc[q]   = out->yc[q_same_y];
                out->rout[q] = r_ref;
                out->rin[q]  = 0.5f * (out->rin[q_same_x] + out->rin[q_same_y]);
            }
        }
    }

    return 0;
}

/* =========================================================================
 * Temporal-Variance 4-Pupil Geometry Detector (3D cube / streaming batch)
 *
 * Principle: In a 4-sided pyramid WFS (modulated or unmodulated, any aperture
 * shape), wavefront phase fluctuations redistribute local flux among the 4
 * sub-pupils at conjugate pupil coordinates, so the sum of the 4 properly
 * aligned pupil images has minimal temporal variance:
 *   Var_t[ sum_q I_q(c_q + u) ] = sum_q Var_t[I_q] + 2 * sum_{p<q} Cov_t[I_p, I_q]
 * Dropping the diagonal self-variance terms eliminates photon/readout noise
 * bias and bilinear noise-smoothing artifacts while strictly avoiding the
 * trivial dark-background solution.
 * ========================================================================= */

static inline int pyrwfs_cmp_double(const void *a, const void *b)
{
    double da = *(const double *) a;
    double db = *(const double *) b;
    return (da < db) ? -1 : ((da > db) ? 1 : 0);
}

static inline double pyrwfs_percentile_double(const double *vals, size_t n, double frac)
{
    if (vals == NULL || n == 0u)
    {
        return 0.0;
    }
    double *tmp = (double *) malloc(n * sizeof(double));
    if (tmp == NULL)
    {
        return vals[0];
    }
    memcpy(tmp, vals, n * sizeof(double));
    qsort(tmp, n, sizeof(double), pyrwfs_cmp_double);
    if (frac <= 0.0)
    {
        double r = tmp[0];
        free(tmp);
        return r;
    }
    if (frac >= 1.0)
    {
        double r = tmp[n - 1u];
        free(tmp);
        return r;
    }
    double pos = frac * (double) (n - 1u);
    size_t i0  = (size_t) floor(pos);
    size_t i1  = (i0 + 1u < n) ? (i0 + 1u) : i0;
    double f   = pos - (double) i0;
    double r   = tmp[i0] + f * (tmp[i1] - tmp[i0]);
    free(tmp);
    return r;
}

static inline uint32_t pyrwfs_next_pow2(uint32_t v)
{
    uint32_t p = 8u;
    while (p < v && p < (1u << 30))
    {
        p <<= 1u;
    }
    return p;
}

/* Self-contained iterative radix-2 Cooley-Tukey 1D FFT (inv=0 forward, inv=1 inverse/N) */
static inline void pyrwfs_fft1d(double *re, double *im, uint32_t n, int inv)
{
    for (uint32_t i = 1u, j = 0u; i < n; i++)
    {
        uint32_t bit = n >> 1u;
        for (; j & bit; bit >>= 1u)
        {
            j ^= bit;
        }
        j ^= bit;
        if (i < j)
        {
            double tr = re[i];
            re[i]     = re[j];
            re[j]     = tr;
            double ti = im[i];
            im[i]     = im[j];
            im[j]     = ti;
        }
    }

    const double sign = inv ? 1.0 : -1.0;
    for (uint32_t len = 2u; len <= n; len <<= 1u)
    {
        double   ang  = sign * (2.0 * 3.14159265358979323846) / (double) len;
        double   wlr  = cos(ang);
        double   wli  = sin(ang);
        uint32_t half = len >> 1u;
        for (uint32_t i = 0u; i < n; i += len)
        {
            double wr = 1.0;
            double wi = 0.0;
            for (uint32_t j = 0u; j < half; j++)
            {
                uint32_t u  = i + j;
                uint32_t v  = u + half;
                double   vr = re[v] * wr - im[v] * wi;
                double   vi = re[v] * wi + im[v] * wr;
                re[v]       = re[u] - vr;
                im[v]       = im[u] - vi;
                re[u] += vr;
                im[u] += vi;
                double nwr = wr * wlr - wi * wli;
                wi         = wr * wli + wi * wlr;
                wr         = nwr;
            }
        }
    }

    if (inv)
    {
        double invn = 1.0 / (double) n;
        for (uint32_t i = 0u; i < n; i++)
        {
            re[i] *= invn;
            im[i] *= invn;
        }
    }
}

/* 2D radix-2 complex FFT (nx, ny must be powers of 2; tmp_re/tmp_im >= ny doubles) */
static inline void pyrwfs_fft2d(double  *re,
                                double  *im,
                                uint32_t nx,
                                uint32_t ny,
                                int      inv,
                                double  *tmp_re,
                                double  *tmp_im)
{
    for (uint32_t y = 0u; y < ny; y++)
    {
        pyrwfs_fft1d(re + (size_t) y * nx, im + (size_t) y * nx, nx, inv);
    }
    for (uint32_t x = 0u; x < nx; x++)
    {
        for (uint32_t y = 0u; y < ny; y++)
        {
            tmp_re[y] = re[(size_t) y * nx + x];
            tmp_im[y] = im[(size_t) y * nx + x];
        }
        pyrwfs_fft1d(tmp_re, tmp_im, ny, inv);
        for (uint32_t y = 0u; y < ny; y++)
        {
            re[(size_t) y * nx + x] = tmp_re[y];
            im[(size_t) y * nx + x] = tmp_im[y];
        }
    }
}

/* Catmull-Rom bicubic interpolation and exact gradient on a 2D double table */
static inline void pyrwfs_bicubic_eval(const double *tab,
                                       int           nx,
                                       int           ny,
                                       double        x,
                                       double        y,
                                       double       *val_out,
                                       double       *gx_out,
                                       double       *gy_out)
{
    int ix = (int) floor(x);
    int iy = (int) floor(y);
    if (ix < 1)
    {
        ix = 1;
    }
    else if (ix > nx - 3)
    {
        ix = nx - 3;
    }
    if (iy < 1)
    {
        iy = 1;
    }
    else if (iy > ny - 3)
    {
        iy = ny - 3;
    }
    double fx = x - (double) ix;
    double fy = y - (double) iy;
    if (fx < 0.0)
    {
        fx = 0.0;
    }
    else if (fx > 1.0)
    {
        fx = 1.0;
    }
    if (fy < 0.0)
    {
        fy = 0.0;
    }
    else if (fy > 1.0)
    {
        fy = 1.0;
    }

    double fx2 = fx * fx;
    double fx3 = fx2 * fx;
    double wx[4] = {
        -0.5 * fx + fx2 - 0.5 * fx3,
        1.0 - 2.5 * fx2 + 1.5 * fx3,
        0.5 * fx + 2.0 * fx2 - 1.5 * fx3,
        -0.5 * fx2 + 0.5 * fx3
    };
    double dwx[4] = {
        -0.5 + 2.0 * fx - 1.5 * fx2,
        -5.0 * fx + 4.5 * fx2,
        0.5 + 4.0 * fx - 4.5 * fx2,
        -1.0 * fx + 1.5 * fx2
    };

    double fy2 = fy * fy;
    double fy3 = fy2 * fy;
    double wy[4] = {
        -0.5 * fy + fy2 - 0.5 * fy3,
        1.0 - 2.5 * fy2 + 1.5 * fy3,
        0.5 * fy + 2.0 * fy2 - 1.5 * fy3,
        -0.5 * fy2 + 0.5 * fy3
    };
    double dwy[4] = {
        -0.5 + 2.0 * fy - 1.5 * fy2,
        -5.0 * fy + 4.5 * fy2,
        0.5 + 4.0 * fy - 4.5 * fy2,
        -1.0 * fy + 1.5 * fy2
    };

    double val = 0.0;
    double gx  = 0.0;
    double gy  = 0.0;
    for (int j = 0; j < 4; j++)
    {
        const double *row = tab + (size_t) (iy + j - 1) * (size_t) nx;
        double        rv  = wx[0] * row[ix - 1] + wx[1] * row[ix] + wx[2] * row[ix + 1] +
                            wx[3] * row[ix + 2];
        double        rgx = dwx[0] * row[ix - 1] + dwx[1] * row[ix] + dwx[2] * row[ix + 1] +
                            dwx[3] * row[ix + 2];
        val += wy[j] * rv;
        gx += wy[j] * rgx;
        gy += dwy[j] * rv;
    }
    *val_out = val;
    *gx_out  = gx;
    *gy_out  = gy;
}

static inline void pyrwfs_p_to_delta(const double p[6], double dx[4], double dy[4])
{
    double ax = p[0], ay = p[1], bx = p[2], by = p[3], cx = p[4], cy = p[5];
    dx[0] = -ax - bx + cx;
    dy[0] = -ay - by + cy;
    dx[1] = -ax + bx - cx;
    dy[1] = -ay + by - cy;
    dx[2] = +ax - bx - cx;
    dy[2] = +ay - by - cy;
    dx[3] = +ax + bx + cx;
    dy[3] = +ay + by + cy;
}

/* Measure Mode C offset (dx, dy) = 0.25*((c0+c3) - (c1+c2)) via soft-Sobel ZNCC
 * between the two tip-tilt-free, centrosymmetric diagonal-pair sums S03 and S12. */
static inline int pyrwfs_align_diag_pair_sums(const float  *img_net,
                                              uint32_t      nx,
                                              uint32_t      ny,
                                              double        cx0,
                                              double        cy0,
                                              const double  del_x[4],
                                              const double  del_y[4],
                                              double        sep_min,
                                              double       *dx_out,
                                              double       *dy_out)
{
    *dx_out = 0.0;
    *dy_out = 0.0;
    int r_c = (int) ceil(0.49 * sep_min) + 2;
    if (r_c < 4)
    {
        return -1;
    }
    int      W    = 2 * r_c + 1;
    size_t   nw2  = (size_t) W * (size_t) W;
    uint32_t P    = pyrwfs_next_pow2((uint32_t) (2 * W));
    size_t   np2  = (size_t) P * (size_t) P;

    float  *s03 = (float *) calloc(nw2, sizeof(float));
    float  *s12 = (float *) calloc(nw2, sizeof(float));
    float  *e03 = (float *) calloc(nw2, sizeof(float));
    float  *e12 = (float *) calloc(nw2, sizeof(float));
    double *ar  = (double *) calloc(np2, sizeof(double));
    double *ai  = (double *) calloc(np2, sizeof(double));
    double *br  = (double *) calloc(np2, sizeof(double));
    double *bi  = (double *) calloc(np2, sizeof(double));
    double *tr  = (double *) malloc((size_t) P * sizeof(double));
    double *ti  = (double *) malloc((size_t) P * sizeof(double));

    if (!s03 || !s12 || !e03 || !e12 || !ar || !ai || !br || !bi || !tr || !ti)
    {
        free(s03); free(s12); free(e03); free(e12);
        free(ar); free(ai); free(br); free(bi); free(tr); free(ti);
        return -1;
    }

    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_c);
        for (int i = 0; i < W; i++)
        {
            double u   = (double) (i - r_c);
            size_t idx = (size_t) j * (size_t) W + (size_t) i;
            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                double xq = cx0 + del_x[q] + u;
                double yq = cy0 + del_y[q] + v;
                if (xq < 0.0 || xq > (double) (nx - 1u) || yq < 0.0 || yq > (double) (ny - 1u))
                {
                    continue;
                }
                int vor_ok = 1;
                for (int r = 0; r < PYRWFS_NPUP; r++)
                {
                    if (r == q)
                    {
                        continue;
                    }
                    double drx = del_x[r] - del_x[q];
                    double dry = del_y[r] - del_y[q];
                    if (u * drx + v * dry > 0.5 * (drx * drx + dry * dry) - 0.5)
                    {
                        vor_ok = 0;
                        break;
                    }
                }
                if (!vor_ok)
                {
                    continue;
                }
                float val = pyrwfs_sample_bilinear(img_net, nx, ny, (float) xq, (float) yq);
                if (q == 0 || q == 3)
                {
                    s03[idx] += val;
                }
                else
                {
                    s12[idx] += val;
                }
            }
        }
    }

    /* Soft-Sobel edge magnitude of each symmetric diagonal sum */
    double gsum0 = 0.0, gsum1 = 0.0;
    size_t gcnt0 = 0u, gcnt1 = 0u;
    for (int j = 1; j < W - 1; j++)
    {
        for (int i = 1; i < W - 1; i++)
        {
            size_t idx = (size_t) j * (size_t) W + (size_t) i;
            float  gx0 = (s03[idx - (size_t) W + 1u] + 2.0f * s03[idx + 1u] +
                          s03[idx + (size_t) W + 1u]) -
                         (s03[idx - (size_t) W - 1u] + 2.0f * s03[idx - 1u] +
                          s03[idx + (size_t) W - 1u]);
            float  gy0 = (s03[idx + (size_t) W - 1u] + 2.0f * s03[idx + (size_t) W] +
                          s03[idx + (size_t) W + 1u]) -
                         (s03[idx - (size_t) W - 1u] + 2.0f * s03[idx - (size_t) W] +
                          s03[idx - (size_t) W + 1u]);
            float  g0  = hypotf(gx0, gy0);
            e03[idx]   = g0;
            if (g0 > 0.0f)
            {
                gsum0 += (double) g0;
                gcnt0++;
            }

            float gx1 = (s12[idx - (size_t) W + 1u] + 2.0f * s12[idx + 1u] +
                         s12[idx + (size_t) W + 1u]) -
                        (s12[idx - (size_t) W - 1u] + 2.0f * s12[idx - 1u] +
                         s12[idx + (size_t) W - 1u]);
            float gy1 = (s12[idx + (size_t) W - 1u] + 2.0f * s12[idx + (size_t) W] +
                         s12[idx + (size_t) W + 1u]) -
                        (s12[idx - (size_t) W - 1u] + 2.0f * s12[idx - (size_t) W] +
                         s12[idx - (size_t) W + 1u]);
            float g1  = hypotf(gx1, gy1);
            e12[idx]  = g1;
            if (g1 > 0.0f)
            {
                gsum1 += (double) g1;
                gcnt1++;
            }
        }
    }
    float sc0 = (gcnt0 > 0u) ? (float) (gsum0 / (double) gcnt0) : 1.0f;
    float sc1 = (gcnt1 > 0u) ? (float) (gsum1 / (double) gcnt1) : 1.0f;
    if (sc0 < 1e-12f)
    {
        sc0 = 1.0f;
    }
    if (sc1 < 1e-12f)
    {
        sc1 = 1.0f;
    }

    double rlim2 = (0.48 * sep_min) * (0.48 * sep_min);
    double m0 = 0.0, m1 = 0.0;
    size_t mc = 0u;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_c);
        for (int i = 0; i < W; i++)
        {
            double u   = (double) (i - r_c);
            size_t idx = (size_t) j * (size_t) W + (size_t) i;
            e03[idx]   = e03[idx] / (e03[idx] + sc0);
            e12[idx]   = e12[idx] / (e12[idx] + sc1);
            if (u * u + v * v <= rlim2)
            {
                m0 += (double) e03[idx];
                m1 += (double) e12[idx];
                mc++;
            }
        }
    }
    if (mc < 16u)
    {
        free(s03); free(s12); free(e03); free(e12);
        free(ar); free(ai); free(br); free(bi); free(tr); free(ti);
        return -1;
    }
    m0 /= (double) mc;
    m1 /= (double) mc;

    double e0_n2 = 0.0, e1_n2 = 0.0;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_c);
        for (int i = 0; i < W; i++)
        {
            double u = (double) (i - r_c);
            if (u * u + v * v <= rlim2)
            {
                size_t src = (size_t) j * (size_t) W + (size_t) i;
                size_t dst = (size_t) j * (size_t) P + (size_t) i;
                double a   = (double) e03[src] - m0;
                double b   = (double) e12[src] - m1;
                ar[dst]    = a;
                br[dst]    = b;
                e0_n2 += a * a;
                e1_n2 += b * b;
            }
        }
    }
    double norm_den = sqrt(e0_n2 * e1_n2);
    if (norm_den < 1e-12)
    {
        free(s03); free(s12); free(e03); free(e12);
        free(ar); free(ai); free(br); free(bi); free(tr); free(ti);
        return -1;
    }

    pyrwfs_fft2d(ar, ai, P, P, 0, tr, ti);
    pyrwfs_fft2d(br, bi, P, P, 0, tr, ti);
    for (size_t i = 0; i < np2; i++)
    {
        /* conj(A) * B */
        double re = ar[i] * br[i] + ai[i] * bi[i];
        double im = ar[i] * bi[i] - ai[i] * br[i];
        ar[i]     = re;
        ai[i]     = im;
    }
    pyrwfs_fft2d(ar, ai, P, P, 1, tr, ti);

    int    mlim = 8;
    if (mlim > (int) (P / 2u) - 2)
    {
        mlim = (int) (P / 2u) - 2;
    }
    int    best_dx = 0;
    int    best_dy = 0;
    double best_v  = -1e30;
    for (int dy = -mlim; dy <= mlim; dy++)
    {
        uint32_t ky = (dy >= 0) ? (uint32_t) dy : (P + (uint32_t) dy);
        for (int dx = -mlim; dx <= mlim; dx++)
        {
            uint32_t kx = (dx >= 0) ? (uint32_t) dx : (P + (uint32_t) dx);
            double   v  = ar[(size_t) ky * P + kx];
            if (v > best_v)
            {
                best_v  = v;
                best_dx = dx;
                best_dy = dy;
            }
        }
    }

    double sub_x = 0.0;
    double sub_y = 0.0;
    if (best_dx > -mlim && best_dx < mlim)
    {
        uint32_t ky  = (best_dy >= 0) ? (uint32_t) best_dy : (P + (uint32_t) best_dy);
        uint32_t kxm = (best_dx - 1 >= 0) ? (uint32_t) (best_dx - 1) : (P + (uint32_t) (best_dx - 1));
        uint32_t kx0 = (best_dx >= 0) ? (uint32_t) best_dx : (P + (uint32_t) best_dx);
        uint32_t kxp = (best_dx + 1 >= 0) ? (uint32_t) (best_dx + 1) : (P + (uint32_t) (best_dx + 1));
        double   cm  = ar[(size_t) ky * P + kxm];
        double   c0  = ar[(size_t) ky * P + kx0];
        double   cp  = ar[(size_t) ky * P + kxp];
        double   den = cm - 2.0 * c0 + cp;
        if (den < -1e-12)
        {
            sub_x = 0.5 * (cm - cp) / den;
            if (sub_x > 0.5)
            {
                sub_x = 0.5;
            }
            else if (sub_x < -0.5)
            {
                sub_x = -0.5;
            }
        }
    }
    if (best_dy > -mlim && best_dy < mlim)
    {
        uint32_t kx  = (best_dx >= 0) ? (uint32_t) best_dx : (P + (uint32_t) best_dx);
        uint32_t kym = (best_dy - 1 >= 0) ? (uint32_t) (best_dy - 1) : (P + (uint32_t) (best_dy - 1));
        uint32_t ky0 = (best_dy >= 0) ? (uint32_t) best_dy : (P + (uint32_t) best_dy);
        uint32_t kyp = (best_dy + 1 >= 0) ? (uint32_t) (best_dy + 1) : (P + (uint32_t) (best_dy + 1));
        double   cm  = ar[(size_t) kym * P + kx];
        double   c0  = ar[(size_t) ky0 * P + kx];
        double   cp  = ar[(size_t) kyp * P + kx];
        double   den = cm - 2.0 * c0 + cp;
        if (den < -1e-12)
        {
            sub_y = 0.5 * (cm - cp) / den;
            if (sub_y > 0.5)
            {
                sub_y = 0.5;
            }
            else if (sub_y < -0.5)
            {
                sub_y = -0.5;
            }
        }
    }

    double zncc = best_v / norm_den;
    int    ok   = (zncc > 0.20) ? 0 : -1;
    if (ok == 0)
    {
        double dx_e = -0.5 * ((double) best_dx + sub_x);
        double dy_e = -0.5 * ((double) best_dy + sub_y);
        if (dx_e > 5.0)
        {
            dx_e = 5.0;
        }
        else if (dx_e < -5.0)
        {
            dx_e = -5.0;
        }
        if (dy_e > 5.0)
        {
            dy_e = 5.0;
        }
        else if (dy_e < -5.0)
        {
            dy_e = -5.0;
        }
        *dx_out = dx_e;
        *dy_out = dy_e;
    }

    free(s03); free(s12); free(e03); free(e12);
    free(ar); free(ai); free(br); free(bi); free(tr); free(ti);
    return ok;
}

/* Fit global center (cx, cy) and outer/inner radii (rout, rin) on the Voronoi-clean
 * 4-pupil aligned sum of a background-subtracted image (4-fold symmetric, TT-invariant). */
static inline int pyrwfs_fit_aligned_sum_circle(const float  *img_net,
                                                uint32_t      nx,
                                                uint32_t      ny,
                                                double        cx0,
                                                double        cy0,
                                                const double  del_x[4],
                                                const double  del_y[4],
                                                double        sep_min,
                                                double       *cx_out,
                                                double       *cy_out,
                                                double       *rout_out,
                                                double       *rin_out)
{
    int r_canv = (int) ceil(0.50 * sep_min) + 6;
    if (r_canv < 6)
    {
        return -1;
    }
    int    W   = 2 * r_canv + 1;
    size_t nw2 = (size_t) W * (size_t) W;
    float *sm  = (float *) calloc(nw2, sizeof(float));
    double *pos_buf = (double *) malloc(nw2 * sizeof(double));
    if (sm == NULL || pos_buf == NULL)
    {
        free(sm);
        free(pos_buf);
        return -1;
    }

    double r_disk = 0.49 * sep_min;
    size_t npos   = 0u;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_canv);
        for (int i = 0; i < W; i++)
        {
            double u   = (double) (i - r_canv);
            double acc = 0.0;
            int    cnt = 0;
            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                double xq = cx0 + del_x[q] + u;
                double yq = cy0 + del_y[q] + v;
                if (xq < 0.0 || xq > (double) (nx - 1u) || yq < 0.0 || yq > (double) (ny - 1u))
                {
                    continue;
                }
                int vor_ok = 1;
                for (int r = 0; r < PYRWFS_NPUP; r++)
                {
                    if (r == q)
                    {
                        continue;
                    }
                    double drx = del_x[r] - del_x[q];
                    double dry = del_y[r] - del_y[q];
                    if (u * drx + v * dry > 0.5 * (drx * drx + dry * dry) - 0.5)
                    {
                        vor_ok = 0;
                        break;
                    }
                }
                if (!vor_ok)
                {
                    continue;
                }
                acc += (double) pyrwfs_sample_bilinear(img_net, nx, ny, (float) xq, (float) yq);
                cnt++;
            }
            float sval = (cnt > 0) ? (float) (acc * (4.0 / (double) cnt)) : 0.0f;
            sm[(size_t) j * (size_t) W + (size_t) i] = sval;
            if (u * u + v * v <= r_disk * r_disk && sval > 0.0f)
            {
                pos_buf[npos++] = (double) sval;
            }
        }
    }

    double med_s = 1.0;
    if (npos > 0u)
    {
        double p30 = pyrwfs_percentile_double(pos_buf, npos, 0.30);
        size_t n2  = 0u;
        for (size_t k = 0; k < npos; k++)
        {
            if (pos_buf[k] > p30)
            {
                pos_buf[n2++] = pos_buf[k];
            }
        }
        med_s = (n2 > 0u) ? pyrwfs_percentile_double(pos_buf, n2, 0.50)
                          : pyrwfs_percentile_double(pos_buf, npos, 0.50);
        if (med_s < 1e-9)
        {
            med_s = 1.0;
        }
    }

    /* Coarse center and radius from binary footprint inside r <= 0.49*sep_min */
    double su = 0.0, sv = 0.0;
    size_t nbw = 0u;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_canv);
        for (int i = 0; i < W; i++)
        {
            double u = (double) (i - r_canv);
            if (u * u + v * v <= r_disk * r_disk &&
                (double) sm[(size_t) j * (size_t) W + (size_t) i] > 0.35 * med_s)
            {
                su += u;
                sv += v;
                nbw++;
            }
        }
    }
    double uc = (nbw > 0u) ? (su / (double) nbw) : 0.0;
    double vc = (nbw > 0u) ? (sv / (double) nbw) : 0.0;
    double rc = (nbw >= 9u) ? (sqrt((double) nbw / 3.1415926535897932) * 1.05) : (0.35 * sep_min);
    if (rc < 3.0)
    {
        rc = 3.0;
    }

    /* 4-pass 360-deg radial-cliff 2-step IRLS Kasa circle fit */
    double r_max_lim = 0.495 * sep_min;
    for (int iter = 0; iter < 4; iter++)
    {
        double rlo = (iter == 0) ? 3.0 : (0.80 * rc);
        double rhi = (iter == 0) ? r_max_lim : (1.18 * rc);
        if (rlo < 3.0)
        {
            rlo = 3.0;
        }
        if (rhi > r_max_lim)
        {
            rhi = r_max_lim;
        }
        double px[72], py[72], pw[72];
        int    npts = 0;
        double rm_c = (double) r_canv - 2.0 - hypot(uc, vc);
        double rm   = (rhi < rm_c) ? rhi : rm_c;
        if (rm > rlo + 1.5)
        {
            double step  = 0.5;
            int    nstep = (int) floor((rm - rlo) / step);
            if (nstep > 255)
            {
                nstep = 255;
            }
            if (nstep >= 4)
            {
                for (int k = 0; k < 72; k++)
                {
                    double th = (2.0 * 3.1415926535897932 * (double) k) / 72.0;
                    double ct = cos(th);
                    double st = sin(th);
                    double drops[256];
                    double md = 0.0;
                    for (int s = 0; s <= nstep; s++)
                    {
                        double r  = rlo + (double) s * step;
                        float  vi = pyrwfs_sample_bilinear(sm, (uint32_t) W, (uint32_t) W,
                                                           (float) (uc + (r - 0.50) * ct + r_canv),
                                                           (float) (vc + (r - 0.50) * st + r_canv));
                        float  vo = pyrwfs_sample_bilinear(sm, (uint32_t) W, (uint32_t) W,
                                                           (float) (uc + (r + 0.50) * ct + r_canv),
                                                           (float) (vc + (r + 0.50) * st + r_canv));
                        double d  = (double) (vi - vo);
                        drops[s]  = d;
                        if (d > md)
                        {
                            md = d;
                        }
                    }
                    if (md <= 0.05 * med_s)
                    {
                        continue;
                    }
                    int best_s = -1;
                    for (int s = nstep - 1; s >= 1; s--)
                    {
                        if (drops[s] >= 0.45 * md && drops[s] >= drops[s - 1] &&
                            drops[s] >= drops[s + 1])
                        {
                            best_s = s;
                            break;
                        }
                    }
                    if (best_s < 0)
                    {
                        continue;
                    }
                    double best_r = rlo + (double) best_s * step;
                    double dm     = drops[best_s - 1];
                    double d0     = drops[best_s];
                    double dp     = drops[best_s + 1];
                    double den    = dm - 2.0 * d0 + dp;
                    if (den < -1e-12)
                    {
                        double off = 0.5 * (dm - dp) / den;
                        if (off > 0.5)
                        {
                            off = 0.5;
                        }
                        else if (off < -0.5)
                        {
                            off = -0.5;
                        }
                        best_r += off * step;
                    }
                    px[npts] = uc + best_r * ct;
                    py[npts] = vc + best_r * st;
                    pw[npts] = (drops[best_s] < med_s) ? drops[best_s] : med_s;
                    npts++;
                }
            }
        }
        if (npts >= 8)
        {
            double fx = uc, fy = vc, fr = rc;
            for (int rf = 0; rf < 2; rf++)
            {
                double A[3][3] = { { 0 } };
                double b[3]    = { 0 };
                int    nkeep   = 0;
                for (int i = 0; i < npts; i++)
                {
                    if (rf == 1 && fabs(hypot(px[i] - fx, py[i] - fy) - fr) > 1.2)
                    {
                        continue;
                    }
                    double w      = pw[i];
                    double rhs    = px[i] * px[i] + py[i] * py[i];
                    double row[3] = { 2.0 * px[i], 2.0 * py[i], 1.0 };
                    for (int a = 0; a < 3; a++)
                    {
                        b[a] += w * row[a] * rhs;
                        for (int c = 0; c < 3; c++)
                        {
                            A[a][c] += w * row[a] * row[c];
                        }
                    }
                    nkeep++;
                }
                double sol[3];
                if (nkeep >= 6 && pyrwfs_solve_3x3(A, b, sol) == 0)
                {
                    double r2 = sol[2] + sol[0] * sol[0] + sol[1] * sol[1];
                    if (r2 > 4.0)
                    {
                        fx = sol[0];
                        fy = sol[1];
                        fr = sqrt(r2);
                    }
                }
            }
            uc = fx;
            vc = fy;
            rc = fr;
        }
    }

    /* Central obscuration Rin inside r < 0.52*rc */
    size_t nring = 0u;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_canv) - vc;
        for (int i = 0; i < W; i++)
        {
            double u = (double) (i - r_canv) - uc;
            double r = hypot(u, v);
            if (r > 0.55 * rc && r < 0.85 * rc)
            {
                pos_buf[nring++] = (double) sm[(size_t) j * (size_t) W + (size_t) i];
            }
        }
    }
    double ring_lvl = (nring > 0u) ? pyrwfs_percentile_double(pos_buf, nring, 0.50) : med_s;
    size_t nhole    = 0u;
    for (int j = 0; j < W; j++)
    {
        double v = (double) (j - r_canv) - vc;
        for (int i = 0; i < W; i++)
        {
            double u = (double) (i - r_canv) - uc;
            if (hypot(u, v) < 0.52 * rc &&
                (double) sm[(size_t) j * (size_t) W + (size_t) i] < 0.42 * ring_lvl)
            {
                nhole++;
            }
        }
    }
    double rin = (nhole >= 4u) ? sqrt((double) nhole / 3.1415926535897932) : 0.0;

    *cx_out   = cx0 + uc;
    *cy_out   = cy0 + vc;
    *rout_out = rc;
    *rin_out  = rin;

    free(sm);
    free(pos_buf);
    return 0;
}

/* Helper struct for 6-DOF cross-covariance objective evaluation inside BFGS */
typedef struct
{
    double *K_tabs[6];
    int     M;
    int     tab_w;
    int     c0x[4];
    int     c0y[4];
    double  dx_e;
    double  dy_e;
    double  lam_c;
} pyrwfs_xcov_obj_t;

static inline double pyrwfs_xcov_eval(const pyrwfs_xcov_obj_t *ctx,
                                      const double             p[6],
                                      double                   grad[6])
{
    static const int    pairs[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
    static const double sx_m[4][3]  = {
        { -1.0, -1.0, +1.0 },
        { -1.0, +1.0, -1.0 },
        { +1.0, -1.0, -1.0 },
        { +1.0, +1.0, +1.0 }
    };
    double del_x[4], del_y[4];
    pyrwfs_p_to_delta(p, del_x, del_y);

    double val    = 0.0;
    double gqx[4] = { 0.0, 0.0, 0.0, 0.0 };
    double gqy[4] = { 0.0, 0.0, 0.0, 0.0 };
    for (int k = 0; k < 6; k++)
    {
        int    pi = pairs[k][0];
        int    qi = pairs[k][1];
        double du = (del_x[qi] - del_x[pi]) - (double) (ctx->c0x[qi] - ctx->c0x[pi]);
        double dv = (del_y[qi] - del_y[pi]) - (double) (ctx->c0y[qi] - ctx->c0y[pi]);
        double v, gx, gy;
        pyrwfs_bicubic_eval(ctx->K_tabs[k], ctx->tab_w, ctx->tab_w, 2.0 * du + (double) ctx->M,
                            2.0 * dv + (double) ctx->M, &v, &gx, &gy);
        gx *= 2.0;
        gy *= 2.0;
        val += v;
        gqx[qi] += gx;
        gqy[qi] += gy;
        gqx[pi] -= gx;
        gqy[pi] -= gy;
    }

    if (grad != NULL)
    {
        for (int m = 0; m < 3; m++)
        {
            double g0 = 0.0, g1 = 0.0;
            for (int q = 0; q < 4; q++)
            {
                g0 += sx_m[q][m] * gqx[q];
                g1 += sx_m[q][m] * gqy[q];
            }
            grad[2 * m]     = g0;
            grad[2 * m + 1] = g1;
        }
    }

    if (ctx->lam_c > 0.0)
    {
        double ex = p[4] - ctx->dx_e;
        double ey = p[5] - ctx->dy_e;
        val += 0.5 * ctx->lam_c * (ex * ex + ey * ey);
        if (grad != NULL)
        {
            grad[4] += ctx->lam_c * ex;
            grad[5] += ctx->lam_c * ey;
        }
    }
    return val;
}

/*
 * Quadrupole objective f_Q(p) = Var((I_0 + I_3) - (I_1 + I_2)) - sum_q Var(I_q)
 * = 2 * (K_03 + K_12 - K_01 - K_02 - K_13 - K_23).
 *
 * Scintillation s = -z * laplacian(W) cancels identically in Q = (I_0 + I_3) - (I_1 + I_2),
 * whereas an isotropic radial-separation error ds produces Q = 8 * ds * G * W_xy.
 * Refining the single isotropic radial-scaling mode ds via f_Q removes the +z_eff / G
 * scintillation bias present in Var(I_0 + I_1 + I_2 + I_3) while preserving all other 5 DOFs.
 */
static inline double pyrwfs_xcov_eval_q(const pyrwfs_xcov_obj_t *ctx,
                                        const double             p[6],
                                        double                   grad[6])
{
    static const int    pairs[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
    static const double qw[6]       = { -1.0, -1.0, +1.0, +1.0, -1.0, -1.0 };
    static const double sx_m[4][3]  = {
        { -1.0, -1.0, +1.0 },
        { -1.0, +1.0, -1.0 },
        { +1.0, -1.0, -1.0 },
        { +1.0, +1.0, +1.0 }
    };
    double del_x[4], del_y[4];
    pyrwfs_p_to_delta(p, del_x, del_y);

    double val    = 0.0;
    double gqx[4] = { 0.0, 0.0, 0.0, 0.0 };
    double gqy[4] = { 0.0, 0.0, 0.0, 0.0 };
    for (int k = 0; k < 6; k++)
    {
        int    pi = pairs[k][0];
        int    qi = pairs[k][1];
        double du = (del_x[qi] - del_x[pi]) - (double) (ctx->c0x[qi] - ctx->c0x[pi]);
        double dv = (del_y[qi] - del_y[pi]) - (double) (ctx->c0y[qi] - ctx->c0y[pi]);
        double v, gx, gy;
        pyrwfs_bicubic_eval(ctx->K_tabs[k], ctx->tab_w, ctx->tab_w, 2.0 * du + (double) ctx->M,
                            2.0 * dv + (double) ctx->M, &v, &gx, &gy);
        gx *= 2.0;
        gy *= 2.0;
        double w = qw[k];
        val += w * v;
        gqx[qi] += w * gx;
        gqy[qi] += w * gy;
        gqx[pi] -= w * gx;
        gqy[pi] -= w * gy;
    }
    if (grad != NULL)
    {
        for (int m = 0; m < 3; m++)
        {
            double g0 = 0.0, g1 = 0.0;
            for (int q = 0; q < 4; q++)
            {
                g0 += sx_m[q][m] * gqx[q];
                g1 += sx_m[q][m] * gqy[q];
            }
            grad[2 * m]     = g0;
            grad[2 * m + 1] = g1;
        }
    }
    return val;
}

static inline void pyrwfs_debias_radial_quadrupole(const pyrwfs_xcov_obj_t *ctx, double p[6])
{
    double sa    = hypot(p[0], p[1]);
    double sb    = hypot(p[2], p[3]);
    double smean = 0.5 * (sa + sb);
    if (smean < 4.0)
    {
        return;
    }
    double       u[4] = { p[0] / smean, p[1] / smean, p[2] / smean, p[3] / smean };
    const double h    = 0.25;
    double       pp[6], pm[6];
    memcpy(pp, p, sizeof(pp));
    memcpy(pm, p, sizeof(pm));
    for (int i = 0; i < 4; i++)
    {
        pp[i] += h * u[i];
        pm[i] -= h * u[i];
    }
    double f0_q   = pyrwfs_xcov_eval_q(ctx, p, NULL);
    double fp_q   = pyrwfs_xcov_eval_q(ctx, pp, NULL);
    double fm_q   = pyrwfs_xcov_eval_q(ctx, pm, NULL);
    double curv_q = (fp_q + fm_q - 2.0 * f0_q) / (h * h);

    double f0_s   = pyrwfs_xcov_eval(ctx, p, NULL);
    double fp_s   = pyrwfs_xcov_eval(ctx, pp, NULL);
    double fm_s   = pyrwfs_xcov_eval(ctx, pm, NULL);
    double curv_s = (fp_s + fm_s - 2.0 * f0_s) / (h * h);

    if (f0_q >= 0.0 || curv_q <= 1e-12 || (curv_s > 1e-12 && curv_q < 0.20 * curv_s))
    {
        return;
    }

    double ds    = 0.0;
    double cur_f = f0_q;
    double pt[6];
    memcpy(pt, p, sizeof(pt));
    for (int it = 0; it < 16; it++)
    {
        double g[6];
        for (int i = 0; i < 4; i++) pt[i] = p[i] + ds * u[i];
        cur_f     = pyrwfs_xcov_eval_q(ctx, pt, g);
        double df = g[0] * u[0] + g[1] * u[1] + g[2] * u[2] + g[3] * u[3];
        if (fabs(df) < 1e-12)
        {
            break;
        }
        const double dh = 0.05;
        double       gp[6], gm[6];
        for (int i = 0; i < 4; i++) pp[i] = p[i] + (ds + dh) * u[i];
        for (int i = 0; i < 4; i++) pm[i] = p[i] + (ds - dh) * u[i];
        (void) pyrwfs_xcov_eval_q(ctx, pp, gp);
        (void) pyrwfs_xcov_eval_q(ctx, pm, gm);
        double dfp = gp[0] * u[0] + gp[1] * u[1] + gp[2] * u[2] + gp[3] * u[3];
        double dfm = gm[0] * u[0] + gm[1] * u[1] + gm[2] * u[2] + gm[3] * u[3];
        double d2f = (dfp - dfm) / (2.0 * dh);
        if (d2f < 0.25 * curv_q)
        {
            d2f = curv_q;
        }
        double step = -df / d2f;
        if (step > 0.25) step = 0.25;
        else if (step < -0.25) step = -0.25;
        if (fabs(step) < 1e-6)
        {
            break;
        }
        double alpha    = 1.0;
        int    accepted = 0;
        for (int ls = 0; ls < 10; ls++)
        {
            double ds_new = ds + alpha * step;
            if (ds_new > 0.5) ds_new = 0.5;
            else if (ds_new < -0.5) ds_new = -0.5;
            for (int i = 0; i < 4; i++) pt[i] = p[i] + ds_new * u[i];
            double fn = pyrwfs_xcov_eval_q(ctx, pt, NULL);
            if (fn < cur_f)
            {
                ds       = ds_new;
                cur_f    = fn;
                accepted = 1;
                break;
            }
            alpha *= 0.5;
        }
        if (!accepted)
        {
            break;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        p[i] += ds * u[i];
    }
}

/* Global integer-basin scan + sub-pixel 2D bicubic minimizer on a (2*M+1)^2 covariance table */
static inline void pyrwfs_min2d_bicubic(const double *tab, int M, double *u_out, double *v_out)
{
    const int tab_w = 2 * M + 1;
    const int lim   = (M > 2) ? (M - 2) : 1;

    int    best_du = 0;
    int    best_dv = 0;
    double best_v  = tab[(size_t) M * (size_t) tab_w + (size_t) M];
    for (int dv = -lim; dv <= lim; dv++)
    {
        const double *row = tab + (size_t) (dv + M) * (size_t) tab_w;
        for (int du = -lim; du <= lim; du++)
        {
            double v = row[du + M];
            if (v < best_v)
            {
                best_v  = v;
                best_du = du;
                best_dv = dv;
            }
        }
    }

    double u = (double) best_du;
    double v = (double) best_dv;
    double f = 0.0, gx = 0.0, gy = 0.0;
    pyrwfs_bicubic_eval(tab, tab_w, tab_w, u + (double) M, v + (double) M, &f, &gx, &gy);

    double B00 = 1.0, B01 = 0.0, B11 = 1.0;
    {
        const double h = 0.5;
        double       fp, fm, d1, d2;
        double       hdiag = 0.0;
        pyrwfs_bicubic_eval(tab, tab_w, tab_w, u + h + (double) M, v + (double) M, &fp, &d1, &d2);
        pyrwfs_bicubic_eval(tab, tab_w, tab_w, u - h + (double) M, v + (double) M, &fm, &d1, &d2);
        double cx = (fp + fm - 2.0 * f) / (h * h);
        pyrwfs_bicubic_eval(tab, tab_w, tab_w, u + (double) M, v + h + (double) M, &fp, &d1, &d2);
        pyrwfs_bicubic_eval(tab, tab_w, tab_w, u + (double) M, v - h + (double) M, &fm, &d1, &d2);
        double cy = (fp + fm - 2.0 * f) / (h * h);
        if (cx > 0.0) hdiag += cx;
        if (cy > 0.0) hdiag += cy;
        if (hdiag > 1e-12)
        {
            B00 = 2.0 / hdiag;
            B11 = 2.0 / hdiag;
        }
    }

    for (int it = 0; it < 28; it++)
    {
        if (gx * gx + gy * gy < 1e-20)
        {
            break;
        }
        double dx = -(B00 * gx + B01 * gy);
        double dy = -(B01 * gx + B11 * gy);
        double dg = dx * gx + dy * gy;
        if (dg >= 0.0)
        {
            double sc = 0.5 * (B00 + B11);
            if (sc <= 0.0) sc = 1.0;
            B00 = sc; B01 = 0.0; B11 = sc;
            dx = -sc * gx;
            dy = -sc * gy;
            dg = dx * gx + dy * gy;
        }
        double dmax = (fabs(dx) > fabs(dy)) ? fabs(dx) : fabs(dy);
        if (dmax > 0.75)
        {
            double sc = 0.75 / dmax;
            dx *= sc;
            dy *= sc;
            dg *= sc;
        }
        if (dmax < 1e-6)
        {
            break;
        }

        double alpha = 1.0;
        double un = u, vn = v, fn = f;
        int    accepted = 0;
        for (int ls = 0; ls < 12; ls++)
        {
            double ut = u + alpha * dx;
            double vt = v + alpha * dy;
            if (ut > (double) lim) ut = (double) lim;
            else if (ut < -(double) lim) ut = -(double) lim;
            if (vt > (double) lim) vt = (double) lim;
            else if (vt < -(double) lim) vt = -(double) lim;
            double d1, d2;
            pyrwfs_bicubic_eval(tab, tab_w, tab_w, ut + (double) M, vt + (double) M, &fn, &d1, &d2);
            if (fn <= f + 1e-4 * alpha * dg)
            {
                un       = ut;
                vn       = vt;
                accepted = 1;
                break;
            }
            alpha *= 0.5;
        }
        if (!accepted)
        {
            break;
        }
        double gxn = 0.0, gyn = 0.0;
        pyrwfs_bicubic_eval(tab, tab_w, tab_w, un + (double) M, vn + (double) M, &fn, &gxn, &gyn);
        double sx = un - u, sy_s = vn - v;
        double yx = gxn - gx, yy_s = gyn - gy;
        double sdoty = sx * yx + sy_s * yy_s;
        u = un; v = vn; f = fn; gx = gxn; gy = gyn;
        if (sdoty > 1e-18)
        {
            double rho  = 1.0 / sdoty;
            double By0  = B00 * yx + B01 * yy_s;
            double By1  = B01 * yx + B11 * yy_s;
            double yBy  = yx * By0 + yy_s * By1;
            double fac  = (1.0 + rho * yBy) * rho;
            B00 += fac * sx * sx - 2.0 * rho * By0 * sx;
            B01 += fac * sx * sy_s - rho * (By0 * sy_s + sx * By1);
            B11 += fac * sy_s * sy_s - 2.0 * rho * By1 * sy_s;
        }
    }

    *u_out = 0.5 * u;
    *v_out = 0.5 * v;
}

/* Auto-detect 4 pupil centers and radii from a 3D WFS cube (nframes x ny x nx)
 * by minimizing the noise-unbiased 4-pupil sum temporal variance.
 * Optional `ref_im` (may be NULL) anchors the global center (cx, cy) to `ref_im`
 * while retaining sub-0.05-pix relative pupil alignment and clean turbulent radii.
 * Falls back gracefully to single-frame `pyrwfs_detect_pupils` if temporal signal is absent. */
static inline int pyrwfs_detect_pupils_cube(const float      *cube,
                                            uint32_t          nx,
                                            uint32_t          ny,
                                            uint32_t          nframes,
                                            const float      *ref_im,
                                            pyrwfs_pupgeom_t *out)
{
    if (cube == NULL || out == NULL || nx < 16u || ny < 16u)
    {
        return -1;
    }
    if (nframes < 4u)
    {
        return pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : cube, nx, ny, 0.25f, out);
    }

    const size_t nimg = (size_t) nx * (size_t) ny;

    /* 0. Estimate corner background & reject dark / shutter-closed frames */
    uint32_t cb = ((nx < ny) ? nx : ny) / 12u;
    if (cb < 2u)
    {
        cb = 2u;
    }
    if (cb > 8u)
    {
        cb = 8u;
    }

    double   *bg_fr    = (double *) malloc((size_t) nframes * sizeof(double));
    double   *fflux    = (double *) malloc((size_t) nframes * sizeof(double));
    uint32_t *good_idx = (uint32_t *) malloc((size_t) nframes * sizeof(uint32_t));
    if (!bg_fr || !fflux || !good_idx)
    {
        free(bg_fr); free(fflux); free(good_idx);
        return pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : cube, nx, ny, 0.25f, out);
    }

    for (uint32_t t = 0u; t < nframes; t++)
    {
        const float *fr = cube + (size_t) t * nimg;
        double       bs = 0.0;
        size_t       bc = 0u;
        for (uint32_t y = 0u; y < cb; y++)
        {
            for (uint32_t x = 0u; x < cb; x++)
            {
                bs += (double) fr[(size_t) y * nx + x];
                bs += (double) fr[(size_t) y * nx + (nx - 1u - x)];
                bs += (double) fr[(size_t) (ny - 1u - y) * nx + x];
                bs += (double) fr[(size_t) (ny - 1u - y) * nx + (nx - 1u - x)];
                bc += 4u;
            }
        }
        bg_fr[t] = (bc > 0u) ? (bs / (double) bc) : 0.0;
    }
    double bg_corner = pyrwfs_percentile_double(bg_fr, nframes, 0.50);

    for (uint32_t t = 0u; t < nframes; t++)
    {
        const float *fr = cube + (size_t) t * nimg;
        double       fs = 0.0;
        for (size_t i = 0; i < nimg; i++)
        {
            fs += (double) fr[i] - bg_corner;
        }
        fflux[t] = fs;
    }
    double   fmed  = pyrwfs_percentile_double(fflux, nframes, 0.50);
    uint32_t ngood = 0u;
    for (uint32_t t = 0u; t < nframes; t++)
    {
        if (fmed <= 0.0 || fflux[t] > 0.25 * fmed)
        {
            good_idx[ngood++] = t;
        }
    }
    free(bg_fr);
    free(fflux);
    if (ngood < 4u)
    {
        free(good_idx);
        return pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : cube, nx, ny, 0.25f, out);
    }

    /* Cap at 128 evenly spaced valid frames for bounded runtime */
    uint32_t T = (ngood > 128u) ? 128u : ngood;
    float   *mean_im  = (float *) calloc(nimg, sizeof(float));
    float   *mean_net = (float *) calloc(nimg, sizeof(float));
    float   *dX       = (float *) malloc((size_t) T * nimg * sizeof(float));
    double  *e_fr     = (double *) malloc((size_t) T * sizeof(double));
    uint32_t *keep_t  = (uint32_t *) malloc((size_t) T * sizeof(uint32_t));
    if (!mean_im || !mean_net || !dX || !e_fr || !keep_t)
    {
        free(good_idx); free(mean_im); free(mean_net); free(dX); free(e_fr); free(keep_t);
        return pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : cube, nx, ny, 0.25f, out);
    }

    for (uint32_t k = 0u; k < T; k++)
    {
        uint32_t     src_t = good_idx[(uint32_t) (((uint64_t) k * ngood) / T)];
        const float *fr    = cube + (size_t) src_t * nimg;
        float       *dst   = dX + (size_t) k * nimg;
        memcpy(dst, fr, nimg * sizeof(float));
        for (size_t i = 0; i < nimg; i++)
        {
            mean_im[i] += fr[i];
        }
    }
    free(good_idx);

    float  invT     = 1.0f / (float) T;
    double mean_s   = 0.0;
    double mean_s2  = 0.0;
    for (size_t i = 0; i < nimg; i++)
    {
        mean_im[i] *= invT;
        double v    = (double) mean_im[i] - bg_corner;
        mean_net[i] = (v > 0.0) ? (float) v : 0.0f;
        mean_s += v;
        mean_s2 += v * v;
    }
    double mean_var = mean_s2 / (double) nimg - (mean_s / (double) nimg) * (mean_s / (double) nimg);
    if (mean_var < 0.0)
    {
        mean_var = 0.0;
    }

    /* Block-detrended fluctuations (blocks of ~16 frames) reject slow pupil jumps / Strehl steps */
    uint32_t nblk = (T >= 24u) ? (T / 16u) : 1u;
    if (nblk < 1u)
    {
        nblk = 1u;
    }
    float *blk_mean = (float *) calloc(nimg, sizeof(float));
    if (blk_mean == NULL)
    {
        free(mean_im); free(mean_net); free(dX); free(e_fr); free(keep_t);
        return pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : cube, nx, ny, 0.25f, out);
    }
    for (uint32_t b = 0u; b < nblk; b++)
    {
        uint32_t t0 = (uint32_t) (((uint64_t) b * T) / nblk);
        uint32_t t1 = (uint32_t) (((uint64_t) (b + 1u) * T) / nblk);
        uint32_t nb = t1 - t0;
        memset(blk_mean, 0, nimg * sizeof(float));
        for (uint32_t k = t0; k < t1; k++)
        {
            const float *fr = dX + (size_t) k * nimg;
            for (size_t i = 0; i < nimg; i++)
            {
                blk_mean[i] += fr[i];
            }
        }
        float invb = 1.0f / (float) nb;
        for (size_t i = 0; i < nimg; i++)
        {
            blk_mean[i] *= invb;
        }
        for (uint32_t k = t0; k < t1; k++)
        {
            float *fr = dX + (size_t) k * nimg;
            double e  = 0.0;
            for (size_t i = 0; i < nimg; i++)
            {
                float dv = fr[i] - blk_mean[i];
                fr[i]    = dv;
                e += (double) dv * (double) dv;
            }
            e_fr[k] = e / (double) nimg;
        }
    }
    free(blk_mean);

    double emed = pyrwfs_percentile_double(e_fr, T, 0.50);
    if (emed <= 1e-10 * (mean_var + 1e-12))
    {
        /* Static cube (no wavefront variation): fall back to single-frame detector */
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net); free(dX); free(e_fr); free(keep_t);
        return rc_fb;
    }

    /* Drop outlier step-transition frames (> 3.5x median L2 energy) */
    uint32_t T_use = 0u;
    for (uint32_t k = 0u; k < T; k++)
    {
        if (e_fr[k] <= 3.5 * emed)
        {
            keep_t[T_use++] = k;
        }
    }
    if (T_use < 4u)
    {
        T_use = T;
        for (uint32_t k = 0u; k < T; k++)
        {
            keep_t[k] = k;
        }
    }
    free(e_fr);

    /* --- Step 1: Global full-image temporal-fluctuation autocovariance A(dx, dy) ---
     * Keep full resolution up to 256x256 so high-Strehl closed-loop residual wavefront
     * anti-correlation peaks (~1-2 pix wide, flanked by positive sidelobes) are not
     * cancelled by 2x2 box binning. Pack 2 real frames per complex FFT (exact Re(IFFT)). */
    int      bin2    = (nx > 256u || ny > 256u) ? 1 : 0;
    uint32_t nxb     = bin2 ? (nx / 2u) : nx;
    uint32_t nyb     = bin2 ? (ny / 2u) : ny;
    double   scale_b = bin2 ? 2.0 : 1.0;
    uint32_t px      = pyrwfs_next_pow2(2u * nxb);
    uint32_t py      = pyrwfs_next_pow2(2u * nyb);
    size_t   np_a    = (size_t) px * (size_t) py;
    size_t   nt_a    = (size_t) ((px > py) ? px : py);
    uint32_t npairs  = (T_use + 1u) / 2u;

    double *P_acc = (double *) calloc(np_a, sizeof(double));
    double *re_a  = (double *) malloc(np_a * sizeof(double));
    double *im_a  = (double *) malloc(np_a * sizeof(double));
    double *tr_a  = (double *) malloc(nt_a * sizeof(double));
    double *ti_a  = (double *) malloc(nt_a * sizeof(double));
    if (!P_acc || !re_a || !im_a || !tr_a || !ti_a)
    {
        free(P_acc); free(re_a); free(im_a); free(tr_a); free(ti_a);
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net); free(dX); free(keep_t);
        return rc_fb;
    }

#ifdef _OPENMP
#    pragma omp parallel
    {
        double *l_re  = (double *) malloc(np_a * sizeof(double));
        double *l_im  = (double *) malloc(np_a * sizeof(double));
        double *l_tr  = (double *) malloc(nt_a * sizeof(double));
        double *l_ti  = (double *) malloc(nt_a * sizeof(double));
        double *l_acc = (double *) calloc(np_a, sizeof(double));
        if (l_re && l_im && l_tr && l_ti && l_acc)
        {
#    pragma omp for schedule(static)
            for (uint32_t pp = 0u; pp < npairs; pp++)
            {
                uint32_t     k0  = 2u * pp;
                uint32_t     k1  = k0 + 1u;
                const float *fr0 = dX + (size_t) keep_t[k0] * nimg;
                const float *fr1 = (k1 < T_use) ? (dX + (size_t) keep_t[k1] * nimg) : NULL;
                memset(l_re, 0, np_a * sizeof(double));
                memset(l_im, 0, np_a * sizeof(double));
                if (bin2)
                {
                    for (uint32_t y = 0u; y < nyb; y++)
                    {
                        const float *r0 = fr0 + (size_t) (2u * y) * nx;
                        const float *r1 = fr0 + (size_t) (2u * y + 1u) * nx;
                        double      *dr = l_re + (size_t) y * px;
                        for (uint32_t x = 0u; x < nxb; x++)
                        {
                            dr[x] = 0.25 * ((double) r0[2u * x] + (double) r0[2u * x + 1u] +
                                            (double) r1[2u * x] + (double) r1[2u * x + 1u]);
                        }
                        if (fr1 != NULL)
                        {
                            const float *q0 = fr1 + (size_t) (2u * y) * nx;
                            const float *q1 = fr1 + (size_t) (2u * y + 1u) * nx;
                            double      *di = l_im + (size_t) y * px;
                            for (uint32_t x = 0u; x < nxb; x++)
                            {
                                di[x] = 0.25 * ((double) q0[2u * x] + (double) q0[2u * x + 1u] +
                                                (double) q1[2u * x] + (double) q1[2u * x + 1u]);
                            }
                        }
                    }
                }
                else
                {
                    for (uint32_t y = 0u; y < nyb; y++)
                    {
                        const float *r0 = fr0 + (size_t) y * nx;
                        double      *dr = l_re + (size_t) y * px;
                        for (uint32_t x = 0u; x < nxb; x++)
                        {
                            dr[x] = (double) r0[x];
                        }
                        if (fr1 != NULL)
                        {
                            const float *q0 = fr1 + (size_t) y * nx;
                            double      *di = l_im + (size_t) y * px;
                            for (uint32_t x = 0u; x < nxb; x++)
                            {
                                di[x] = (double) q0[x];
                            }
                        }
                    }
                }
                pyrwfs_fft2d(l_re, l_im, px, py, 0, l_tr, l_ti);
                for (size_t i = 0; i < np_a; i++)
                {
                    l_acc[i] += l_re[i] * l_re[i] + l_im[i] * l_im[i];
                }
            }
#    pragma omp critical
            {
                for (size_t i = 0; i < np_a; i++)
                {
                    P_acc[i] += l_acc[i];
                }
            }
        }
        free(l_re); free(l_im); free(l_tr); free(l_ti); free(l_acc);
    }
#else
    for (uint32_t pp = 0u; pp < npairs; pp++)
    {
        uint32_t     k0  = 2u * pp;
        uint32_t     k1  = k0 + 1u;
        const float *fr0 = dX + (size_t) keep_t[k0] * nimg;
        const float *fr1 = (k1 < T_use) ? (dX + (size_t) keep_t[k1] * nimg) : NULL;
        memset(re_a, 0, np_a * sizeof(double));
        memset(im_a, 0, np_a * sizeof(double));
        if (bin2)
        {
            for (uint32_t y = 0u; y < nyb; y++)
            {
                const float *r0 = fr0 + (size_t) (2u * y) * nx;
                const float *r1 = fr0 + (size_t) (2u * y + 1u) * nx;
                double      *dr = re_a + (size_t) y * px;
                for (uint32_t x = 0u; x < nxb; x++)
                {
                    dr[x] = 0.25 * ((double) r0[2u * x] + (double) r0[2u * x + 1u] +
                                    (double) r1[2u * x] + (double) r1[2u * x + 1u]);
                }
                if (fr1 != NULL)
                {
                    const float *q0 = fr1 + (size_t) (2u * y) * nx;
                    const float *q1 = fr1 + (size_t) (2u * y + 1u) * nx;
                    double      *di = im_a + (size_t) y * px;
                    for (uint32_t x = 0u; x < nxb; x++)
                    {
                        di[x] = 0.25 * ((double) q0[2u * x] + (double) q0[2u * x + 1u] +
                                        (double) q1[2u * x] + (double) q1[2u * x + 1u]);
                    }
                }
            }
        }
        else
        {
            for (uint32_t y = 0u; y < nyb; y++)
            {
                const float *r0 = fr0 + (size_t) y * nx;
                double      *dr = re_a + (size_t) y * px;
                for (uint32_t x = 0u; x < nxb; x++)
                {
                    dr[x] = (double) r0[x];
                }
                if (fr1 != NULL)
                {
                    const float *q0 = fr1 + (size_t) y * nx;
                    double      *di = im_a + (size_t) y * px;
                    for (uint32_t x = 0u; x < nxb; x++)
                    {
                        di[x] = (double) q0[x];
                    }
                }
            }
        }
        pyrwfs_fft2d(re_a, im_a, px, py, 0, tr_a, ti_a);
        for (size_t i = 0; i < np_a; i++)
        {
            P_acc[i] += re_a[i] * re_a[i] + im_a[i] * im_a[i];
        }
    }
#endif
    double inv_tu = 1.0 / (double) T_use;
    for (size_t i = 0; i < np_a; i++)
    {
        re_a[i] = P_acc[i] * inv_tu;
        im_a[i] = 0.0;
    }
    pyrwfs_fft2d(re_a, im_a, px, py, 1, tr_a, ti_a);
    free(P_acc);
    free(im_a);
    free(tr_a);
    free(ti_a);

    /* Find top-16 strict 2D local negative minima in Q1 (d03) and Q4 (d12) */
    int dx_lo = (int) ceil(0.20 * (double) nxb);
    int dx_hi = (int) floor(0.85 * (double) nxb);
    int dy_lo = (int) ceil(0.20 * (double) nyb);
    int dy_hi = (int) floor(0.85 * (double) nyb);
    if (dx_hi > (int) (px / 2u) - 2)
    {
        dx_hi = (int) (px / 2u) - 2;
    }
    if (dy_hi > (int) (py / 2u) - 2)
    {
        dy_hi = (int) (py / 2u) - 2;
    }

    int    c03_x[16], c03_y[16], c12_x[16], c12_y[16];
    double c03_v[16], c12_v[16];
    int    n03 = 0, n12 = 0;
    double a_s2 = 0.0;
    size_t a_cnt = 0u;

    for (int quad = 0; quad < 2; quad++)
    {
        int y_start = (quad == 0) ? dy_lo : -dy_hi;
        int y_end   = (quad == 0) ? dy_hi : -dy_lo;
        for (int dy = y_start; dy <= y_end; dy++)
        {
            uint32_t ky0 = (dy >= 0) ? (uint32_t) dy : (py + (uint32_t) dy);
            uint32_t kym = (dy - 1 >= 0) ? (uint32_t) (dy - 1) : (py + (uint32_t) (dy - 1));
            uint32_t kyp = (dy + 1 >= 0) ? (uint32_t) (dy + 1) : (py + (uint32_t) (dy + 1));
            for (int dx = dx_lo; dx <= dx_hi; dx++)
            {
                uint32_t kx0 = (uint32_t) dx;
                uint32_t kxm = (uint32_t) (dx - 1);
                uint32_t kxp = (uint32_t) (dx + 1);
                double   v   = re_a[(size_t) ky0 * px + kx0];
                a_s2 += v * v;
                a_cnt++;
                if (v >= 0.0)
                {
                    continue;
                }
                if (v < re_a[(size_t) kym * px + kx0] && v < re_a[(size_t) kyp * px + kx0] &&
                    v < re_a[(size_t) ky0 * px + kxm] && v < re_a[(size_t) ky0 * px + kxp] &&
                    v <= re_a[(size_t) kym * px + kxm] && v <= re_a[(size_t) kym * px + kxp] &&
                    v <= re_a[(size_t) kyp * px + kxm] && v <= re_a[(size_t) kyp * px + kxp])
                {
                    int    *cx_arr = (quad == 0) ? c03_x : c12_x;
                    int    *cy_arr = (quad == 0) ? c03_y : c12_y;
                    double *cv_arr = (quad == 0) ? c03_v : c12_v;
                    int    *nc     = (quad == 0) ? &n03 : &n12;
                    /* Insert sorted ascending (most negative first) */
                    int pos = *nc;
                    if (pos < 16)
                    {
                        (*nc)++;
                    }
                    else if (v >= cv_arr[15])
                    {
                        continue;
                    }
                    else
                    {
                        pos = 15;
                    }
                    while (pos > 0 && v < cv_arr[pos - 1])
                    {
                        cx_arr[pos] = cx_arr[pos - 1];
                        cy_arr[pos] = cy_arr[pos - 1];
                        cv_arr[pos] = cv_arr[pos - 1];
                        pos--;
                    }
                    cx_arr[pos] = dx;
                    cy_arr[pos] = dy;
                    cv_arr[pos] = v;
                }
            }
        }
    }

    /* Select geometrically consistent diagonal pair (d03, d12):
     * Pass 0 enforces tight symmetry (4% length, 6% axis) among strong peaks so periodic
     * DM ripple/waffle sidelobes cannot override the true pupil separation; passes 1-2
     * relax progressively for strongly anamorphic or sheared optical layouts. */
    int    b_i = -1, b_j = -1;
    double best_sc = 1e30;
    for (int pass = 0; pass < 3 && b_i < 0; pass++)
    {
        double r_tol    = (pass == 0) ? 0.04 : ((pass == 1) ? 0.16 : 0.30);
        double axis_tol = (pass == 0) ? 0.06 : 0.35;
        double str_min  = (pass == 0) ? 0.45 : 0.0;
        for (int i = 0; i < n03; i++)
        {
            if (str_min > 0.0 && (-c03_v[i]) < str_min * (-c03_v[0]))
            {
                continue;
            }
            double r1 = hypot((double) c03_x[i], (double) c03_y[i]);
            for (int j = 0; j < n12; j++)
            {
                if (str_min > 0.0 && (-c12_v[j]) < str_min * (-c12_v[0]))
                {
                    continue;
                }
                double r2   = hypot((double) c12_x[j], (double) c12_y[j]);
                double rmax = (r1 > r2) ? r1 : r2;
                if (fabs(r1 - r2) > r_tol * rmax)
                {
                    continue;
                }
                if (fabs((double) (c03_x[i] - c12_x[j])) >
                    axis_tol * (double) (c03_x[i] + c12_x[j]))
                {
                    continue;
                }
                if (fabs((double) (c03_y[i] + c12_y[j])) >
                    axis_tol * (double) (c03_y[i] - c12_y[j]))
                {
                    continue;
                }
                double sc = c03_v[i] + c12_v[j];
                if (sc < best_sc)
                {
                    best_sc = sc;
                    b_i     = i;
                    b_j     = j;
                }
            }
        }
    }

    double a_rms = (a_cnt > 0u) ? sqrt(a_s2 / (double) a_cnt) : 0.0;
    if (b_i < 0 || b_j < 0 || (-c03_v[b_i]) < 2.2 * a_rms || (-c12_v[b_j]) < 2.2 * a_rms)
    {
        /* No significant cross-pupil anti-correlation (e.g. pure noise): fallback */
        free(re_a);
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net); free(dX); free(keep_t);
        return rc_fb;
    }

    /* Parabolic sub-pixel refinement of global diagonal peaks */
    double d_diag[2][2];
    int    pk_x[2] = { c03_x[b_i], c12_x[b_j] };
    int    pk_y[2] = { c03_y[b_i], c12_y[b_j] };
    for (int k = 0; k < 2; k++)
    {
        int      ix  = pk_x[k];
        int      iy  = pk_y[k];
        uint32_t ky0 = (iy >= 0) ? (uint32_t) iy : (py + (uint32_t) iy);
        uint32_t kym = (iy - 1 >= 0) ? (uint32_t) (iy - 1) : (py + (uint32_t) (iy - 1));
        uint32_t kyp = (iy + 1 >= 0) ? (uint32_t) (iy + 1) : (py + (uint32_t) (iy + 1));
        uint32_t kx0 = (uint32_t) ix;
        uint32_t kxm = (uint32_t) (ix - 1);
        uint32_t kxp = (uint32_t) (ix + 1);
        double   c0  = re_a[(size_t) ky0 * px + kx0];
        double   sx  = 0.0, sy = 0.0;
        double   denx = re_a[(size_t) ky0 * px + kxm] - 2.0 * c0 + re_a[(size_t) ky0 * px + kxp];
        double   deny = re_a[(size_t) kym * px + kx0] - 2.0 * c0 + re_a[(size_t) kyp * px + kx0];
        if (denx > 1e-12)
        {
            sx = 0.5 * (re_a[(size_t) ky0 * px + kxm] - re_a[(size_t) ky0 * px + kxp]) / denx;
            if (sx > 0.5) sx = 0.5;
            else if (sx < -0.5) sx = -0.5;
        }
        if (deny > 1e-12)
        {
            sy = 0.5 * (re_a[(size_t) kym * px + kx0] - re_a[(size_t) kyp * px + kx0]) / deny;
            if (sy > 0.5) sy = 0.5;
            else if (sy < -0.5) sy = -0.5;
        }
        d_diag[k][0] = ((double) ix + sx) * scale_b;
        d_diag[k][1] = ((double) iy + sy) * scale_b;
    }
    free(re_a);

    double p0[6] = {
        0.25 * (d_diag[0][0] + d_diag[1][0]),
        0.25 * (d_diag[0][1] + d_diag[1][1]),
        0.25 * (d_diag[0][0] - d_diag[1][0]),
        0.25 * (d_diag[0][1] - d_diag[1][1]),
        0.0,
        0.0
    };
    double del0_x[4], del0_y[4];
    pyrwfs_p_to_delta(p0, del0_x, del0_y);

    /* --- Step 2: Coarse global center (cx0, cy0) from aligned cancellation map C(x, y) --- */
    double min_dx = del0_x[0], max_dx = del0_x[0];
    double min_dy = del0_y[0], max_dy = del0_y[0];
    for (int q = 1; q < 4; q++)
    {
        if (del0_x[q] < min_dx) min_dx = del0_x[q];
        if (del0_x[q] > max_dx) max_dx = del0_x[q];
        if (del0_y[q] < min_dy) min_dy = del0_y[q];
        if (del0_y[q] > max_dy) max_dy = del0_y[q];
    }
    int xmin = (int) ceil(-min_dx);
    int xmax = (int) floor((double) (nx - 1u) - max_dx);
    int ymin = (int) ceil(-min_dy);
    int ymax = (int) floor((double) (ny - 1u) - max_dy);
    if (xmin < 1) xmin = 1;
    if (ymin < 1) ymin = 1;
    if (xmax > (int) nx - 2) xmax = (int) nx - 2;
    if (ymax > (int) ny - 2) ymax = (int) ny - 2;

    double cx0 = 0.5 * (double) (nx - 1u);
    double cy0 = 0.5 * (double) (ny - 1u);
    if (xmax > xmin + 4 && ymax > ymin + 4)
    {
        int     bw_w = xmax - xmin + 1;
        int     bw_h = ymax - ymin + 1;
        size_t  nb   = (size_t) bw_w * (size_t) bw_h;
        double *cmap = (double *) calloc(nb, sizeof(double));
        if (cmap != NULL)
        {
            int idx_dx[4], idx_dy[4];
            for (int q = 0; q < 4; q++)
            {
                idx_dx[q] = (int) lround(del0_x[q]);
                idx_dy[q] = (int) lround(del0_y[q]);
            }
            for (int y = ymin; y <= ymax; y++)
            {
                for (int x = xmin; x <= xmax; x++)
                {
                    int    qx[4], qy[4];
                    int    ok_all = 1;
                    for (int q = 0; q < 4; q++)
                    {
                        qx[q] = x + idx_dx[q];
                        qy[q] = y + idx_dy[q];
                        if (qx[q] < 0 || qx[q] >= (int) nx || qy[q] < 0 || qy[q] >= (int) ny)
                        {
                            ok_all = 0;
                            break;
                        }
                    }
                    if (!ok_all)
                    {
                        continue;
                    }
                    size_t off0 = (size_t) qy[0] * nx + (size_t) qx[0];
                    size_t off1 = (size_t) qy[1] * nx + (size_t) qx[1];
                    size_t off2 = (size_t) qy[2] * nx + (size_t) qx[2];
                    size_t off3 = (size_t) qy[3] * nx + (size_t) qx[3];
                    double vind = 0.0;
                    double vsum = 0.0;
                    for (uint32_t kk = 0u; kk < T_use; kk++)
                    {
                        const float *fr = dX + (size_t) keep_t[kk] * nimg;
                        double       v0 = (double) fr[off0];
                        double       v1 = (double) fr[off1];
                        double       v2 = (double) fr[off2];
                        double       v3 = (double) fr[off3];
                        double       s  = v0 + v1 + v2 + v3;
                        vind += v0 * v0 + v1 * v1 + v2 * v2 + v3 * v3;
                        vsum += s * s;
                    }
                    double cval = (vind - vsum) * inv_tu;
                    cmap[(size_t) (y - ymin) * (size_t) bw_w + (size_t) (x - xmin)] =
                        (cval > 0.0) ? cval : 0.0;
                }
            }
            double sep_a0   = hypot(2.0 * p0[0], 2.0 * p0[1]);
            double sep_b0   = hypot(2.0 * p0[2], 2.0 * p0[3]);
            double sep_min0 = (sep_a0 < sep_b0) ? sep_a0 : sep_b0;

            /* Locate the true 4-pupil overlap peak via 2D box-smoothed integral image of cmap
             * (4-pupil overlap has ~4-6x higher integrated cancellation than any 2-pupil ghost),
             * then take the local centroid strictly within radius 0.48*sep_min0 of that peak. */
            double *ii_buf = (double *) calloc((size_t) (bw_h + 1) * (size_t) (bw_w + 1), sizeof(double));
            int     pk_x   = xmin + bw_w / 2;
            int     pk_y   = ymin + bw_h / 2;
            if (ii_buf != NULL)
            {
                int iw = bw_w + 1;
                for (int j = 0; j < bw_h; j++)
                {
                    double rsum = 0.0;
                    for (int i = 0; i < bw_w; i++)
                    {
                        rsum += cmap[(size_t) j * (size_t) bw_w + (size_t) i];
                        ii_buf[(size_t) (j + 1) * (size_t) iw + (size_t) (i + 1)] =
                            ii_buf[(size_t) j * (size_t) iw + (size_t) (i + 1)] + rsum;
                    }
                }
                int br = (int) lround(0.48 * sep_min0);
                if (br < 4)
                {
                    br = 4;
                }
                double best_box = -1.0;
                for (int j = 0; j < bw_h; j++)
                {
                    int j0 = (j - br > 0) ? (j - br) : 0;
                    int j1 = (j + br + 1 < bw_h) ? (j + br + 1) : bw_h;
                    for (int i = 0; i < bw_w; i++)
                    {
                        int    i0 = (i - br > 0) ? (i - br) : 0;
                        int    i1 = (i + br + 1 < bw_w) ? (i + br + 1) : bw_w;
                        double bs = ii_buf[(size_t) j1 * (size_t) iw + (size_t) i1] -
                                    ii_buf[(size_t) j0 * (size_t) iw + (size_t) i1] -
                                    ii_buf[(size_t) j1 * (size_t) iw + (size_t) i0] +
                                    ii_buf[(size_t) j0 * (size_t) iw + (size_t) i0];
                        if (bs > best_box)
                        {
                            best_box = bs;
                            pk_x     = xmin + i;
                            pk_y     = ymin + j;
                        }
                    }
                }
                free(ii_buf);
            }

            double loc_r  = 0.48 * sep_min0;
            double loc_r2 = loc_r * loc_r;
            double *loc_v = (double *) malloc(nb * sizeof(double));
            size_t  nloc  = 0u;
            if (loc_v != NULL)
            {
                for (int y = ymin; y <= ymax; y++)
                {
                    double dy = (double) (y - pk_y);
                    for (int x = xmin; x <= xmax; x++)
                    {
                        double dx = (double) (x - pk_x);
                        if (dx * dx + dy * dy <= loc_r2)
                        {
                            loc_v[nloc++] = cmap[(size_t) (y - ymin) * (size_t) bw_w + (size_t) (x - xmin)];
                        }
                    }
                }
            }
            double c95 = (nloc > 0u) ? pyrwfs_percentile_double(loc_v, nloc, 0.95) : 0.0;
            free(loc_v);
            if (c95 > 0.0)
            {
                double sx_c = 0.0, sy_c = 0.0;
                size_t cnt_c = 0u;
                double thr_c = 0.15 * c95;
                for (int y = ymin; y <= ymax; y++)
                {
                    double dy = (double) (y - pk_y);
                    for (int x = xmin; x <= xmax; x++)
                    {
                        double dx = (double) (x - pk_x);
                        if (dx * dx + dy * dy <= loc_r2 &&
                            cmap[(size_t) (y - ymin) * (size_t) bw_w + (size_t) (x - xmin)] > thr_c)
                        {
                            sx_c += (double) x;
                            sy_c += (double) y;
                            cnt_c++;
                        }
                    }
                }
                if (cnt_c >= 16u)
                {
                    cx0 = sx_c / (double) cnt_c;
                    cy0 = sy_c / (double) cnt_c;
                }
            }
            free(cmap);
        }
    }

    /* --- Step 3: Exact Voronoi-masked integer-grid 6-pair cross-covariance tables K_pq --- */
    int c0x[4], c0y[4];
    for (int q = 0; q < 4; q++)
    {
        c0x[q] = (int) lround(cx0 + del0_x[q]);
        c0y[q] = (int) lround(cy0 + del0_y[q]);
    }
    double sep_a   = hypot(2.0 * p0[0], 2.0 * p0[1]);
    double sep_b   = hypot(2.0 * p0[2], 2.0 * p0[3]);
    double sep_min = (sep_a < sep_b) ? sep_a : sep_b;
    if (sep_min < 8.0)
    {
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net); free(dX); free(keep_t);
        return rc_fb;
    }

    int      hw    = (int) ceil(0.55 * sep_min) + 2;
    int      W     = 2 * hw + 1;
    int      M     = 12;
    uint32_t P_fft = pyrwfs_next_pow2((uint32_t) (W + M + 2));
    if (M > (int) (P_fft / 4u))
    {
        M = (int) (P_fft / 4u);
    }
    size_t npf   = (size_t) P_fft * (size_t) P_fft;
    int    M2    = 2 * M;
    int    tab_w = 2 * M2 + 1;
    size_t ntab  = (size_t) tab_w * (size_t) tab_w;

    uint8_t *vor_id = (uint8_t *) malloc(nimg);
    double  *spq_re = (double *) calloc(6u * npf, sizeof(double));
    double  *spq_im = (double *) calloc(6u * npf, sizeof(double));
    double  *tr_f   = (double *) malloc((size_t) P_fft * sizeof(double));
    double  *ti_f   = (double *) malloc((size_t) P_fft * sizeof(double));

    pyrwfs_xcov_obj_t xobj;
    memset(&xobj, 0, sizeof(xobj));
    xobj.M     = M2;
    xobj.tab_w = tab_w;
    for (int q = 0; q < 4; q++)
    {
        xobj.c0x[q] = c0x[q];
        xobj.c0y[q] = c0y[q];
    }
    int tab_ok = 1;
    for (int k = 0; k < 6; k++)
    {
        xobj.K_tabs[k] = (double *) malloc(ntab * sizeof(double));
        if (xobj.K_tabs[k] == NULL)
        {
            tab_ok = 0;
        }
    }

    if (!vor_id || !spq_re || !spq_im || !tr_f || !ti_f || !tab_ok)
    {
        free(vor_id); free(spq_re); free(spq_im); free(tr_f); free(ti_f);
        for (int k = 0; k < 6; k++) free(xobj.K_tabs[k]);
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net); free(dX); free(keep_t);
        return rc_fb;
    }

    /* Precompute strictly disjoint Voronoi cells */
    for (uint32_t y = 0u; y < ny; y++)
    {
        for (uint32_t x = 0u; x < nx; x++)
        {
            double  best_d2 = 1e30;
            uint8_t best_q  = 0u;
            for (int q = 0; q < 4; q++)
            {
                double dx = (double) x - (double) c0x[q];
                double dy = (double) y - (double) c0y[q];
                double d2 = dx * dx + dy * dy;
                if (d2 < best_d2)
                {
                    best_d2 = d2;
                    best_q  = (uint8_t) q;
                }
            }
            vor_id[(size_t) y * nx + x] = best_q;
        }
    }

    static const int pairs[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
    const double     rmask2      = (0.56 * sep_min) * (0.56 * sep_min);

#ifdef _OPENMP
#    pragma omp parallel
    {
        double *l_fqr = (double *) malloc(4u * npf * sizeof(double));
        double *l_fqi = (double *) malloc(4u * npf * sizeof(double));
        double *l_sr  = (double *) calloc(6u * npf, sizeof(double));
        double *l_si  = (double *) calloc(6u * npf, sizeof(double));
        double *l_tr  = (double *) malloc((size_t) P_fft * sizeof(double));
        double *l_ti  = (double *) malloc((size_t) P_fft * sizeof(double));
        if (l_fqr && l_fqi && l_sr && l_si && l_tr && l_ti)
        {
#    pragma omp for schedule(static)
            for (uint32_t kk = 0u; kk < T_use; kk++)
            {
                const float *fr = dX + (size_t) keep_t[kk] * nimg;
                for (int q = 0; q < 4; q++)
                {
                    double *qr = l_fqr + (size_t) q * npf;
                    double *qi = l_fqi + (size_t) q * npf;
                    memset(qr, 0, npf * sizeof(double));
                    memset(qi, 0, npf * sizeof(double));
                    int ox = c0x[q] - hw;
                    int oy = c0y[q] - hw;
                    int x0 = (ox > 0) ? ox : 0;
                    int x1 = (ox + W < (int) nx) ? (ox + W) : (int) nx;
                    int y0 = (oy > 0) ? oy : 0;
                    int y1 = (oy + W < (int) ny) ? (oy + W) : (int) ny;
                    for (int y = y0; y < y1; y++)
                    {
                        double         ry  = (double) (y - c0y[q]);
                        const float   *src = fr + (size_t) y * nx;
                        const uint8_t *vid = vor_id + (size_t) y * nx;
                        double        *dst = qr + (size_t) (y - oy) * P_fft;
                        for (int x = x0; x < x1; x++)
                        {
                            double rx = (double) (x - c0x[q]);
                            if (vid[x] == (uint8_t) q && rx * rx + ry * ry <= rmask2)
                            {
                                dst[x - ox] = (double) src[x];
                            }
                        }
                    }
                    pyrwfs_fft2d(qr, qi, P_fft, P_fft, 0, l_tr, l_ti);
                }
                for (int k = 0; k < 6; k++)
                {
                    const double *pr = l_fqr + (size_t) pairs[k][0] * npf;
                    const double *pi = l_fqi + (size_t) pairs[k][0] * npf;
                    const double *qr = l_fqr + (size_t) pairs[k][1] * npf;
                    const double *qi = l_fqi + (size_t) pairs[k][1] * npf;
                    double       *sr = l_sr + (size_t) k * npf;
                    double       *si = l_si + (size_t) k * npf;
                    for (size_t i = 0; i < npf; i++)
                    {
                        sr[i] += pr[i] * qr[i] + pi[i] * qi[i];
                        si[i] += pr[i] * qi[i] - pi[i] * qr[i];
                    }
                }
            }
#    pragma omp critical
            {
                for (size_t i = 0; i < 6u * npf; i++)
                {
                    spq_re[i] += l_sr[i];
                    spq_im[i] += l_si[i];
                }
            }
        }
        free(l_fqr); free(l_fqi); free(l_sr); free(l_si); free(l_tr); free(l_ti);
    }
#else
    double *fq_re = (double *) malloc(4u * npf * sizeof(double));
    double *fq_im = (double *) malloc(4u * npf * sizeof(double));
    if (fq_re && fq_im)
    {
        for (uint32_t kk = 0u; kk < T_use; kk++)
        {
            const float *fr = dX + (size_t) keep_t[kk] * nimg;
            for (int q = 0; q < 4; q++)
            {
                double *qr = fq_re + (size_t) q * npf;
                double *qi = fq_im + (size_t) q * npf;
                memset(qr, 0, npf * sizeof(double));
                memset(qi, 0, npf * sizeof(double));
                int ox = c0x[q] - hw;
                int oy = c0y[q] - hw;
                int x0 = (ox > 0) ? ox : 0;
                int x1 = (ox + W < (int) nx) ? (ox + W) : (int) nx;
                int y0 = (oy > 0) ? oy : 0;
                int y1 = (oy + W < (int) ny) ? (oy + W) : (int) ny;
                for (int y = y0; y < y1; y++)
                {
                    double         ry  = (double) (y - c0y[q]);
                    const float   *src = fr + (size_t) y * nx;
                    const uint8_t *vid = vor_id + (size_t) y * nx;
                    double        *dst = qr + (size_t) (y - oy) * P_fft;
                    for (int x = x0; x < x1; x++)
                    {
                        double rx = (double) (x - c0x[q]);
                        if (vid[x] == (uint8_t) q && rx * rx + ry * ry <= rmask2)
                        {
                            dst[x - ox] = (double) src[x];
                        }
                    }
                }
                pyrwfs_fft2d(qr, qi, P_fft, P_fft, 0, tr_f, ti_f);
            }
            for (int k = 0; k < 6; k++)
            {
                const double *pr = fq_re + (size_t) pairs[k][0] * npf;
                const double *pi = fq_im + (size_t) pairs[k][0] * npf;
                const double *qr = fq_re + (size_t) pairs[k][1] * npf;
                const double *qi = fq_im + (size_t) pairs[k][1] * npf;
                double       *sr = spq_re + (size_t) k * npf;
                double       *si = spq_im + (size_t) k * npf;
                for (size_t i = 0; i < npf; i++)
                {
                    sr[i] += pr[i] * qr[i] + pi[i] * qi[i];
                    si[i] += pr[i] * qi[i] - pi[i] * qr[i];
                }
            }
        }
    }
    free(fq_re);
    free(fq_im);
#endif

    free(vor_id);
    free(dX);
    free(keep_t);

    /* 2x Fourier zero-padded IFFT of each of the 6 cross-spectra -> half-pixel (4*M+1)^2 tables
     * (eliminates Catmull-Rom half-integer locking bias on 45-deg narrow diagonal peaks) */
    uint32_t P2   = 2u * P_fft;
    uint32_t half = P_fft / 2u;
    size_t   np2  = (size_t) P2 * (size_t) P2;
    double  *z_re = (double *) malloc(np2 * sizeof(double));
    double  *z_im = (double *) malloc(np2 * sizeof(double));
    double  *tr2  = (double *) malloc((size_t) P2 * sizeof(double));
    double  *ti2  = (double *) malloc((size_t) P2 * sizeof(double));
    if (!z_re || !z_im || !tr2 || !ti2)
    {
        free(z_re); free(z_im); free(tr2); free(ti2);
        free(spq_re); free(spq_im); free(tr_f); free(ti_f);
        for (int k = 0; k < 6; k++) free(xobj.K_tabs[k]);
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net);
        return rc_fb;
    }
    const double sc_pad = 4.0 * inv_tu;
    for (int k = 0; k < 6; k++)
    {
        const double *sr = spq_re + (size_t) k * npf;
        const double *si = spq_im + (size_t) k * npf;
        memset(z_re, 0, np2 * sizeof(double));
        memset(z_im, 0, np2 * sizeof(double));
        for (uint32_t ky = 0u; ky < P_fft; ky++)
        {
            uint32_t ty0 = (ky < half) ? ky : ((ky == half) ? half : (P2 - P_fft + ky));
            int      sy2 = (ky == half);
            double   wy  = sy2 ? 0.5 : 1.0;
            for (uint32_t kx = 0u; kx < P_fft; kx++)
            {
                uint32_t tx0 = (kx < half) ? kx : ((kx == half) ? half : (P2 - P_fft + kx));
                int      sx2 = (kx == half);
                double   w   = wy * (sx2 ? 0.5 : 1.0) * sc_pad;
                double   vr  = sr[(size_t) ky * P_fft + kx] * w;
                double   vi  = si[(size_t) ky * P_fft + kx] * w;
                z_re[(size_t) ty0 * P2 + tx0] += vr;
                z_im[(size_t) ty0 * P2 + tx0] += vi;
                if (sx2)
                {
                    uint32_t tx1 = P2 - half;
                    z_re[(size_t) ty0 * P2 + tx1] += vr;
                    z_im[(size_t) ty0 * P2 + tx1] += vi;
                }
                if (sy2)
                {
                    uint32_t ty1 = P2 - half;
                    z_re[(size_t) ty1 * P2 + tx0] += vr;
                    z_im[(size_t) ty1 * P2 + tx0] += vi;
                    if (sx2)
                    {
                        uint32_t tx1 = P2 - half;
                        z_re[(size_t) ty1 * P2 + tx1] += vr;
                        z_im[(size_t) ty1 * P2 + tx1] += vi;
                    }
                }
            }
        }
        pyrwfs_fft2d(z_re, z_im, P2, P2, 1, tr2, ti2);
        double *tab = xobj.K_tabs[k];
        for (int dv = -M2; dv <= M2; dv++)
        {
            uint32_t ky = (dv >= 0) ? (uint32_t) dv : (P2 + (uint32_t) dv);
            for (int du = -M2; du <= M2; du++)
            {
                uint32_t kx = (du >= 0) ? (uint32_t) du : (P2 + (uint32_t) du);
                tab[(size_t) (dv + M2) * (size_t) tab_w + (size_t) (du + M2)] =
                    z_re[(size_t) ky * P2 + kx];
            }
        }
    }
    free(z_re);
    free(z_im);
    free(tr2);
    free(ti2);
    free(spq_re);
    free(spq_im);
    free(tr_f);
    free(ti_f);

    /* Step 3a: Globally-seeded decoupled opposite-diagonal refinement (K_03 and K_12).
     * Both W_x and W_y anti-correlate identically in opposite pairs (0,3) and (1,2),
     * uniquely fixing p[0..3] without adjacent-pair anisotropy or Mode-C scintillation
     * double-well bias. */
    double u03 = 0.0, v03 = 0.0, u12 = 0.0, v12 = 0.0;
    pyrwfs_min2d_bicubic(xobj.K_tabs[2], M2, &u03, &v03);
    pyrwfs_min2d_bicubic(xobj.K_tabs[3], M2, &u12, &v12);

    double d03_x = (double) (c0x[3] - c0x[0]) + u03;
    double d03_y = (double) (c0y[3] - c0y[0]) + v03;
    double d12_x = (double) (c0x[2] - c0x[1]) + u12;
    double d12_y = (double) (c0y[2] - c0y[1]) + v12;

    double p[6] = {
        0.25 * (d03_x + d12_x),
        0.25 * (d03_y + d12_y),
        0.25 * (d03_x - d12_x),
        0.25 * (d03_y - d12_y),
        0.0,
        0.0
    };

    /* Step 3b: Mode C (p[4], p[5]) from centrosymmetric diagonal-pair sums S03 <-> S12
     * evaluated at the sub-pixel refined diagonal vectors p[0..3]. */
    double del_d_x[4], del_d_y[4];
    pyrwfs_p_to_delta(p, del_d_x, del_d_y);
    double sep_ad   = hypot(2.0 * p[0], 2.0 * p[1]);
    double sep_bd   = hypot(2.0 * p[2], 2.0 * p[3]);
    double sep_mind = (sep_ad < sep_bd) ? sep_ad : sep_bd;
    double dx_e = 0.0, dy_e = 0.0;
    int    rc_mc = pyrwfs_align_diag_pair_sums(mean_net, nx, ny, cx0, cy0, del_d_x, del_d_y,
                                               (sep_mind >= 8.0) ? sep_mind : sep_min, &dx_e,
                                               &dy_e);
    xobj.dx_e  = dx_e;
    xobj.dy_e  = dy_e;
    xobj.lam_c = 0.0;
    if (rc_mc == 0)
    {
        p[4] = dx_e;
        p[5] = dy_e;
    }
    else
    {
        /* Fallback for edge-less images: only refine Mode C on f_S where strictly convex */
        double g0[6];
        double f0 = pyrwfs_xcov_eval(&xobj, p, g0);
        for (int k = 4; k < 6; k++)
        {
            double pt[6];
            memcpy(pt, p, sizeof(pt));
            pt[k]     = 0.5;
            double fp = pyrwfs_xcov_eval(&xobj, pt, NULL);
            pt[k]     = -0.5;
            double fm = pyrwfs_xcov_eval(&xobj, pt, NULL);
            double ck = (fp + fm - 2.0 * f0) / 0.25;
            if (ck > 1e-12)
            {
                double step = -g0[k] / ck;
                if (step > 1.5) step = 1.5;
                else if (step < -1.5) step = -1.5;
                pt[k] = step;
                if (pyrwfs_xcov_eval(&xobj, pt, NULL) < f0)
                {
                    p[k] = step;
                }
            }
        }
    }

    /* Step 3c: Remove isotropic scintillation radial bias via quadrupole f_Q */
    pyrwfs_debias_radial_quadrupole(&xobj, p);

    for (int k = 0; k < 6; k++)
    {
        free(xobj.K_tabs[k]);
    }

    /* Safety clamp against wild step outside table */
    double max_dev = 0.5 * (double) (M - 2);
    if (max_dev < 1.0)
    {
        max_dev = 1.0;
    }
    for (int k = 0; k < 6; k++)
    {
        double base = (k < 4) ? p0[k] : ((k == 4) ? dx_e : dy_e);
        if (p[k] > base + max_dev)
        {
            p[k] = base + max_dev;
        }
        else if (p[k] < base - max_dev)
        {
            p[k] = base - max_dev;
        }
    }

    double del_x[4], del_y[4];
    pyrwfs_p_to_delta(p, del_x, del_y);
    sep_a   = hypot(2.0 * p[0], 2.0 * p[1]);
    sep_b   = hypot(2.0 * p[2], 2.0 * p[3]);
    sep_min = (sep_a < sep_b) ? sep_a : sep_b;

    /* --- Step 4: Global center (cx, cy) and radii (rout, rin) on aligned 4-pupil sum --- */
    double cx = cx0, cy = cy0, rout = 0.40 * sep_min, rin = 0.0;
    if (pyrwfs_fit_aligned_sum_circle(mean_net, nx, ny, cx0, cy0, del_x, del_y, sep_min, &cx, &cy,
                                      &rout, &rin) != 0)
    {
        int rc_fb = pyrwfs_detect_pupils((ref_im != NULL) ? ref_im : mean_im, nx, ny, 0.25f, out);
        free(mean_im); free(mean_net);
        return rc_fb;
    }

    if (ref_im != NULL)
    {
        /* Anchor global center (cx, cy) to ref_im while keeping clean turbulent radii (rout, rin) */
        double bg_r_sum = 0.0;
        size_t bg_r_cnt = 0u;
        for (uint32_t y = 0u; y < cb; y++)
        {
            for (uint32_t x = 0u; x < cb; x++)
            {
                bg_r_sum += (double) ref_im[(size_t) y * nx + x];
                bg_r_sum += (double) ref_im[(size_t) y * nx + (nx - 1u - x)];
                bg_r_sum += (double) ref_im[(size_t) (ny - 1u - y) * nx + x];
                bg_r_sum += (double) ref_im[(size_t) (ny - 1u - y) * nx + (nx - 1u - x)];
                bg_r_cnt += 4u;
            }
        }
        double bg_r = (bg_r_cnt > 0u) ? (bg_r_sum / (double) bg_r_cnt) : bg_corner;
        float *ref_net = (float *) malloc(nimg * sizeof(float));
        if (ref_net != NULL)
        {
            for (size_t i = 0; i < nimg; i++)
            {
                double v   = (double) ref_im[i] - bg_r;
                ref_net[i] = (v > 0.0) ? (float) v : 0.0f;
            }
            double cx_r = cx, cy_r = cy, rout_r = rout, rin_r = rin;
            if (pyrwfs_fit_aligned_sum_circle(ref_net, nx, ny, cx, cy, del_x, del_y, sep_min,
                                              &cx_r, &cy_r, &rout_r, &rin_r) == 0)
            {
                cx = cx_r;
                cy = cy_r;
            }
            free(ref_net);
        }
    }

    free(mean_im);
    free(mean_net);

    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        out->xc[q]   = (float) (cx + del_x[q]);
        out->yc[q]   = (float) (cy + del_y[q]);
        out->rout[q] = (float) rout;
        out->rin[q]  = (float) rin;
        out->corr[q] = 1.0f;
    }
    return 0;
}
static inline void pyrwfs_build_default_zones(uint32_t                nx,
                                              uint32_t                ny,
                                              const pyrwfs_pupgeom_t *g,
                                              float                   spider_angle_deg,
                                              float                   spider_halfwidth,
                                              uint8_t                *zones)
{
    double cx   = 0.25 * (g->xc[0] + g->xc[1] + g->xc[2] + g->xc[3]);
    double cy   = 0.25 * (g->yc[0] + g->yc[1] + g->yc[2] + g->yc[3]);
    double sx   = 0.25 * ((g->xc[2] + g->xc[3]) - (g->xc[0] + g->xc[1]));
    double sy   = 0.25 * ((g->yc[1] + g->yc[3]) - (g->yc[0] + g->yc[2]));
    double ravg = 0.25 * (g->rout[0] + g->rout[1] + g->rout[2] + g->rout[3]);

    double ang = (double) spider_angle_deg * (3.1415926535897932 / 180.0);
    double ca  = cos(ang);
    double sa  = sin(ang);
    double shw = (double) spider_halfwidth;

    /* Compact far-corner box for background monitoring (strictly outside outer halo) */
    double bg_margin = 0.05 * (double) ((nx < ny) ? nx : ny);
    if (bg_margin < 3.0)
    {
        bg_margin = 3.0;
    }
    if (bg_margin > 8.0)
    {
        bg_margin = 8.0;
    }

    for (uint32_t y = 0; y < ny; y++)
    {
        for (uint32_t x = 0; x < nx; x++)
        {
            uint8_t z = PYRWFS_ZONE_NONE;
            int     in_pup = -1;
            int     in_obs = 0;

            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                double dx = (double) x - (double) g->xc[q];
                double dy = (double) y - (double) g->yc[q];
                double r  = hypot(dx, dy);
                if (r <= (double) g->rout[q])
                {
                    if (g->rin[q] > 0.5f && r < (double) g->rin[q])
                    {
                        in_obs = 1;
                    }
                    else
                    {
                        in_pup = q;
                        z      = (uint8_t) (PYRWFS_ZONE_PUP0 + (uint32_t) q);
                        /* Eroded interior flag */
                        double erode = (g->rout[q] > 8.0f) ? 1.5 : 1.0;
                        if (r <= (double) g->rout[q] - erode &&
                            (g->rin[q] <= 0.5f || r >= (double) g->rin[q] + erode))
                        {
                            z |= PYRWFS_FLAG_INNER_PUP;
                        }
                        /* Inner rim flag (quadrant facing center (cx, cy)) */
                        double sgnx = (g->xc[q] >= (float) cx) ? 1.0 : -1.0;
                        double sgny = (g->yc[q] >= (float) cy) ? 1.0 : -1.0;
                        if (dx * sgnx < 0.0 && dy * sgny < 0.0 &&
                            r > 0.72 * (double) g->rout[q])
                        {
                            z |= PYRWFS_FLAG_INNER_RIM;
                        }
                        /* Optional spider-flanking bands (two orthogonal arms at spider_angle) */
                        if (shw > 0.0)
                        {
                            double xr = fabs(dx * ca + dy * sa);
                            double yr = fabs(-dx * sa + dy * ca);
                            if (xr <= shw || yr <= shw)
                            {
                                z |= PYRWFS_FLAG_SPIDER;
                            }
                        }
                    }
                    break;
                }
            }

            if (in_pup < 0)
            {
                double ax = fabs((double) x - cx);
                double ay = fabs((double) y - cy);

                if (in_obs)
                {
                    z = PYRWFS_ZONE_OUTER;
                }
                else if (ax < sx && ay < sy)
                {
                    /* Between the 4 pupil centers: split into central diamond vs H/V bridges */
                    double gap_x = sx - 0.55 * ravg;
                    double gap_y = sy - 0.55 * ravg;
                    if (gap_x < 2.0)
                    {
                        gap_x = 2.0;
                    }
                    if (gap_y < 2.0)
                    {
                        gap_y = 2.0;
                    }
                    if (ax <= gap_x && ay <= gap_y)
                    {
                        z = PYRWFS_ZONE_DIAMOND;
                    }
                    else if (ax > ay)
                    {
                        /* Near horizontal midline (between top & bottom pupils) */
                        z = PYRWFS_ZONE_BRIDGE_H;
                    }
                    else
                    {
                        /* Near vertical midline (between left & right pupils) */
                        z = PYRWFS_ZONE_BRIDGE_V;
                    }
                }
                else
                {
                    /* Check outer diffraction halo around any pupil vs far corners */
                    double dmin = 1e9;
                    for (int q = 0; q < PYRWFS_NPUP; q++)
                    {
                        double d = hypot((double) x - (double) g->xc[q],
                                         (double) y - (double) g->yc[q]) -
                                   (double) g->rout[q];
                        if (d < dmin)
                        {
                            dmin = d;
                        }
                    }
                    int is_corner = ((double) x < bg_margin || (double) x >= (double) nx - bg_margin) &&
                                    ((double) y < bg_margin || (double) y >= (double) ny - bg_margin);
                    if (is_corner && dmin > 2.0)
                    {
                        z = PYRWFS_ZONE_BG;
                    }
                    else
                    {
                        z = PYRWFS_ZONE_OUTER;
                    }
                }
            }

            zones[(size_t) y * nx + x] = z;
        }
    }
}

/* =========================================================================
 * Edge-based Sub-Pixel 4-Pupil Registration (wavefront-gradient immune)
 * ========================================================================= */
static inline void pyrwfs_compute_soft_sobel(const float *im,
                                             uint32_t     nx,
                                             uint32_t     ny,
                                             float       *edge_out)
{
    size_t npix = (size_t) nx * ny;
    memset(edge_out, 0, npix * sizeof(float));

    double gsum = 0.0;
    size_t gcnt = 0;

    for (uint32_t y = 1u; y + 1u < ny; y++)
    {
        const float *rm = im + (size_t) (y - 1u) * nx;
        const float *r0 = im + (size_t) y * nx;
        const float *rp = im + (size_t) (y + 1u) * nx;
        float       *eo = edge_out + (size_t) y * nx;

        for (uint32_t x = 1u; x + 1u < nx; x++)
        {
            float gx = (rm[x + 1u] + 2.0f * r0[x + 1u] + rp[x + 1u]) -
                       (rm[x - 1u] + 2.0f * r0[x - 1u] + rp[x - 1u]);
            float gy = (rp[x - 1u] + 2.0f * rp[x] + rp[x + 1u]) -
                       (rm[x - 1u] + 2.0f * rm[x] + rm[x + 1u]);
            float g = hypotf(gx, gy);
            eo[x]   = g;
            if (g > 0.0f)
            {
                gsum += (double) g;
                gcnt++;
            }
        }
    }

    /* Soft-clip G / (G + scale) so intense flat-WF apex diffraction spikes do not
     * dominate over the geometric outer rim, central obscuration, and spiders. */
    float scale = (gcnt > 0u) ? (float) (gsum / (double) gcnt) : 1.0f;
    if (scale < 1e-12f)
    {
        scale = 1.0f;
    }
    for (size_t i = 0; i < npix; i++)
    {
        float g     = edge_out[i];
        edge_out[i] = g / (g + scale);
    }
}

static inline int pyrwfs_pupreg_init(pyrwfs_pupreg_state_t  *st,
                                     uint32_t                nx,
                                     uint32_t                ny,
                                     const float            *ref_im,
                                     const pyrwfs_pupgeom_t *nom,
                                     int                     edge_mode)
{
    memset(st, 0, sizeof(*st));
    st->nx        = nx;
    st->ny        = ny;
    st->edge_mode = edge_mode;
    st->nom       = *nom;

    size_t npix = (size_t) nx * ny;
    st->ref_edge = (float *) calloc(npix, sizeof(float));
    st->cur_edge = (float *) calloc(npix, sizeof(float));
    st->wgt_map  = (float *) calloc(npix, sizeof(float));
    if (st->ref_edge == NULL || st->cur_edge == NULL || st->wgt_map == NULL)
    {
        free(st->ref_edge);
        free(st->cur_edge);
        free(st->wgt_map);
        memset(st, 0, sizeof(*st));
        return -1;
    }

    pyrwfs_compute_soft_sobel(ref_im, nx, ny, st->ref_edge);

    double cx = 0.25 * (nom->xc[0] + nom->xc[1] + nom->xc[2] + nom->xc[3]);
    double cy = 0.25 * (nom->yc[0] + nom->yc[1] + nom->yc[2] + nom->yc[3]);

    for (size_t i = 0; i < npix; i++)
    {
        st->wgt_map[i] = 1.0f;
    }

    if (edge_mode == 1)
    {
        /* Downweight the Strehl-sensitive inner diffraction quadrant facing (cx, cy) */
        for (int q = 0; q < PYRWFS_NPUP; q++)
        {
            double sgnx = (nom->xc[q] >= (float) cx) ? 1.0 : -1.0;
            double sgny = (nom->yc[q] >= (float) cy) ? 1.0 : -1.0;
            int    rad  = (int) ceilf(nom->rout[q] + 4.0f);
            int    x0   = (int) floorf(nom->xc[q]) - rad;
            int    x1   = (int) ceilf(nom->xc[q]) + rad;
            int    y0   = (int) floorf(nom->yc[q]) - rad;
            int    y1   = (int) ceilf(nom->yc[q]) + rad;
            if (x0 < 0)
            {
                x0 = 0;
            }
            if (y0 < 0)
            {
                y0 = 0;
            }
            if (x1 >= (int) nx)
            {
                x1 = (int) nx - 1;
            }
            if (y1 >= (int) ny)
            {
                y1 = (int) ny - 1;
            }
            for (int y = y0; y <= y1; y++)
            {
                double dy = (double) y - (double) nom->yc[q];
                for (int x = x0; x <= x1; x++)
                {
                    double dx = (double) x - (double) nom->xc[q];
                    double r  = hypot(dx, dy);
                    if (dx * sgnx < 0.0 && dy * sgny < 0.0 &&
                        r > 0.65 * (double) nom->rout[q])
                    {
                        st->wgt_map[(size_t) y * nx + (size_t) x] = 0.15f;
                    }
                }
            }
        }
    }

    st->ready = 1;
    return 0;
}

static inline void pyrwfs_pupreg_free(pyrwfs_pupreg_state_t *st)
{
    if (st == NULL)
    {
        return;
    }
    free(st->ref_edge);
    free(st->cur_edge);
    free(st->wgt_map);
    memset(st, 0, sizeof(*st));
}

/* Measure 4-pupil shifts & decompose into 6 global DOFs + residual */
static inline int pyrwfs_pupreg_run(pyrwfs_pupreg_state_t *st,
                                    const float           *im,
                                    int                    maxshift,
                                    pyrwfs_pupgeom_t      *geom_out,
                                    pyrwfs_pupdof_t       *dof_out)
{
    if (st == NULL || !st->ready || im == NULL || geom_out == NULL || dof_out == NULL)
    {
        return -1;
    }
    if (maxshift < 1)
    {
        maxshift = 1;
    }
    if (maxshift > 16)
    {
        maxshift = 16;
    }

    const uint32_t nx = st->nx;
    const uint32_t ny = st->ny;
    pyrwfs_compute_soft_sobel(im, nx, ny, st->cur_edge);

    double dxq[PYRWFS_NPUP];
    double dyq[PYRWFS_NPUP];
    double cpk[PYRWFS_NPUP];

    const int ngrid = 2 * maxshift + 1;

    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        int rad = (int) ceilf(st->nom.rout[q] + 3.0f);
        int x0  = (int) lroundf(st->nom.xc[q]) - rad;
        int x1  = (int) lroundf(st->nom.xc[q]) + rad;
        int y0  = (int) lroundf(st->nom.yc[q]) - rad;
        int y1  = (int) lroundf(st->nom.yc[q]) + rad;

        /* Clamp window so shifted reads stay in bounds */
        if (x0 < maxshift + 1)
        {
            x0 = maxshift + 1;
        }
        if (y0 < maxshift + 1)
        {
            y0 = maxshift + 1;
        }
        if (x1 > (int) nx - maxshift - 2)
        {
            x1 = (int) nx - maxshift - 2;
        }
        if (y1 > (int) ny - maxshift - 2)
        {
            y1 = (int) ny - maxshift - 2;
        }

        if (x1 <= x0 || y1 <= y0)
        {
            dxq[q] = 0.0;
            dyq[q] = 0.0;
            cpk[q] = 0.0;
            continue;
        }

        /* Weighted reference mean and variance in window */
        double wsum = 0.0;
        double rsum = 0.0;
        for (int y = y0; y <= y1; y++)
        {
            const float *re = st->ref_edge + (size_t) y * nx;
            const float *wm = st->wgt_map + (size_t) y * nx;
            for (int x = x0; x <= x1; x++)
            {
                double w = (double) wm[x];
                wsum += w;
                rsum += w * (double) re[x];
            }
        }
        double rmean = (wsum > 0.0) ? (rsum / wsum) : 0.0;
        double rvar  = 0.0;
        for (int y = y0; y <= y1; y++)
        {
            const float *re = st->ref_edge + (size_t) y * nx;
            const float *wm = st->wgt_map + (size_t) y * nx;
            for (int x = x0; x <= x1; x++)
            {
                double dr = (double) re[x] - rmean;
                rvar += (double) wm[x] * dr * dr;
            }
        }
        if (rvar <= 1e-18)
        {
            dxq[q] = 0.0;
            dyq[q] = 0.0;
            cpk[q] = 0.0;
            continue;
        }

        /* Evaluate ZNCC grid over [-maxshift, +maxshift]^2 */
        double cmap[33 * 33]; /* ngrid <= 33 since maxshift <= 16 */
        int    best_ix = maxshift;
        int    best_iy = maxshift;
        double best_c  = -2.0;

        for (int iy = 0; iy < ngrid; iy++)
        {
            int sy = iy - maxshift;
            for (int ix = 0; ix < ngrid; ix++)
            {
                int    sx   = ix - maxshift;
                double csum = 0.0;
                double c2   = 0.0;
                double rc   = 0.0;

                for (int y = y0; y <= y1; y++)
                {
                    const float *re = st->ref_edge + (size_t) y * nx;
                    const float *wm = st->wgt_map + (size_t) y * nx;
                    const float *ce = st->cur_edge + (size_t) (y + sy) * nx;
                    for (int x = x0; x <= x1; x++)
                    {
                        double w  = (double) wm[x];
                        double cv = (double) ce[x + sx];
                        double dr = (double) re[x] - rmean;
                        csum += w * cv;
                        c2 += w * cv * cv;
                        rc += w * dr * cv;
                    }
                }
                double cvar  = c2 - (csum * csum) / wsum;
                double denom = sqrt(rvar * ((cvar > 1e-18) ? cvar : 1e-18));
                double corr  = rc / denom;
                cmap[iy * ngrid + ix] = corr;
                if (corr > best_c)
                {
                    best_c  = corr;
                    best_ix = ix;
                    best_iy = iy;
                }
            }
        }

        /* 1D parabolic sub-pixel refinement in x and y around interior peak */
        double sub_x = 0.0;
        double sub_y = 0.0;
        if (best_ix > 0 && best_ix + 1 < ngrid)
        {
            double cm    = cmap[best_iy * ngrid + (best_ix - 1)];
            double c0    = cmap[best_iy * ngrid + best_ix];
            double cp    = cmap[best_iy * ngrid + (best_ix + 1)];
            double denom = cm - 2.0 * c0 + cp;
            if (denom < -1e-12)
            {
                sub_x = 0.5 * (cm - cp) / denom;
                if (sub_x > 0.6)
                {
                    sub_x = 0.6;
                }
                else if (sub_x < -0.6)
                {
                    sub_x = -0.6;
                }
            }
        }
        if (best_iy > 0 && best_iy + 1 < ngrid)
        {
            double cm    = cmap[(best_iy - 1) * ngrid + best_ix];
            double c0    = cmap[best_iy * ngrid + best_ix];
            double cp    = cmap[(best_iy + 1) * ngrid + best_ix];
            double denom = cm - 2.0 * c0 + cp;
            if (denom < -1e-12)
            {
                sub_y = 0.5 * (cm - cp) / denom;
                if (sub_y > 0.6)
                {
                    sub_y = 0.6;
                }
                else if (sub_y < -0.6)
                {
                    sub_y = -0.6;
                }
            }
        }

        dxq[q] = (double) (best_ix - maxshift) + sub_x;
        dyq[q] = (double) (best_iy - maxshift) + sub_y;
        cpk[q] = best_c;
    }

    /* Populate updated geometry */
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        geom_out->xc[q]   = st->nom.xc[q] + (float) dxq[q];
        geom_out->yc[q]   = st->nom.yc[q] + (float) dyq[q];
        geom_out->rout[q] = st->nom.rout[q];
        geom_out->rin[q]  = st->nom.rin[q];
        geom_out->corr[q] = (float) cpk[q];
    }

    /* 6-DOF geometric decomposition of the 4 pupil shifts */
    double cx0 = 0.25 * (st->nom.xc[0] + st->nom.xc[1] + st->nom.xc[2] + st->nom.xc[3]);
    double cy0 = 0.25 * (st->nom.yc[0] + st->nom.yc[1] + st->nom.yc[2] + st->nom.yc[3]);
    double sx0 = 0.25 * ((st->nom.xc[2] + st->nom.xc[3]) - (st->nom.xc[0] + st->nom.xc[1]));
    double sy0 = 0.25 * ((st->nom.yc[1] + st->nom.yc[3]) - (st->nom.yc[0] + st->nom.yc[2]));

    double dx  = 0.25 * (dxq[0] + dxq[1] + dxq[2] + dxq[3]);
    double dy  = 0.25 * (dyq[0] + dyq[1] + dyq[2] + dyq[3]);
    double dsx = 0.25 * ((dxq[2] + dxq[3]) - (dxq[0] + dxq[1]));
    double dsy = 0.25 * ((dyq[1] + dyq[3]) - (dyq[0] + dyq[2]));

    double r2sum  = 0.0;
    double rotsum = 0.0;
    double skwsum = 0.0;
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        double u  = (double) st->nom.xc[q] - cx0;
        double v  = (double) st->nom.yc[q] - cy0;
        double rx = dxq[q] - dx;
        double ry = dyq[q] - dy;
        r2sum += u * u + v * v;
        rotsum += u * ry - v * rx;
        skwsum += u * ry + v * rx;
    }
    double rot   = (r2sum > 1e-12) ? (rotsum / r2sum) : 0.0;
    double skew  = (r2sum > 1e-12) ? (skwsum / r2sum) : 0.0;
    double scale = 0.5 * (((fabs(sx0) > 1e-6) ? (dsx / sx0) : 0.0) +
                          ((fabs(sy0) > 1e-6) ? (dsy / sy0) : 0.0));

    /* Non-rigid residual across the 4 pupils after subtracting affine 6-DOF model */
    double res2 = 0.0;
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        double u   = (double) st->nom.xc[q] - cx0;
        double v   = (double) st->nom.yc[q] - cy0;
        double sgn_x = (q == 0 || q == 1) ? -1.0 : 1.0;
        double sgn_y = (q == 0 || q == 2) ? -1.0 : 1.0;
        double mx  = dx + sgn_x * dsx - rot * v + 0.5 * skew * v;
        double my  = dy + sgn_y * dsy + rot * u + 0.5 * skew * u;
        double ex  = dxq[q] - mx;
        double ey  = dyq[q] - my;
        res2 += ex * ex + ey * ey;
    }

    dof_out->dx    = (float) dx;
    dof_out->dy    = (float) dy;
    dof_out->dsx   = (float) dsx;
    dof_out->dsy   = (float) dsy;
    dof_out->rot   = (float) rot;
    dof_out->scale = (float) scale;
    dof_out->skew  = (float) skew;
    dof_out->resid = (float) sqrt(res2 / (double) PYRWFS_NPUP);

    return 0;
}

/* =========================================================================
 * Sub-Pixel 4-Pupil Cutout & Registration Gather -> Isum, Sx, Sy
 * ========================================================================= */
static inline void pyrwfs_extract4_run_bg(const float            *im,
                                          uint32_t                nx,
                                          uint32_t                ny,
                                          const pyrwfs_pupgeom_t *geom,
                                          uint32_t                npix,
                                          int                     norm_mode,
                                          float                   bg_sub,
                                          float                  *isum_out,
                                          float                  *sx_out,
                                          float                  *sy_out)
{
    const float half    = 0.5f * (float) (npix - 1u);
    double      sum_all = 0.0;
    size_t      cnt_pos = 0;

    for (uint32_t j = 0; j < npix; j++)
    {
        float v = (float) j - half;
        for (uint32_t i = 0; i < npix; i++)
        {
            float  u   = (float) i - half;
            size_t idx = (size_t) j * npix + i;

            float i0 =
                pyrwfs_sample_bilinear(im, nx, ny, geom->xc[0] + u, geom->yc[0] + v) - bg_sub;
            float i1 =
                pyrwfs_sample_bilinear(im, nx, ny, geom->xc[1] + u, geom->yc[1] + v) - bg_sub;
            float i2 =
                pyrwfs_sample_bilinear(im, nx, ny, geom->xc[2] + u, geom->yc[2] + v) - bg_sub;
            float i3 =
                pyrwfs_sample_bilinear(im, nx, ny, geom->xc[3] + u, geom->yc[3] + v) - bg_sub;

            float isum    = i0 + i1 + i2 + i3;
            isum_out[idx] = isum;
            sx_out[idx]   = (i2 + i3) - (i0 + i1);
            sy_out[idx]   = (i1 + i3) - (i0 + i2);
            if (isum > 0.0f)
            {
                sum_all += (double) isum;
                cnt_pos++;
            }
        }
    }

    float gnorm   = (cnt_pos > 0u) ? (float) (sum_all / (double) cnt_pos) : 1.0f;
    float floor_v = 1e-6f * ((gnorm > 1.0f) ? gnorm : 1.0f);

    if (norm_mode == 1)
    {
        /* Local per-pixel Ragazzoni normalization */
        for (size_t idx = 0; idx < (size_t) npix * npix; idx++)
        {
            float isum = isum_out[idx];
            if (isum > floor_v)
            {
                float inv = 1.0f / isum;
                sx_out[idx] *= inv;
                sy_out[idx] *= inv;
            }
            else
            {
                sx_out[idx] = 0.0f;
                sy_out[idx] = 0.0f;
            }
        }
    }
    else
    {
        /* Global mean-flux normalization */
        float inv = 1.0f / ((gnorm > floor_v) ? gnorm : floor_v);
        for (size_t idx = 0; idx < (size_t) npix * npix; idx++)
        {
            sx_out[idx] *= inv;
            sy_out[idx] *= inv;
        }
    }
}

static inline void pyrwfs_extract4_run(const float            *im,
                                       uint32_t                nx,
                                       uint32_t                ny,
                                       const pyrwfs_pupgeom_t *geom,
                                       uint32_t                npix,
                                       int                     norm_mode,
                                       float                  *isum_out,
                                       float                  *sx_out,
                                       float                  *sy_out)
{
    pyrwfs_extract4_run_bg(im, nx, ny, geom, npix, norm_mode, 0.0f, isum_out, sx_out, sy_out);
}

/* =========================================================================
 * Single-pass packed-zone flux budget, quad-cell, saturation & NL metrics
 * ========================================================================= */
static inline void pyrwfs_compute_frame_metrics(const float            *im,
                                                const uint8_t          *zones,
                                                const float            *ref_norm,
                                                const float            *even_basis,
                                                uint32_t                nx,
                                                uint32_t                ny,
                                                const pyrwfs_tlm_cfg_t *cfg,
                                                pyrwfs_frame_metrics_t *out)
{
    memset(out, 0, sizeof(*out));
    const size_t npix = (size_t) nx * ny;

    /* Pass 1: single fused scan over image + packed uint8_t zones */
    double bg_sum  = 0.0;
    double bg_sum2 = 0.0;
    size_t bg_cnt  = 0;

    double qsum[PYRWFS_NPUP] = { 0.0, 0.0, 0.0, 0.0 };
    size_t qcnt[PYRWFS_NPUP] = { 0u, 0u, 0u, 0u };
    double s_diam = 0.0;
    size_t c_diam = 0;
    double s_bH   = 0.0;
    size_t c_bH   = 0;
    double s_bV   = 0.0;
    size_t c_bV   = 0;
    double s_out  = 0.0;
    size_t c_out  = 0;

    size_t n_sat_hi   = 0;
    size_t n_sat_dark = 0;
    size_t n_pup      = 0;

    const float sat_hi     = (float) cfg->sat_hi;
    const float dark_floor = (float) cfg->dark_floor;

    for (size_t i = 0; i < npix; i++)
    {
        uint8_t code = zones[i] & PYRWFS_ZONE_MASK;
        float   v    = im[i];
        switch (code)
        {
        case PYRWFS_ZONE_PUP0:
        case PYRWFS_ZONE_PUP1:
        case PYRWFS_ZONE_PUP2:
        case PYRWFS_ZONE_PUP3:
        {
            int q = (int) (code - PYRWFS_ZONE_PUP0);
            qsum[q] += (double) v;
            qcnt[q]++;
            n_pup++;
            if (v > sat_hi)
            {
                n_sat_hi++;
            }
            if (v < dark_floor)
            {
                n_sat_dark++;
            }
            break;
        }
        case PYRWFS_ZONE_DIAMOND:
            s_diam += (double) v;
            c_diam++;
            break;
        case PYRWFS_ZONE_BRIDGE_H:
            s_bH += (double) v;
            c_bH++;
            break;
        case PYRWFS_ZONE_BRIDGE_V:
            s_bV += (double) v;
            c_bV++;
            break;
        case PYRWFS_ZONE_OUTER:
            s_out += (double) v;
            c_out++;
            break;
        case PYRWFS_ZONE_BG:
            bg_sum += (double) v;
            bg_sum2 += (double) v * (double) v;
            bg_cnt++;
            break;
        default:
            break;
        }
    }

    double bg_mean = (bg_cnt > 0u) ? (bg_sum / (double) bg_cnt) : 0.0;
    double bg_var  = (bg_cnt > 1u) ? (bg_sum2 / (double) bg_cnt - bg_mean * bg_mean) : 0.0;
    if (bg_var < 0.0)
    {
        bg_var = 0.0;
    }

    double bg_sub = cfg->sub_bg ? bg_mean : cfg->bg_const;
    double fpup   = 0.0;
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        double fq         = qsum[q] - bg_sub * (double) qcnt[q];
        out->pup_flux[q]  = fq;
        fpup += fq;
    }
    double fdiam = s_diam - bg_sub * (double) c_diam;
    double fbH   = s_bH - bg_sub * (double) c_bH;
    double fbV   = s_bV - bg_sub * (double) c_bV;
    double fout  = s_out - bg_sub * (double) c_out;
    double ftot  = fpup + fdiam + fbH + fbV + fout;

    double inv_tot = (ftot > 1e-9) ? (1.0 / ftot) : 0.0;
    double inv_pup = (fpup > 1e-9) ? (1.0 / fpup) : 0.0;

    out->ftot          = ftot;
    out->bg_mean       = bg_mean;
    out->bg_std        = sqrt(bg_var);
    out->sat_hi_frac   = (n_pup > 0u) ? ((double) n_sat_hi / (double) n_pup) : 0.0;
    out->sat_dark_frac = (n_pup > 0u) ? ((double) n_sat_dark / (double) n_pup) : 0.0;

    out->f_pup  = fpup * inv_tot;
    out->f_diam = fdiam * inv_tot;
    out->f_bH   = fbH * inv_tot;
    out->f_bV   = fbV * inv_tot;
    out->f_out  = fout * inv_tot;
    out->df_pup = out->f_pup - cfg->f_pup0;
    out->aniso  = (fbH - fbV) / (fabs(fbH) + fabs(fbV) + 1e-12);

    out->qx    = ((out->pup_flux[2] + out->pup_flux[3]) - (out->pup_flux[0] + out->pup_flux[1])) * inv_pup;
    out->qy    = ((out->pup_flux[1] + out->pup_flux[3]) - (out->pup_flux[0] + out->pup_flux[2])) * inv_pup;
    out->qdiag = ((out->pup_flux[0] + out->pup_flux[3]) - (out->pup_flux[1] + out->pup_flux[2])) * inv_pup;

    out->flux_ok = (ftot > cfg->flux_min && out->sat_hi_frac < 0.02) ? 1 : 0;

    /* Optional nonlinearity projection onto even-order Strehl mode */
    if (ref_norm != NULL && inv_tot > 0.0)
    {
        double dot_even = 0.0;
        double dI2_pup  = 0.0;
        for (size_t i = 0; i < npix; i++)
        {
            uint8_t code = zones[i] & PYRWFS_ZONE_MASK;
            if (code == PYRWFS_ZONE_NONE || code == PYRWFS_ZONE_BG)
            {
                continue;
            }
            double dI = ((double) im[i] - bg_sub) * inv_tot - (double) ref_norm[i];
            if (even_basis != NULL)
            {
                dot_even += dI * (double) even_basis[i];
            }
            if (code >= PYRWFS_ZONE_PUP0 && code <= PYRWFS_ZONE_PUP3)
            {
                dI2_pup += dI * dI;
            }
        }
        out->c_even   = dot_even;
        double rms_p  = (n_pup > 0u) ? sqrt(dI2_pup / (double) n_pup) : 0.0;
        out->nl_ratio = (rms_p > 1e-12) ? (fabs(dot_even) / sqrt(dI2_pup + 1e-24)) : 0.0;
    }
}

/* =========================================================================
 * Build default unit-norm even-mode basis (in-pupil positive, between+outer
 * negative, zero-mean over active zones) when no custom basis is provided
 * ========================================================================= */
static inline void pyrwfs_build_default_even_basis(const uint8_t *zones,
                                                   const float   *ref_norm,
                                                   uint32_t       nx,
                                                   uint32_t       ny,
                                                   float         *even_out)
{
    size_t npix = (size_t) nx * ny;
    memset(even_out, 0, npix * sizeof(float));

    double mean = 0.0;
    size_t cnt  = 0;
    for (size_t i = 0; i < npix; i++)
    {
        uint8_t code = zones[i] & PYRWFS_ZONE_MASK;
        if (code >= PYRWFS_ZONE_PUP0 && code <= PYRWFS_ZONE_OUTER)
        {
            double v = (code <= PYRWFS_ZONE_PUP3) ? 1.0 : -1.0;
            if (ref_norm != NULL && code > PYRWFS_ZONE_PUP3)
            {
                /* Weight negative lobe by flat-WF diffraction brightness */
                v = -(double) ref_norm[i];
            }
            even_out[i] = (float) v;
            mean += v;
            cnt++;
        }
    }
    if (cnt == 0u)
    {
        return;
    }
    mean /= (double) cnt;
    double n2 = 0.0;
    for (size_t i = 0; i < npix; i++)
    {
        uint8_t code = zones[i] & PYRWFS_ZONE_MASK;
        if (code >= PYRWFS_ZONE_PUP0 && code <= PYRWFS_ZONE_OUTER)
        {
            double v    = (double) even_out[i] - mean;
            even_out[i] = (float) v;
            n2 += v * v;
        }
    }
    if (n2 > 1e-24)
    {
        float invn = (float) (1.0 / sqrt(n2));
        for (size_t i = 0; i < npix; i++)
        {
            even_out[i] *= invn;
        }
    }
}

/* =========================================================================
 * Initialize / free full telemetry state
 * ========================================================================= */
static inline int pyrwfs_tlm_init(pyrwfs_tlm_state_t     *st,
                                  const pyrwfs_tlm_cfg_t *cfg,
                                  const pyrwfs_pupgeom_t *nom,
                                  const uint8_t          *zones_in,
                                  const float            *ref_im,
                                  const float            *even_in)
{
    memset(st, 0, sizeof(*st));
    st->cfg  = *cfg;
    st->nom  = *nom;
    st->live = *nom;

    if (st->cfg.npix == 0u)
    {
        float rmax = 0.0f;
        for (int q = 0; q < PYRWFS_NPUP; q++)
        {
            if (nom->rout[q] > rmax)
            {
                rmax = nom->rout[q];
            }
        }
        uint32_t np = (uint32_t) (2 * (int) ceilf(rmax) + 4);
        if (np < 8u)
        {
            np = 8u;
        }
        if (np & 1u)
        {
            np++;
        }
        st->cfg.npix = np;
    }

    const uint32_t nx   = st->cfg.nx;
    const uint32_t ny   = st->cfg.ny;
    const uint32_t npix = st->cfg.npix;
    const size_t   nimg = (size_t) nx * ny;
    const size_t   np2  = (size_t) npix * npix;

    st->zones_nom      = (uint8_t *) calloc(nimg, sizeof(uint8_t));
    st->zones          = (uint8_t *) calloc(nimg, sizeof(uint8_t));
    st->ref_norm_nom   = (float *) calloc(nimg, sizeof(float));
    st->ref_norm       = (float *) calloc(nimg, sizeof(float));
    st->even_basis_nom = (float *) calloc(nimg, sizeof(float));
    st->even_basis     = (float *) calloc(nimg, sizeof(float));
    st->im_avg         = (float *) calloc(nimg, sizeof(float));
    st->p_in_mask      = (uint8_t *) calloc(np2, sizeof(uint8_t));
    st->p_sp_mask      = (uint8_t *) calloc(np2, sizeof(uint8_t));
    st->isum           = (float *) calloc(np2, sizeof(float));
    st->sx             = (float *) calloc(np2, sizeof(float));
    st->sy             = (float *) calloc(np2, sizeof(float));
    st->isum_avg         = (float *) calloc(np2, sizeof(float));
    st->isum0_norm       = (float *) calloc(np2, sizeof(float));
    st->isum_rel_prev    = (float *) calloc(np2, sizeof(float));
    st->scint_map        = (float *) calloc(np2, sizeof(float));
    st->scint_debias_map = (float *) calloc(np2, sizeof(float));
    st->lvar_map         = (float *) calloc(np2, sizeof(float));

    if (!st->zones_nom || !st->zones || !st->ref_norm_nom || !st->ref_norm ||
        !st->even_basis_nom || !st->even_basis || !st->im_avg || !st->p_in_mask ||
        !st->p_sp_mask || !st->isum || !st->sx || !st->sy || !st->isum_avg || !st->isum0_norm ||
        !st->isum_rel_prev || !st->scint_map || !st->scint_debias_map || !st->lvar_map)
    {
        return -1;
    }

    if (zones_in != NULL)
    {
        memcpy(st->zones_nom, zones_in, nimg);
    }
    else
    {
        pyrwfs_build_default_zones(nx, ny, nom, 45.0f, 1.5f, st->zones_nom);
    }
    memcpy(st->zones, st->zones_nom, nimg);

    /* Project pupil-interior and spider flags onto the registered npix x npix pupil grid */
    float half = 0.5f * (float) (npix - 1u);
    for (uint32_t j = 0; j < npix; j++)
    {
        float v = (float) j - half;
        for (uint32_t i = 0; i < npix; i++)
        {
            float u     = (float) i - half;
            int   in_ok = 1;
            int   sp_ok = 0;
            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                int ix = (int) lroundf(nom->xc[q] + u);
                int iy = (int) lroundf(nom->yc[q] + v);
                if (ix < 0 || iy < 0 || ix >= (int) nx || iy >= (int) ny)
                {
                    in_ok = 0;
                    continue;
                }
                uint8_t zv = st->zones[(size_t) iy * nx + (size_t) ix];
                if ((zv & PYRWFS_FLAG_INNER_PUP) == 0u)
                {
                    in_ok = 0;
                }
                if ((zv & PYRWFS_FLAG_SPIDER) != 0u)
                {
                    sp_ok = 1;
                }
            }
            st->p_in_mask[(size_t) j * npix + i] = (uint8_t) in_ok;
            st->p_sp_mask[(size_t) j * npix + i] = (uint8_t) (in_ok && sp_ok);
        }
    }

    if (ref_im != NULL)
    {
        st->has_ref = 1;
        pyrwfs_frame_metrics_t fm0;
        pyrwfs_compute_frame_metrics(ref_im, st->zones, NULL, NULL, nx, ny, &st->cfg, &fm0);
        double bg_sub = st->cfg.sub_bg ? fm0.bg_mean : st->cfg.bg_const;
        double inv_t  = (fm0.ftot > 1e-9) ? (1.0 / fm0.ftot) : 0.0;
        for (size_t i = 0; i < nimg; i++)
        {
            st->ref_norm_nom[i] = (float) (((double) ref_im[i] - bg_sub) * inv_t);
        }
        memcpy(st->ref_norm, st->ref_norm_nom, nimg * sizeof(float));
        if (st->cfg.f_pup0 <= 0.0 && fm0.f_pup > 0.0)
        {
            st->cfg.f_pup0 = fm0.f_pup;
        }
        /* Precompute reference normalized 4-pupil sum (background-subtracted when sub_bg) */
        pyrwfs_extract4_run_bg(ref_im, nx, ny, nom, npix, st->cfg.norm_mode, (float) bg_sub,
                               st->isum0_norm, st->sx, st->sy);
        double isum_m = 0.0;
        size_t icnt   = 0;
        for (size_t i = 0; i < np2; i++)
        {
            if (st->p_in_mask[i])
            {
                isum_m += (double) st->isum0_norm[i];
                icnt++;
            }
        }
        float invm = (icnt > 0u && isum_m > 1e-9) ? (float) ((double) icnt / isum_m) : 1.0f;
        for (size_t i = 0; i < np2; i++)
        {
            st->isum0_norm[i] *= invm;
        }
        /* Initialize edge-based pupil registration state */
        if (pyrwfs_pupreg_init(&st->reg, nx, ny, ref_im, nom, st->cfg.edge_mode) != 0)
        {
            return -1;
        }
    }

    if (even_in != NULL)
    {
        memcpy(st->even_basis_nom, even_in, nimg * sizeof(float));
        st->has_even = 1;
    }
    else
    {
        pyrwfs_build_default_even_basis(st->zones_nom, st->has_ref ? st->ref_norm_nom : NULL, nx,
                                        ny, st->even_basis_nom);
        st->has_even = 1;
    }
    memcpy(st->even_basis, st->even_basis_nom, nimg * sizeof(float));

    st->pup_corr_min = 1.0;
    st->maps_seeded  = 0;
    return 0;
}

/* Recalibrate nominal pupil geometry in-place (preserving allocated npix SHM size) */
static inline int pyrwfs_tlm_recalibrate_geom(pyrwfs_tlm_state_t     *st,
                                              const pyrwfs_pupgeom_t *new_nom,
                                              const float            *ref_im,
                                              const uint8_t          *custom_zones,
                                              const float            *custom_even,
                                              float                   sp_ang,
                                              float                   sp_hw,
                                              double                  f_pup0_override)
{
    if (st == NULL || new_nom == NULL)
    {
        return -1;
    }
    const uint32_t nx   = st->cfg.nx;
    const uint32_t ny   = st->cfg.ny;
    const uint32_t npix = st->cfg.npix;
    const size_t   nimg = (size_t) nx * ny;
    const size_t   np2  = (size_t) npix * npix;

    st->nom      = *new_nom;
    st->live     = *new_nom;
    memset(&st->dof, 0, sizeof(st->dof));
    st->zones_sx = 0;
    st->zones_sy = 0;

    if (custom_zones == NULL)
    {
        pyrwfs_build_default_zones(nx, ny, new_nom, sp_ang, sp_hw, st->zones_nom);
    }
    else
    {
        memcpy(st->zones_nom, custom_zones, nimg);
    }
    memcpy(st->zones, st->zones_nom, nimg);

    /* Reproject pupil-interior and spider flags onto the existing npix x npix grid */
    float half = 0.5f * (float) (npix - 1u);
    for (uint32_t j = 0u; j < npix; j++)
    {
        float v = (float) j - half;
        for (uint32_t i = 0u; i < npix; i++)
        {
            float u     = (float) i - half;
            int   in_ok = 1;
            int   sp_ok = 0;
            for (int q = 0; q < PYRWFS_NPUP; q++)
            {
                int ix = (int) lroundf(new_nom->xc[q] + u);
                int iy = (int) lroundf(new_nom->yc[q] + v);
                if (ix < 0 || iy < 0 || ix >= (int) nx || iy >= (int) ny)
                {
                    in_ok = 0;
                    continue;
                }
                uint8_t zv = st->zones[(size_t) iy * nx + (size_t) ix];
                if ((zv & PYRWFS_FLAG_INNER_PUP) == 0u)
                {
                    in_ok = 0;
                }
                if ((zv & PYRWFS_FLAG_SPIDER) != 0u)
                {
                    sp_ok = 1;
                }
            }
            st->p_in_mask[(size_t) j * npix + i] = (uint8_t) in_ok;
            st->p_sp_mask[(size_t) j * npix + i] = (uint8_t) (in_ok && sp_ok);
        }
    }

    if (ref_im != NULL)
    {
        st->has_ref = 1;
        pyrwfs_frame_metrics_t fm0;
        pyrwfs_compute_frame_metrics(ref_im, st->zones, NULL, NULL, nx, ny, &st->cfg, &fm0);
        double bg_sub = st->cfg.sub_bg ? fm0.bg_mean : st->cfg.bg_const;
        double inv_t  = (fm0.ftot > 1e-9) ? (1.0 / fm0.ftot) : 0.0;
        for (size_t i = 0; i < nimg; i++)
        {
            st->ref_norm_nom[i] = (float) (((double) ref_im[i] - bg_sub) * inv_t);
        }
        memcpy(st->ref_norm, st->ref_norm_nom, nimg * sizeof(float));
        if (f_pup0_override <= 0.0 && fm0.f_pup > 0.0)
        {
            st->cfg.f_pup0 = fm0.f_pup;
        }
        pyrwfs_extract4_run_bg(ref_im, nx, ny, new_nom, npix, st->cfg.norm_mode, (float) bg_sub,
                               st->isum0_norm, st->sx, st->sy);
        double isum_m = 0.0;
        size_t icnt   = 0u;
        for (size_t i = 0; i < np2; i++)
        {
            if (st->p_in_mask[i])
            {
                isum_m += (double) st->isum0_norm[i];
                icnt++;
            }
        }
        float invm = (icnt > 0u && isum_m > 1e-9) ? (float) ((double) icnt / isum_m) : 1.0f;
        for (size_t i = 0; i < np2; i++)
        {
            st->isum0_norm[i] *= invm;
        }
        pyrwfs_pupreg_free(&st->reg);
        if (pyrwfs_pupreg_init(&st->reg, nx, ny, ref_im, new_nom, st->cfg.edge_mode) != 0)
        {
            return -1;
        }
    }
    else if (st->reg.ready)
    {
        pyrwfs_pupreg_free(&st->reg);
        (void) pyrwfs_pupreg_init(&st->reg, nx, ny, st->im_avg, new_nom, st->cfg.edge_mode);
    }

    if (custom_even == NULL)
    {
        pyrwfs_build_default_even_basis(st->zones_nom, st->has_ref ? st->ref_norm_nom : NULL, nx,
                                        ny, st->even_basis_nom);
    }
    else
    {
        memcpy(st->even_basis_nom, custom_even, nimg * sizeof(float));
    }
    memcpy(st->even_basis, st->even_basis_nom, nimg * sizeof(float));

    /* Reset 2D EMA maps so post-calibration maps have zero pre-lock ghosting */
    memset(st->isum_avg, 0, np2 * sizeof(float));
    memset(st->isum_rel_prev, 0, np2 * sizeof(float));
    memset(st->scint_map, 0, np2 * sizeof(float));
    memset(st->scint_debias_map, 0, np2 * sizeof(float));
    memset(st->lvar_map, 0, np2 * sizeof(float));
    st->scint_idx    = 0.0;
    st->scint_debias = 0.0;
    st->maps_seeded  = 0;
    st->has_prev_rel = 0;
    st->pup_corr_min = 1.0;
    return 0;
}

static inline void pyrwfs_shift_live_maps(pyrwfs_tlm_state_t *st, int sx, int sy)
{
    if (st->zones_sx == sx && st->zones_sy == sy)
    {
        return;
    }
    const uint32_t nx = st->cfg.nx;
    const uint32_t ny = st->cfg.ny;
    for (uint32_t y = 0; y < ny; y++)
    {
        int src_y = (int) y - sy;
        for (uint32_t x = 0; x < nx; x++)
        {
            int    src_x = (int) x - sx;
            size_t dst_i = (size_t) y * nx + x;
            if (src_x >= 0 && src_x < (int) nx && src_y >= 0 && src_y < (int) ny)
            {
                size_t src_i       = (size_t) src_y * nx + (size_t) src_x;
                st->zones[dst_i]   = st->zones_nom[src_i];
                st->ref_norm[dst_i] = st->ref_norm_nom[src_i];
                st->even_basis[dst_i] = st->even_basis_nom[src_i];
            }
            else
            {
                st->zones[dst_i]      = PYRWFS_ZONE_BG;
                st->ref_norm[dst_i]   = 0.0f;
                st->even_basis[dst_i] = 0.0f;
            }
        }
    }
    st->zones_sx = sx;
    st->zones_sy = sy;
}

static inline void pyrwfs_tlm_free(pyrwfs_tlm_state_t *st)
{
    if (st == NULL)
    {
        return;
    }
    pyrwfs_pupreg_free(&st->reg);
    free(st->zones_nom);
    free(st->zones);
    free(st->ref_norm_nom);
    free(st->ref_norm);
    free(st->even_basis_nom);
    free(st->even_basis);
    free(st->im_avg);
    free(st->p_in_mask);
    free(st->p_sp_mask);
    free(st->isum);
    free(st->sx);
    free(st->sy);
    free(st->isum_avg);
    free(st->isum0_norm);
    free(st->isum_rel_prev);
    free(st->scint_map);
    free(st->scint_debias_map);
    free(st->lvar_map);
    memset(st, 0, sizeof(*st));
}

/* =========================================================================
 * Process one WFS frame -> updates state & fills `tlm_vec[PYRWFS_TLM_NCOL]`
 * ========================================================================= */
static inline void pyrwfs_tlm_step(pyrwfs_tlm_state_t     *st,
                                   const float            *im,
                                   double                  ts,
                                   int                     do_reg,
                                   pyrwfs_frame_metrics_t *fm_out,
                                   float                   tlm_vec[PYRWFS_TLM_NCOL])
{
    const uint32_t nx   = st->cfg.nx;
    const uint32_t ny   = st->cfg.ny;
    const uint32_t npix = st->cfg.npix;
    const size_t   nimg = (size_t) nx * ny;
    const size_t   np2  = (size_t) npix * npix;

    /* 1. Single-pass zone flux budget, quad-cell, saturation & NL projection */
    pyrwfs_frame_metrics_t fm;
    pyrwfs_compute_frame_metrics(im, st->zones, st->has_ref ? st->ref_norm : NULL,
                                 st->has_even ? st->even_basis : NULL, nx, ny, &st->cfg, &fm);

    /* 2. Quad-cell EMA mean & TT jitter RMS */
    if (st->nframe == 0u)
    {
        st->qx_mean = fm.qx;
        st->qy_mean = fm.qy;
        st->qx_var  = 0.0;
        st->qy_var  = 0.0;
    }
    else
    {
        double as   = st->cfg.a_slow;
        double af   = st->cfg.a_fast;
        st->qx_mean = (1.0 - as) * st->qx_mean + as * fm.qx;
        st->qy_mean = (1.0 - as) * st->qy_mean + as * fm.qy;
        double dqx  = fm.qx - st->qx_mean;
        double dqy  = fm.qy - st->qy_mean;
        st->qx_var  = (1.0 - af) * st->qx_var + af * dqx * dqx;
        st->qy_var  = (1.0 - af) * st->qy_var + af * dqy * dqy;
    }

    /* 3. EMA-averaged frame for edge registration */
    if (st->nframe == 0u)
    {
        memcpy(st->im_avg, im, nimg * sizeof(float));
        if (!st->reg.ready)
        {
            (void) pyrwfs_pupreg_init(&st->reg, nx, ny, im, &st->nom, st->cfg.edge_mode);
        }
    }
    else
    {
        float ar  = (float) st->cfg.a_reg;
        float omr = 1.0f - ar;
        for (size_t i = 0; i < nimg; i++)
        {
            st->im_avg[i] = omr * st->im_avg[i] + ar * im[i];
        }
    }

    /* 4. Optional decimated edge-based 4-pupil alignment update */
    if (do_reg && fm.flux_ok && st->reg.ready)
    {
        pyrwfs_pupgeom_t cand;
        pyrwfs_pupdof_t  dof;
        if (pyrwfs_pupreg_run(&st->reg, st->im_avg, st->cfg.maxshift, &cand, &dof) == 0)
        {
            double cmin = (double) cand.corr[0];
            for (int q = 1; q < PYRWFS_NPUP; q++)
            {
                if ((double) cand.corr[q] < cmin)
                {
                    cmin = (double) cand.corr[q];
                }
            }
            st->pup_corr_min = cmin;
            st->dof          = dof;
            if (cmin >= st->cfg.corr_min)
            {
                st->live = cand;
                pyrwfs_shift_live_maps(st, (int) lroundf(dof.dx), (int) lroundf(dof.dy));
            }
            else
            {
                st->live = st->nom;
                pyrwfs_shift_live_maps(st, 0, 0);
                for (int q = 0; q < PYRWFS_NPUP; q++)
                {
                    st->live.corr[q] = cand.corr[q];
                }
            }
        }
    }

    /* 5. Registered 4-pupil gather -> Isum, Sx, Sy (background-subtracted) */
    float frm_bg = st->cfg.sub_bg ? (float) fm.bg_mean : (float) st->cfg.bg_const;
    pyrwfs_extract4_run_bg(im, nx, ny, &st->live, npix, st->cfg.norm_mode, frm_bg, st->isum,
                           st->sx, st->sy);

    /* 6. Scintillation map (raw & lag-1 noise-debiased), local phase-variance map, and spider LWE */
    double as     = st->cfg.a_slow;
    double mean_i = 0.0;
    size_t cnt_in = 0;
    double sc_sum = 0.0;
    double sd_sum = 0.0;
    double sp_sum = 0.0;
    size_t cnt_sp = 0;

    int seeded_now = 0;
    if (!st->maps_seeded)
    {
        memcpy(st->isum_avg, st->isum, np2 * sizeof(float));
        memset(st->isum_rel_prev, 0, np2 * sizeof(float));
        memset(st->scint_map, 0, np2 * sizeof(float));
        memset(st->scint_debias_map, 0, np2 * sizeof(float));
        st->maps_seeded  = 1;
        st->has_prev_rel = 0;
        seeded_now       = 1;
    }
    else if (fm.flux_ok)
    {
        float asf = (float) as;
        float oms = 1.0f - asf;
        for (size_t i = 0; i < np2; i++)
        {
            st->isum_avg[i] = oms * st->isum_avg[i] + asf * st->isum[i];
        }
    }

    for (size_t i = 0; i < np2; i++)
    {
        if (st->p_in_mask[i])
        {
            mean_i += (double) st->isum_avg[i];
            cnt_in++;
        }
    }
    double mean_p = (cnt_in > 0u) ? (mean_i / (double) cnt_in) : 0.0;
    double inv_mi = (mean_p > 1e-9) ? (1.0 / mean_p) : 0.0;
    double ifloor = 0.15 * mean_p;
    if (ifloor < 1e-6)
    {
        ifloor = 1e-6;
    }

    /* Exact EMA lag-1 white-noise cancellation: for white noise e_t = n_t - EMA(n)_t,
     * E[e_t * e_{t-1}] = -(a/2) * E[e_t^2], so (e_t * e_{t-1} + (a/2) * e_t^2) / (1 + a/2)
     * has identically zero photon/readout noise bias for any EMA weight a in (0, 1). */
    const double deb_w   = 0.5 * as;
    const double deb_inv = 1.0 / (1.0 + deb_w);
    const int    do_deb  = (!seeded_now && st->has_prev_rel && fm.flux_ok);

    for (size_t i = 0; i < np2; i++)
    {
        double ia  = (double) st->isum_avg[i];
        double den = (ia > ifloor) ? ia : ifloor;
        double rel = (st->p_in_mask[i] && mean_p > 1e-9) ? (((double) st->isum[i] - ia) / den) : 0.0;
        double sc  = (double) st->scint_map[i];
        double sd  = (double) st->scint_debias_map[i];
        if (fm.flux_ok)
        {
            sc               = (1.0 - as) * sc + as * rel * rel;
            st->scint_map[i] = (float) sc;
            if (do_deb)
            {
                double rprev            = (double) st->isum_rel_prev[i];
                double sample_deb       = (rel * rprev + deb_w * rel * rel) * deb_inv;
                sd                      = (1.0 - as) * sd + as * sample_deb;
                st->scint_debias_map[i] = (float) sd;
            }
            st->isum_rel_prev[i] = (float) rel;
        }
        st->lvar_map[i] = (float) (ia * inv_mi - (double) st->isum0_norm[i]);

        if (st->p_in_mask[i])
        {
            sc_sum += sc;
            sd_sum += sd;
        }
        if (st->p_sp_mask[i])
        {
            sp_sum += rel * rel;
            cnt_sp++;
        }
    }
    if (fm.flux_ok && !seeded_now)
    {
        st->has_prev_rel = 1;
    }
    else if (!fm.flux_ok)
    {
        st->has_prev_rel = 0;
    }
    st->scint_idx    = (cnt_in > 0u) ? (sc_sum / (double) cnt_in) : 0.0;
    st->scint_debias = (cnt_in > 0u) ? (sd_sum / (double) cnt_in) : 0.0;
    st->lwe_metric   = (cnt_sp > 0u) ? sqrt(sp_sum / (double) cnt_sp) : 0.0;

    /* 7. Slope curl vs divergence ratio on interior pupil pixels */
    double curl2 = 0.0;
    double div2  = 0.0;
    for (uint32_t j = 1u; j + 1u < npix; j++)
    {
        for (uint32_t i = 1u; i + 1u < npix; i++)
        {
            size_t idx = (size_t) j * npix + i;
            if (!st->p_in_mask[idx])
            {
                continue;
            }
            double dsx_dx = 0.5 * ((double) st->sx[idx + 1u] - (double) st->sx[idx - 1u]);
            double dsy_dy = 0.5 * ((double) st->sy[idx + npix] - (double) st->sy[idx - npix]);
            double dsy_dx = 0.5 * ((double) st->sy[idx + 1u] - (double) st->sy[idx - 1u]);
            double dsx_dy = 0.5 * ((double) st->sx[idx + npix] - (double) st->sx[idx - npix]);
            double div    = dsx_dx + dsy_dy;
            double curl   = dsy_dx - dsx_dy;
            div2 += div * div;
            curl2 += curl * curl;
        }
    }
    st->curl_ratio = sqrt(curl2) / (sqrt(div2) + 1e-12);

    st->nframe++;

    if (fm_out != NULL)
    {
        *fm_out = fm;
    }
    if (tlm_vec != NULL)
    {
        tlm_vec[PYRWFS_TLM_TS]            = (float) ts;
        tlm_vec[PYRWFS_TLM_FTOT]          = (float) fm.ftot;
        tlm_vec[PYRWFS_TLM_BG_MEAN]       = (float) fm.bg_mean;
        tlm_vec[PYRWFS_TLM_BG_STD]        = (float) fm.bg_std;
        tlm_vec[PYRWFS_TLM_SAT_HI_FRAC]   = (float) fm.sat_hi_frac;
        tlm_vec[PYRWFS_TLM_SAT_DARK_FRAC] = (float) fm.sat_dark_frac;
        tlm_vec[PYRWFS_TLM_F_PUP]         = (float) fm.f_pup;
        tlm_vec[PYRWFS_TLM_F_DIAM]        = (float) fm.f_diam;
        tlm_vec[PYRWFS_TLM_F_BH]          = (float) fm.f_bH;
        tlm_vec[PYRWFS_TLM_F_BV]          = (float) fm.f_bV;
        tlm_vec[PYRWFS_TLM_F_OUT]         = (float) fm.f_out;
        tlm_vec[PYRWFS_TLM_DF_PUP]        = (float) fm.df_pup;
        tlm_vec[PYRWFS_TLM_ANISO]         = (float) fm.aniso;
        tlm_vec[PYRWFS_TLM_NL_RATIO]      = (float) fm.nl_ratio;
        tlm_vec[PYRWFS_TLM_QX_MEAN]       = (float) st->qx_mean;
        tlm_vec[PYRWFS_TLM_QY_MEAN]       = (float) st->qy_mean;
        tlm_vec[PYRWFS_TLM_QX_RMS]        = (float) sqrt((st->qx_var > 0.0) ? st->qx_var : 0.0);
        tlm_vec[PYRWFS_TLM_QY_RMS]        = (float) sqrt((st->qy_var > 0.0) ? st->qy_var : 0.0);
        tlm_vec[PYRWFS_TLM_QDIAG]         = (float) fm.qdiag;
        tlm_vec[PYRWFS_TLM_SCINT_IDX]     = (float) st->scint_idx;
        tlm_vec[PYRWFS_TLM_CURL_RATIO]    = (float) st->curl_ratio;
        tlm_vec[PYRWFS_TLM_LWE_METRIC]    = (float) st->lwe_metric;
        tlm_vec[PYRWFS_TLM_PUP_DX]        = st->dof.dx;
        tlm_vec[PYRWFS_TLM_PUP_DY]        = st->dof.dy;
        tlm_vec[PYRWFS_TLM_PUP_DS]        = 0.5f * (st->dof.dsx + st->dof.dsy);
        tlm_vec[PYRWFS_TLM_PUP_DAST]      = st->dof.dsx - st->dof.dsy;
        tlm_vec[PYRWFS_TLM_PUP_ROT]       = st->dof.rot * 1000.0f; /* mrad */
        tlm_vec[PYRWFS_TLM_PUP_CORR]      = (float) st->pup_corr_min;
        tlm_vec[PYRWFS_TLM_FLUX_OK]       = (float) fm.flux_ok;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* CACAO_PYRWFS_TLM_CORE_H */
