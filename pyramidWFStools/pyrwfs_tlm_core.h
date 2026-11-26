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
    float   *isum_avg;    /* EMA 4-pupil sum */
    float   *isum0_norm;  /* reference normalized 4-pupil sum */
    float   *scint_map;   /* EMA normalized intensity variance map */
    float   *lvar_map;    /* local phase-variance / vignetting map */

    /* Running scalar EMAs */
    uint64_t nframe;
    double   qx_mean;
    double   qy_mean;
    double   qx_var;
    double   qy_var;
    double   scint_idx;
    double   curl_ratio;
    double   lwe_metric;
    double   pup_corr_min;
    int      has_ref;
    int      has_even;
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
 * Build default packed zone-label map from nominal 4-pupil geometry
 * ========================================================================= */
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
    st->isum_avg       = (float *) calloc(np2, sizeof(float));
    st->isum0_norm     = (float *) calloc(np2, sizeof(float));
    st->scint_map      = (float *) calloc(np2, sizeof(float));
    st->lvar_map       = (float *) calloc(np2, sizeof(float));

    if (!st->zones_nom || !st->zones || !st->ref_norm_nom || !st->ref_norm ||
        !st->even_basis_nom || !st->even_basis || !st->im_avg || !st->p_in_mask ||
        !st->p_sp_mask || !st->isum || !st->sx || !st->sy || !st->isum_avg || !st->isum0_norm ||
        !st->scint_map || !st->lvar_map)
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
    free(st->scint_map);
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

    /* 6. Scintillation map, local phase-variance map, and spider LWE metric */
    double as     = st->cfg.a_slow;
    double mean_i = 0.0;
    size_t cnt_in = 0;
    double sc_sum = 0.0;
    double sp_sum = 0.0;
    size_t cnt_sp = 0;

    if (st->nframe == 0u)
    {
        memcpy(st->isum_avg, st->isum, np2 * sizeof(float));
        memset(st->scint_map, 0, np2 * sizeof(float));
    }
    else
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

    for (size_t i = 0; i < np2; i++)
    {
        double ia  = (double) st->isum_avg[i];
        double den = (ia > ifloor) ? ia : ifloor;
        double rel = (st->p_in_mask[i] && mean_p > 1e-9) ? (((double) st->isum[i] - ia) / den) : 0.0;
        double sc  = (1.0 - as) * (double) st->scint_map[i] + as * rel * rel;
        st->scint_map[i] = (float) sc;
        st->lvar_map[i]  = (float) (ia * inv_mi - (double) st->isum0_norm[i]);

        if (st->p_in_mask[i])
        {
            sc_sum += sc;
        }
        if (st->p_sp_mask[i])
        {
            sp_sum += rel * rel;
            cnt_sp++;
        }
    }
    st->scint_idx  = (cnt_in > 0u) ? (sc_sum / (double) cnt_in) : 0.0;
    st->lwe_metric = (cnt_sp > 0u) ? sqrt(sp_sum / (double) cnt_sp) : 0.0;

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
