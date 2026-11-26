// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    pyrwfs_calc_ext.c
 * @brief   Thin Tier 2 `@calc` C extensions wrapping `pyrwfs_tlm_core.h` (`libcacaopyrwfs_calc.so`)
 *
 * Installed to `lib/milk/calc.d/libcacaopyrwfs_calc.so` and auto-loaded by `@calc` and
 * `milk-fpsexec-milk-calc`. Links only against `libm` and OpenMP.
 *
 * Exported `@calc` operators:
 *   - nomgeom = pyrwfs_detect(im, thr=0.25)
 *   - pupgeom, pupdof = pyrwfs_pupreg(im, ref, nomgeom, maxshift=6, edge_mode=1)
 *   - isum, sx, sy = pyrwfs_extract4(im, pupgeom, npix=0, norm_mode=0)
 *   - tlm = pyrwfs_metrics(im, zones, [ref], sub_bg=1, sat_hi=60000, dark_floor=0,
 *                          flux_min=100, f_pup0=0)
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "milk_calc_ext.h"
#include "pyrwfs_tlm_core.h"

/* =========================================================================
 * Helper: read [4 x >=3] float geometry image into pyrwfs_pupgeom_t
 * ========================================================================= */
static int ext_read_geom(const IMAGE *im, pyrwfs_pupgeom_t *g)
{
    if (im == NULL || im->md[0].naxis < 2 || im->md[0].size[0] < 3u || im->md[0].size[1] < 4u)
    {
        return -1;
    }
    uint32_t     ncol = im->md[0].size[0];
    const float *src  = im->array.F;
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        const float *row = src + (size_t) q * ncol;
        g->xc[q]         = row[0];
        g->yc[q]         = row[1];
        g->rout[q]       = row[2];
        g->rin[q]        = (ncol >= 4u) ? row[3] : 0.0f;
        g->corr[q]       = 1.0f;
    }
    return 0;
}

/* =========================================================================
 * 1. nomgeom = pyrwfs_detect(im, thr=0.25)
 * ========================================================================= */
#define PYR_DETECT_PARAMS(M)                                                                    \
    M(".thr", thr, FPTYPE_FLOAT64, 0.25, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,                \
      "Binary footprint threshold fraction")

#define PYR_DETECT_PORTS(M)                                                                     \
    M(IN, im, MILK_PORT_IMAGE, 0, "2D PyWFS intensity image")                                   \
    M(OUT, nomgeom, MILK_PORT_IMAGE, 0, "Detected [4 x 4] pupil geometry [xc, yc, rout, rin]")

MILK_CALC_OP_DECLARE(pyrwfs_detect, PYR_DETECT_PARAMS, PYR_DETECT_PORTS);

static int pyrwfs_detect_init(milk_calc_op_ctx_t *ctx, const void *P, void **state)
{
    (void) P;
    (void) state;
    const IMAGE *im = MILK_IN(ctx, im);
    if (im == NULL || im->md[0].naxis < 2 || im->md[0].size[0] < 8u || im->md[0].size[1] < 8u)
    {
        return -1;
    }
    milk_op_set_out_shape(ctx, nomgeom, 4u, PYRWFS_NPUP);
    return 0;
}

static int pyrwfs_detect_exec(milk_calc_op_ctx_t *ctx, const void *P, void *state)
{
    (void) state;
    const struct pyrwfs_detect_par *par = (const struct pyrwfs_detect_par *) P;
    const IMAGE                    *im  = MILK_IN(ctx, im);
    IMAGE                          *out = MILK_OUT(ctx, nomgeom);
    if (im == NULL || out == NULL)
    {
        return -1;
    }
    pyrwfs_pupgeom_t g;
    if (pyrwfs_detect_pupils(im->array.F, im->md[0].size[0], im->md[0].size[1], (float) par->thr,
                             &g) != 0)
    {
        return -1;
    }
    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        float *row = out->array.F + (size_t) q * 4u;
        row[0]     = g.xc[q];
        row[1]     = g.yc[q];
        row[2]     = g.rout[q];
        row[3]     = g.rin[q];
    }
    return 0;
}

static void pyrwfs_detect_free(void *state)
{
    (void) state;
}

/* =========================================================================
 * 2. pupgeom, pupdof = pyrwfs_pupreg(im, ref, nomgeom, maxshift=6, edge_mode=1)
 * ========================================================================= */
#define PYR_PUPREG_PARAMS(M)                                                                    \
    M(".maxshift", maxshift, FPTYPE_INT64, 6, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,           \
      "Registration search half-width [pix]")                                                   \
    M(".edge_mode", edge_mode, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT,                           \
      "0=full soft-Sobel, 1=downweight inner diffraction arc")

#define PYR_PUPREG_PORTS(M)                                                                     \
    M(IN, im, MILK_PORT_IMAGE, 0, "Current 2D PyWFS image (typically EMA-smoothed)")            \
    M(IN, ref, MILK_PORT_IMAGE, 0, "Reference 2D PyWFS image")                                  \
    M(IN, nomgeom, MILK_PORT_IMAGE, 0, "Nominal [4 x 4] pupil geometry [xc, yc, rout, rin]")    \
    M(OUT, pupgeom, MILK_PORT_IMAGE, 0, "Tracked [4 x 4] pupil geometry [xc, yc, rout, corr]")  \
    M(OUT, pupdof, MILK_PORT_IMAGE, 0, "1D [8] DOFs [dx, dy, dsx, dsy, rot, scale, skew, res]")

MILK_CALC_OP_DECLARE(pyrwfs_pupreg, PYR_PUPREG_PARAMS, PYR_PUPREG_PORTS);

static int pyrwfs_pupreg_ext_init(milk_calc_op_ctx_t *ctx, const void *P, void **state)
{
    const struct pyrwfs_pupreg_par *par = (const struct pyrwfs_pupreg_par *) P;
    const IMAGE                    *im  = MILK_IN(ctx, im);
    const IMAGE                    *ref = MILK_IN(ctx, ref);
    const IMAGE                    *ng  = MILK_IN(ctx, nomgeom);
    if (im == NULL || ref == NULL || ng == NULL || im->md[0].naxis < 2 || ref->md[0].naxis < 2 ||
        im->md[0].size[0] != ref->md[0].size[0] || im->md[0].size[1] != ref->md[0].size[1])
    {
        return -1;
    }

    pyrwfs_pupgeom_t nom;
    if (ext_read_geom(ng, &nom) != 0)
    {
        return -1;
    }

    pyrwfs_pupreg_state_t *st = (pyrwfs_pupreg_state_t *) calloc(1u, sizeof(*st));
    if (st == NULL)
    {
        return -1;
    }
    if (pyrwfs_pupreg_init(st, ref->md[0].size[0], ref->md[0].size[1], ref->array.F, &nom,
                           (int) par->edge_mode) != 0)
    {
        free(st);
        return -1;
    }

    *state = st;
    milk_op_set_out_shape(ctx, pupgeom, 4u, PYRWFS_NPUP);
    milk_op_set_out_shape(ctx, pupdof, PYRWFS_DOF_NCOL, 0u);
    return 0;
}

static int pyrwfs_pupreg_ext_exec(milk_calc_op_ctx_t *ctx, const void *P, void *state)
{
    const struct pyrwfs_pupreg_par *par  = (const struct pyrwfs_pupreg_par *) P;
    pyrwfs_pupreg_state_t          *st   = (pyrwfs_pupreg_state_t *) state;
    const IMAGE                    *im   = MILK_IN(ctx, im);
    IMAGE                          *ogeo = MILK_OUT(ctx, pupgeom);
    IMAGE                          *odof = MILK_OUT(ctx, pupdof);
    if (st == NULL || im == NULL || ogeo == NULL || odof == NULL)
    {
        return -1;
    }

    pyrwfs_pupgeom_t g;
    pyrwfs_pupdof_t  d;
    if (pyrwfs_pupreg_run(st, im->array.F, (int) par->maxshift, &g, &d) != 0)
    {
        return -1;
    }

    for (int q = 0; q < PYRWFS_NPUP; q++)
    {
        float *row = ogeo->array.F + (size_t) q * 4u;
        row[0]     = g.xc[q];
        row[1]     = g.yc[q];
        row[2]     = g.rout[q];
        row[3]     = g.corr[q];
    }
    odof->array.F[0] = d.dx;
    odof->array.F[1] = d.dy;
    odof->array.F[2] = d.dsx;
    odof->array.F[3] = d.dsy;
    odof->array.F[4] = d.rot;
    odof->array.F[5] = d.scale;
    odof->array.F[6] = d.skew;
    odof->array.F[7] = d.resid;
    return 0;
}

static void pyrwfs_pupreg_ext_free(void *state)
{
    pyrwfs_pupreg_state_t *st = (pyrwfs_pupreg_state_t *) state;
    if (st != NULL)
    {
        pyrwfs_pupreg_free(st);
        free(st);
    }
}

/* =========================================================================
 * 3. isum, sx, sy = pyrwfs_extract4(im, pupgeom, npix=0, norm_mode=0)
 * ========================================================================= */
#define PYR_EXTRACT4_PARAMS(M)                                                                  \
    M(".npix", npix, FPTYPE_INT64, 0, FPFLAG_DEFAULT_INPUT,                                     \
      "Output pupil grid size (0=auto from rout)")                                              \
    M(".norm_mode", norm_mode, FPTYPE_INT64, 0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,         \
      "0=global mean Isum norm, 1=local per-pixel norm")

#define PYR_EXTRACT4_PORTS(M)                                                                   \
    M(IN, im, MILK_PORT_IMAGE, 0, "2D PyWFS image")                                             \
    M(IN, pupgeom, MILK_PORT_IMAGE, 0, "[4 x >=3] pupil geometry [xc, yc, rout, ...]")          \
    M(OUT, isum, MILK_PORT_IMAGE, 0, "Registered 4-pupil sum (npix x npix)")                    \
    M(OUT, sx, MILK_PORT_IMAGE, 0, "Registered pyramid X-slope (npix x npix)")                  \
    M(OUT, sy, MILK_PORT_IMAGE, 0, "Registered pyramid Y-slope (npix x npix)")

MILK_CALC_OP_DECLARE(pyrwfs_extract4, PYR_EXTRACT4_PARAMS, PYR_EXTRACT4_PORTS);

static int pyrwfs_extract4_ext_init(milk_calc_op_ctx_t *ctx, const void *P, void **state)
{
    (void) state;
    const struct pyrwfs_extract4_par *par = (const struct pyrwfs_extract4_par *) P;
    const IMAGE                      *im  = MILK_IN(ctx, im);
    const IMAGE                      *pg  = MILK_IN(ctx, pupgeom);
    if (im == NULL || pg == NULL || im->md[0].naxis < 2)
    {
        return -1;
    }
    pyrwfs_pupgeom_t g;
    if (ext_read_geom(pg, &g) != 0)
    {
        return -1;
    }

    uint32_t npix = (par->npix > 0) ? (uint32_t) par->npix : 0u;
    if (npix == 0u)
    {
        float rmax = 0.0f;
        for (int q = 0; q < PYRWFS_NPUP; q++)
        {
            if (g.rout[q] > rmax)
            {
                rmax = g.rout[q];
            }
        }
        npix = (uint32_t) (2 * (int) ceilf(rmax) + 4);
        if (npix < 8u)
        {
            npix = 8u;
        }
        if (npix & 1u)
        {
            npix++;
        }
    }
    milk_op_set_out_shape(ctx, isum, npix, npix);
    milk_op_set_out_shape(ctx, sx, npix, npix);
    milk_op_set_out_shape(ctx, sy, npix, npix);
    return 0;
}

static int pyrwfs_extract4_ext_exec(milk_calc_op_ctx_t *ctx, const void *P, void *state)
{
    (void) state;
    const struct pyrwfs_extract4_par *par   = (const struct pyrwfs_extract4_par *) P;
    const IMAGE                      *im    = MILK_IN(ctx, im);
    const IMAGE                      *pg    = MILK_IN(ctx, pupgeom);
    IMAGE                            *oisum = MILK_OUT(ctx, isum);
    IMAGE                            *osx   = MILK_OUT(ctx, sx);
    IMAGE                            *osy   = MILK_OUT(ctx, sy);
    if (im == NULL || pg == NULL || oisum == NULL || osx == NULL || osy == NULL)
    {
        return -1;
    }

    pyrwfs_pupgeom_t g;
    if (ext_read_geom(pg, &g) != 0)
    {
        return -1;
    }
    pyrwfs_extract4_run(im->array.F, im->md[0].size[0], im->md[0].size[1], &g, oisum->md[0].size[0],
                        (int) par->norm_mode, oisum->array.F, osx->array.F, osy->array.F);
    return 0;
}

static void pyrwfs_extract4_ext_free(void *state)
{
    (void) state;
}

/* =========================================================================
 * 4. tlm = pyrwfs_metrics(im, zones, [ref], sub_bg=1, sat_hi=60000, ...)
 *    Single-pass frame flux budget, quad-cell, saturation & NL projection
 * ========================================================================= */
#define PYR_METRICS_PARAMS(M)                                                                   \
    M(".sub_bg", sub_bg, FPTYPE_INT64, 1, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,               \
      "Subtract corner background mean before flux budget")                                     \
    M(".sat_hi", sat_hi, FPTYPE_FLOAT64, 60000.0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,       \
      "High saturation floor [ADU]")                                                            \
    M(".dark_floor", dark_floor, FPTYPE_FLOAT64, 0.0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,   \
      "Dark-floor saturation threshold [ADU]")                                                  \
    M(".flux_min", flux_min, FPTYPE_FLOAT64, 100.0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,     \
      "Minimum net flux for flux_ok flag")                                                      \
    M(".f_pup0", f_pup0, FPTYPE_FLOAT64, 0.0, FPFLAG_DEFAULT_INPUT | FPFLAG_WRITERUN,           \
      "Reference in-pupil flux fraction (0=auto from ref)")

#define PYR_METRICS_PORTS(M)                                                                    \
    M(IN, im, MILK_PORT_IMAGE, 0, "2D PyWFS intensity image")                                   \
    M(IN, zones, MILK_PORT_IMAGE, 0, "Packed zone-label image (same shape as im)")              \
    M(IN, ref, MILK_PORT_IMAGE, MILK_PORT_OPT, "Optional flat-WF reference image")              \
    M(OUT, tlm, MILK_PORT_IMAGE, 0, "1D [18] frame metrics vector")

MILK_CALC_OP_DECLARE(pyrwfs_metrics, PYR_METRICS_PARAMS, PYR_METRICS_PORTS);

typedef struct
{
    uint32_t nx;
    uint32_t ny;
    uint8_t *zones;
    float   *ref_norm;
    float   *even_basis;
    double   f_pup0_auto;
    int      has_ref;
} pyr_metrics_state_t;

static int pyrwfs_metrics_ext_init(milk_calc_op_ctx_t *ctx, const void *P, void **state)
{
    const struct pyrwfs_metrics_par *par = (const struct pyrwfs_metrics_par *) P;
    const IMAGE                     *im  = MILK_IN(ctx, im);
    const IMAGE                     *zn  = MILK_IN(ctx, zones);
    const IMAGE                     *ref = MILK_IN(ctx, ref);
    if (im == NULL || zn == NULL || im->md[0].naxis < 2 || zn->md[0].naxis < 2 ||
        im->md[0].size[0] != zn->md[0].size[0] || im->md[0].size[1] != zn->md[0].size[1])
    {
        return -1;
    }

    uint32_t nx   = im->md[0].size[0];
    uint32_t ny   = im->md[0].size[1];
    size_t   npix = (size_t) nx * ny;

    pyr_metrics_state_t *st = (pyr_metrics_state_t *) calloc(1u, sizeof(*st));
    if (st == NULL)
    {
        return -1;
    }
    st->nx         = nx;
    st->ny         = ny;
    st->zones      = (uint8_t *) malloc(npix);
    st->ref_norm   = (float *) calloc(npix, sizeof(float));
    st->even_basis = (float *) calloc(npix, sizeof(float));
    if (!st->zones || !st->ref_norm || !st->even_basis)
    {
        free(st->zones);
        free(st->ref_norm);
        free(st->even_basis);
        free(st);
        return -1;
    }

    for (size_t i = 0; i < npix; i++)
    {
        long v       = lroundf(zn->array.F[i]);
        st->zones[i] = (uint8_t) ((v >= 0 && v <= 255) ? v : 0);
    }

    if (ref != NULL && ref->md[0].size[0] == nx && ref->md[0].size[1] == ny)
    {
        pyrwfs_tlm_cfg_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.nx     = nx;
        cfg.ny     = ny;
        cfg.sub_bg = (par->sub_bg != 0);
        pyrwfs_frame_metrics_t fm0;
        pyrwfs_compute_frame_metrics(ref->array.F, st->zones, NULL, NULL, nx, ny, &cfg, &fm0);
        double bg_sub = cfg.sub_bg ? fm0.bg_mean : 0.0;
        double inv_t  = (fm0.ftot > 1e-9) ? (1.0 / fm0.ftot) : 0.0;
        for (size_t i = 0; i < npix; i++)
        {
            st->ref_norm[i] = (float) (((double) ref->array.F[i] - bg_sub) * inv_t);
        }
        st->f_pup0_auto = fm0.f_pup;
        st->has_ref     = 1;
        pyrwfs_build_default_even_basis(st->zones, st->ref_norm, nx, ny, st->even_basis);
    }

    *state = st;
    milk_op_set_out_shape(ctx, tlm, 18u, 0u);
    return 0;
}

static int pyrwfs_metrics_ext_exec(milk_calc_op_ctx_t *ctx, const void *P, void *state)
{
    const struct pyrwfs_metrics_par *par = (const struct pyrwfs_metrics_par *) P;
    pyr_metrics_state_t             *st  = (pyr_metrics_state_t *) state;
    const IMAGE                     *im  = MILK_IN(ctx, im);
    IMAGE                           *out = MILK_OUT(ctx, tlm);
    if (st == NULL || im == NULL || out == NULL)
    {
        return -1;
    }

    pyrwfs_tlm_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.nx         = st->nx;
    cfg.ny         = st->ny;
    cfg.sub_bg     = (par->sub_bg != 0);
    cfg.sat_hi     = par->sat_hi;
    cfg.dark_floor = par->dark_floor;
    cfg.flux_min   = par->flux_min;
    cfg.f_pup0     = (par->f_pup0 > 0.0) ? par->f_pup0 : st->f_pup0_auto;

    pyrwfs_frame_metrics_t fm;
    pyrwfs_compute_frame_metrics(im->array.F, st->zones, st->has_ref ? st->ref_norm : NULL,
                                 st->has_ref ? st->even_basis : NULL, st->nx, st->ny, &cfg, &fm);

    float *v = out->array.F;
    v[0]     = (float) fm.ftot;
    v[1]     = (float) fm.bg_mean;
    v[2]     = (float) fm.bg_std;
    v[3]     = (float) fm.sat_hi_frac;
    v[4]     = (float) fm.sat_dark_frac;
    v[5]     = (float) fm.f_pup;
    v[6]     = (float) fm.f_diam;
    v[7]     = (float) fm.f_bH;
    v[8]     = (float) fm.f_bV;
    v[9]     = (float) fm.f_out;
    v[10]    = (float) fm.df_pup;
    v[11]    = (float) fm.aniso;
    v[12]    = (float) fm.nl_ratio;
    v[13]    = (float) fm.qx;
    v[14]    = (float) fm.qy;
    v[15]    = (float) fm.qdiag;
    v[16]    = (float) fm.c_even;
    v[17]    = (float) fm.flux_ok;
    return 0;
}

static void pyrwfs_metrics_ext_free(void *state)
{
    pyr_metrics_state_t *st = (pyr_metrics_state_t *) state;
    if (st != NULL)
    {
        free(st->zones);
        free(st->ref_norm);
        free(st->even_basis);
        free(st);
    }
}

MILK_CALC_EXT_EXPORT(MILK_CALC_REG_OP(pyrwfs_detect,
                                      pyrwfs_detect,
                                      pyrwfs_detect_init,
                                      pyrwfs_detect_exec,
                                      pyrwfs_detect_free),
                     MILK_CALC_REG_OP(pyrwfs_pupreg,
                                      pyrwfs_pupreg,
                                      pyrwfs_pupreg_ext_init,
                                      pyrwfs_pupreg_ext_exec,
                                      pyrwfs_pupreg_ext_free),
                     MILK_CALC_REG_OP(pyrwfs_extract4,
                                      pyrwfs_extract4,
                                      pyrwfs_extract4_ext_init,
                                      pyrwfs_extract4_ext_exec,
                                      pyrwfs_extract4_ext_free),
                     MILK_CALC_REG_OP(pyrwfs_metrics,
                                      pyrwfs_metrics,
                                      pyrwfs_metrics_ext_init,
                                      pyrwfs_metrics_ext_exec,
                                      pyrwfs_metrics_ext_free));
