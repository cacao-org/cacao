// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file    eoflin_phys.c
 * @brief   Pass-2 Filtered-X DM kernel, windlayers physics-Gram Bayesian prior, and
 *          extended-memory / regime-weighted Gram accumulators for mkEOFPF.
 */

// MILK_CMAKE_MANDATE_LAPACKE
// MILK_CMAKE_REQUEST_BLAS
// MILK_CMAKE_REQUEST_CFITSIO

#ifdef _OPENMP
#    include <omp.h>
#endif
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef USE_CFITSIO
#    include <fitsio.h>
#endif

#include "milk_blas_lapacke.h"
#include "AOloopControl_PredictiveControl/eoflin_phys.h"

#define EP_PI 3.14159265358979323846

/* ---------------------------------------------------------------------------
 * Minimal self-contained radix-2 2D FFT (double precision, zero FFTW dep)
 * --------------------------------------------------------------------------- */

static uint32_t next_pow2_u32(uint32_t v)
{
    uint32_t p = 2;
    while (p < v && p < 512u)
    {
        p <<= 1;
    }
    return p;
}

static void fft1d_f64(double *re, double *im, uint32_t n)
{
    for (uint32_t i = 1, j = 0; i < n; i++)
    {
        uint32_t bit = n >> 1;
        for (; (j & bit) != 0u; bit >>= 1)
        {
            j ^= bit;
        }
        j ^= bit;
        if (i < j)
        {
            double tr = re[i];
            double ti = im[i];
            re[i]     = re[j];
            im[i]     = im[j];
            re[j]     = tr;
            im[j]     = ti;
        }
    }
    for (uint32_t len = 2; len <= n; len <<= 1)
    {
        double   ang = -2.0 * EP_PI / (double) len;
        double   wlr = cos(ang);
        double   wli = sin(ang);
        uint32_t h   = len >> 1;
        for (uint32_t i = 0; i < n; i += len)
        {
            double wr = 1.0;
            double wi = 0.0;
            for (uint32_t j = 0; j < h; j++)
            {
                uint32_t u  = i + j;
                uint32_t v  = u + h;
                double   vr = re[v] * wr - im[v] * wi;
                double   vi = re[v] * wi + im[v] * wr;
                re[v]       = re[u] - vr;
                im[v]       = im[u] - vi;
                re[u]      += vr;
                im[u]      += vi;
                double nwr  = wr * wlr - wi * wli;
                wi          = wr * wli + wi * wlr;
                wr          = nwr;
            }
        }
    }
}

static void fft2d_f64(double *re, double *im, uint32_t n)
{
    double col_r[512];
    double col_i[512];
    for (uint32_t y = 0; y < n; y++)
    {
        fft1d_f64(re + (size_t) y * n, im + (size_t) y * n, n);
    }
    for (uint32_t x = 0; x < n; x++)
    {
        for (uint32_t y = 0; y < n; y++)
        {
            col_r[y] = re[(size_t) y * n + x];
            col_i[y] = im[(size_t) y * n + x];
        }
        fft1d_f64(col_r, col_i, n);
        for (uint32_t y = 0; y < n; y++)
        {
            re[(size_t) y * n + x] = col_r[y];
            im[(size_t) y * n + x] = col_i[y];
        }
    }
}

/* ---------------------------------------------------------------------------
 * 1. Pass-2 Filtered-X DM impulse kernel (`H_diag.fits` / `H_mimo.fits`)
 * --------------------------------------------------------------------------- */

void eofpf_dm_kern_free(EOFPF_DM_KERN *k)
{
    if (k != NULL)
    {
        free(k->h_diag);
        free(k->h_mimo);
        memset(k, 0, sizeof(*k));
    }
}

int eofpf_dm_kern_load(const char *raw_path, uint32_t nbmode, EOFPF_DM_KERN *k)
{
    eofpf_dm_kern_free(k);
    if (raw_path == NULL || raw_path[0] == '\0' || strcmp(raw_path, "none") == 0)
    {
        return 0;
    }

#ifdef USE_CFITSIO
    const char *path   = (raw_path[0] == '!') ? (raw_path + 1) : raw_path;
    fitsfile   *fptr   = NULL;
    int         status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        fprintf(stderr, "mkEOFPF: cannot open dm_kernel '%s'\n", path);
        return -1;
    }
    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || naxis < 1 || naxis > 3)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    fits_get_img_size(fptr, naxis, naxes, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    size_t nelem = (size_t) naxes[0] * (size_t) ((naxis >= 2) ? naxes[1] : 1) *
                   (size_t) ((naxis == 3) ? naxes[2] : 1);
    float *raw = (float *) malloc(nelem * sizeof(float));
    if (raw == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, (LONGLONG) nelem, NULL, raw, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(raw);
        return -1;
    }

    if (naxis == 1)
    {
        uint32_t L = (uint32_t) naxes[0];
        if (L < 2 || L > EOFPF_MAX_DM_TAPS)
        {
            free(raw);
            return -1;
        }
        k->h_diag = (float *) calloc((size_t) L * nbmode, sizeof(float));
        if (k->h_diag == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < L; l++)
        {
            for (uint32_t m = 0; m < nbmode; m++)
            {
                k->h_diag[(size_t) l * nbmode + m] = isfinite(raw[l]) ? raw[l] : 0.0f;
            }
        }
        k->mode   = 1;
        k->n_dyn  = nbmode;
        k->n_taps = L;
    }
    else if (naxis == 2)
    {
        uint32_t f_nin = (uint32_t) naxes[0];
        uint32_t L     = (uint32_t) naxes[1];
        if (f_nin == 0 || L < 2 || L > EOFPF_MAX_DM_TAPS)
        {
            free(raw);
            return -1;
        }
        uint32_t n_dyn = (f_nin == 1) ? nbmode : ((f_nin < nbmode) ? f_nin : nbmode);
        k->h_diag      = (float *) calloc((size_t) L * n_dyn, sizeof(float));
        if (k->h_diag == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < L; l++)
        {
            for (uint32_t m = 0; m < n_dyn; m++)
            {
                float v = (f_nin == 1) ? raw[l] : raw[(size_t) l * f_nin + m];
                k->h_diag[(size_t) l * n_dyn + m] = isfinite(v) ? v : 0.0f;
            }
        }
        k->mode   = (f_nin == 1) ? 1 : 2;
        k->n_dyn  = n_dyn;
        k->n_taps = L;
    }
    else
    {
        uint32_t f_nin = (uint32_t) naxes[0];
        uint32_t f_ns  = (uint32_t) naxes[1];
        uint32_t L     = (uint32_t) naxes[2];
        if (f_nin == 0 || f_nin != f_ns || L < 2 || L > EOFPF_MAX_DM_TAPS)
        {
            free(raw);
            return -1;
        }
        uint32_t n_dyn = (f_nin < nbmode) ? f_nin : nbmode;
        k->h_mimo      = (float *) calloc((size_t) L * n_dyn * n_dyn, sizeof(float));
        if (k->h_mimo == NULL)
        {
            free(raw);
            return -1;
        }
        for (uint32_t l = 0; l < L; l++)
        {
            for (uint32_t r = 0; r < n_dyn; r++)
            {
                for (uint32_t c = 0; c < n_dyn; c++)
                {
                    float v = raw[(size_t) l * f_nin * f_nin + (size_t) r * f_nin + c];
                    k->h_mimo[(size_t) l * n_dyn * n_dyn + (size_t) r * n_dyn + c] =
                        isfinite(v) ? v : 0.0f;
                }
            }
        }
        k->mode   = 3;
        k->n_dyn  = n_dyn;
        k->n_taps = L;
    }
    free(raw);

    double num = 0.0;
    double den = 0.0;
    for (uint32_t l = 0; l < k->n_taps; l++)
    {
        double hm = 0.0;
        if (k->mode == 3)
        {
            for (uint32_t m = 0; m < k->n_dyn; m++)
            {
                hm += (double) k->h_mimo[(size_t) l * k->n_dyn * k->n_dyn +
                                         (size_t) m * k->n_dyn + m];
            }
        }
        else
        {
            for (uint32_t m = 0; m < k->n_dyn; m++)
            {
                hm += (double) k->h_diag[(size_t) l * k->n_dyn + m];
            }
        }
        hm  /= (double) k->n_dyn;
        num += (double) l * hm;
        den += hm;
    }
    k->centroid = (fabs(den) > 1e-6) ? (float) (num / den) : 2.0f;
    return 1;
#else
    (void) nbmode;
    return -1;
#endif
}

void eofpf_dm_kern_convolve(const EOFPF_DM_KERN *k,
                            const float         *in_fm,
                            uint32_t             n_frames,
                            uint32_t             nbmode,
                            uint32_t             t_start,
                            uint32_t             t_end,
                            float                lat_fb,
                            float               *out_fm)
{
    if (k == NULL || k->mode == 0 || t_end <= t_start || t_end > n_frames)
    {
        return;
    }
    uint32_t L      = k->n_taps;
    uint32_t n_dyn  = (k->n_dyn < nbmode) ? k->n_dyn : nbmode;
    float    lfb    = (lat_fb > 0.0f) ? lat_fb : k->centroid;
    uint32_t fb_i   = (uint32_t) lfb;
    float    fb_a   = lfb - (float) fb_i;
    float    fb_om  = 1.0f - fb_a;
    uint32_t t;

#pragma omp parallel for schedule(static) if ((t_end - t_start) * nbmode > 8192)
    for (t = t_start; t < t_end; t++)
    {
        float *dst = out_fm + (size_t) t * nbmode;
        memset(dst, 0, sizeof(float) * nbmode);

        if (k->mode == 3)
        {
            for (uint32_t l = 0; l < L; l++)
            {
                if (t < l)
                {
                    break;
                }
                const float *src = in_fm + (size_t) (t - l) * nbmode;
                const float *Hl  = k->h_mimo + (size_t) l * k->n_dyn * k->n_dyn;
                for (uint32_t r = 0; r < n_dyn; r++)
                {
                    const float *row = Hl + (size_t) r * k->n_dyn;
                    float        acc = 0.0f;
                    for (uint32_t c = 0; c < n_dyn; c++)
                    {
                        acc += row[c] * src[c];
                    }
                    dst[r] += acc;
                }
            }
        }
        else
        {
            for (uint32_t l = 0; l < L; l++)
            {
                if (t < l)
                {
                    break;
                }
                const float *src = in_fm + (size_t) (t - l) * nbmode;
                const float *hl  = k->h_diag + (size_t) l * k->n_dyn;
                for (uint32_t m = 0; m < n_dyn; m++)
                {
                    dst[m] += hl[m] * src[m];
                }
            }
        }

        if (n_dyn < nbmode && t >= fb_i + 1)
        {
            const float *s0 = in_fm + (size_t) (t - fb_i) * nbmode;
            const float *s1 = in_fm + (size_t) (t - fb_i - 1) * nbmode;
            for (uint32_t m = n_dyn; m < nbmode; m++)
            {
                dst[m] = fb_om * s0[m] + fb_a * s1[m];
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * 2. windlayers Physics-Gram Bayesian Prior (`eof_wind_study.py` C port)
 * --------------------------------------------------------------------------- */

void eofpf_phys_free(EOFPF_PHYS_PRIOR *pp)
{
    if (pp == NULL)
    {
        return;
    }
    if (pp->sxx_unit != NULL)
    {
        for (uint32_t b = 0; b < pp->nblocks; b++)
        {
            free(pp->sxx_unit[b]);
        }
        free(pp->sxx_unit);
    }
    if (pp->syx_unit != NULL)
    {
        for (uint32_t b = 0; b < pp->nblocks; b++)
        {
            free(pp->syx_unit[b]);
        }
        free(pp->syx_unit);
    }
    free(pp->fx);
    free(pp->fy);
    free(pp->psd);
    free(pp->wf_re);
    free(pp->wf_im);
    free(pp->c0_unit);
    free(pp->r1_unit);
    memset(pp, 0, sizeof(*pp));
}

int eofpf_phys_init(EOFPF_PHYS_PRIOR *pp,
                    const char       *dmmodes_path,
                    uint32_t          eff_modes,
                    uint32_t          B,
                    uint32_t          P,
                    float             pup_diam,
                    float             dm_dx,
                    float             l0_m)
{
    eofpf_phys_free(pp);
    if (dmmodes_path == NULL || dmmodes_path[0] == '\0' || strcmp(dmmodes_path, "none") == 0)
    {
        return 0;
    }

#ifdef USE_CFITSIO
    const char *path   = (dmmodes_path[0] == '!') ? (dmmodes_path + 1) : dmmodes_path;
    fitsfile   *fptr   = NULL;
    int         status = 0;
    if (fits_open_file(&fptr, path, READONLY, &status) != 0)
    {
        fprintf(stderr, "mkEOFPF: cannot open dmmodes '%s'\n", path);
        return -1;
    }
    int  naxis    = 0;
    long naxes[3] = { 1, 1, 1 };
    fits_get_img_dim(fptr, &naxis, &status);
    if (status != 0 || naxis != 3)
    {
        fits_close_file(fptr, &status);
        fprintf(stderr, "mkEOFPF: dmmodes '%s' must be 3D [nx,ny,nmodes]\n", path);
        return -1;
    }
    fits_get_img_size(fptr, 3, naxes, &status);
    if (status != 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    uint32_t nx     = (uint32_t) naxes[0];
    uint32_t ny     = (uint32_t) naxes[1];
    uint32_t n_file = (uint32_t) naxes[2];
    uint32_t npix   = nx * ny;
    uint32_t n_m    = (n_file < eff_modes) ? n_file : eff_modes;
    if (nx == 0 || ny == 0 || n_m == 0)
    {
        fits_close_file(fptr, &status);
        return -1;
    }

    size_t nelem = (size_t) npix * n_m;
    float *raw   = (float *) malloc(nelem * sizeof(float));
    if (raw == NULL)
    {
        fits_close_file(fptr, &status);
        return -1;
    }
    int anynul = 0;
    fits_read_img(fptr, TFLOAT, 1, (LONGLONG) nelem, NULL, raw, &anynul, &status);
    fits_close_file(fptr, &status);
    if (status != 0)
    {
        free(raw);
        return -1;
    }

    /* Build dual basis W = (M M^T + 1e-6 * tr/nm * I)^{-1} M (n_m x npix). */
    double *M = (double *) malloc((size_t) n_m * npix * sizeof(double));
    double *G = (double *) malloc((size_t) n_m * n_m * sizeof(double));
    if (!M || !G)
    {
        free(raw);
        free(M);
        free(G);
        return -1;
    }
    for (size_t i = 0; i < nelem; i++)
    {
        M[i] = isfinite(raw[i]) ? (double) raw[i] : 0.0;
    }
    free(raw);

    cblas_dsyrk(CblasRowMajor, CblasUpper, CblasNoTrans, (int) n_m, (int) npix, 1.0, M, (int) npix,
                0.0, G, (int) n_m);
    double tr = 0.0;
    for (uint32_t i = 0; i < n_m; i++)
    {
        tr += G[(size_t) i * n_m + i];
    }
    double reg = 1.0e-6 * ((tr > 1.0e-30) ? (tr / (double) n_m) : 1.0);
    for (uint32_t i = 0; i < n_m; i++)
    {
        G[(size_t) i * n_m + i] += reg;
    }
    lapack_int info = LAPACKE_dposv(LAPACK_ROW_MAJOR, 'U', (lapack_int) n_m, (lapack_int) npix, G,
                                    (lapack_int) n_m, M, (lapack_int) npix);
    free(G);
    if (info != 0)
    {
        fprintf(stderr, "mkEOFPF: dmmodes dual basis Cholesky failed (info=%d)\n", (int) info);
        free(M);
        return -1;
    }

    /* Prepare half-plane 2D frequency grid and von Karman PSD. */
    uint32_t max_side = (nx > ny) ? nx : ny;
    uint32_t npad     = next_pow2_u32(2u * max_side);
    if (npad < 32u)
    {
        npad = 32u;
    }
    if (npad > 128u)
    {
        npad = 128u;
    }
    uint32_t nh = npad / 2u + 1u;
    uint32_t nf = npad * nh;

    double dx = (dm_dx > 1.0e-6f) ? (double) dm_dx
                                  : (((pup_diam > 0.1f) ? (double) pup_diam : 8.0) / (double) nx);
    double l0 = (l0_m > 0.5f) ? (double) l0_m : 25.0;

    pp->n_modes  = n_m;
    pp->B        = B;
    pp->P        = P;
    pp->nblocks  = (n_m + B - 1u) / B;
    pp->npad     = npad;
    pp->nf       = nf;
    pp->dx       = dx;
    pp->l0_m     = l0;
    pp->fx       = (double *) malloc(nf * sizeof(double));
    pp->fy       = (double *) malloc(nf * sizeof(double));
    pp->psd      = (double *) malloc(nf * sizeof(double));
    pp->wf_re    = (float *) malloc((size_t) n_m * nf * sizeof(float));
    pp->wf_im    = (float *) malloc((size_t) n_m * nf * sizeof(float));
    pp->c0_unit  = (double *) calloc(n_m, sizeof(double));
    pp->r1_unit  = (double *) calloc(n_m, sizeof(double));
    pp->sxx_unit = (double **) calloc(pp->nblocks, sizeof(double *));
    pp->syx_unit = (double **) calloc(pp->nblocks, sizeof(double *));

    if (!pp->fx || !pp->fy || !pp->psd || !pp->wf_re || !pp->wf_im || !pp->c0_unit ||
        !pp->r1_unit || !pp->sxx_unit || !pp->syx_unit)
    {
        free(M);
        eofpf_phys_free(pp);
        return -1;
    }

    double df    = 1.0 / ((double) npad * dx);
    double invl2 = 1.0 / (l0 * l0);
    for (uint32_t ky = 0; ky < npad; ky++)
    {
        int    iy = (ky <= npad / 2u) ? (int) ky : ((int) ky - (int) npad);
        double fy = (double) iy * df;
        for (uint32_t kx = 0; kx < nh; kx++)
        {
            double   fx   = (double) kx * df;
            uint32_t fidx = ky * nh + kx;
            double   fr2  = fx * fx + fy * fy;
            double   wsym = (kx == 0u || kx == npad / 2u) ? 1.0 : 2.0;
            pp->fx[fidx]  = fx;
            pp->fy[fidx]  = fy;
            pp->psd[fidx] = (ky == 0u && kx == 0u) ? 0.0 : (wsym * pow(fr2 + invl2, -11.0 / 6.0));
        }
    }

    for (uint32_t blk = 0; blk < pp->nblocks; blk++)
    {
        uint32_t s0       = blk * B;
        uint32_t b        = (s0 + B <= n_m) ? B : (n_m - s0);
        uint32_t D        = P * b;
        pp->sxx_unit[blk] = (double *) calloc((size_t) D * D, sizeof(double));
        pp->syx_unit[blk] = (double *) calloc((size_t) b * D, sizeof(double));
        if (!pp->sxx_unit[blk] || !pp->syx_unit[blk])
        {
            free(M);
            eofpf_phys_free(pp);
            return -1;
        }
    }

    /* 2D FFT each dual mode W[m] into half-plane Fourier tables (parallel across modes). */
    uint32_t m;
#pragma omp parallel for schedule(static)
    for (m = 0; m < n_m; m++)
    {
        double *re = (double *) calloc((size_t) npad * npad, sizeof(double));
        double *im = (double *) calloc((size_t) npad * npad, sizeof(double));
        if (re && im)
        {
            const double *wm = M + (size_t) m * npix;
            uint32_t      cy = (ny < npad) ? ny : npad;
            uint32_t      cx = (nx < npad) ? nx : npad;
            for (uint32_t y = 0; y < cy; y++)
            {
                memcpy(re + (size_t) y * npad, wm + (size_t) y * nx, cx * sizeof(double));
            }
            fft2d_f64(re, im, npad);
            float *dr = pp->wf_re + (size_t) m * nf;
            float *di = pp->wf_im + (size_t) m * nf;
            for (uint32_t ky = 0; ky < npad; ky++)
            {
                for (uint32_t kx = 0; kx < nh; kx++)
                {
                    uint32_t fidx = ky * nh + kx;
                    dr[fidx]      = (float) re[(size_t) ky * npad + kx];
                    di[fidx]      = (float) im[(size_t) ky * npad + kx];
                }
            }
        }
        free(re);
        free(im);
    }

    free(M);
    pp->ready = 1;
    return 1;
#else
    (void) eff_modes;
    (void) B;
    (void) P;
    (void) pup_diam;
    (void) dm_dx;
    (void) l0_m;
    return -1;
#endif
}

int eofpf_phys_update_layers(EOFPF_PHYS_PRIOR *pp,
                             const float      *wl_tbl,
                             uint32_t          ncol,
                             uint32_t          nrow,
                             float             lat_fr,
                             float             frate_hz,
                             int              *wind_switched_out)
{
    if (wind_switched_out != NULL)
    {
        *wind_switched_out = 0;
    }
    if (pp == NULL || !pp->ready || wl_tbl == NULL || ncol < 16 || nrow == 0)
    {
        return 0;
    }

    double vx[EOFPF_MAX_WL_LAYER];
    double vy[EOFPF_MAX_WL_LAYER];
    double wt[EOFPF_MAX_WL_LAYER];
    int    nl    = 0;
    double w_sum = 0.0;
    uint32_t nr  = (nrow < EOFPF_MAX_WL_LAYER) ? nrow : EOFPF_MAX_WL_LAYER;

    for (uint32_t r = 0; r < nr; r++)
    {
        const float *row = wl_tbl + (size_t) r * ncol;
        if (row[15] < 0.5f)
        {
            continue;
        }
        double lx = (double) row[0];
        double ly = (double) row[1];
        double lw = (double) row[8]; /* frac_k */
        if (!isfinite(lw) || lw <= 0.005)
        {
            lw = (double) row[5]; /* pk_k fallback */
        }
        if (!isfinite(lx) || !isfinite(ly) || !isfinite(lw) || lw <= 0.005)
        {
            continue;
        }
        if (hypot(lx, ly) < 0.15)
        {
            continue; /* skip static pattern */
        }
        vx[nl] = lx;
        vy[nl] = ly;
        wt[nl] = lw;
        w_sum += lw;
        nl++;
    }
    if (nl == 0 || w_sum <= 1.0e-12)
    {
        return 0;
    }

    double vbx = 0.0;
    double vby = 0.0;
    for (int l = 0; l < nl; l++)
    {
        wt[l] /= w_sum;
        vbx   += wt[l] * vx[l];
        vby   += wt[l] * vy[l];
    }
    double vvar = 0.0;
    for (int l = 0; l < nl; l++)
    {
        double dxv = vx[l] - vbx;
        double dyv = vy[l] - vby;
        vvar += wt[l] * (dxv * dxv + dyv * dyv);
    }

    if (pp->has_layers && wind_switched_out != NULL)
    {
        double dv   = hypot(vbx - pp->vbar_x, vby - pp->vbar_y);
        double vref = hypot(pp->vbar_x, pp->vbar_y);
        double thr  = (3.0 > 0.25 * vref) ? 3.0 : (0.25 * vref);
        if (dv > thr)
        {
            *wind_switched_out = 1;
        }
    }
    pp->vbar_x   = vbx;
    pp->vbar_y   = vby;
    pp->v_std    = sqrt(vvar);
    pp->n_layers = nl;

    uint32_t P       = pp->P;
    uint32_t li      = (uint32_t) ((lat_fr >= 0.0f) ? lat_fr : 2.0f);
    double   la      = (double) lat_fr - (double) li;
    uint32_t max_lag = li + P;
    if (max_lag > 96u)
    {
        max_lag = 96u;
    }
    uint8_t need[98];
    memset(need, 0, sizeof(need));
    for (uint32_t m = 0; m < P; m++)
    {
        need[m] = 1;
    }
    for (uint32_t m = li; m <= li + P && m <= max_lag; m++)
    {
        need[m] = 1;
    }
    need[0] = 1;
    if (1u <= max_lag)
    {
        need[1] = 1;
    }

    double   dt   = (frate_hz > 1.0f) ? (1.0 / (double) frate_hz) : 0.001;
    uint32_t nf   = pp->nf;
    uint32_t B    = pp->B;
    double  *ph_r = (double *) malloc(nf * sizeof(double));
    double  *ph_i = (double *) malloc(nf * sizeof(double));
    double  *A_b  = (double *) malloc((size_t) B * nf * sizeof(double));
    double  *B_b  = (double *) malloc((size_t) B * nf * sizeof(double));
    double  *T1   = (double *) malloc((size_t) B * nf * sizeof(double));
    double  *T2   = (double *) malloc((size_t) B * nf * sizeof(double));
    double  *Rlag = (double *) malloc((size_t) (max_lag + 1u) * B * B * sizeof(double));
    if (!ph_r || !ph_i || !A_b || !B_b || !T1 || !T2 || !Rlag)
    {
        free(ph_r);
        free(ph_i);
        free(A_b);
        free(B_b);
        free(T1);
        free(T2);
        free(Rlag);
        return -1;
    }

    for (uint32_t blk = 0; blk < pp->nblocks; blk++)
    {
        uint32_t s0 = blk * B;
        uint32_t b  = (s0 + B <= pp->n_modes) ? B : (pp->n_modes - s0);
        uint32_t D  = P * b;

        for (uint32_t j = 0; j < b; j++)
        {
            const float *sr = pp->wf_re + (size_t) (s0 + j) * nf;
            const float *si = pp->wf_im + (size_t) (s0 + j) * nf;
            double      *dr = A_b + (size_t) j * nf;
            double      *di = B_b + (size_t) j * nf;
            for (uint32_t f = 0; f < nf; f++)
            {
                dr[f] = (double) sr[f];
                di[f] = (double) si[f];
            }
        }

        for (uint32_t lag = 0; lag <= max_lag; lag++)
        {
            if (!need[lag])
            {
                continue;
            }
            double tau = (double) lag * dt;
            for (uint32_t f = 0; f < nf; f++)
            {
                double cr = 0.0;
                double ci = 0.0;
                for (int l = 0; l < nl; l++)
                {
                    double arg = 2.0 * EP_PI * (pp->fx[f] * vx[l] + pp->fy[f] * vy[l]) * tau;
                    cr        += wt[l] * cos(arg);
                    ci        += wt[l] * sin(arg);
                }
                ph_r[f] = pp->psd[f] * cr;
                ph_i[f] = pp->psd[f] * ci;
            }

            uint32_t j;
#pragma omp parallel for schedule(static)
            for (j = 0; j < b; j++)
            {
                const double *ar  = A_b + (size_t) j * nf;
                const double *br  = B_b + (size_t) j * nf;
                double       *t1r = T1 + (size_t) j * nf;
                double       *t2r = T2 + (size_t) j * nf;
                for (uint32_t f = 0; f < nf; f++)
                {
                    t1r[f] = ar[f] * ph_r[f] - br[f] * ph_i[f];
                    t2r[f] = ar[f] * ph_i[f] + br[f] * ph_r[f];
                }
            }

            double *Rl = Rlag + (size_t) lag * b * b;
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) b, (int) b, (int) nf, 1.0,
                        T1, (int) nf, A_b, (int) nf, 0.0, Rl, (int) b);
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, (int) b, (int) b, (int) nf, 1.0,
                        T2, (int) nf, B_b, (int) nf, 1.0, Rl, (int) b);
        }

        /* Cache per-mode unit lag-0 and lag-1 diagonals for scale/noise calibration. */
        const double *R0 = Rlag;
        const double *R1 = (max_lag >= 1u) ? (Rlag + (size_t) b * b) : R0;
        for (uint32_t j = 0; j < b; j++)
        {
            pp->c0_unit[s0 + j] = R0[(size_t) j * b + j];
            pp->r1_unit[s0 + j] = R1[(size_t) j * b + j];
        }

        /* Assemble block-Toeplitz Sxx_unit [D x D] and Syx_unit [b x D]. */
        double *sxx = pp->sxx_unit[blk];
        double *syx = pp->syx_unit[blk];
        for (uint32_t k1 = 0; k1 < P; k1++)
        {
            for (uint32_t k2 = 0; k2 < P; k2++)
            {
                if (k2 >= k1)
                {
                    const double *Rb = Rlag + (size_t) (k2 - k1) * b * b;
                    for (uint32_t r = 0; r < b; r++)
                    {
                        memcpy(sxx + (size_t) (k1 * b + r) * D + k2 * b, Rb + (size_t) r * b,
                               b * sizeof(double));
                    }
                }
                else
                {
                    const double *Rb = Rlag + (size_t) (k1 - k2) * b * b;
                    for (uint32_t r = 0; r < b; r++)
                    {
                        for (uint32_t c = 0; c < b; c++)
                        {
                            sxx[(size_t) (k1 * b + r) * D + k2 * b + c] = Rb[(size_t) c * b + r];
                        }
                    }
                }
            }
        }
        for (uint32_t k = 0; k < P; k++)
        {
            uint32_t      l0  = (li + k <= max_lag) ? (li + k) : max_lag;
            uint32_t      l1  = (li + k + 1u <= max_lag) ? (li + k + 1u) : max_lag;
            const double *Ra  = Rlag + (size_t) l0 * b * b;
            const double *Rb  = Rlag + (size_t) l1 * b * b;
            double        om  = 1.0 - la;
            for (uint32_t r = 0; r < b; r++)
            {
                double *dst = syx + (size_t) r * D + k * b;
                for (uint32_t c = 0; c < b; c++)
                {
                    dst[c] = om * Ra[(size_t) r * b + c] + la * Rb[(size_t) r * b + c];
                }
            }
        }
    }

    free(ph_r);
    free(ph_i);
    free(A_b);
    free(B_b);
    free(T1);
    free(T2);
    free(Rlag);
    pp->has_layers = 1;
    return 1;
}

void eofpf_phys_blend_block(const EOFPF_PHYS_PRIOR *pp,
                            uint32_t                blk,
                            const double           *G_emp,
                            const double           *R_emp,
                            double                  w_emp,
                            double                  n_prior,
                            double                 *G_out,
                            double                 *R_out,
                            double                 *prior_y2_out)
{
    if (prior_y2_out != NULL)
    {
        *prior_y2_out = 0.0;
    }
    if (pp == NULL || !pp->has_layers || blk >= pp->nblocks || n_prior <= 0.0 || w_emp <= 1.0)
    {
        return;
    }
    uint32_t s0   = blk * pp->B;
    uint32_t b    = (s0 + pp->B <= pp->n_modes) ? pp->B : (pp->n_modes - s0);
    uint32_t P    = pp->P;
    uint32_t D    = P * b;
    double   invw = 1.0 / w_emp;

    double sc[512];
    double nv[512];
    double py2 = 0.0;
    for (uint32_t j = 0; j < b; j++)
    {
        double c0_e  = G_emp[(size_t) j * D + j] * invw;
        double r1_e  = (P >= 2u) ? (G_emp[(size_t) j * D + (b + j)] * invw) : (0.95 * c0_e);
        double c0_u  = pp->c0_unit[s0 + j];
        double r1_u  = pp->r1_unit[s0 + j];
        double a_max = (c0_u > 1.0e-30) ? (c0_e / c0_u) : 0.0;
        double alpha = (r1_u > 1.0e-30) ? (r1_e / r1_u) : a_max;
        if (alpha < 0.0)
        {
            alpha = 0.0;
        }
        if (alpha > a_max)
        {
            alpha = a_max;
        }
        double sig = alpha * c0_u;
        double n_j = c0_e - sig;
        if (n_j < 1.0e-3 * c0_e)
        {
            n_j = 1.0e-3 * c0_e;
        }
        sc[j] = sqrt((alpha > 1.0e-12) ? alpha : 1.0e-12);
        nv[j] = n_j;
        py2  += n_prior * (sc[j] * sc[j] * c0_u + n_j);
    }
    if (prior_y2_out != NULL)
    {
        *prior_y2_out = py2;
    }

    const double *sxx = pp->sxx_unit[blk];
    const double *syx = pp->syx_unit[blk];
    for (uint32_t k1 = 0; k1 < P; k1++)
    {
        for (uint32_t i = 0; i < b; i++)
        {
            uint32_t row_d = k1 * b + i;
            double   sci   = n_prior * sc[i];
            for (uint32_t k2 = 0; k2 < P; k2++)
            {
                for (uint32_t j = 0; j < b; j++)
                {
                    uint32_t col_d = k2 * b + j;
                    G_out[(size_t) row_d * D + col_d] +=
                        sci * sc[j] * sxx[(size_t) row_d * D + col_d];
                }
            }
            G_out[(size_t) row_d * D + row_d] += n_prior * nv[i];
            for (uint32_t j = 0; j < b; j++)
            {
                /* R_out is [D x b] = X^T Y = Syx^T. */
                R_out[(size_t) row_d * b + j] +=
                    sci * sc[j] * syx[(size_t) j * D + row_d];
            }
        }
    }
    (void) R_emp;
}

/* ---------------------------------------------------------------------------
 * 3. Extended-Memory & Regime-Weighted Gram Library (`algo/eoflin`)
 * --------------------------------------------------------------------------- */

void eofpf_gram_free(EOFPF_GRAM_STATE *gs)
{
    if (gs == NULL)
    {
        return;
    }
    for (uint32_t k = 0; k < EOFPF_REG_MAX_K; k++)
    {
        free(gs->G[k]);
        free(gs->R[k]);
    }
    free(gs->g_off);
    free(gs->r_off);
    memset(gs, 0, sizeof(*gs));
}

int eofpf_gram_init(EOFPF_GRAM_STATE *gs,
                    uint32_t          eff_modes,
                    uint32_t          B,
                    uint32_t          P,
                    uint32_t          k_max)
{
    eofpf_gram_free(gs);
    if (k_max < 1u)
    {
        k_max = 1u;
    }
    if (k_max > EOFPF_REG_MAX_K)
    {
        k_max = EOFPF_REG_MAX_K;
    }
    uint32_t nblocks = (eff_modes + B - 1u) / B;
    gs->nblocks      = nblocks;
    gs->B            = B;
    gs->P            = P;
    gs->eff_modes    = eff_modes;
    gs->k_max        = k_max;
    gs->g_off        = (size_t *) calloc(nblocks, sizeof(size_t));
    gs->r_off        = (size_t *) calloc(nblocks, sizeof(size_t));
    if (!gs->g_off || !gs->r_off)
    {
        eofpf_gram_free(gs);
        return -1;
    }
    size_t go = 0;
    size_t ro = 0;
    for (uint32_t blk = 0; blk < nblocks; blk++)
    {
        uint32_t s0    = blk * B;
        uint32_t b     = (s0 + B <= eff_modes) ? B : (eff_modes - s0);
        uint32_t D     = P * b;
        gs->g_off[blk] = go;
        gs->r_off[blk] = ro;
        go            += (size_t) D * D;
        ro            += (size_t) D * b;
    }
    gs->g_tot = go;
    gs->r_tot = ro;
    for (uint32_t k = 0; k < k_max; k++)
    {
        gs->G[k] = (double *) calloc(go, sizeof(double));
        gs->R[k] = (double *) calloc(ro, sizeof(double));
        if (!gs->G[k] || !gs->R[k])
        {
            eofpf_gram_free(gs);
            return -1;
        }
    }
    return 0;
}

void eofpf_gram_discount_all(EOFPF_GRAM_STATE *gs, double factor)
{
    if (gs == NULL || factor >= 0.999999)
    {
        return;
    }
    for (uint32_t k = 0; k < gs->k_used; k++)
    {
        gs->w_eff[k]    *= factor;
        gs->y2_sum[k]   *= factor;
        gs->zoh2_sum[k] *= factor;
        for (size_t i = 0; i < gs->g_tot; i++)
        {
            gs->G[k][i] *= factor;
        }
        for (size_t i = 0; i < gs->r_tot; i++)
        {
            gs->R[k][i] *= factor;
        }
    }
}

static double desc_dist2(const double a[EOFPF_REG_DIM], const double b[EOFPF_REG_DIM])
{
    double s = 0.0;
    for (int i = 0; i < EOFPF_REG_DIM; i++)
    {
        double d = a[i] - b[i];
        s       += d * d;
    }
    return s;
}

uint32_t eofpf_gram_select_anchor(EOFPF_GRAM_STATE *gs,
                                  const double      desc_now[EOFPF_REG_DIM],
                                  double            rlim)
{
    if (gs->k_max <= 1u)
    {
        if (gs->k_used == 0u)
        {
            gs->k_used = 1u;
            memcpy(gs->desc[0], desc_now, sizeof(double) * EOFPF_REG_DIM);
        }
        gs->cur_k = 0u;
        return 0u;
    }
    if (gs->k_used == 0u)
    {
        gs->k_used = 1u;
        memcpy(gs->desc[0], desc_now, sizeof(double) * EOFPF_REG_DIM);
        gs->cur_k = 0u;
        return 0u;
    }
    uint32_t best_k  = 0u;
    double   best_d2 = desc_dist2(desc_now, gs->desc[0]);
    for (uint32_t k = 1u; k < gs->k_used; k++)
    {
        double d2 = desc_dist2(desc_now, gs->desc[k]);
        if (d2 < best_d2)
        {
            best_d2 = d2;
            best_k  = k;
        }
    }
    if (best_d2 > rlim * rlim && gs->k_used < gs->k_max)
    {
        uint32_t nk = gs->k_used++;
        memcpy(gs->desc[nk], desc_now, sizeof(double) * EOFPF_REG_DIM);
        gs->cur_k = nk;
        return nk;
    }
    /* Gentle centroid tracking toward assigned chunk descriptor. */
    for (int i = 0; i < EOFPF_REG_DIM; i++)
    {
        gs->desc[best_k][i] = 0.90 * gs->desc[best_k][i] + 0.10 * desc_now[i];
    }
    gs->cur_k = best_k;
    return best_k;
}

void eofpf_gram_accum_block(EOFPF_GRAM_STATE *gs,
                            uint32_t          anchor,
                            uint32_t          blk,
                            const double     *dG,
                            const double     *dR,
                            double            decay)
{
    uint32_t s0   = blk * gs->B;
    uint32_t b    = (s0 + gs->B <= gs->eff_modes) ? gs->B : (gs->eff_modes - s0);
    uint32_t D    = gs->P * b;
    size_t   ng   = (size_t) D * D;
    size_t   nr   = (size_t) D * b;
    double  *gptr = gs->G[anchor] + gs->g_off[blk];
    double  *rptr = gs->R[anchor] + gs->r_off[blk];

    if (decay < 0.999999)
    {
        for (size_t i = 0; i < ng; i++)
        {
            gptr[i] = decay * gptr[i] + dG[i];
        }
        for (size_t i = 0; i < nr; i++)
        {
            rptr[i] = decay * rptr[i] + dR[i];
        }
    }
    else
    {
        for (size_t i = 0; i < ng; i++)
        {
            gptr[i] += dG[i];
        }
        for (size_t i = 0; i < nr; i++)
        {
            rptr[i] += dR[i];
        }
    }
}

void eofpf_gram_blend_block(const EOFPF_GRAM_STATE *gs,
                            uint32_t                blk,
                            const double            desc_now[EOFPF_REG_DIM],
                            double                  kappa,
                            double                  bw,
                            double                 *G_out,
                            double                 *R_out,
                            double                 *w_out)
{
    uint32_t s0 = blk * gs->B;
    uint32_t b  = (s0 + gs->B <= gs->eff_modes) ? gs->B : (gs->eff_modes - s0);
    uint32_t D  = gs->P * b;
    size_t   ng = (size_t) D * D;
    size_t   nr = (size_t) D * b;

    memset(G_out, 0, ng * sizeof(double));
    memset(R_out, 0, nr * sizeof(double));
    double w_tot = 0.0;

    if (kappa < 0.0)
    {
        kappa = 0.0;
    }
    if (kappa > 1.0)
    {
        kappa = 1.0;
    }
    double inv_2bw2 = (bw > 1.0e-6) ? (0.5 / (bw * bw)) : 0.0;

    for (uint32_t k = 0; k < gs->k_used; k++)
    {
        if (gs->w_eff[k] <= 0.0)
        {
            continue;
        }
        double wk = 1.0;
        if (gs->k_used > 1u && inv_2bw2 > 0.0)
        {
            double d2 = desc_dist2(desc_now, gs->desc[k]);
            wk        = kappa + (1.0 - kappa) * exp(-d2 * inv_2bw2);
        }
        const double *gk = gs->G[k] + gs->g_off[blk];
        const double *rk = gs->R[k] + gs->r_off[blk];
        for (size_t i = 0; i < ng; i++)
        {
            G_out[i] += wk * gk[i];
        }
        for (size_t i = 0; i < nr; i++)
        {
            R_out[i] += wk * rk[i];
        }
        w_tot += wk * gs->w_eff[k];
    }
    if (w_out != NULL)
    {
        *w_out = w_tot;
    }
}
