// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file compute_masksWFSDM.c
 * @brief Compute maskswfsdm module
 */

/**
 * @file compute_straight_CM.c
 *
 */


#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include "COREMOD_arith/COREMOD_arith.h"

#include "COREMOD_iofits/COREMOD_iofits.h"

#include "info/info.h"

#include "computeCalib.h"

static char     zrespWFS[FUNCTION_PARAMETER_STRMAXLEN];
static uint32_t dmxsize;
static uint32_t dmysize;

static float dmmaskperc0;
static float dmmaskcoeff0;
static float dmmaskperc1;
static float dmmaskcoeff1;

static float wfsmaskperc0;
static float wfsmaskcoeff0;
static float wfsmaskperc1;
static float wfsmaskcoeff1;

#define FPS_PARAMS(X)                                                                    \
    X(".zrespM", zrespWFS, FPTYPE_STRING, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),  \
      "input zonal WFS RM")                                                              \
    X(".dmxsize", &dmxsize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "DM x size")                                                                       \
    X(".dmysize", &dmysize, FPTYPE_UINT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "DM y size")                                                                       \
    X(".dmmask.perc0", &dmmaskperc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,            \
      "DM mask percentile 0")                                                            \
    X(".dmmask.coeff0", &dmmaskcoeff0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,          \
      "DM mask coefficient 0")                                                           \
    X(".dmmask.perc1", &dmmaskperc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,            \
      "DM mask percentile 1")                                                            \
    X(".dmmask.coeff1", &dmmaskcoeff1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,          \
      "DM mask coefficient 1")                                                           \
    X(".wfsmask.perc0", &wfsmaskperc0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,          \
      "WFS mask percentile 0")                                                           \
    X(".wfsmask.coeff0", &wfsmaskcoeff0, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,        \
      "WFS mask coefficient 0")                                                          \
    X(".wfsmask.perc1", &wfsmaskperc1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,          \
      "WFS mask percentile 1")                                                           \
    X(".wfsmask.coeff1", &wfsmaskcoeff1, FPTYPE_FLOAT32, 1, FPFLAG_DEFAULT_INPUT,        \
      "WFS mask coefficient 1")

MILK_PARAMS_DEFINE(compute_masksWFSDM, FPS_PARAMS);


// Optional custom configuration checks.
// Runs at every configuration check loop iteration
//
static errno_t customCONFcheck()
{
    if (milk_data.fpsptr != NULL)
    {
    }

    return RETURN_SUCCESS;
}


// detailed help
static __attribute__((unused)) errno_t help_function()
{
    return RETURN_SUCCESS;
}


static MILK_HOT errno_t compute_masksWFSDM_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();


    list_image_ID();


    imageID IDzrm = image_ID(zrespWFS, dcimg, dcnimg);
    printf("IDzrm = %ld\n", IDzrm);
    uint32_t sizexWFS = dcimg[IDzrm].md[0].size[0];
    uint32_t sizeyWFS = dcimg[IDzrm].md[0].size[1];
    uint64_t sizeWFS  = sizexWFS;
    sizeWFS *= sizeyWFS;

    uint32_t NBpoke = dcimg[IDzrm].md[0].size[2];


    imageID IDWFSmap;
    create_2Dimage_ID("wfsmap", sizexWFS, sizeyWFS, &IDWFSmap);

    imageID IDDMmap;
    create_2Dimage_ID("dmmap", dmxsize, dmysize, &IDDMmap);

    imageID IDWFSmask;
    create_2Dimage_ID("wfsmask", sizexWFS, sizeyWFS, &IDWFSmask);

    imageID IDDMmask;
    create_2Dimage_ID("dmmask", dmxsize, dmysize, &IDDMmask);

    MILK_PROCINFO_INIT(self)
    MILK_PROCINFO_LOOPSTART(self)
    {
        printf("Preparing DM map ... ");
        fflush(stdout);
        for (uint32_t poke = 0; poke < NBpoke; poke++)
        {
            double rms = 0.0;
            for (uint64_t ii = 0; ii < sizeWFS; ii++)
            {
                double tmpv = dcimg[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            dcimg[IDDMmap].array.F[poke] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing WFS map ... ");
        fflush(stdout);
        for (uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            double rms = 0.0;
            for (uint32_t poke = 0; poke < NBpoke; poke++)
            {
                double tmpv = dcimg[IDzrm].array.F[poke * sizeWFS + ii];
                rms += tmpv * tmpv;
            }
            dcimg[IDWFSmap].array.F[ii] = rms;
        }
        printf("done\n");
        fflush(stdout);

        printf("Preparing DM mask ... ");
        fflush(stdout);

        // pre-filtering
        // gauss_filter(DMmap_name, "dmmapg", 5.0, 8);
        // IDDMmap1 = image_ID("dmmapg", dcimg, dcnimg);

        // (map/map1)*pow(map,0.25)

        // DMmask: select pixels
        double lim0 = dmmaskcoeff0 * img_percentile("dmmap", dmmaskperc0);

        imageID IDtmp;
        create_2Dimage_ID("_tmpdmmap", dmxsize, dmysize, &IDtmp);
        for (uint64_t ii = 0; ii < dmxsize * dmysize; ii++)
        {
            dcimg[IDtmp].array.F[ii] = dcimg[IDDMmap].array.F[ii] - lim0;
        }
        double lim = dmmaskcoeff1 * img_percentile("_tmpdmmap", dmmaskperc1);

        for (uint32_t poke = 0; poke < NBpoke; poke++)
        {
            if (dcimg[IDtmp].array.F[poke] < lim)
            {
                dcimg[IDDMmask].array.F[poke] = 0.0;
            }
            else
            {
                dcimg[IDDMmask].array.F[poke] = 1.0;
            }
        }
        delete_image_ID("_tmpdmmap", DELETE_IMAGE_ERRMODE_WARNING);
        printf("done\n");
        fflush(stdout);

        // WFSmask : select pixels
        printf("Preparing WFS mask ... ");
        fflush(stdout);

        lim0 = wfsmaskcoeff0 * img_percentile("wfsmap", wfsmaskperc0);
        create_2Dimage_ID("_tmpwfsmap", sizexWFS, sizeyWFS, &IDtmp);
        for (uint64_t ii = 0; ii < sizexWFS * sizeyWFS; ii++)
        {
            dcimg[IDtmp].array.F[ii] = dcimg[IDWFSmap].array.F[ii] - lim0;
        }
        lim = wfsmaskcoeff1 * img_percentile("_tmpwfsmap", wfsmaskperc1);

        for (uint64_t ii = 0; ii < sizeWFS; ii++)
        {
            if (dcimg[IDWFSmap].array.F[ii] < lim)
            {
                dcimg[IDWFSmask].array.F[ii] = 0.0;
            }
            else
            {
                dcimg[IDWFSmask].array.F[ii] = 1.0;
            }
        }
        delete_image_ID("_tmpwfsmap", DELETE_IMAGE_ERRMODE_WARNING);
        printf("done\n");
        fflush(stdout);
    }
    MILK_PROCINFO_END(self)


    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_computeCalib_compute_masksWFSDM = {
    .kind    = MILK_RUNNABLE_FPS,
    .app     = { .fps_name         = "compmasksWFSDM",
                 .cmdkey           = "compmasksWFSDM",
                 .description      = "compute WFS and DM masks",
                 .description_long = "Compute pupil masks for WFS and DM from response matrix "
                                     "data. Identifies active subapertures and actuators." },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(compute_masksWFSDM),
    .compute   = compute_masksWFSDM_compute,
    .confcheck = customCONFcheck,
};
EXPORT_FPSEXEC(cacao, "cacaocc-compmaskwfsdm", runnable_computeCalib_compute_masksWFSDM);
