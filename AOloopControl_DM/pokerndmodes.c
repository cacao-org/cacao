// SPDX-FileCopyrightText: 2026 Olivier Guyon et al
//
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * @file pokerndmodes.c
 * @brief poke mode values
 */

#include "ImageStreamIO/ImageStruct.h"
#include <math.h>

#include "milk_runnable.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "statistic/statistic.h"

#include "AOloopControl_DM/AOloopControl_DM.h"

// poke mode values
//
static float *pokemval  = NULL;
static float *pokemfreq = NULL;
static float *pokempha  = NULL;


// Local variables pointers
static char  outsname[FUNCTION_PARAMETER_STRMAXLEN];
static char  modecsname[FUNCTION_PARAMETER_STRMAXLEN];
static float pokeampl = 0;
static float pokefreq = 0;

#define FPS_PARAMS(X)                                                                            \
    X(".outsname", outsname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),    \
      "output stream")                                                                           \
    X(".mode_cube", modecsname, FPTYPE_STREAMNAME, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), \
      "modes to be poked")                                                                       \
    X(".pokeampl", &pokeampl, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),      \
      "poke amplitude")                                                                          \
    X(".pokefreq", &pokefreq, FPTYPE_FLOAT32, 1, (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT),      \
      "poke frequency")

MILK_PARAMS_DEFINE(pokerndmodes, FPS_PARAMS);


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


static errno_t pokerndmodes(IMGID outimg, IMGID modecimg)
{
    static int      NBmode = 0;
    static uint64_t iter   = 0;


    if (pokemval == NULL)
    {
        printf("Initializing\n");
        NBmode = modecimg.md->size[2];
        printf("%d modes\n", NBmode);
        pokemval  = (float *) malloc(sizeof(float) * NBmode);
        pokemfreq = (float *) malloc(sizeof(float) * NBmode);
        pokempha  = (float *) malloc(sizeof(float) * NBmode);

        for (int m = 0; m < NBmode; m++)
        {
            pokemval[m]  = (pokeampl) * (1.0f - 2.0f * ran1());
            pokemfreq[m] = (pokefreq) * (0.5f + 0.5f * ran1());
            pokempha[m]  = 2.0f * M_PI * ran1();
        }
    }
    /*    else
        {
            for(int m = 0; m < NBmode; m++)
            {
                pokemval[m] += (*pokeampl) * (1.0 - 2.0 * ran1());
                pokemval[m] *= *pokemult;
            }
        }*/


    for (int m = 0; m < NBmode; m++)
    {
        pokempha[m] += pokemfreq[m] * ran1();
        pokemfreq[m] += (pokefreq) * 0.01f * (1.0f - 2.0f * ran1());

        if (pokemfreq[m] < 0.5 * (pokefreq))
        {
            pokemfreq[m] = 0.5f * (pokefreq);
        }

        if (pokemfreq[m] > (pokefreq))
        {
            pokemfreq[m] = (pokefreq);
        }

        while (pokempha[m] > 2.0f * M_PI)
        {
            pokempha[m] -= 2.0f * M_PI;
        }

        pokemval[m] = (pokeampl) *sinf(pokempha[m]);
    }

    for (uint64_t ii = 0; ii < outimg.md->size[0] * outimg.md->size[1]; ii++)
    {
        outimg.im->array.F[ii] = 0.0f;
    }
    for (int m = 0; m < NBmode; m++)
    {
        for (uint64_t ii = 0; ii < outimg.md->size[0] * outimg.md->size[1]; ii++)
        {
            outimg.im->array.F[ii] +=
                pokemval[m] *
                modecimg.im->array.F[m * outimg.md->size[0] * outimg.md->size[1] + ii];
        }
    }

    iter++;

    return RETURN_SUCCESS;
}


static MILK_HOT errno_t pokerndmodes_compute(const MILK_RUNNABLE *self)
{
    DEBUG_TRACE_FSTART();

    IMGID outimg = imgid_make_from_name(outsname);
    resolveIMGID(&outimg, ERRMODE_WARN, dcimg, dcnimg);
    if (outimg.ID == -1)
    {
        return RETURN_FAILURE;
    }

    IMGID modecimg = imgid_make_from_name(modecsname);
    resolveIMGID(&modecimg, ERRMODE_WARN, dcimg, dcnimg);
    if (modecimg.ID == -1)
    {
        return RETURN_FAILURE;
    }

    MILK_PROCINFO_INIT(self)

    MILK_PROCINFO_LOOPSTART(self)

    pokerndmodes(outimg, modecimg);
    processinfo_update_output_stream(processinfo, outimg.im, NULL);

    MILK_PROCINFO_END(self)

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


const MILK_RUNNABLE runnable_AOloopControl_DM_pokerndmodes = {
    .kind = MILK_RUNNABLE_FPS,
    .app = {
        .fps_name    = "pokerndmodes",
        .cmdkey      = "pokerndmodes",
        .description = "poke modes with random amplitudes",
        .description_long =
            "Poke deformable mirror modes with random amplitudes for "
            "system identification and response matrix calibration.",
    },
    .srcfile = __FILE__,
    MILK_PARAMS_REF(pokerndmodes),
    .compute = pokerndmodes_compute,
};
