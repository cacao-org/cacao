/**
 * @file pokerndmodes.c
 * @brief poke mode values
 */

#include "ImageStreamIO/ImageStruct.h"
#include <math.h>

#include "CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"

#include "statistic/statistic.h"

// poke mode values
//
static float *pokemval = NULL;
static float *pokemfreq = NULL;
static float *pokempha = NULL;


static FPS_APP_INFO FPS_app_info = {
    .fps_name    = "pokerndmodes",
    .cmdkey      = "pokerndmodes",
    .description = "poke modes with random amplitudes"
};

// Local variables pointers
static char outsname[
    FUNCTION_PARAMETER_STRMAXLEN];
static char modecsname[
    FUNCTION_PARAMETER_STRMAXLEN];
static float pokeampl = 0;
static float pokefreq = 0;

#define FPS_PARAMS(X) \
    X(".outsname", outsname, \
      FPTYPE_STREAMNAME, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "output stream") \
    X(".mode_cube", modecsname, \
      FPTYPE_STREAMNAME, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "modes to be poked") \
    X(".pokeampl", &pokeampl, \
      FPTYPE_FLOAT32, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "poke amplitude") \
    X(".pokefreq", &pokefreq, \
      FPTYPE_FLOAT32, 1, \
      (FPFLAG_DEFAULT_INPUT | FPFLAG_CLI_INPUT), "poke frequency")

FPS_V2_SECTION5(FPS_PARAMS)


// detailed help
static errno_t __attribute__((unused)) help_function()
{
    return RETURN_SUCCESS;
}


static errno_t pokerndmodes(IMGID outimg, IMGID modecimg)
{

    static int NBmode = 0;
    static uint64_t iter = 0;


    if(pokemval == NULL)
    {
        printf("Initializing\n");
        NBmode = modecimg.md->size[2];
        printf("%d modes\n", NBmode);
        pokemval = (float *) malloc(sizeof(float) * NBmode);
        pokemfreq = (float *) malloc(sizeof(float) * NBmode);
        pokempha = (float *) malloc(sizeof(float) * NBmode);

        for(int m = 0; m < NBmode; m++)
        {
            pokemval[m] = (pokeampl) * (1.0f - 2.0f * ran1());
            pokemfreq[m] = (pokefreq) * (0.5f + 0.5f * ran1());
            pokempha[m] = 2.0f * M_PI * ran1();
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


    for(int m = 0; m < NBmode; m++)
    {
        pokempha[m] += pokemfreq[m] * ran1();
        pokemfreq[m] += (pokefreq) * 0.01f * (1.0f - 2.0f * ran1());

        if(pokemfreq[m] < 0.5 * (pokefreq))
        {
            pokemfreq[m] = 0.5f * (pokefreq);
        }

        if(pokemfreq[m] > (pokefreq))
        {
            pokemfreq[m] = (pokefreq);
        }

        while(pokempha[m] > 2.0f * M_PI)
        {
            pokempha[m] -= 2.0f * M_PI;
        }

        pokemval[m] = (pokeampl) * sinf(pokempha[m]);
    }

    for(uint64_t ii = 0; ii < outimg.md->size[0]*outimg.md->size[1]; ii++)
    {
        outimg.im->array.F[ii] = 0.0f;
    }
    for(int m = 0; m < NBmode; m++)
    {
        for(uint64_t ii = 0; ii < outimg.md->size[0]*outimg.md->size[1]; ii++)
        {
            outimg.im->array.F[ii] += pokemval[m] * modecimg.im->array.F[m *
                                      outimg.md->size[0] * outimg.md->size[1] + ii];
        }

    }

    iter++;

    return RETURN_SUCCESS;
}


static errno_t compute_function()
{
    DEBUG_TRACE_FSTART();

    IMGID outimg = imgid_make_from_name(outsname);
    resolveIMGID(
        &outimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (outimg.ID == -1) return RETURN_FAILURE;

    IMGID modecimg = imgid_make_from_name(modecsname);
    resolveIMGID(
        &modecimg, ERRMODE_WARN,
        data.core.image,
        data.core.NB_MAX_IMAGE);
        if (modecimg.ID == -1) return RETURN_FAILURE;

    printf(" COMPUTE Flags = %ld\n", CLIcmddata.cmdsettings->flags);
    INSERT_STD_PROCINFO_COMPUTEFUNC_INIT

    // custom initialization
    printf(" COMPUTE Flags = %ld\n", CLIcmddata.cmdsettings->flags);
    if(CLIcmddata.cmdsettings->flags & CLICMDFLAG_PROCINFO)
    {
        // procinfo is accessible here
    }

    INSERT_STD_PROCINFO_COMPUTEFUNC_LOOPSTART

    pokerndmodes(outimg, modecimg);
    processinfo_update_output_stream(processinfo, outimg.im, NULL);

    INSERT_STD_PROCINFO_COMPUTEFUNC_END

    DEBUG_TRACE_FEXIT();
    return RETURN_SUCCESS;
}


#ifndef FPS_STANDALONE
static errno_t CLIfunction(void)
{
    return safe_fps_generic_CLIfunction(
        &FPS_app_info, farg, &CLIcmddata,
        my_bindings, nb_bindings,
        compute_function);
}

// Register function in CLI
errno_t
CLIADDCMD_AOloopControl_DM__pokerndmodes()
{
    safe_fps_fill_farg_examples(
        farg, my_bindings, nb_bindings);

    INSERT_STD_CLIREGISTERFUNC

    return RETURN_SUCCESS;
}
#endif

#ifdef FPS_STANDALONE
FPS_MAIN_STANDALONE_V2(
    FPS_app_info,
    FPS_PARAMS,
    compute_function)
#endif
