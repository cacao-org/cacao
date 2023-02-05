/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 *
 * AO engine uses stream data structure
 *
 *
 *
 */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "cacaocc"

// Module short description
#define MODULE_DESCRIPTION "AO loop control compute calibration"

// Application to which module belongs
#define MODULE_APPLICATION "cacao"

#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                        HEADER FILES */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#include <stdio.h>
#include <string.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_math.h>

#include "CommandLineInterface/CLIcore.h"

#include "AOloopControl/AOloopControl.h"
#include "computeCalib/computeCalib.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_tools/COREMOD_tools.h"

#include "info/info.h"

#include "compute_control_modes.h"
#include "compute_masksWFSDM.h"
#include "compute_straight_CM.h"
#include "generateRMWFS.h"
#include "computeHadamard.h"

#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                      DEFINES, MACROS */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

#define MAX_MBLOCK 20

#ifdef _OPENMP
#include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
#endif

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                  GLOBAL DATA DECLARATION */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

// extern int aoloopcontrol_var.PIXSTREAM_NBSLICES;
// extern long aoloopcontrol_var.aoconfID_pixstream_wfspixindex;;

// extern int *aoloopcontrol_var.DM_active_map;
// extern int *aoloopcontrol_var.WFS_active_map;

/* ===============================================================================================
 */
/*                                     MAIN DATA STRUCTURES */
/* ===============================================================================================
 */

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf;            // declared in AOloopControl.c
extern AOloopControl_var   aoloopcontrol_var; // declared in AOloopControl.c

// static long aoconfID_imWFS2_active[100];

/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(AOloopControl_computeCalib)

/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */

// CLI commands
//
// function CLI_checkarg used to check arguments
// CLI_checkarg ( CLI argument index , type code )
//
// type codes:
// 1: float
// 2: long
// 3: string, not existing image
// 4: existing image
// 5: string
//

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl - 5. COMPUTING CALIBRATION
 *  Compute control matrix, modes */
/* ===============================================================================================
 */
/* ===============================================================================================
 */

/** @brief CLI function for AOloopControl_mkSlavedAct */
errno_t AOloopControl_computeCalib_mkSlavedAct_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 1) + CLI_checkarg(3, 3) == 0)
    {
        AOloopControl_computeCalib_mkSlavedAct(data.cmdargtoken[1].val.string,
                                               data.cmdargtoken[2].val.numf,
                                               data.cmdargtoken[3].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_mkloDMmodes */
errno_t AOloopControl_computeCalib_mkloDMmodes_cli()
{
    if(CLI_checkarg(1, 3) + CLI_checkarg(2, 2) + CLI_checkarg(3, 2) +
            CLI_checkarg(4, 1) + CLI_checkarg(5, 1) + CLI_checkarg(6, 1) +
            CLI_checkarg(7, 1) + CLI_checkarg(8, 1) + CLI_checkarg(9, 1) +
            CLI_checkarg(10, 2) ==
            0)
    {
        AOloopControl_computeCalib_mkloDMmodes(data.cmdargtoken[1].val.string,
                                               data.cmdargtoken[2].val.numl,
                                               data.cmdargtoken[3].val.numl,
                                               data.cmdargtoken[4].val.numf,
                                               data.cmdargtoken[5].val.numf,
                                               data.cmdargtoken[6].val.numf,
                                               data.cmdargtoken[7].val.numf,
                                               data.cmdargtoken[8].val.numf,
                                               data.cmdargtoken[9].val.numf,
                                               data.cmdargtoken[10].val.numl);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_computeCalib_ComputeCM */

errno_t AOloopControl_computeCalib_ComputeCM_cli()
{
    // Try FPS implementation

    // Set data.fpsname, providing default value as first arg, and set
    // data.FPS_CMDCODE value. Default FPS name will be used if CLI process has
    // NOT been named. See code in function_parameter.c for detailed rules.

    function_parameter_getFPSargs_from_CLIfunc("compfCM");

    if(data.FPS_CMDCODE != 0)  // use FPS implementation
    {
        // set pointers to CONF and RUN functions
        data.FPS_CONFfunc = AOcontrolLoop_computeCalib_ComputeCM_FPCONF;
        data.FPS_RUNfunc  = AOcontrolLoop_computeCalib_ComputeCM_RUN;
        function_parameter_execFPScmd();
        return RETURN_SUCCESS;
    }
    else
    {
        return RETURN_FAILURE;
    }
}

/** @brief CLI function for AOloopControl_mkCM */
errno_t AOloopControl_computeCalib_mkCM_cli()
{
    function_parameter_getFPSargs_from_CLIfunc("compsCM");

    if(data.FPS_CMDCODE != 0)  // use FPS implementation
    {
        // set pointers to CONF and RUN functions
        data.FPS_CONFfunc = AOloopControl_computeCalib_mkCM_FPCONF;
        data.FPS_RUNfunc  = AOloopControl_computeCalib_mkCM_RUN;
        function_parameter_execFPScmd();
        return RETURN_SUCCESS;
    }

    // call non FPS implementation - all parameters specified at function launch

    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 1) == 0)
    {

        AOloopControl_computeCalib_mkCM(data.cmdargtoken[1].val.string,
                                        data.cmdargtoken[3].val.numf);
        return RETURN_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_mkCM */
/*errno_t AOloopControl_computeCalib_mkCM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,1)==0) {
        AOloopControl_computeCalib_mkCM(data.cmdargtoken[1].val.string,
data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf); return 0;
    }
    else return 1;
}*/

/** @brief CLI function for AOloopControl_mkModes */
errno_t AOloopControl_computeCalib_mkModes_cli()
{
    if(CLI_checkarg(1, 3) + CLI_checkarg(2, 2) + CLI_checkarg(3, 2) +
            CLI_checkarg(4, 1) + CLI_checkarg(5, 1) + CLI_checkarg(6, 1) +
            CLI_checkarg(7, 1) + CLI_checkarg(8, 1) + CLI_checkarg(9, 1) +
            CLI_checkarg(10, 2) + CLI_checkarg(11, 2) + CLI_checkarg(12, 1) ==
            0)
    {
        AOloopControl_computeCalib_mkModes(data.cmdargtoken[1].val.string,
                                           data.cmdargtoken[2].val.numl,
                                           data.cmdargtoken[3].val.numl,
                                           data.cmdargtoken[4].val.numf,
                                           data.cmdargtoken[5].val.numf,
                                           data.cmdargtoken[6].val.numf,
                                           data.cmdargtoken[7].val.numf,
                                           data.cmdargtoken[8].val.numf,
                                           data.cmdargtoken[9].val.numf,
                                           data.cmdargtoken[10].val.numl,
                                           data.cmdargtoken[11].val.numl,
                                           data.cmdargtoken[12].val.numf,
                                           "NULL");

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_mkModes_Simple */
errno_t AOloopControl_computeCalib_mkModes_Simple_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 2) + CLI_checkarg(3, 2) +
            CLI_checkarg(4, 1) ==
            0)
    {
        AOloopControl_computeCalib_mkModes_Simple(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numl,
            data.cmdargtoken[3].val.numl,
            data.cmdargtoken[4].val.numf);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_computeCM */
errno_t AOloopControl_computeCalib_computeCM_cli()
{
    if(CLI_checkarg(1, 2) + CLI_checkarg(2, 4) + CLI_checkarg(3, 3) +
            CLI_checkarg(4, 1) + CLI_checkarg(5, 2) + CLI_checkarg(6, 1) ==
            0)
    {
        AOloopControl_computeCalib_compute_ControlMatrix(
            LOOPNUMBER,
            data.cmdargtoken[1].val.numl,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            "evecM",
            data.cmdargtoken[4].val.numf,
            data.cmdargtoken[5].val.numl,
            data.cmdargtoken[6].val.numf);
        save_fits("evecM", "evecM.fits");
        delete_image_ID("evecM", DELETE_IMAGE_ERRMODE_WARNING);
        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_loadCM */
errno_t AOloopControl_computeCalib_loadCM_cli()
{
    if(CLI_checkarg(1, 3) == 0)
    {
        AOloopControl_computeCalib_loadCM(LOOPNUMBER,
                                          data.cmdargtoken[1].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_mkHadamardModes */
errno_t AOloopControl_computeCalib_mkHadamardModes_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 3) == 0)
    {
        AOloopControl_computeCalib_mkHadamardModes(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_Hadamard_decodeRM */
errno_t AOloopControl_computeCalib_Hadamard_decodeRM_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 4) + CLI_checkarg(3, 4) +
            CLI_checkarg(4, 3) ==
            0)
    {
        AOloopControl_computeCalib_Hadamard_decodeRM(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_mkCalib_map_mask */
errno_t AOloopControl_computeCalib_mkCalib_map_mask_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 3) + CLI_checkarg(3, 3) +
            CLI_checkarg(4, 1) + CLI_checkarg(5, 1) + CLI_checkarg(6, 1) +
            CLI_checkarg(7, 1) + CLI_checkarg(8, 1) + CLI_checkarg(9, 1) +
            CLI_checkarg(10, 1) + CLI_checkarg(11, 1) ==
            0)
    {
        AOloopControl_computeCalib_mkCalib_map_mask(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.numf,
            data.cmdargtoken[5].val.numf,
            data.cmdargtoken[6].val.numf,
            data.cmdargtoken[7].val.numf,
            data.cmdargtoken[8].val.numf,
            data.cmdargtoken[9].val.numf,
            data.cmdargtoken[10].val.numf,
            data.cmdargtoken[11].val.numf);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_Process_zrespM */
errno_t AOloopControl_computeCalib_Process_zrespM_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 4) + CLI_checkarg(3, 3) +
            CLI_checkarg(4, 3) + CLI_checkarg(5, 3) ==
            0)
    {
        AOloopControl_computeCalib_Process_zrespM(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string,
            data.cmdargtoken[5].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_ProcessZrespM */
errno_t AOloopControl_computeCalib_ProcessZrespM_cli()
{
    if(CLI_checkarg(1, 3) + CLI_checkarg(2, 3) + CLI_checkarg(3, 3) +
            CLI_checkarg(4, 3) + CLI_checkarg(5, 1) + CLI_checkarg(6, 2) ==
            0)
    {
        AOloopControl_computeCalib_ProcessZrespM_medianfilt(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string,
            data.cmdargtoken[5].val.numf,
            data.cmdargtoken[6].val.numl);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/** @brief CLI function for AOloopControl_compute_CombinedControlMatrix */
errno_t AOloopControl_computeCalib_compute_CombinedControlMatrix_cli()
{
    if(CLI_checkarg(1, 4) + CLI_checkarg(2, 4) + CLI_checkarg(3, 4) +
            CLI_checkarg(4, 4) + CLI_checkarg(5, 3) + CLI_checkarg(6, 3) ==
            0)
    {
        AOloopControl_computeCalib_compute_CombinedControlMatrix(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.string,
            data.cmdargtoken[3].val.string,
            data.cmdargtoken[4].val.string,
            data.cmdargtoken[5].val.string,
            data.cmdargtoken[6].val.string);

        return CLICMD_SUCCESS;
    }
    else
    {
        return CLICMD_INVALID_ARG;
    }
}

/* ===============================================================================================
 */
/* ===============================================================================================
 */
/*                                    FUNCTIONS SOURCE CODE */
/* ===============================================================================================
 */
/* ===============================================================================================
 */
/** @name AOloopControl_IOtools functions */

static errno_t init_module_CLI()
{

    /* ===============================================================================================
    */
    /* ===============================================================================================
    */
    /** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION */
    /* ===============================================================================================
    */
    /* ===============================================================================================
    */

    RegisterCLIcommand("aolmkH",
                       __FILE__,
                       AOloopControl_computeCalib_mkHadamardModes_cli,
                       "make Hadamard poke sequence",
                       "<DM pixel mask> <output fname [string]>",
                       "aolmkH dm50mask h50pokec",
                       "long AOloopControl_computeCalib_mkHadamardModes(char "
                       "*DMmask_name, char outname)");

    RegisterCLIcommand(
        "aolHaddec",
        __FILE__,
        AOloopControl_computeCalib_Hadamard_decodeRM_cli,
        "decode Hadamard matrix",
        "<input RM> <Hadamard matrix> <DMpix index frame> <output RM>",
        "aolHaddec imRMh Hmat pixiind imRM",
        "long AOloopControl_computeCalib_Hadamard_decodeRM(char *inname, char "
        "*Hmatname, char "
        "*indexname, char *outname)");

    RegisterCLIcommand("aolmkslact",
                       __FILE__,
                       AOloopControl_computeCalib_mkSlavedAct_cli,
                       "create slaved actuators map based on proximity",
                       "<maskRM> <distance> <outslact>",
                       "aolmkslact DMmaskRM 2.5 DMslavedact",
                       "long AOloopControl_computeCalib_mkSlavedAct(char "
                       "*IDmaskRM_name, float pixrad, char *IDout_name)");

    RegisterCLIcommand("aolmklodmmodes",
                       __FILE__,
                       AOloopControl_computeCalib_mkloDMmodes_cli,
                       "make low order DM modes",
                       "<output modes> <sizex> <sizey> <max CPA> <delta CPA> "
                       "<cx> <cy> <r0> <r1> <masking mode>",
                       "aolmklodmmodes modes 50 50 5.0 0.8 1",
                       "long AOloopControl_computeCalib_mkloDMmodes(char "
                       "*ID_name, long msizex, long msizey, float "
                       "CPAmax, float deltaCPA, double xc, double yx, double "
                       "r0, double r1, int MaskMode)");

    RegisterCLIcommand("aolcomputeCM",
                       __FILE__,
                       AOloopControl_computeCalib_ComputeCM_cli,
                       "compute control matrix, high level function FPS only",
                       "<CODE>",
                       "aolcomputeCM _CONFRUN_",
                       "FPS only");

    RegisterCLIcommand("aolRM2CM",
                       __FILE__,
                       AOloopControl_computeCalib_mkCM_cli,
                       "make control matrix from response matrix",
                       "<RMimage> <SVDlim>",
                       "aolRM2CM respM contrM 0.1",
                       "long AOloopControl_computeCalib_mkCM(char *respm_name, "
                       "char *cm_name, float SVDlim)");

    RegisterCLIcommand("aolmkmodes",
                       __FILE__,
                       AOloopControl_computeCalib_mkModes_cli,
                       "make control modes",
                       "<output modes> <sizex> <sizey> <max CPA> <delta CPA> "
                       "<cx> <cy> <r0> <r1> <masking mode> <block> <SVDlim>",
                       "aolmkmodes modes 50 50 5.0 0.8 1 2 0.01",
                       "long AOloopControl_computeCalib_mkModes(char *ID_name, "
                       "long msizex, long msizey, float CPAmax, float "
                       "deltaCPA, double xc, double yx, double r0, double r1, "
                       "int MaskMode, int BlockNB, float SVDlim, char "
                       "*stagedir)");

    RegisterCLIcommand(
        "aolmkmodesM",
        __FILE__,
        AOloopControl_computeCalib_mkModes_Simple_cli,
        "make control modes in modal DM mode",
        "<input WFS modes> <NBmblock> <block> <SVDlim>",
        "aolmkmodesM wfsallmodes 5 2 0.01",
        "long AOloopControl_computeCalib_mkModes_Simple(char *IDin_name, long "
        "NBmblock, long Cmblock, float SVDlim)");

    RegisterCLIcommand(
        "aolRMmkmasks",
        __FILE__,
        AOloopControl_computeCalib_mkCalib_map_mask_cli,
        "make sensitivity maps and masks from response matrix",
        "<zrespm fname [string]> <output WFS response map fname [string]>  "
        "<output DM response map fname [string]> "
        "<percentile low> <coefficient low> <percentile high> <coefficient "
        "high>",
        "aolRMmkmasks .. 0.2 1.0 0.5 0.3 0.05 1.0 0.65 0.3",
        "int AOloopControl_computeCalib_mkCalib_map_mask(long loop, char "
        "*zrespm_name, char *WFSmap_name, char "
        "*DMmap_name, float dmmask_perclow, float dmmask_coefflow, float "
        "dmmask_perchigh, float dmmask_coeffhigh, "
        "float wfsmask_perclow, float wfsmask_coefflow, float "
        "wfsmask_perchigh, float wfsmask_coeffhigh)");

    RegisterCLIcommand(
        "aolproczrm",
        __FILE__,
        AOloopControl_computeCalib_Process_zrespM_cli,
        "process zonal resp mat, WFS ref -> DM and WFS response maps",
        "<input zrespm fname [string]> <input WFS ref fname [string]> <output "
        "zrespm [string]> <output "
        "WFS response map fname [string]>  <output DM response map fname "
        "[string]>",
        "aolproczrm zrespmat0 wfsref0 zrespm wfsmap dmmap",
        "int AOloopControl_computeCalib_Process_zrespM(long loop, char "
        "*IDzrespm0_name, char "
        "*IDwfsref_name, char *IDzrespm_name, char *WFSmap_name, char "
        "*DMmap_name)");

    RegisterCLIcommand(
        "aolcleanzrm",
        __FILE__,
        AOloopControl_computeCalib_ProcessZrespM_cli,
        "clean zonal resp mat, WFS ref, DM and WFS response maps",
        "<zrespm fname [string]> <output WFS ref fname [string]>  <output WFS "
        "response map fname "
        "[string]>  <output DM response map fname [string]> <RM ampl [um]>",
        "aolcleanzrm zrm wfsref wfsmap dmmap 0.05",
        "int AOloopControl_computeCalib_ProcessZrespM_medianfilt(long loop, "
        "char *zrespm_name, char "
        "*WFSref0_name, char *WFSmap_name, char *DMmap_name, double ampl)");

    RegisterCLIcommand(
        "aolcompcmatc",
        __FILE__,
        AOloopControl_computeCalib_compute_CombinedControlMatrix_cli,
        "compute combined control matrix",
        "<modal control matrix> <modes> <wfs mask> <dm mask> <combined cmat> "
        "<combined cmat, only active elements>",
        "aolcompcmatc cmat fmodes wfsmask dmmask cmatc cmatcact",
        "long AOloopControl_computeCalib_compute_CombinedControlMatrix(char "
        "*IDcmat_name, char *IDmodes_name, char* "
        "IDwfsmask_name, char *IDdmmask_name, char *IDcmatc_name, char "
        "*IDcmatc_active_name)");

    RegisterCLIcommand(
        "aolcmmake",
        __FILE__,
        AOloopControl_computeCalib_computeCM_cli,
        "make control matrix",
        "<NBmodes removed> <RespMatrix> <ContrMatrix> <beta> <nbremovedstep> "
        "<eigenvlim>",
        "aolcmmake 8 respm cmat",
        "int AOloopControl_computeCalib_compute_ControlMatrix(long loop, long "
        "NB_MODE_REMOVED, char *ID_Rmatrix_name, "
        "char *ID_Cmatrix_name, char *ID_VTmatrix_name, double Beta, long "
        "NB_MODE_REMOVED_STEP, float eigenvlim)");

    RegisterCLIcommand("aolloadcm",
                       __FILE__,
                       AOloopControl_computeCalib_loadCM_cli,
                       "load new control matrix from file",
                       "<fname>",
                       "aolloadcm cm32.fits",
                       "long AOloopControl_computeCalib_loadCM(long loop, "
                       "const char *CMfname)");

    CLIADDCMD_cacao_computeCalib__compute_control_modes();

    CLIADDCMD_AOloopControl_computeCalib__compsCM();

    CLIADDCMD_AOloopControl_computeCalib__compmasksWFSDM();

    CLIADDCMD_AOloopControl_computeCalib__generateRMWFS();

    CLIADDCMD_AOloopControl_computeCalib__mkHadamard();


    // add atexit functions here
    // atexit((void*) myfunc);

    return RETURN_SUCCESS;
}

/**
 * @ingroup FPSconf
 *
 * @brief Configuration for compute calib FPS
 *
 * Calls #FPS_SETUP_INIT to perform initialization
 *
 * @{
 */
errno_t AOcontrolLoop_computeCalib_ComputeCM_FPCONF()
{
    // ===========================
    // SETUP FPS
    // ===========================
    FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);
    fps_add_processinfo_entries(&fps);

    // int SMfd_zRMacqu = -1;
    // int SMfd_loRMacqu = -1;
    // int SMfd_DMcomb = -1;

    // ===========================
    // ALLOCATE FPS ENTRIES IF NOT ALREADY EXIST
    // ===========================
    void *pNull = NULL;
    // __attribute__((unused)) uint64_t FPFLAG;

    long loop_default[4] = {0, 0, 10, 0};
    //__attribute__((unused)) long fpi_loop =
    function_parameter_add_entry(&fps,
                                 ".AOloopindex",
                                 "loop index",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &loop_default,
                                 NULL);

    double SVDlim_default[4] = {0.01, 0.000001, 1.0, 0.01};
    //__attribute__((unused)) long fpi_SVDlim =
    function_parameter_add_entry(&fps,
                                 ".SVDlim",
                                 "RM poke amplitude",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &SVDlim_default,
                                 NULL);

    double CPAmax_default[4] = {10.0, 0.1, 100.0, 10.0};
    //__attribute__((unused)) long fpi_CPAmax =
    function_parameter_add_entry(&fps,
                                 ".CPAmax",
                                 "maximum controlled CPA",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &CPAmax_default,
                                 NULL);

    double deltaCPA_default[4] = {0.8, 0.1, 2.0, 0.8};
    //__attribute__((unused)) long fpi_deltaCPA =
    function_parameter_add_entry(&fps,
                                 ".deltaCPA",
                                 "delta CPA",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &deltaCPA_default,
                                 NULL);

    double alignCX_default[4] = {0.0, 0.0, 2000.0, 0.0};
    //__attribute__((unused)) long fpi_alignCX =
    function_parameter_add_entry(&fps,
                                 ".align.CX",
                                 "DM mask center X (if no DMmaskRM)",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &alignCX_default,
                                 NULL);

    double alignCY_default[4] = {0.0, 0.0, 2000.0, 0.0};
    //__attribute__((unused)) long fpi_alignCY =
    function_parameter_add_entry(&fps,
                                 ".align.CY",
                                 "DM mask center Y (if no DMmaskRM)",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &alignCY_default,
                                 NULL);

    double alignID_default[4] = {0.0, 0.0, 2000.0, 0.0};
    //__attribute__((unused)) long fpi_alignID =
    function_parameter_add_entry(&fps,
                                 ".align.ID",
                                 "mask I.D. (if no DMmaskRM)",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &alignID_default,
                                 NULL);

    double alignOD_default[4] = {0.0, 0.0, 2000.0, 0.0};
    //__attribute__((unused)) long fpi_alignOD =
    function_parameter_add_entry(&fps,
                                 ".align.OD",
                                 "mask O.D. (if no DMmaskRM)",
                                 FPTYPE_FLOAT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &alignOD_default,
                                 NULL);

    long DMxsize_default[4] = {10, 1, 2000, 10};
    //__attribute__((unused)) long fpi_DMxsize =
    function_parameter_add_entry(&fps,
                                 ".DMxsize",
                                 "DM x size",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &DMxsize_default,
                                 NULL);

    long DMysize_default[4] = {10, 1, 2000, 10};
    //__attribute__((unused)) long fpi_DMysize =
    function_parameter_add_entry(&fps,
                                 ".DMysize",
                                 "DM y size",
                                 FPTYPE_INT64,
                                 FPFLAG_DEFAULT_INPUT,
                                 &DMysize_default,
                                 NULL);

    // FPS

    long fpi_FPS_zRMacqu = 0;
    function_parameter_add_entry(&fps,
                                 ".FPS_zRMacqu",
                                 "FPS zonal RM acquisition",
                                 FPTYPE_FPSNAME,
                                 FPFLAG_DEFAULT_INPUT, // | FPFLAG_FPS_RUN_REQUIRED,
                                 pNull,
                                 &fpi_FPS_zRMacqu);
    FUNCTION_PARAMETER_STRUCT FPS_zRMacqu = {0};
    fps.parray[fpi_FPS_zRMacqu].info.fps.FPSNBparamMAX = 0;

    long fpi_FPS_loRMacqu = 0;
    function_parameter_add_entry(&fps,
                                 ".FPS_loRMacqu",
                                 "FPS low order modal RM acquisition",
                                 FPTYPE_FPSNAME,
                                 FPFLAG_DEFAULT_INPUT, // | FPFLAG_FPS_RUN_REQUIRED,
                                 pNull,
                                 &fpi_FPS_loRMacqu);
    FUNCTION_PARAMETER_STRUCT FPS_loRMacqu = {0};
    fps.parray[fpi_FPS_loRMacqu].info.fps.FPSNBparamMAX = 0;

    long fpi_FPS_DMcomb = 0;
    function_parameter_add_entry(&fps,
                                 ".FPS_DMcomb",
                                 "FPS DMcomb",
                                 FPTYPE_FPSNAME,
                                 FPFLAG_DEFAULT_INPUT, // | FPFLAG_FPS_RUN_REQUIRED,
                                 pNull,
                                 &fpi_FPS_DMcomb);
    FUNCTION_PARAMETER_STRUCT FPS_DMcomb = {0};

    fps.parray[fpi_FPS_DMcomb].info.fps.FPSNBparamMAX = 0;

    //__attribute__((unused)) long fpi_fname_dmslaved =
    function_parameter_add_entry(&fps,
                                 ".DMslaved",
                                 "DM slaved actuators",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_zrespM =
    function_parameter_add_entry(&fps,
                                 ".zrespM",
                                 "Zonal response matrix",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_dmmaskRM =
    function_parameter_add_entry(&fps,
                                 ".DMmaskRM",
                                 "actuators directly controlled",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_WFSmask =
    function_parameter_add_entry(&fps,
                                 ".WFSmask",
                                 "WFS mask",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_loRM =
    function_parameter_add_entry(&fps,
                                 ".loRM",
                                 "low order modal response matrix",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_loRMmodes =
    function_parameter_add_entry(&fps,
                                 ".loRMmodes",
                                 "low order RM modes",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT |
                                 FPFLAG_FILE_RUN_REQUIRED,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_extrablockM =
    function_parameter_add_entry(&fps,
                                 ".option.extrablockM",
                                 "extra modes block",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);

    //__attribute__((unused)) long fpi_fname_exclmodes =
    function_parameter_add_entry(&fps,
                                 ".option.exclmodes",
                                 "excluded modes",
                                 FPTYPE_FITSFILENAME,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);

    // Update Actions
    long fpi_update_RMfiles = 0;
    function_parameter_add_entry(&fps,
                                 ".upRMfiles",
                                 "update RM files",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 &fpi_update_RMfiles);

    long fpi_update_align = 0;
    function_parameter_add_entry(&fps,
                                 ".upAlign",
                                 "update default align (if no DMmaskRM)",
                                 FPTYPE_ONOFF,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 &fpi_update_align);

    // settings for output files and dir

    // long fpi_out_dirname      =
    function_parameter_add_entry(&fps,
                                 ".out.dirname",
                                 "output directory",
                                 FPTYPE_DIRNAME,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);
    //(void) fpi_out_dirname;

    //__attribute__((unused)) long fpi_out_label      =
    function_parameter_add_entry(&fps,
                                 ".out.label",
                                 "output label",
                                 FPTYPE_STRING,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);

    // long fpi_out_timestring = 0;
    function_parameter_add_entry(&fps,
                                 ".out.timestring",
                                 "output timestring",
                                 FPTYPE_STRING,
                                 FPFLAG_DEFAULT_INPUT,
                                 pNull,
                                 NULL);
    //    (void) fpi_out_timestring;

    // External scripts (post)
    /*    long fpi_exec_logdata =
          function_parameter_add_entry(&fps, ".log2fs",
                                       "log to filesystem",
                                       FPTYPE_EXECFILENAME,
     FPFLAG_DEFAULT_INPUT, pNull); (void) fpi_exec_logdata;
    */

    if(!fps.localstatus & FPS_LOCALSTATUS_CONFLOOP)  // stop fps
    {
        return RETURN_SUCCESS;
    }

    // =====================================
    // PARAMETER LOGIC AND UPDATE LOOP
    // =====================================
    while(fps.localstatus & FPS_LOCALSTATUS_CONFLOOP)
    {
        usleep(50);

        if(function_parameter_FPCONFloopstep(&fps) ==
                1) // Apply logic if update is needed
        {
            //
            //  Connect to aux FPS
            //
            if(fps.parray[fpi_FPS_zRMacqu].info.fps.FPSNBparamMAX < 1)
            {
                functionparameter_ConnectExternalFPS(&fps,
                                                     fpi_FPS_zRMacqu,
                                                     &FPS_zRMacqu);
                //                FPS_zRMacqu_NBparam =
                //                function_parameter_struct_connect(fps.parray[fpi_FPS_zRMacqu].val.string[0],
                //                &FPS_zRMacqu, FPSCONNECT_SIMPLE);
            }
            if(fps.parray[fpi_FPS_loRMacqu].info.fps.FPSNBparamMAX < 1)
            {
                functionparameter_ConnectExternalFPS(&fps,
                                                     fpi_FPS_loRMacqu,
                                                     &FPS_loRMacqu);
                // FPS_loRMacqu_NBparam =
                // function_parameter_struct_connect(fps.parray[fpi_FPS_loRMacqu].val.string[0],
                // &FPS_loRMacqu, FPSCONNECT_SIMPLE);
            }
            if(fps.parray[fpi_FPS_DMcomb].info.fps.FPSNBparamMAX < 1)
            {
                functionparameter_ConnectExternalFPS(&fps,
                                                     fpi_FPS_DMcomb,
                                                     &FPS_DMcomb);
                // FPS_DMcomb_NBparam =
                // function_parameter_struct_connect(fps.parray[fpi_FPS_DMcomb].val.string[0],
                // &FPS_DMcomb, FPSCONNECT_SIMPLE);
            }

            // Update RM files
            if(fps.parray[fpi_update_RMfiles].fpflag & FPFLAG_ONOFF)
            {

                if(fps.parray[fpi_FPS_zRMacqu].info.fps.FPSNBparamMAX > 0)
                {
                    char datadir[FUNCTION_PARAMETER_STRMAXLEN];
                    char fname[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(
                        datadir,
                        functionparameter_GetParamPtr_STRING(&FPS_zRMacqu,
                                ".conf.datadir"),
                        FUNCTION_PARAMETER_STRMAXLEN);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/dmslaved.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".DMslaved",
                                                           fname);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/zrespM_mn.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".zrespM",
                                                           fname);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/dmmask_mksl.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".DMmaskRM",
                                                           fname);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/wfsmask_mkm.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".WFSmask",
                                                           fname);
                }

                if(fps.parray[fpi_FPS_loRMacqu].info.fps.FPSNBparamMAX > 0)
                {
                    char datadir[FUNCTION_PARAMETER_STRMAXLEN];
                    char fname[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(
                        datadir,
                        functionparameter_GetParamPtr_STRING(&FPS_loRMacqu,
                                ".conf.datadir"),
                        FUNCTION_PARAMETER_STRMAXLEN);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/respM.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".loRM",
                                                           fname);

                    SNPRINTF_CHECK(fname,
                                   FUNCTION_PARAMETER_STRMAXLEN,
                                   "%s/RMpokeCube.fits",
                                   datadir);
                    functionparameter_SetParamValue_STRING(&fps,
                                                           ".loRMmodes",
                                                           fname);
                }

                // set back to OFF
                fps.parray[fpi_update_RMfiles].fpflag &= ~FPFLAG_ONOFF;
            }

            // update align params for auto mask
            if(fps.parray[fpi_update_align].fpflag & FPFLAG_ONOFF)
            {
                if(fps.parray[fpi_FPS_DMcomb].info.fps.FPSNBparamMAX > 0)
                {
                    int DMxsize =
                        functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                ".DMxsize");
                    int DMysize =
                        functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                ".DMysize");
                    __attribute__((unused)) int DMMODE =
                        functionparameter_GetParamValue_INT64(&FPS_DMcomb,
                                ".DMMODE");

                    float cx = 0.5 * DMxsize - 0.5;
                    float cy = 0.5 * DMysize - 0.5;
                    float od = 0.45 * DMxsize;
                    float id = 0.05 * DMxsize;

                    functionparameter_SetParamValue_INT64(&fps,
                                                          ".DMxsize",
                                                          DMxsize);
                    functionparameter_SetParamValue_INT64(&fps,
                                                          ".DMysize",
                                                          DMysize);
                    functionparameter_SetParamValue_FLOAT64(&fps,
                                                            ".align.CX",
                                                            cx);
                    functionparameter_SetParamValue_FLOAT64(&fps,
                                                            ".align.CY",
                                                            cy);
                    functionparameter_SetParamValue_FLOAT64(&fps,
                                                            ".align.OD",
                                                            od);
                    functionparameter_SetParamValue_FLOAT64(&fps,
                                                            ".align.ID",
                                                            id);
                }
                fps.parray[fpi_update_align].fpflag &= ~FPFLAG_ONOFF;
            }

            // update align parameters

            functionparameter_CheckParametersAll(
                &fps); // check all parameter values
        }
    }

    function_parameter_FPCONFexit(&fps);

    if(fps.parray[fpi_FPS_zRMacqu].info.fps.FPSNBparamMAX > 0)
    {
        function_parameter_struct_disconnect(&FPS_zRMacqu);
    }

    if(fps.parray[fpi_FPS_loRMacqu].info.fps.FPSNBparamMAX > 0)
    {
        function_parameter_struct_disconnect(&FPS_loRMacqu);
    }

    if(fps.parray[fpi_FPS_DMcomb].info.fps.FPSNBparamMAX > 0)
    {
        function_parameter_struct_disconnect(&FPS_DMcomb);
    }

    return RETURN_SUCCESS;
}
/** @} */ // end of group

/**
 * @ingroup FPSrun
 *
 * @brief Compute control matrix
 *
 *
 * @{
 */
errno_t AOcontrolLoop_computeCalib_ComputeCM_RUN()
{
    // ===========================
    // CONNECT TO FPS
    // ===========================
    /*  int SMfd = -1;
    FUNCTION_PARAMETER_STRUCT fps;

    if(function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_RUN, &SMfd)
    == -1) { printf("ERROR: fps \"%s\" does not exist -> running without FPS
    interface\n", fpsname); return RETURN_FAILURE;
    }*/

    FPS_SETUP_INIT(data.FPS_name, data.FPS_CMDCODE);

    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================
    __attribute__((unused)) long loop =
        functionparameter_GetParamValue_INT64(&fps, ".AOloopindex");
    float SVDlim   = functionparameter_GetParamValue_FLOAT64(&fps, ".SVDlim");
    float CPAmax   = functionparameter_GetParamValue_FLOAT64(&fps, ".CPAmax");
    float deltaCPA = functionparameter_GetParamValue_FLOAT64(&fps, ".deltaCPA");

    float align_CX = functionparameter_GetParamValue_FLOAT64(&fps, ".align.CX");
    float align_CY = functionparameter_GetParamValue_FLOAT64(&fps, ".align.CY");
    float align_ID = functionparameter_GetParamValue_FLOAT64(&fps, ".align.ID");
    float align_OD = functionparameter_GetParamValue_FLOAT64(&fps, ".align.OD");

    long DMxsize = functionparameter_GetParamValue_INT64(&fps, ".DMxsize");
    long DMysize = functionparameter_GetParamValue_INT64(&fps, ".DMysize");

    char fname_dmslaved[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_dmslaved,
            functionparameter_GetParamPtr_STRING(&fps, ".DMslaved"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_dmslaved, "dmslaved", LOADFITS_ERRMODE_WARNING, NULL);

    char fname_zrespM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_zrespM,
            functionparameter_GetParamPtr_STRING(&fps, ".zrespM"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_zrespM, "zrespM", LOADFITS_ERRMODE_WARNING, NULL);

    char fname_DMmaskRM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_DMmaskRM,
            functionparameter_GetParamPtr_STRING(&fps, ".DMmaskRM"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_DMmaskRM, "dmmaskRM", LOADFITS_ERRMODE_WARNING, NULL);

    char fname_WFSmask[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_WFSmask,
            functionparameter_GetParamPtr_STRING(&fps, ".WFSmask"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_WFSmask, "wfsmask", LOADFITS_ERRMODE_WARNING, NULL);

    char fname_loRM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_loRM,
            functionparameter_GetParamPtr_STRING(&fps, ".loRM"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_loRM, "RMMresp", LOADFITS_ERRMODE_WARNING, NULL);

    char fname_loRMmodes[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_loRMmodes,
            functionparameter_GetParamPtr_STRING(&fps, ".loRMmodes"),
            FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_loRMmodes, "RMMmodes", LOADFITS_ERRMODE_WARNING, NULL);

    char outdirname[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(outdirname,
            functionparameter_GetParamPtr_STRING(&fps, ".out.dirname"),
            FUNCTION_PARAMETER_STRMAXLEN);

    EXECUTE_SYSTEM_COMMAND("mkdir -p ./%s/mkmodestmp", fps.md->datadir);

    list_image_ID();

    // Get time
    time_t     tnow;
    struct tm *tmnow;
    char       datestring[200];

    time(&tnow);
    tmnow = gmtime(&tnow);

    printf("TIMESTRING:  %04d%02d%02dT%02d%02d%02d\n",
           1900 + tmnow->tm_year,
           1 + tmnow->tm_mon,
           tmnow->tm_mday,
           tmnow->tm_hour,
           tmnow->tm_min,
           tmnow->tm_sec);
    sprintf(datestring,
            "%04d%02d%02dT%02d%02d%02d",
            1900 + tmnow->tm_year,
            1 + tmnow->tm_mon,
            tmnow->tm_mday,
            tmnow->tm_hour,
            tmnow->tm_min,
            tmnow->tm_sec);

    // MaskMode = 0  : tapered masking
    // MaskMode = 1  : STRICT masking
    //
    // if BlockNB < 0 : do all blocks
    // if BlockNB >= 0 : only update single block (untested)
    int MaskMode = 0;
    int BlockNB  = -1;

    AOloopControl_computeCalib_mkModes("fmodes",
                                       DMxsize,
                                       DMysize,
                                       CPAmax,
                                       deltaCPA,
                                       align_CX,
                                       align_CY,
                                       align_ID,
                                       align_OD,
                                       MaskMode,
                                       BlockNB,
                                       SVDlim,
                                       fps.md->datadir);

    printf("[%s %d] Save to disk\n", __FILE__, __LINE__);
    fflush(stdout);

    // save results to disk
    char fnamesrc[STRINGMAXLEN_FULLFILENAME];
    char fnamedest[STRINGMAXLEN_FULLFILENAME];
    char fnametxt[STRINGMAXLEN_FULLFILENAME];

    EXECUTE_SYSTEM_COMMAND("mkdir -p DMmodes");
    EXECUTE_SYSTEM_COMMAND("mkdir -p respM");
    EXECUTE_SYSTEM_COMMAND("mkdir -p contrM");
    EXECUTE_SYSTEM_COMMAND("mkdir -p contrMc");
    EXECUTE_SYSTEM_COMMAND("mkdir -p contrMcact");

    printf("[%s %d] Save to disk\n", __FILE__, __LINE__);
    fflush(stdout);

    WRITE_FULLFILENAME(fnamesrc,
                       "./%s/mkmodestmp/fmodesall.fits",
                       fps.md->datadir);
    WRITE_FULLFILENAME(fnamedest, "DMmodes/DMmodes_%s.fits", datestring);
    WRITE_FULLFILENAME(fnametxt,
                       "./%s/shmim.DMmodes.fname.txt",
                       fps.md->datadir);

    EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
    WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

    WRITE_FULLFILENAME(fnamesrc,
                       "./%s/mkmodestmp/fmodesWFSall.fits",
                       fps.md->datadir);
    WRITE_FULLFILENAME(fnamedest, "respM/respM_%s.fits", datestring);
    WRITE_FULLFILENAME(fnametxt, "./%s/shmim.respM.fname.txt", fps.md->datadir);

    EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
    WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

    WRITE_FULLFILENAME(fnamesrc,
                       "./%s/mkmodestmp/cmatall.fits",
                       fps.md->datadir);
    WRITE_FULLFILENAME(fnamedest, "contrM/contrM_%s.fits", datestring);
    WRITE_FULLFILENAME(fnametxt,
                       "./%s/shmim.contrM.fname.txt",
                       fps.md->datadir);

    EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
    WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

    EXECUTE_SYSTEM_COMMAND(
        "cp ./%s/mkmodestmp/NBmodes.txt ./%s/param_NBmodes.txt",
        fps.md->datadir,
        fps.md->datadir);

    int OKloop = 1;
    for(int i = 0; i < 20; i++)
    {
        WRITE_FULLFILENAME(fnamesrc,
                           "./%s/mkmodestmp/fmodes_%02d.fits",
                           fps.md->datadir,
                           i);

        if(OKloop == 1)
        {
            if(is_fits_file(fnamesrc) == 1)
            {
                printf("[%s %d] Archive [%d] to disk\n", __FILE__, __LINE__, i);
                fflush(stdout);

                printf("Found file %s\n", fnamesrc);

                WRITE_FULLFILENAME(fnamesrc,
                                   "./%s/mkmodestmp/fmodes_%02d.fits",
                                   fps.md->datadir,
                                   i);
                WRITE_FULLFILENAME(fnamedest,
                                   "DMmodes/DMmodes%02d_%s.fits",
                                   i,
                                   datestring);
                WRITE_FULLFILENAME(fnametxt,
                                   "./%s/shmim.DMmodes%02d.fname.txt",
                                   fps.md->datadir,
                                   i);

                EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
                WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

                WRITE_FULLFILENAME(fnamesrc,
                                   "./%s/mkmodestmp/fmodesWFS_%02d.fits",
                                   fps.md->datadir,
                                   i);
                WRITE_FULLFILENAME(fnamedest,
                                   "respM/respM%02d_%s.fits",
                                   i,
                                   datestring);
                WRITE_FULLFILENAME(fnametxt,
                                   "./%s/shmim.respM%02d.fname.txt",
                                   fps.md->datadir,
                                   i);

                EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
                WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

                WRITE_FULLFILENAME(fnamesrc,
                                   "./%s/mkmodestmp/cmat_%02d.fits",
                                   fps.md->datadir,
                                   i);
                WRITE_FULLFILENAME(fnamedest,
                                   "contrM/contrM%02d_%s.fits",
                                   i,
                                   datestring);
                WRITE_FULLFILENAME(fnametxt,
                                   "./%s/shmim.contrM%02d.fname.txt",
                                   fps.md->datadir,
                                   i);

                EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
                WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

                WRITE_FULLFILENAME(fnamesrc,
                                   "./%s/mkmodestmp/cmatc_%02d.fits",
                                   fps.md->datadir,
                                   i);
                WRITE_FULLFILENAME(fnamedest,
                                   "contrMc/contrMc%02d_%s.fits",
                                   i,
                                   datestring);
                WRITE_FULLFILENAME(fnametxt,
                                   "./%s/shmim.contrMc%02d.fname.txt",
                                   fps.md->datadir,
                                   i);

                EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
                WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);

                WRITE_FULLFILENAME(fnamesrc,
                                   "./%s/mkmodestmp/cmatcact_%02d.fits",
                                   fps.md->datadir,
                                   i);
                WRITE_FULLFILENAME(fnamedest,
                                   "contrMcact/contrMcact%02d_%s.fits",
                                   i,
                                   datestring);
                WRITE_FULLFILENAME(fnametxt,
                                   "./%s/shmim.contrMcact%02d.fname.txt",
                                   fps.md->datadir,
                                   i);

                EXECUTE_SYSTEM_COMMAND("cp %s %s", fnamesrc, fnamedest);
                WRITE_STRING_TO_FILE(fnametxt, "%s", fnamedest);
            }
            else
            {
                OKloop = 0;
                printf("Cannot find file %s -> stopping\n", fnamesrc);
            }
        }
    }

    functionparameter_SaveFPS2disk(&fps);

    functionparameter_SaveFPS2disk_dir(&fps, fps.md->datadir);
    EXECUTE_SYSTEM_COMMAND("rm %s/loglist.dat 2> /dev/null", fps.md->datadir);
    // EXECUTE_SYSTEM_COMMAND("echo \"sCMat.fits\" >> %s/loglist.dat",
    // outdirname);

    // create archive script
    functionparameter_write_archivescript(&fps);

    printf("[%s %d] DONE - disconnecting from FPS\n", __FILE__, __LINE__);
    fflush(stdout);
    function_parameter_RUNexit(&fps);

    return RETURN_SUCCESS;
}
/** @} */ // end of group
