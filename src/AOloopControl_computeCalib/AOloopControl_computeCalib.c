/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *
 * 
 * @bug No known bugs.
 * 
 * 
 */



#define _GNU_SOURCE

// uncomment for test print statements to stdout
//#define _PRINT_TEST



/* =============================================================================================== */
/* =============================================================================================== */
/*                                        HEADER FILES                                             */
/* =============================================================================================== */
/* =============================================================================================== */

#include <stdio.h>
#include <string.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h>


#include "CommandLineInterface/CLIcore.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"
#include "00CORE/00CORE.h"
#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_computeCalib/AOloopControl_computeCalib.h"



#ifdef HAVE_CUDA
#include "cudacomp/cudacomp.h"
#endif


/* =============================================================================================== */
/* =============================================================================================== */
/*                                      DEFINES, MACROS                                            */
/* =============================================================================================== */
/* =============================================================================================== */

#define MAX_MBLOCK 20

# ifdef _OPENMP
# include <omp.h>
#define OMP_NELEMENT_LIMIT 1000000
# endif




/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */

//extern int aoloopcontrol_var.PIXSTREAM_NBSLICES;
//extern long aoloopcontrol_var.aoconfID_pixstream_wfspixindex;;

//extern int *aoloopcontrol_var.DM_active_map;
//extern int *aoloopcontrol_var.WFS_active_map;



/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */


static int INITSTATUS_AOloopControl_computeCalib = 0;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c


long aoconfID_imWFS2_active[100];









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




/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl - 5. COMPUTING CALIBRATION
 *  Compute control matrix, modes */
/* =============================================================================================== */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_mkSlavedAct */
int_fast8_t AOloopControl_computeCalib_mkSlavedAct_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,1)+CLI_checkarg(3,3)==0) {
        AOloopControl_computeCalib_mkSlavedAct(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numf, data.cmdargtoken[3].val.string);
        return 0;
    }
    else	return 1;
}

/** @brief CLI function for AOloopControl_mkloDMmodes */
int_fast8_t AOloopControl_computeCalib_mkloDMmodes_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,2)==0) {
        AOloopControl_computeCalib_mkloDMmodes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numl);
        return 0;
    }
    else return 1;
}






/** @brief CLI function for AOloopControl_computeCalib_ComputeCM */

int_fast8_t AOloopControl_computeCalib_ComputeCM_cli() {
    char fpsname[200];

    // First, we try to execute function through FPS interface
    if(CLI_checkarg(1, 5) == 0) { // check that first arg is string
        // unsigned int OptionalArg00 = data.cmdargtoken[2].val.numl;
        // Set FPS interface name
        // By convention, if there are optional arguments, they should be appended to the fps name
        //
        if(data.processnameflag == 0) { // name fps to something different than the process name
            if(strlen(data.cmdargtoken[2].val.string)>0)
                sprintf(fpsname, "compCM-%s", data.cmdargtoken[2].val.string);
            else
                sprintf(fpsname, "compCM");
        } else { // Automatically set fps name to be process name up to first instance of character '.'
            strcpy(fpsname, data.processname0);
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_FPSINIT_") == 0) {  // Initialize FPS 
            AOcontrolLoop_computeCalib_ComputeCM_FPCONF(fpsname, CMDCODE_FPSINIT);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_CONFSTART_") == 0) {  // Start conf process
            AOcontrolLoop_computeCalib_ComputeCM_FPCONF(fpsname, CMDCODE_CONFSTART);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_CONFSTOP_") == 0) { // Stop conf process
            AOcontrolLoop_computeCalib_ComputeCM_FPCONF(fpsname, CMDCODE_CONFSTOP);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_RUNSTART_") == 0) { // Run process
            AOcontrolLoop_computeCalib_ComputeCM_RUN(fpsname);
            return RETURN_SUCCESS;
        }
        if(strcmp(data.cmdargtoken[1].val.string, "_RUNSTOP_") == 0) { // Stop process
            //     AOcontrolLoop__STOP(OptionalArg00);
            return RETURN_SUCCESS;
        }
    }    
     return RETURN_FAILURE;
}






/** @brief CLI function for AOloopControl_mkCM */
int_fast8_t AOloopControl_computeCalib_mkCM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,1)==0) {
        AOloopControl_computeCalib_mkCM(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkModes */
int_fast8_t AOloopControl_computeCalib_mkModes_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,2)+CLI_checkarg(11,2)+CLI_checkarg(12,1)==0) {
        AOloopControl_computeCalib_mkModes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numl, data.cmdargtoken[11].val.numl, data.cmdargtoken[12].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkModes_Simple */
int_fast8_t AOloopControl_computeCalib_mkModes_Simple_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,2)+CLI_checkarg(3,2)+CLI_checkarg(4,1)==0) {
        AOloopControl_computeCalib_mkModes_Simple(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.numl, data.cmdargtoken[4].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_computeCM */
int_fast8_t AOloopControl_computeCalib_computeCM_cli() {
    if(CLI_checkarg(1,2)+CLI_checkarg(2,4)+CLI_checkarg(3,3)+CLI_checkarg(4,1)+CLI_checkarg(5,2)+CLI_checkarg(6,1)==0) {
        AOloopControl_computeCalib_compute_ControlMatrix(LOOPNUMBER, data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, "evecM", data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numl, data.cmdargtoken[6].val.numf);
        save_fits("evecM","!evecM.fits");
        delete_image_ID("evecM");
    } else return 1;
}

/** @brief CLI function for AOloopControl_loadCM */
int_fast8_t AOloopControl_computeCalib_loadCM_cli() {
    if(CLI_checkarg(1,3)==0) {
        AOloopControl_computeCalib_loadCM(LOOPNUMBER, data.cmdargtoken[1].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkHadamardModes */
int_fast8_t AOloopControl_computeCalib_mkHadamardModes_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)==0) {
        AOloopControl_computeCalib_mkHadamardModes(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_Hadamard_decodeRM */
int_fast8_t AOloopControl_computeCalib_Hadamard_decodeRM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,3)==0) {
        AOloopControl_computeCalib_Hadamard_decodeRM(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_mkCalib_map_mask */
int_fast8_t AOloopControl_computeCalib_mkCalib_map_mask_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,3)+CLI_checkarg(3,3)+CLI_checkarg(4,1)+CLI_checkarg(5,1)+CLI_checkarg(6,1)+CLI_checkarg(7,1)+CLI_checkarg(8,1)+CLI_checkarg(9,1)+CLI_checkarg(10,1)+CLI_checkarg(11,1)==0) {
        AOloopControl_computeCalib_mkCalib_map_mask(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.numf, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numf, data.cmdargtoken[7].val.numf, data.cmdargtoken[8].val.numf, data.cmdargtoken[9].val.numf, data.cmdargtoken[10].val.numf, data.cmdargtoken[11].val.numf);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_Process_zrespM */
int_fast8_t AOloopControl_computeCalib_Process_zrespM_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,3)==0) {
        AOloopControl_computeCalib_Process_zrespM(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_ProcessZrespM */
int_fast8_t AOloopControl_computeCalib_ProcessZrespM_cli() {
    if(CLI_checkarg(1,3)+CLI_checkarg(2,3)+CLI_checkarg(3,3)+CLI_checkarg(4,3)+CLI_checkarg(5,1)+CLI_checkarg(6,2)==0) {
        AOloopControl_computeCalib_ProcessZrespM_medianfilt(LOOPNUMBER, data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.numf, data.cmdargtoken[6].val.numl);
        return 0;
    }
    else return 1;
}

/** @brief CLI function for AOloopControl_compute_CombinedControlMatrix */
int_fast8_t AOloopControl_computeCalib_compute_CombinedControlMatrix_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,4)+CLI_checkarg(4,4)+CLI_checkarg(5,3)+CLI_checkarg(6,3)==0) {
        AOloopControl_computeCalib_compute_CombinedControlMatrix(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string, data.cmdargtoken[4].val.string, data.cmdargtoken[5].val.string, data.cmdargtoken[6].val.string);
        return 0;
    }
    else return 1;
}
















/* =============================================================================================== */
/* =============================================================================================== */
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_IOtools functions */


void __attribute__ ((constructor)) libinit_AOloopControl_computeCalib()
{
	if ( INITSTATUS_AOloopControl_computeCalib == 0)
	{
		init_AOloopControl_computeCalib();
		RegisterModule(__FILE__, "cacao", "AO loop control compute calibration");
		INITSTATUS_AOloopControl_computeCalib = 1;
	}
}

            
int_fast8_t init_AOloopControl_computeCalib()
{

/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


    RegisterCLIcommand("aolmkH", __FILE__, AOloopControl_computeCalib_mkHadamardModes_cli, "make Hadamard poke sequence", "<DM pixel mask> <output fname [string]>", "aolmkH dm50mask h50pokec", "long AOloopControl_computeCalib_mkHadamardModes(char *DMmask_name, char outname)");

    RegisterCLIcommand("aolHaddec", __FILE__, AOloopControl_computeCalib_Hadamard_decodeRM_cli, "decode Hadamard matrix", "<input RM> <Hadamard matrix> <DMpix index frame> <output RM>", "aolHaddec imRMh Hmat pixiind imRM", "long AOloopControl_computeCalib_Hadamard_decodeRM(char *inname, char *Hmatname, char *indexname, char *outname)");

    RegisterCLIcommand("aolmkslact",__FILE__, AOloopControl_computeCalib_mkSlavedAct_cli, "create slaved actuators map based on proximity", "<maskRM> <distance> <outslact>", "aolmkslact DMmaskRM 2.5 DMslavedact", "long AOloopControl_computeCalib_mkSlavedAct(char *IDmaskRM_name, float pixrad, char *IDout_name)");

    RegisterCLIcommand("aolmklodmmodes",__FILE__, AOloopControl_computeCalib_mkloDMmodes_cli, "make low order DM modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode>", "aolmklodmmodes modes 50 50 5.0 0.8 1", "long AOloopControl_computeCalib_mkloDMmodes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode)");


	RegisterCLIcommand(
		"aolcomputeCM",
		__FILE__, 
		AOloopControl_computeCalib_ComputeCM_cli, 
		"compute control matrix, high level function FPS only", 
		"<CODE>", 
		"aolcomputeCM _CONFRUN_", 
		"FPS only");


    RegisterCLIcommand("aolRM2CM",__FILE__, AOloopControl_computeCalib_mkCM_cli, "make control matrix from response matrix", "<RMimage> <CMimage> <SVDlim>", "aolRM2CM respM contrM 0.1", "long AOloopControl_computeCalib_mkCM(char *respm_name, char *cm_name, float SVDlim)");

    RegisterCLIcommand("aolmkmodes", __FILE__, AOloopControl_computeCalib_mkModes_cli, "make control modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode> <block> <SVDlim>", "aolmkmodes modes 50 50 5.0 0.8 1 2 0.01", "long AOloopControl_computeCalib_mkModes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode, int BlockNB, float SVDlim)");

    RegisterCLIcommand("aolmkmodesM", __FILE__, AOloopControl_computeCalib_mkModes_Simple_cli, "make control modes in modal DM mode", "<input WFS modes> <NBmblock> <block> <SVDlim>", "aolmkmodesM wfsallmodes 5 2 0.01", "long AOloopControl_computeCalib_mkModes_Simple(char *IDin_name, long NBmblock, long Cmblock, float SVDlim)");

    RegisterCLIcommand("aolRMmkmasks",__FILE__, AOloopControl_computeCalib_mkCalib_map_mask_cli, "make sensitivity maps and masks from response matrix", "<zrespm fname [string]> <output WFS response map fname [string]>  <output DM response map fname [string]> <percentile low> <coefficient low> <percentile high> <coefficient high>", "aolRMmkmasks .. 0.2 1.0 0.5 0.3 0.05 1.0 0.65 0.3", "int AOloopControl_computeCalib_mkCalib_map_mask(long loop, char *zrespm_name, char *WFSmap_name, char *DMmap_name, float dmmask_perclow, float dmmask_coefflow, float dmmask_perchigh, float dmmask_coeffhigh, float wfsmask_perclow, float wfsmask_coefflow, float wfsmask_perchigh, float wfsmask_coeffhigh)");

    RegisterCLIcommand("aolproczrm",__FILE__, AOloopControl_computeCalib_Process_zrespM_cli, "process zonal resp mat, WFS ref -> DM and WFS response maps", "<input zrespm fname [string]> <input WFS ref fname [string]> <output zrespm [string]> <output WFS response map fname [string]>  <output DM response map fname [string]>", "aolproczrm zrespmat0 wfsref0 zrespm wfsmap dmmap", "int AOloopControl_computeCalib_Process_zrespM(long loop, char *IDzrespm0_name, char *IDwfsref_name, char *IDzrespm_name, char *WFSmap_name, char *DMmap_name)");

    RegisterCLIcommand("aolcleanzrm",__FILE__, AOloopControl_computeCalib_ProcessZrespM_cli, "clean zonal resp mat, WFS ref, DM and WFS response maps", "<zrespm fname [string]> <output WFS ref fname [string]>  <output WFS response map fname [string]>  <output DM response map fname [string]> <RM ampl [um]>", "aolcleanzrm zrm wfsref wfsmap dmmap 0.05", "int AOloopControl_computeCalib_ProcessZrespM_medianfilt(long loop, char *zrespm_name, char *WFSref0_name, char *WFSmap_name, char *DMmap_name, double ampl)");

    RegisterCLIcommand("aolcompcmatc",__FILE__, AOloopControl_computeCalib_compute_CombinedControlMatrix_cli, "compute combined control matrix", "<modal control matrix> <modes> <wfs mask> <dm mask> <combined cmat> <combined cmat, only active elements>", "aolcompcmatc cmat fmodes wfsmask dmmask cmatc cmatcact", "long AOloopControl_computeCalib_compute_CombinedControlMatrix(char *IDcmat_name, char *IDmodes_name, char* IDwfsmask_name, char *IDdmmask_name, char *IDcmatc_name, char *IDcmatc_active_name)");

    RegisterCLIcommand("aolcmmake",__FILE__, AOloopControl_computeCalib_computeCM_cli, "make control matrix", "<NBmodes removed> <RespMatrix> <ContrMatrix> <beta> <nbremovedstep> <eigenvlim>", "aolcmmake 8 respm cmat", "int AOloopControl_computeCalib_compute_ControlMatrix(long loop, long NB_MODE_REMOVED, char *ID_Rmatrix_name, char *ID_Cmatrix_name, char *ID_VTmatrix_name, double Beta, long NB_MODE_REMOVED_STEP, float eigenvlim)");



    RegisterCLIcommand("aolloadcm", __FILE__, AOloopControl_computeCalib_loadCM_cli, "load new control matrix from file", "<fname>", "aolloadcm cm32.fits", "long AOloopControl_computeCalib_loadCM(long loop, const char *CMfname)");




    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}







errno_t AOcontrolLoop_computeCalib_ComputeCM_FPCONF(
    char *fpsname,
    uint32_t CMDmode
) {
    uint16_t loopstatus;


    // ===========================
    // SETUP FPS
    // ===========================
    FUNCTION_PARAMETER_STRUCT fps = function_parameter_FPCONFsetup(fpsname, CMDmode, &loopstatus);


    // ===========================
    // ALLOCATE FPS ENTRIES IF NOT ALREADY EXIST
    // ===========================
    void *pNull = NULL;
    uint64_t FPFLAG;


    long loop_default[4] = { 0, 0, 10, 0 };
    long fpi_loop = function_parameter_add_entry(&fps, ".loop",
                    "loop index",
                    FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, &loop_default);

    double SVDlim_default[4] = { 0.01, 0.000001, 1.0, 0.01 };
    long fpi_SVDlim = function_parameter_add_entry(&fps, ".SVDlim",
                      "RM poke amplitude",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &SVDlim_default);


    double CPAmax_default[4] = { 10.0, 0.1, 100.0, 10.0 };
    long fpi_CPAmax = function_parameter_add_entry(&fps, ".CPAmax",
                      "maximum controlled CPA",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &CPAmax_default);

    double deltaCPA_default[4] = { 0.8, 0.1, 2.0, 0.8 };
    long fpi_deltaCPA = function_parameter_add_entry(&fps, ".deltaCPA",
                      "delta CPA",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &deltaCPA_default);

    double alignCX_default[4] = { 0.0, 0.0, 2000.0, 0.0 };
    long fpi_alignCX = function_parameter_add_entry(&fps, ".align.CX",
                      "DM mask center X (if no DMmaskRM)",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &alignCX_default);

    double alignCY_default[4] = { 0.0, 0.0, 2000.0, 0.0 };
    long fpi_alignCY = function_parameter_add_entry(&fps, ".align.CY",
                      "DM mask center Y (if no DMmaskRM)",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &alignCY_default);

    double alignID_default[4] = { 0.0, 0.0, 2000.0, 0.0 };
    long fpi_alignID = function_parameter_add_entry(&fps, ".align.ID",
                      "mask I.D. (if no DMmaskRM)",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &alignID_default);

    double alignOD_default[4] = { 0.0, 0.0, 2000.0, 0.0 };
    long fpi_alignOD = function_parameter_add_entry(&fps, ".align.OD",
                      "mask O.D. (if no DMmaskRM)",
                      FPTYPE_FLOAT64, FPFLAG_DEFAULT_INPUT, &alignOD_default);

    long DMxsize_default[4] = { 10, 1, 2000, 10 };
    long fpi_DMxsize = function_parameter_add_entry(&fps, ".DMxsize",
                      "DM x size",
                      FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, &DMxsize_default);

    long DMysize_default[4] = { 10, 1, 2000, 10 };
    long fpi_DMysize = function_parameter_add_entry(&fps, ".DMysize",
                      "DM y size",
                      FPTYPE_INT64, FPFLAG_DEFAULT_INPUT, &DMysize_default);


	// FPS 
	
    long fpi_FPS_zRMacqu = function_parameter_add_entry(&fps, ".FPS_zRMacqu",
                           "FPS zonal RM acquisition",
                           FPTYPE_FPSNAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FPS_RUN_REQUIRED, pNull);
    FUNCTION_PARAMETER_STRUCT FPS_zRMacqu;
    long FPS_zRMacqu_NBparam = 0;

    long fpi_FPS_loRMacqu = function_parameter_add_entry(&fps, ".FPS_loRMacqu",
                            "FPS low order modal RM acquisition",
                            FPTYPE_FPSNAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FPS_RUN_REQUIRED, pNull);
    FUNCTION_PARAMETER_STRUCT FPS_loRMacqu;
    long FPS_loRMacqu_NBparam = 0;


    long fpi_FPS_DMcomb = function_parameter_add_entry(&fps, ".FPS_DMcomb",
                         "FPS DMcomb",
                         FPTYPE_FPSNAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FPS_RUN_REQUIRED, pNull);
	FUNCTION_PARAMETER_STRUCT FPS_DMcomb;
	long FPS_DMcomb_NBparam = 0;





    long fpi_fname_dmslaved = function_parameter_add_entry(&fps, ".DMslaved",
                              "DM slaved actuators",
                              FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);

    long fpi_fname_zrespM = function_parameter_add_entry(&fps, ".zrespM",
                            "Zonal response matrix",
                            FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);

    long fpi_fname_dmmaskRM = function_parameter_add_entry(&fps, ".DMmaskRM",
                              "actuators directly controlled",
                              FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);

    long fpi_fname_WFSmask = function_parameter_add_entry(&fps, ".WFSmask",
                             "WFS mask",
                             FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);

    long fpi_fname_loRM = function_parameter_add_entry(&fps, ".loRM",
                          "low order modal response matrix",
                          FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);

    long fpi_fname_loRMmodes = function_parameter_add_entry(&fps, ".loRMmodes",
                               "low order RM modes",
                               FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT|FPFLAG_FILE_RUN_REQUIRED, pNull);


    long fpi_fname_extrablockM = function_parameter_add_entry(&fps, ".option.extrablockM",
                                 "extra modes block",
                                 FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT, pNull);

    long fpi_fname_exclmodes = function_parameter_add_entry(&fps, ".option.exclmodes",
                               "excluded modes",
                               FPTYPE_FITSFILENAME, FPFLAG_DEFAULT_INPUT, pNull);


    // Update Actions
    long fpi_update_RMfiles = function_parameter_add_entry(&fps, ".upRMfiles",
                              "update RM files",
                              FPTYPE_ONOFF, FPFLAG_DEFAULT_INPUT, pNull);

    long fpi_update_align = function_parameter_add_entry(&fps, ".upAlign",
                              "update default align (if no DMmaskRM)",
                              FPTYPE_ONOFF, FPFLAG_DEFAULT_INPUT, pNull);



    if( loopstatus == 0 ) // stop fps
        return RETURN_SUCCESS;



    // =====================================
    // PARAMETER LOGIC AND UPDATE LOOP
    // =====================================
    while ( loopstatus == 1 )
    {
        usleep(50);

        if( function_parameter_FPCONFloopstep(&fps, CMDmode, &loopstatus) == 1) // Apply logic if update is needed
        {
            //
            //  Connect to aux FPS
            //
            if ( FPS_zRMacqu_NBparam < 1 ) {
                FPS_zRMacqu_NBparam = function_parameter_struct_connect(fps.parray[fpi_FPS_zRMacqu].val.string[0], &FPS_zRMacqu, FPSCONNECT_SIMPLE);
            }
            if ( FPS_loRMacqu_NBparam < 1 ) {
                FPS_loRMacqu_NBparam = function_parameter_struct_connect(fps.parray[fpi_FPS_loRMacqu].val.string[0], &FPS_loRMacqu, FPSCONNECT_SIMPLE);
            }
            if ( FPS_DMcomb_NBparam < 1 ) {
				FPS_DMcomb_NBparam = function_parameter_struct_connect(fps.parray[fpi_FPS_DMcomb].val.string[0], &FPS_DMcomb, FPSCONNECT_SIMPLE);
			}            

            // Update RM files
            if(fps.parray[fpi_update_RMfiles].fpflag & FPFLAG_ONOFF) {

                if ( FPS_zRMacqu_NBparam > 0 ) {
                    char outdir[FUNCTION_PARAMETER_STRMAXLEN];
                    char fname[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(outdir,  functionparameter_GetParamPtr_STRING(&FPS_zRMacqu, ".out.dir"),  FUNCTION_PARAMETER_STRMAXLEN);

                    sprintf(fname, "%s/dmslaved.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".DMslaved", fname);

                    sprintf(fname, "%s/zrespM.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".zrespM", fname);

                    sprintf(fname, "%s/dmmaskRM.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".DMmaskRM", fname);

                    sprintf(fname, "%s/wfsmask.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".WFSmask", fname);
                }

                if ( FPS_loRMacqu_NBparam > 0 ) {
                    char outdir[FUNCTION_PARAMETER_STRMAXLEN];
                    char fname[FUNCTION_PARAMETER_STRMAXLEN];

                    strncpy(outdir,  functionparameter_GetParamPtr_STRING(&FPS_loRMacqu, ".out.dir"),  FUNCTION_PARAMETER_STRMAXLEN);

                    sprintf(fname, "%s/respM.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".loRM", fname);

                    sprintf(fname, "%s/RMpokeCube.fits", outdir);
                    functionparameter_SetParamValue_STRING(&fps, ".loRMmodes", fname);
                }

                // set back to OFF
                fps.parray[fpi_update_RMfiles].fpflag &= ~FPFLAG_ONOFF;
            }
            
            
            // update align params for auto mask
            if(fps.parray[fpi_update_align].fpflag & FPFLAG_ONOFF) {
				if ( FPS_DMcomb_NBparam > 0 ) {
					int DMxsize = functionparameter_GetParamValue_INT64 ( &FPS_DMcomb, ".DMxsize" );
					int DMysize = functionparameter_GetParamValue_INT64 ( &FPS_DMcomb, ".DMysize" );
					int DMMODE = functionparameter_GetParamValue_INT64 ( &FPS_DMcomb, ".DMMODE" );

					float cx = 0.5*DMxsize - 0.5;
					float cy = 0.5*DMysize - 0.5;
					float od = 0.45*DMxsize;
					float id = 0.05*DMxsize;
					
					functionparameter_SetParamValue_INT64(&fps, ".DMxsize", DMxsize);
					functionparameter_SetParamValue_INT64(&fps, ".DMysize", DMysize);
					functionparameter_SetParamValue_FLOAT64(&fps, ".align.CX", cx);
					functionparameter_SetParamValue_FLOAT64(&fps, ".align.CY", cy);
					functionparameter_SetParamValue_FLOAT64(&fps, ".align.OD", od);
					functionparameter_SetParamValue_FLOAT64(&fps, ".align.ID", id);
				}
				fps.parray[fpi_update_align].fpflag &= ~FPFLAG_ONOFF;
			}
				
				

            
            
            // update align parameters
            


            functionparameter_CheckParametersAll(&fps);  // check all parameter values
        }
    }


    function_parameter_FPCONFexit( &fps );

    if ( FPS_zRMacqu_NBparam > 0 ) {
        function_parameter_struct_disconnect( &FPS_zRMacqu );
    }


    if ( FPS_loRMacqu_NBparam > 0 ) {
        function_parameter_struct_disconnect( &FPS_loRMacqu );
    }

    if ( FPS_DMcomb_NBparam > 0 ) {
		function_parameter_struct_disconnect( &FPS_DMcomb );
	}
    

    return RETURN_SUCCESS;

}





errno_t AOcontrolLoop_computeCalib_ComputeCM_RUN(
    char *fpsname
) {
    // ===========================
    // CONNECT TO FPS
    // ===========================
    FUNCTION_PARAMETER_STRUCT fps;

    if(function_parameter_struct_connect(fpsname, &fps, FPSCONNECT_RUN) == -1) {
        printf("ERROR: fps \"%s\" does not exist -> running without FPS interface\n", fpsname);
        return RETURN_FAILURE;
    }

    // ===============================
    // GET FUNCTION PARAMETER VALUES
    // ===============================
    long loop         = functionparameter_GetParamValue_INT64(&fps, ".loop");
    float SVDlim      = functionparameter_GetParamValue_FLOAT64(&fps, ".SVDlim");
    float CPAmax      = functionparameter_GetParamValue_FLOAT64(&fps, ".CPAmax");
    float deltaCPA    = functionparameter_GetParamValue_FLOAT64(&fps, ".deltaCPA");

    float align_CX    = functionparameter_GetParamValue_FLOAT64(&fps, ".align.CX");
    float align_CY    = functionparameter_GetParamValue_FLOAT64(&fps, ".align.CY");
    float align_ID    = functionparameter_GetParamValue_FLOAT64(&fps, ".align.ID");
    float align_OD    = functionparameter_GetParamValue_FLOAT64(&fps, ".align.OD");

    long DMxsize    = functionparameter_GetParamValue_INT64(&fps, ".DMxsize");
    long DMysize    = functionparameter_GetParamValue_INT64(&fps, ".DMysize");

    char fname_dmslaved[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_dmslaved,  functionparameter_GetParamPtr_STRING(&fps, ".DMslaved"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_dmslaved, "dmslaved", 1);

    char fname_zrespM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_zrespM,  functionparameter_GetParamPtr_STRING(&fps, ".zrespM"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_zrespM, "zrespM", 1);

    char fname_DMmaskRM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_DMmaskRM,  functionparameter_GetParamPtr_STRING(&fps, ".DMmaskRM"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_DMmaskRM, "dmmaskRM", 1);

    char fname_WFSmask[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_WFSmask,  functionparameter_GetParamPtr_STRING(&fps, ".WFSmask"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_WFSmask, "wfsmask", 1);

    char fname_loRM[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_loRM,  functionparameter_GetParamPtr_STRING(&fps, ".loRM"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_loRM, "LOrespM", 1);

    char fname_loRMmodes[FUNCTION_PARAMETER_STRMAXLEN];
    strncpy(fname_loRMmodes,  functionparameter_GetParamPtr_STRING(&fps, ".loRMmodes"),  FUNCTION_PARAMETER_STRMAXLEN);
    load_fits(fname_loRMmodes, "RMMmodes", 1);

    list_image_ID();



    // Get time
    time_t tnow;
    struct tm *tmnow;
    char datestring[200];

    time(&tnow);
    tmnow = gmtime(&tnow);

    printf("TIMESTRING:  %04d %02d %02d  %02d:%02d:%02d\n", 1900+tmnow->tm_year, tmnow->tm_mon, tmnow->tm_mday, tmnow->tm_hour, tmnow->tm_min, tmnow->tm_sec);
    sprintf(datestring, "%04d-%02d-%02d_%02d:%02d:%02d", 1900+tmnow->tm_year, tmnow->tm_mon, tmnow->tm_mday, tmnow->tm_hour, tmnow->tm_min, tmnow->tm_sec);

    // MaskMode = 0  : tapered masking
    // MaskMode = 1  : STRICT masking
    //
    // if BlockNB < 0 : do all blocks
    // if BlockNB >= 0 : only update single block (untested)
    int MaskMode = 0;
    int BlockNB = -1;



    AOloopControl_computeCalib_mkModes("fmodes", DMxsize, DMysize, CPAmax, deltaCPA, align_CX, align_CY, align_ID, align_OD, MaskMode, BlockNB, SVDlim);

    // save results to disk
    char fnamesrc[500];
    char fnamedest[500];
    char fnametxt[500];
    char stagedir[] = "conf_staged";
    char command[500];
    FILE *fp;

    sprintf(fnamesrc, "./mkmodestmp/fmodesall.fits");
    sprintf(fnamedest, "DMmodes/DMmodes_%s.fits", datestring);
    sprintf(fnametxt, "./%s/shmim.DMmodes.name.txt", stagedir);

    sprintf(command, "cp %s %s", fnamesrc, fnamedest);
    if(system(command) != 0) {
        printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
    }
    fp = fopen(fnametxt, "w");
    fprintf(fp, "%s", fnamedest);
    fclose(fp);



    sprintf(fnamesrc, "./mkmodestmp/fmodesWFSall.fits");
    sprintf(fnamedest, "respM/respM_%s.fits", datestring);
    sprintf(fnametxt, "./%s/shmim.respM.name.txt", stagedir);

    sprintf(command, "cp %s %s", fnamesrc, fnamedest);
    if(system(command) != 0) {
        printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
    }
    fp = fopen(fnametxt, "w");
    fprintf(fp, "%s", fnamedest);
    fclose(fp);


    sprintf(fnamesrc, "./mkmodestmp/cmatall.fits");
    sprintf(fnamedest, "contrM/contrM_%s.fits", datestring);
    sprintf(fnametxt, "./%s/shmim.contrM.name.txt", stagedir);

    sprintf(command, "cp %s %s", fnamesrc, fnamedest);
    if(system(command) != 0) {
        printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
    }
    fp = fopen(fnametxt, "w");
    fprintf(fp, "%s", fnamedest);
    fclose(fp);



    sprintf(command, "cp ./mkmodestmp/NBmodes.txt ./%s/param_NBmodes.txt", stagedir);
    if(system(command) != 0) {
        printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
    }



    int OKloop = 1;
    for (int i=0; i<20; i++)
    {
        sprintf(fnamesrc, "./mkmodestmp/fmodes_%02d.fits", i);

        if(OKloop==1) {
            if(is_fits_file(fnamesrc)==1)
            {
                printf("Found file %s\n", fnamesrc);

                sprintf(fnamesrc, "./mkmodestmp/fmodes_%02d.fits", i);
                sprintf(fnamedest, "DMmodes/DMmodes%02d_%s.fits", i, datestring);
                sprintf(fnametxt, "./%s/shmim.DMmodes%02d.name.txt", stagedir, i);
                sprintf(command, "cp %s %s", fnamesrc, fnamedest);
                if(system(command) != 0) {
                    printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
                }
                fp = fopen(fnametxt, "w");
                fprintf(fp, "%s", fnamedest);
                fclose(fp);


                sprintf(fnamesrc, "./mkmodestmp/fmodesWFS_%02d.fits", i);
                sprintf(fnamedest, "respM/respM%02d_%s.fits", i, datestring);
                sprintf(fnametxt, "./%s/shmim.respM%02d.name.txt", stagedir, i);
                sprintf(command, "cp %s %s", fnamesrc, fnamedest);
                if(system(command) != 0) {
                    printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
                }
                fp = fopen(fnametxt, "w");
                fprintf(fp, "%s", fnamedest);
                fclose(fp);


                sprintf(fnamesrc, "./mkmodestmp/cmat_%02d.fits", i);
                sprintf(fnamedest, "contrM/contrM%02d_%s.fits", i, datestring);
                sprintf(fnametxt, "./%s/shmim.contrM%02d.name.txt", stagedir, i);
                sprintf(command, "cp %s %s", fnamesrc, fnamedest);
                if(system(command) != 0) {
                    printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
                }
                fp = fopen(fnametxt, "w");
                fprintf(fp, "%s", fnamedest);
                fclose(fp);


                sprintf(fnamesrc, "./mkmodestmp/cmatc_%02d.fits", i);
                sprintf(fnamedest, "contrMc/contrMc%02d_%s.fits", i, datestring);
                sprintf(fnametxt, "./%s/shmim.contrMc%02d.name.txt", stagedir, i);
                sprintf(command, "cp %s %s", fnamesrc, fnamedest);
                if(system(command) != 0) {
                    printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
                }
                fp = fopen(fnametxt, "w");
                fprintf(fp, "%s", fnamedest);
                fclose(fp);


                sprintf(fnamesrc, "./mkmodestmp/cmatcact_%02d.fits", i);
                sprintf(fnamedest, "contrMcact/contrMcact%02d_%s.fits", i, datestring);
                sprintf(fnametxt, "./%s/shmim.contrMcact%02d.name.txt", stagedir, i);
                sprintf(command, "cp %s %s", fnamesrc, fnamedest);
                if(system(command) != 0) {
                    printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");
                }
                fp = fopen(fnametxt, "w");
                fprintf(fp, "%s", fnamedest);
                fclose(fp);
            }
            else {
                OKloop = 0;
                printf("Cannot find file %s -> stopping\n", fnamesrc);
            }
        }

    }




    function_parameter_struct_disconnect( &fps );


    return RETURN_SUCCESS;

}



















