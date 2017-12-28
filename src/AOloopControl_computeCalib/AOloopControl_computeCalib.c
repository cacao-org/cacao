/**
 * @file    AOloopControl_computeCalib.c
 * @brief   Adaptive Optics Control loop engine compute calibration
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    26 Dec 2017 
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
#include "info/info.h"

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

extern DATA data;

extern long LOOPNUMBER; // current loop index

extern AOLOOPCONTROL_CONF *AOconf; // declared in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c
extern int AOloopcontrol_meminit;

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
	init_AOloopControl_computeCalib();
//	printf(" ...... Loading module %s\n", __FILE__);
}

            
int_fast8_t init_AOloopControl_computeCalib()
{

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].package, "cacao");
    strcpy(data.module[data.NBmodule].info, "AO loop control compute calibration");
    data.NBmodule++;





/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl_computeCalib - 1. COMPUTING CALIBRATION                                                 */
/* =============================================================================================== */
/* =============================================================================================== */


    RegisterCLIcommand("aolmkH", __FILE__, AOloopControl_computeCalib_mkHadamardModes_cli, "make Hadamard poke sequence", "<DM pixel mask> <output fname [string]>", "aolmkH dm50mask h50pokec", "long AOloopControl_computeCalib_mkHadamardModes(char *DMmask_name, char outname)");

    RegisterCLIcommand("aolHaddec", __FILE__, AOloopControl_computeCalib_Hadamard_decodeRM_cli, "decode Hadamard matrix", "<input RM> <Hadamard matrix> <DMpix index frame> <output RM>", "aolHaddec imRMh Hmat pixiind imRM", "long AOloopControl_computeCalib_Hadamard_decodeRM(char *inname, char *Hmatname, char *indexname, char *outname)");

    RegisterCLIcommand("aolmkslact",__FILE__, AOloopControl_computeCalib_mkSlavedAct_cli, "create slaved actuators map based on proximity", "<maskRM> <distance> <outslact>", "aolmkslact DMmaskRM 2.5 DMslavedact", "long AOloopControl_computeCalib_mkSlavedAct(char *IDmaskRM_name, float pixrad, char *IDout_name)");

    RegisterCLIcommand("aolmklodmmodes",__FILE__, AOloopControl_computeCalib_mkloDMmodes_cli, "make low order DM modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode>", "aolmklodmmodes modes 50 50 5.0 0.8 1", "long AOloopControl_computeCalib_mkloDMmodes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode)");

    RegisterCLIcommand("aolRM2CM",__FILE__, AOloopControl_computeCalib_mkCM_cli, "make control matrix from response matrix", "<RMimage> <CMimage> <SVDlim>", "aolRM2CM respM contrM 0.1", "long AOloopControl_computeCalib_mkCM(char *respm_name, char *cm_name, float SVDlim)");

    RegisterCLIcommand("aolmkmodes", __FILE__, AOloopControl_computeCalib_mkModes_cli, "make control modes", "<output modes> <sizex> <sizey> <max CPA> <delta CPA> <cx> <cy> <r0> <r1> <masking mode> <block> <SVDlim>", "aolmkmodes modes 50 50 5.0 0.8 1 2 0.01", "long AOloopControl_computeCalib_mkModes(char *ID_name, long msizex, long msizey, float CPAmax, float deltaCPA, double xc, double yx, double r0, double r1, int MaskMode, int BlockNB, float SVDlim)");

    RegisterCLIcommand("aolmkmodesM", __FILE__, AOloopControl_computeCalib_mkModes_Simple_cli, "make control modes in modal DM mode", "<input WFS modes> <NBmblock> <block> <SVDlim>", "aolmkmodesM wfsallmodes 5 2 0.01", "long AOloopControl_computeCalib_mkModes_Simple(char *IDin_name, long NBmblock, long Cmblock, float SVDlim)");

    RegisterCLIcommand("aolRMmkmasks",__FILE__, AOloopControl_computeCalib_mkCalib_map_mask_cli, "make sensitivity maps and masks from response matrix", "<zrespm fname [string]> <output WFS response map fname [string]>  <output DM response map fname [string]> <percentile low> <coefficient low> <percentile high> <coefficient high>", "aolRMmkmasks .. 0.2 1.0 0.5 0.3 0.05 1.0 0.65 0.3", "int AOloopControl_computeCalib_mkCalib_map_mask(long loop, char *zrespm_name, char *WFSmap_name, char *DMmap_name, float dmmask_perclow, float dmmask_coefflow, float dmmask_perchigh, float dmmask_coeffhigh, float wfsmask_perclow, float wfsmask_coefflow, float wfsmask_perchigh, float wfsmask_coeffhigh)");

    RegisterCLIcommand("aolproczrm",__FILE__, AOloopControl_computeCalib_Process_zrespM_cli, "process zonal resp mat, WFS ref -> DM and WFS response maps", "<input zrespm fname [string]> <input WFS ref fname [string]> <output zrespm [string]> <output WFS response map fname [string]>  <output DM response map fname [string]>", "aolproczrm zrespmat0 wfsref0 zrespm wfsmap dmmap", "int AOloopControl_computeCalib_Process_zrespM(long loop, char *IDzrespm0_name, char *IDwfsref_name, char *IDzrespm_name, char *WFSmap_name, char *DMmap_name)");

    RegisterCLIcommand("aolcleanzrm",__FILE__, AOloopControl_computeCalib_ProcessZrespM_cli, "clean zonal resp mat, WFS ref, DM and WFS response maps", "<zrespm fname [string]> <output WFS ref fname [string]>  <output WFS response map fname [string]>  <output DM response map fname [string]> <RM ampl [um]>", "aolcleanzrm zrm wfsref wfsmap dmmap 0.05", "int AOloopControl_computeCalib_ProcessZrespM(long loop, char *zrespm_name, char *WFSref0_name, char *WFSmap_name, char *DMmap_name, double ampl)");

    RegisterCLIcommand("aolcompcmatc",__FILE__, AOloopControl_computeCalib_compute_CombinedControlMatrix_cli, "compute combined control matrix", "<modal control matrix> <modes> <wfs mask> <dm mask> <combined cmat> <combined cmat, only active elements>", "aolcompcmatc cmat fmodes wfsmask dmmask cmatc cmatcact", "long AOloopControl_computeCalib_compute_CombinedControlMatrix(char *IDcmat_name, char *IDmodes_name, char* IDwfsmask_name, char *IDdmmask_name, char *IDcmatc_name, char *IDcmatc_active_name)");

    RegisterCLIcommand("aolcmmake",__FILE__, AOloopControl_computeCalib_computeCM_cli, "make control matrix", "<NBmodes removed> <RespMatrix> <ContrMatrix> <beta> <nbremovedstep> <eigenvlim>", "aolcmmake 8 respm cmat", "int AOloopControl_computeCalib_compute_ControlMatrix(long loop, long NB_MODE_REMOVED, char *ID_Rmatrix_name, char *ID_Cmatrix_name, char *ID_VTmatrix_name, double Beta, long NB_MODE_REMOVED_STEP, float eigenvlim)");



    RegisterCLIcommand("aolloadcm", __FILE__, AOloopControl_computeCalib_loadCM_cli, "load new control matrix from file", "<fname>", "aolloadcm cm32.fits", "long AOloopControl_computeCalib_loadCM(long loop, const char *CMfname)");




    // add atexit functions here
    // atexit((void*) myfunc);

    return 0;
}




// dmmask_perclow = 0.2
// dmmask_coefflow = 1.0
// dmmask_perchigh = 0.7
// dmmask_coeffhigh = 0.3


