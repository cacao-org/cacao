/**
 * @file    AOloopControl_compTools.c
 * @brief   Adaptive Optics Control loop engine misc computation tools
 * 
 * AO engine uses stream data structure
 *  
 * @author  O. Guyon
 * @date    28 Aug 2017
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

#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include <malloc.h>
#include <stdio.h>
#include <string.h>





#include <fitsio.h>

#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "COREMOD_tools/COREMOD_tools.h"
#include "COREMOD_arith/COREMOD_arith.h"
#include "info/info.h"

#include "statistic/statistic.h"

#include "AOloopControl/AOloopControl.h"
#include "AOloopControl_compTools/AOloopControl_compTools.h"










/* =============================================================================================== */
/* =============================================================================================== */
/*                                  GLOBAL DATA DECLARATION                                        */
/* =============================================================================================== */
/* =============================================================================================== */




/* =============================================================================================== */
/*                                     MAIN DATA STRUCTURES                                        */
/* =============================================================================================== */

extern DATA data;

#define NB_AOloopcontrol 10 // max number of loops
long LOOPNUMBER = 0; // current loop index

int AOloopcontrol_meminit = 0;
static int AOlooploadconf_init = 0;

#define AOconfname "/tmp/AOconf.shm"
AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array












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
/*                                    FUNCTIONS SOURCE CODE                                        */
/* =============================================================================================== */
/* =============================================================================================== */
/** @name AOloopControl functions */




/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
/* =============================================================================================== */

/** @brief CLI function for AOloopControl_CrossProduct */
int_fast8_t AOloopControl_compTools_CrossProduct_cli() {
    if(CLI_checkarg(1,4)+CLI_checkarg(2,4)+CLI_checkarg(3,3)==0) {
        AOloopControl_compTools_CrossProduct(data.cmdargtoken[1].val.string, data.cmdargtoken[2].val.string, data.cmdargtoken[3].val.string);
        return 0;
    }
    else return 1;
}


/** @brief CLI function for AOloopControl_mkSimpleZpokeM */
int_fast8_t AOloopControl_compTools_mkSimpleZpokeM_cli()
{
    if(CLI_checkarg(1,2)+CLI_checkarg(2,2)+CLI_checkarg(3,3)==0)    {
        AOloopControl_compTools_mkSimpleZpokeM(data.cmdargtoken[1].val.numl, data.cmdargtoken[2].val.numl, data.cmdargtoken[3].val.string);
        return 0;
    }
    else        return 1;
}








void __attribute__ ((constructor)) libinit_AOloopControl_compTools()
{
	init_AOloopControl_compTools();
	printf(" ...... Loading module %s\n", __FILE__);
}



int_fast8_t init_AOloopControl_compTools()
{
    FILE *fp;

    strcpy(data.module[data.NBmodule].name, __FILE__);
    strcpy(data.module[data.NBmodule].info, "cacao   - AO loop control - computation tools");
    data.NBmodule++;


/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
/* =============================================================================================== */

    RegisterCLIcommand("aolcrossp", __FILE__, AOloopControl_compTools_CrossProduct_cli, "compute cross product between two cubes. Apply mask if image xpmask exists", "<cube1> <cube2> <output image>", "aolcrossp imc0 imc1 crosspout", "AOloopControl_compTools_CrossProduct(char *ID1_name, char *ID1_name, char *IDout_name)");

    RegisterCLIcommand("aolmksimplezpM", __FILE__, AOloopControl_compTools_mkSimpleZpokeM_cli, "make simple poke sequence", "<dmsizex> <dmsizey> <output image>", "aolmksimplezpM 50 50 pokeM", "long AOloopControl_compTools_mkSimpleZpokeM( long dmxsize, long dmysize, char *IDout_name)");

    // add atexit functions here
    // atexit((void*) myfunc);

}










/* =============================================================================================== */
/** @name AOloopControl_compTools - 1. COMPUTATION UTILITIES & TOOLS                               */
/* =============================================================================================== */




// measures cross product between 2 cubes
long AOloopControl_compTools_CrossProduct(const char *ID1_name, const char *ID2_name, const char *IDout_name)
{
    long ID1, ID2, IDout;
    long xysize1, xysize2;
    long zsize1, zsize2;
    long z1, z2;
    long ii;
    long IDmask;




    ID1 = image_ID(ID1_name);
    ID2 = image_ID(ID2_name);

    xysize1 = data.image[ID1].md[0].size[0]*data.image[ID1].md[0].size[1];
    xysize2 = data.image[ID2].md[0].size[0]*data.image[ID2].md[0].size[1];
    zsize1 = data.image[ID1].md[0].size[2];
    zsize2 = data.image[ID2].md[0].size[2];

    if(xysize1!=xysize2)
    {
        printf("ERROR: cubes %s and %s have different xysize: %ld %ld\n", ID1_name, ID2_name, xysize1, xysize2);
        exit(0);
    }

    IDmask = image_ID("xpmask");


    IDout = create_2Dimage_ID(IDout_name, zsize1, zsize2);
    for(ii=0; ii<zsize1*zsize2; ii++)
        data.image[IDout].array.F[ii] = 0.0;

    if(IDmask==-1)
    {
        printf("No mask\n");
        fflush(stdout);


        for(z1=0; z1<zsize1; z1++)
            for(z2=0; z2<zsize2; z2++)
            {
                for(ii=0; ii<xysize1; ii++)
                {
                    data.image[IDout].array.F[z2*zsize1+z1] += data.image[ID1].array.F[z1*xysize1+ii] * data.image[ID2].array.F[z2*xysize2+ii];
                }
            }
    }
    else
    {
        printf("Applying mask\n");
        fflush(stdout);

        for(z1=0; z1<zsize1; z1++)
            for(z2=0; z2<zsize2; z2++)
            {
                for(ii=0; ii<xysize1; ii++)
                {
                    data.image[IDout].array.F[z2*zsize1+z1] += data.image[IDmask].array.F[ii]*data.image[IDmask].array.F[ii]*data.image[ID1].array.F[z1*xysize1+ii] * data.image[ID2].array.F[z2*xysize2+ii];
                }
            }
    }


    return(IDout);
}






// create simple poke matrix
long AOloopControl_compTools_mkSimpleZpokeM( long dmxsize, long dmysize, char *IDout_name)
{
    long IDout;
    uint_fast16_t dmxysize;
    uint_fast16_t ii, jj, kk;


    dmxysize = dmxsize * dmysize;

    IDout = create_3Dimage_ID(IDout_name, dmxsize, dmysize, dmxysize);

    for(kk=0; kk<dmxysize; kk++)
        data.image[IDout].array.F[kk*dmxysize + kk] = 1.0;

    return(IDout);
}



