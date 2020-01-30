/**
 * @file    AOloopControl_process_files.c
 * @brief   AO loop control - Process log files   
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * 
 */
 
 
#define _GNU_SOURCE

#include "CommandLineInterface/CLIcore.h"
#include "AOloopControl/AOloopControl.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "fft/fft.h"


#include <string.h>
#include <math.h>



// defined in AOloopControl.c
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array

// defined in AOloopControl.c
extern AOloopControl_var aoloopcontrol_var;






errno_t AOloopControl_logprocess_modeval(
    const char *IDname
)
{
    imageID ID;
    long    NBmodes;
    long    NBframes;

    imageID IDout_ave;
    imageID IDout_rms;

    long m;
    imageID ID1dtmp;
    FILE *fp;

    imageID ID1dPSD;
    char    fname[200];



    ID = image_ID(IDname);
    NBmodes = data.image[ID].md[0].size[0]*data.image[ID].md[0].size[1];
    NBframes = data.image[ID].md[0].size[2];

    IDout_ave = create_2Dimage_ID("modeval_ol_ave", data.image[ID].md[0].size[0], data.image[ID].md[0].size[1]);
    IDout_rms = create_2Dimage_ID("modeval_ol_rms", data.image[ID].md[0].size[0], data.image[ID].md[0].size[1]);

    ID1dtmp = create_2Dimage_ID("modeval1d", data.image[ID].md[0].size[2], 1);
    ID1dPSD = create_2Dimage_ID("modevalPSD", data.image[ID].md[0].size[2]/2, 1);

    if(system("mkdir -p modePSD") != 0)
        printERROR(__FILE__, __func__, __LINE__, "system() returns non-zero value");

    fp = fopen("moveval_stats.dat", "w");
    for(m=0; m<NBmodes; m++)
    {
        double ave = 0.0;
        double rms;
        long kk;
		FILE *fpPSD;
		long IDft;


        for(kk=0; kk<NBframes; kk++)
            ave += data.image[ID].array.F[kk*NBmodes+m];
        ave /= NBframes;
        data.image[IDout_ave].array.F[m] = ave;
        rms = 0.0;
        for(kk=0; kk<NBframes; kk++)
        {
			double tmpv;
			
            tmpv = (data.image[ID].array.F[kk*NBmodes+m]-ave);
            rms += tmpv*tmpv;
        }
        rms = sqrt(rms/NBframes);
        data.image[IDout_rms].array.F[m] = rms;


        for(kk=0; kk<NBframes; kk++)
            data.image[ID1dtmp].array.F[kk] = data.image[ID].array.F[kk*NBmodes+m];
        do1drfft("modeval1d", "modeval1d_FT");
        IDft = image_ID("modeval1d_FT");

        if(sprintf(fname, "./modePSD/modevalPSD_%04ld.dat", m) < 1)
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        fpPSD = fopen(fname, "w");
        for(kk=0; kk<NBframes/2; kk++)
        {
            data.image[ID1dPSD].array.F[kk] = data.image[IDft].array.CF[kk].re*data.image[IDft].array.CF[kk].re + data.image[IDft].array.CF[kk].im*data.image[IDft].array.CF[kk].im;
            fprintf(fpPSD, "%03ld %g\n", kk, data.image[ID1dPSD].array.F[kk]);
        }
        delete_image_ID("modeval1d_FT");
        fclose(fpPSD);


        fprintf(fp, "%4ld  %12.8f  %12.8f\n", m, data.image[IDout_ave].array.F[m], data.image[IDout_rms].array.F[m]);
    }
    fclose(fp);



    return RETURN_SUCCESS;
}




errno_t AOloopControl_TweakRM(
    char *ZRMinname,
    char *DMinCname,
    char *WFSinCname,
    char *DMmaskname,
    char *WFSmaskname,
    __attribute__((unused)) char *RMoutname
)
{
    imageID IDzrmin;
    imageID IDdmin;
    imageID IDwfsin;
    imageID IDdmmask;
    uint32_t wfsxsize, wfsysize;
    uint32_t dmxsize, dmysize;
    uint64_t dmsize;
    uint32_t NBframes;


    // input response matrix
    IDzrmin = image_ID(ZRMinname);
    wfsxsize = data.image[IDzrmin].md[0].size[0];
    wfsysize = data.image[IDzrmin].md[0].size[1];

    // DM input frames
    IDdmin = image_ID(DMinCname);
    dmxsize = data.image[IDdmin].md[0].size[0];
    dmysize = data.image[IDdmin].md[0].size[1];
    dmsize = dmxsize*dmysize;

    if(dmsize != data.image[IDzrmin].md[0].size[2])
    {
        printf("ERROR: total number of DM actuators (%ld) does not match zsize of RM (%ld)\n", dmsize, (long) data.image[IDzrmin].md[0].size[2]);
        exit(0);
    }

    NBframes = data.image[IDdmin].md[0].size[2];


    // input WFS frames
    IDwfsin = image_ID(WFSinCname);
    if((data.image[IDwfsin].md[0].size[0] != wfsxsize) || (data.image[IDwfsin].md[0].size[1] != wfsysize) || (data.image[IDwfsin].md[0].size[2] != NBframes))
    {
        printf("ERROR: size of WFS mask image \"%s\" (%u %u %u) does not match expected size (%u %u %u)\n",
               WFSmaskname, data.image[IDwfsin].md[0].size[0], data.image[IDwfsin].md[0].size[1], data.image[IDwfsin].md[0].size[2], wfsxsize, wfsysize, NBframes);
        exit(0);
    }

    // DM mask
    IDdmmask = image_ID(DMmaskname);
    if((data.image[IDdmmask].md[0].size[0] != dmxsize) || (data.image[IDdmmask].md[0].size[1] != dmysize))
    {
        printf("ERROR: size of DM mask image \"%s\" (%u %u) does not match expected size (%u %u)\n",
               DMmaskname, data.image[IDdmmask].md[0].size[0], data.image[IDdmmask].md[0].size[1], dmxsize, dmysize);
        exit(0);
    }



    // ARRANGE DATA IN MATRICES


    return RETURN_SUCCESS;
}
