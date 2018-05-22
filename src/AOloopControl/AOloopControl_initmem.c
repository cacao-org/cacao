/**
 * @file    AOloopControl_initmem.c 
 * @brief   Aload / setup configuration 
 * 
 * REAL TIME COMPUTING ROUTINES
 *  
 * @author  O. Guyon
 * @date    24 nov 2017
 *
 * 
 * 
 * @bug No known bugs.
 * 
 */

#define _GNU_SOURCE

#include "AOloopControl.h"


//libraries created by O. Guyon 
#include "CommandLineInterface/CLIcore.h"
#include <unistd.h>
#include <fcntl.h>
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"


#include <sys/mman.h>
#include <sys/stat.h>


#define AOconfname "/tmp/AOconf.shm"
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;
#define NB_AOloopcontrol 10 // max number of loops

static int GPUcntMax = 100;



/**
 * ## Purpose
 * 
 * Initialize memory of the loop  
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	int
 * 				value of the mode
 * 
 *
 *  
 */

/***  */

int_fast8_t AOloopControl_InitializeMemory(int mode)
{
    int SM_fd;
    struct stat file_stat;
    int create = 0;
    long loop;
    int tmpi;
	char imname[200];


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 0, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    loop = aoloopcontrol_var.LOOPNUMBER;

    SM_fd = open(AOconfname, O_RDWR);
    if(SM_fd==-1)
    {
        printf("Cannot import file \"%s\" -> creating file\n", AOconfname);
        create = 1;
    }
    else
    {
        fstat(SM_fd, &file_stat);
        printf("File %s size: %zd\n", AOconfname, file_stat.st_size);
        if(file_stat.st_size!=sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol)
        {
            printf("File \"%s\" size is wrong -> recreating file\n", AOconfname);
            create = 1;
            close(SM_fd);
        }
    }

    if(create==1)
    {
        int result;

        SM_fd = open(AOconfname, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);

        if (SM_fd == -1) {
            perror("Error opening file for writing");
            exit(0);
        }

        result = lseek(SM_fd, sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol-1, SEEK_SET);
        if (result == -1) {
            close(SM_fd);
            perror("Error calling lseek() to 'stretch' the file");
            exit(0);
        }

        result = write(SM_fd, "", 1);
        if (result != 1) {
            close(SM_fd);
            perror("Error writing last byte of the file");
            exit(0);
        }
    }


    AOconf = (AOLOOPCONTROL_CONF*) mmap(0, sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol, PROT_READ | PROT_WRITE, MAP_SHARED, SM_fd, 0);
    if (AOconf == MAP_FAILED) {
        close(SM_fd);
        perror("Error mmapping the file");
        exit(0);
    }


    if((mode==0)||(create==1))
    {
        char cntname[200];

        AOconf[loop].on = 0;
        AOconf[loop].DMprimaryWriteON = 0;
        AOconf[loop].DMfilteredWriteON = 0;
        AOconf[loop].AUTOTUNE_LIMITS_ON = 0;
        AOconf[loop].AUTOTUNE_GAINS_ON = 0;
        AOconf[loop].ARPFon = 0;
        AOconf[loop].ARPFgainAutoMin = 0.00;
        AOconf[loop].ARPFgainAutoMax = 1.00;
        AOconf[loop].LOOPiteration = 0;

        AOconf[loop].cnt = 0;
        

        AOconf[loop].cntmax = 0;
        AOconf[loop].init_CMc = 0;

        if(sprintf(cntname, "aol%ld_logdata", loop) < 1) // contains loop count (cnt0) and loop gain
            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

        if((aoloopcontrol_var.aoconfIDlogdata = image_ID(cntname))==-1)
        {
            uint32_t *sizearray;
            sizearray = (uint32_t*) malloc(sizeof(uint32_t)*2);
            sizearray[0] = 1;
            sizearray[1] = 1;
            aoloopcontrol_var.aoconfIDlogdata = create_image_ID(cntname, 2, sizearray, _DATATYPE_FLOAT, 1, 0);
            free(sizearray);
        }
        
        
        // logging
        AOconf[loop].RTstreamLOG_buff = 0;
        AOconf[loop].RTstreamLOG_frame = 0;
        
        AOconf[loop].RTstreamLOG_wfsim_ENABLE = 1;
        AOconf[loop].RTstreamLOG_wfsim_ON = 0;
        AOconf[loop].RTstreamLOG_wfsim_save = 0;
        AOconf[loop].RTstreamLOG_wfsim_saveToggle = 0;
        
        AOconf[loop].RTstreamLOG_imWFS0_ENABLE = 1;
        AOconf[loop].RTstreamLOG_imWFS0_ON = 0;
        AOconf[loop].RTstreamLOG_imWFS0_save = 0;
        AOconf[loop].RTstreamLOG_imWFS0_saveToggle = 0;

        AOconf[loop].RTstreamLOG_imWFS1_ENABLE = 1;
        AOconf[loop].RTstreamLOG_imWFS1_ON = 0;
        AOconf[loop].RTstreamLOG_imWFS1_save = 0;
        AOconf[loop].RTstreamLOG_imWFS1_saveToggle = 0;
        
        AOconf[loop].RTstreamLOG_imWFS2_ENABLE = 1;
        AOconf[loop].RTstreamLOG_imWFS2_ON = 0;
        AOconf[loop].RTstreamLOG_imWFS2_save = 0;
        AOconf[loop].RTstreamLOG_imWFS2_saveToggle = 0;        

        AOconf[loop].RTstreamLOG_modeval_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modeval_dm_corr_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_dm_corr_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_dm_corr_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_dm_corr_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modeval_dm_now_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_dm_now_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_dm_now_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_dm_now_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modeval_dm_now_filt_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_dm_now_filt_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_dm_now_filt_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_dm_now_filt_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modevalPF_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modevalPF_ON = 0;
        AOconf[loop].RTstreamLOG_modevalPF_save = 0 ;
        AOconf[loop].RTstreamLOG_modevalPF_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modevalPFsync_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modevalPFsync_ON = 0;
        AOconf[loop].RTstreamLOG_modevalPFsync_save = 0 ;
        AOconf[loop].RTstreamLOG_modevalPFsync_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modevalPFres_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modevalPFres_ON = 0;
        AOconf[loop].RTstreamLOG_modevalPFres_save = 0 ;
        AOconf[loop].RTstreamLOG_modevalPFres_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modeval_dm_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_dm_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_dm_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_dm_saveToggle = 0;

        AOconf[loop].RTstreamLOG_modeval_ol_ENABLE = 1;
        AOconf[loop].RTstreamLOG_modeval_ol_ON = 0;
        AOconf[loop].RTstreamLOG_modeval_ol_save = 0 ;
        AOconf[loop].RTstreamLOG_modeval_ol_saveToggle = 0;
        
        AOconf[loop].RTstreamLOG_dmdisp_ENABLE = 1;
        AOconf[loop].RTstreamLOG_dmdisp_ON = 0;
        AOconf[loop].RTstreamLOG_dmdisp_save = 0;
        AOconf[loop].RTstreamLOG_dmdisp_saveToggle = 0;        
    }


    if(create==1)
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
        {
            AOconf[loop].init = 0;
            AOconf[loop].on = 0;
            AOconf[loop].DMprimaryWriteON = 0;
            AOconf[loop].DMfilteredWriteON = 0;
            AOconf[loop].ARPFon = 0;
            AOconf[loop].LOOPiteration = 0;
            AOconf[loop].cnt = 0;
            AOconf[loop].cntmax = 0;
            AOconf[loop].maxlimit = 0.3;
            AOconf[loop].mult = 1.00;
            AOconf[loop].gain = 0.0;
            AOconf[loop].AUTOTUNE_LIMITS_perc = 1.0; // percentile threshold
            AOconf[loop].AUTOTUNE_LIMITS_mcoeff = 1.0; // multiplicative coeff
            AOconf[loop].AUTOTUNE_LIMITS_delta = 1.0e-3;
            AOconf[loop].ARPFgain = 0.0;
			AOconf[loop].ARPFgainAutoMin = 0.99;
			AOconf[loop].ARPFgainAutoMax = 1.01;
            AOconf[loop].WFSnormfloor = 0.0;
            AOconf[loop].framesAve = 1;
            AOconf[loop].DMmodesNBblock = 1;
            AOconf[loop].GPUusesem = 1;

            AOconf[loop].loopfrequ = 2000.0;
            AOconf[loop].hardwlatency = 0.0011;
            AOconf[loop].hardwlatency_frame = 2.2;
            AOconf[loop].complatency = 0.0001;
            AOconf[loop].complatency_frame = 0.2;
            AOconf[loop].wfsmextrlatency = 0.0003;
            AOconf[loop].wfsmextrlatency_frame = 0.6;
        }
    }
    else
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
            if(AOconf[loop].init == 1)
            {
                printf("LIST OF ACTIVE LOOPS:\n");
                printf("----- Loop %ld   (%s) ----------\n", loop, AOconf[loop].name);
                printf("  WFS:  %s  [%ld]  %ld x %ld\n", AOconf[loop].WFSname, aoloopcontrol_var.aoconfID_wfsim, AOconf[loop].sizexWFS, AOconf[loop].sizeyWFS);
                printf("   DM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmCname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
                printf("DM RM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].dmRMname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].sizexDM, AOconf[loop].sizeyDM);
            }
    }

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
    {

        printf("INITIALIZING GPUset ARRAYS\n");
        fflush(stdout);

        aoloopcontrol_var.GPUset0 = (int*) malloc(sizeof(int)*GPUcntMax);

        uint_fast16_t k;

        for(k=0; k<GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_GPUset0dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                aoloopcontrol_var.GPUset0[k] = tmpi;
            }
            else
                aoloopcontrol_var.GPUset0[k] = k;
        }


        aoloopcontrol_var.GPUset1 = (int*) malloc(sizeof(int)*GPUcntMax);
        for(k=0; k<GPUcntMax; k++)
        {
            FILE *fp;
            char fname[200];

            if(sprintf(fname, "./conf/param_GPUset1dev%d.txt", (int) k) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            fp = fopen(fname, "r");
            if(fp!=NULL)
            {
                if(fscanf(fp, "%50d" , &tmpi) != 1)
                    printERROR(__FILE__, __func__, __LINE__, "Cannot read parameter from file");

                fclose(fp);
                aoloopcontrol_var.GPUset1[k] = tmpi;
            }
            else
                aoloopcontrol_var.GPUset1[k] = k;
        }
    }

    aoloopcontrol_var.AOloopcontrol_meminit = 1;


#ifdef AOLOOPCONTROL_LOGFUNC
	AOLOOPCONTROL_logfunc_level = 0;
    CORE_logFunctionCall( AOLOOPCONTROL_logfunc_level, AOLOOPCONTROL_logfunc_level_max, 1, __FILE__, __FUNCTION__, __LINE__, "");
#endif

    return 0;
}

