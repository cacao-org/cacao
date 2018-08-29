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
#include <string.h>

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
 * mode     	int
 * 		- 0 initialize memory when connecting
 * 		- 1 connect only, do not modify/initialize
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
            printf("File has size : %zd\n", file_stat.st_size);
            printf("Should be     : %zd  (%d)\n", sizeof(AOLOOPCONTROL_CONF)*NB_AOloopcontrol, NB_AOloopcontrol);
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
		long i;

        AOconf[loop].aorun.on = 0;
        AOconf[loop].aorun.DMprimaryWriteON = 0;
        AOconf[loop].aorun.DMfilteredWriteON = 0;
        AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_ON = 0;
        AOconf[loop].AOAutoTune.AUTOTUNE_GAINS_ON = 0;
        AOconf[loop].aorun.ARPFon = 0;
        AOconf[loop].aorun.ARPFgainAutoMin = 0.00;
        AOconf[loop].aorun.ARPFgainAutoMax = 1.00;
        AOconf[loop].aorun.LOOPiteration = 0;

        AOconf[loop].aorun.cnt = 0;
        

        AOconf[loop].aorun.cntmax = 0;
        AOconf[loop].aorun.init_CMc = 0;

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
        AOloopControl_RTstreamLOG_init(loop);
    }





    if(create==1)
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
        {
            AOconf[loop].aorun.init = 0;
            AOconf[loop].aorun.on = 0;
            AOconf[loop].aorun.DMprimaryWriteON = 0;
            AOconf[loop].aorun.DMfilteredWriteON = 0;
            AOconf[loop].aorun.ARPFon = 0;
            AOconf[loop].aorun.LOOPiteration = 0;
            AOconf[loop].aorun.cnt = 0;
            AOconf[loop].aorun.cntmax = 0;
            AOconf[loop].aorun.maxlimit = 0.3;
            AOconf[loop].aorun.mult = 1.00;
            AOconf[loop].aorun.gain = 0.0;            
            AOconf[loop].aorun.ARPFgain = 0.0;
			AOconf[loop].aorun.ARPFgainAutoMin = 0.99;
			AOconf[loop].aorun.ARPFgainAutoMax = 1.01;
			
			            
            AOconf[loop].AOcompute.ComputeWFSsol_FLAG = 1;
            AOconf[loop].AOcompute.GPUusesem = 1;

            AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_perc = 1.0; // percentile threshold
            AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_mcoeff = 1.0; // multiplicative coeff
            AOconf[loop].AOAutoTune.AUTOTUNE_LIMITS_delta = 1.0e-3;

            AOconf[loop].AOpmodecoeffs.DMmodesNBblock = 1;

            AOconf[loop].WFSim.WFSnormfloor = 0.0;
			AOconf[loop].WFSim.WFSnormalize = 1;
			AOconf[loop].WFSim.WFSrefzero = 0;

            AOconf[loop].AOtiminginfo.loopfrequ = 2000.0;
            AOconf[loop].AOtiminginfo.hardwlatency = 0.0011;
            AOconf[loop].AOtiminginfo.hardwlatency_frame = 2.2;
            AOconf[loop].AOtiminginfo.complatency = 0.0001;
            AOconf[loop].AOtiminginfo.complatency_frame = 0.2;
            AOconf[loop].AOtiminginfo.wfsmextrlatency = 0.0003;
            AOconf[loop].AOtiminginfo.wfsmextrlatency_frame = 0.6;
        }
    }
    else
    {
        for(loop=0; loop<NB_AOloopcontrol; loop++)
            if(AOconf[loop].aorun.init == 1)
            {
                printf("LIST OF ACTIVE LOOPS:\n");
                printf("----- Loop %ld   (%s) ----------\n", loop, AOconf[loop].name);
                printf("  WFS:  %s  [%ld]  %ld x %ld\n", AOconf[loop].WFSim.WFSname, aoloopcontrol_var.aoconfID_wfsim, AOconf[loop].WFSim.sizexWFS, AOconf[loop].WFSim.sizeyWFS);
                printf("   DM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].DMctrl.dmCname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].DMctrl.sizexDM, AOconf[loop].DMctrl.sizeyDM);
                printf("DM RM:  %s  [%ld]  %ld x %ld\n", AOconf[loop].DMctrl.dmRMname, aoloopcontrol_var.aoconfID_dmC, AOconf[loop].DMctrl.sizexDM, AOconf[loop].DMctrl.sizeyDM);
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

