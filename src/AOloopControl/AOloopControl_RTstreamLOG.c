/**
 * @file    AOloopControl_RTstreamLOG.c 
 * @brief   Logging of real-time streams
 * 
 * Real-time stream logging
 *  
 * @author  O. Guyon
 *
 * 
 * 
 */



#define _GNU_SOURCE

#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>


#include <ncurses.h>
#include <termios.h>
#include <fcntl.h> 


#include "CommandLineInterface/CLIcore.h"
#include "00CORE/00CORE.h"
#include "COREMOD_memory/COREMOD_memory.h"
#include "COREMOD_iofits/COREMOD_iofits.h"
#include "ImageStreamIO/ImageStruct.h"

#include "AOloopControl/AOloopControl.h"

extern AOLOOPCONTROL_CONF *AOconf; 
extern AOloopControl_var aoloopcontrol_var; // declared in AOloopControl.c




int AOloopControl_RTstreamLOG_init(int loop)
{
	long i;
	long SIZEwfsim = 3000;
	long SIZEdm = 3000;
	
	// default
	AOconf[loop].RTLOGsize = 9000;
	
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
        {
			AOconf[loop].RTSLOGarray[i].active = 0;
			strcpy(AOconf[loop].RTSLOGarray[i].name, "NULL");
			AOconf[loop].RTSLOGarray[i].ENABLE = 1;
			AOconf[loop].RTSLOGarray[i].INIT = 0;
			AOconf[loop].RTSLOGarray[i].SIZE = AOconf[loop].RTLOGsize;
			AOconf[loop].RTSLOGarray[i].ON = 0;
			AOconf[loop].RTSLOGarray[i].save = 0;
			AOconf[loop].RTSLOGarray[i].saveToggle = 0;
			AOconf[loop].RTSLOGarray[i].NBcubeSaved = -1;
        }
	
	    i = RTSLOGindex_wfsim;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "wfsim"); //   U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].SIZE = SIZEwfsim;
     
        i = RTSLOGindex_imWFS0;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS0");// U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
		AOconf[loop].RTSLOGarray[i].SIZE = SIZEwfsim;
     
        i = RTSLOGindex_imWFS1;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS1");  // U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
        AOconf[loop].RTSLOGarray[i].SIZE = SIZEwfsim;
     
        i = RTSLOGindex_imWFS2;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS2"); // U in AOcompute()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
        AOconf[loop].RTSLOGarray[i].SIZE = SIZEwfsim;
        
        
// managed by AOloopControl_ComputeOpenLoopModes()

        i = RTSLOGindex_modeval;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval"); // U
                
        i = RTSLOGindex_modeval_ol;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_ol");// U 
        AOconf[loop].RTSLOGarray[i].save = 1;
        AOconf[loop].RTSLOGarray[i].ON = 1;                

        i = RTSLOGindex_modeval_dm;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm");// U
                
        i = RTSLOGindex_modeval_dm_corr;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_corr");// U
                        
        i = RTSLOGindex_modeval_dm_now;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now");// U
                                
        i = RTSLOGindex_modeval_dm_now_filt;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now_filt");// U

        i = RTSLOGindex_modevalPF;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPF");// U
                                        
        i = RTSLOGindex_modevalPFsync;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFsync");// U
                
        i = RTSLOGindex_modevalPFres;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFres");// U

        i = RTSLOGindex_dmC;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "dmC");// U

        i = RTSLOGindex_dmdisp;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "dmdisp");// 
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
        AOconf[loop].RTSLOGarray[i].ENABLE = SIZEdm;
	
	
	return 0;
}





int AOloopControl_RTstreamLOG_setup(long loop, long rtlindex, char *streamname)
{
    if(aoloopcontrol_var.RTSLOGarrayInitFlag[RTSLOGindex_modeval] == 1) // ensure local ownership
    {
        if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE == 1)&&(AOconf[loop].RTSLOGarray[rtlindex].INIT == 0))
        {
            long IDstream;
            uint32_t *imsize;
            int retval = 0;
            char imname[500];
            uint64_t nelement;
            long infosize = 5;
            uint8_t atype;
            int SHARED = 1;

            IDstream = image_ID(streamname);

            imsize = (uint32_t*) malloc(sizeof(uint32_t)*3);
            imsize[0] = data.image[IDstream].md[0].size[0];
            imsize[1] = data.image[IDstream].md[0].size[1];
            imsize[2] = AOconf[loop].RTSLOGarray[rtlindex].SIZE;

            atype = data.image[IDstream].md[0].atype;


            if(sprintf(imname, "aol%ld_%s_logbuff0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            AOconf[loop].RTSLOGarray[rtlindex].IDbuff0 = create_image_ID(imname, 3, imsize, atype, SHARED, 0);

            if(sprintf(imname, "aol%ld_%s_logbuff1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            AOconf[loop].RTSLOGarray[rtlindex].IDbuff1 = create_image_ID(imname, 3, imsize, atype, SHARED, 0);

            // nelement for a SINGLE SLICE
            nelement = (uint64_t) imsize[0];
            nelement *= imsize[1];

            switch(atype)
            {
            // memsize for EACH SLICE

            case _DATATYPE_UINT8 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT8*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.UI8;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.UI8;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.UI8;
                break;

            case _DATATYPE_UINT16 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT16*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.UI16;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.UI16;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.UI16;
                break;

            case _DATATYPE_UINT32 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT32*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.UI32;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.UI32;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.UI32;
                break;

            case _DATATYPE_UINT64 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT64*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.UI64;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.UI64;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.UI64;
                break;

            case _DATATYPE_INT8 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT8*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.SI8;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.SI8;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.SI8;
                break;

            case _DATATYPE_INT16 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT16*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.SI16;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.SI16;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.SI16;
                break;

            case _DATATYPE_INT32 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT32*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.SI32;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.SI32;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.SI32;
                break;

            case _DATATYPE_INT64 :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT64*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.SI64;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.SI64;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.SI64;
                break;

            case _DATATYPE_FLOAT :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_FLOAT*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.F;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.F;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.F;
                break;

            case _DATATYPE_DOUBLE :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_DOUBLE*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.D;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.D;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.D;
                break;

            case _DATATYPE_COMPLEX_FLOAT :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_COMPLEX_FLOAT*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.CF;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.CF;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (void*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.CF;
                break;

            case _DATATYPE_COMPLEX_DOUBLE :
                AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_COMPLEX_DOUBLE*nelement);
                AOconf[loop].RTSLOGarray[rtlindex].srcptr      = (void*) data.image[IDstream].array.CD;
                AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.CD;
                AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.CD;
                break;

            default :
                printf("Unknown data type\n");
                exit(0);
                break;
            }



            imsize[0] = infosize;
            imsize[1] = AOconf[loop].RTSLOGarray[rtlindex].SIZE;
            imsize[2] = 1;

            if(sprintf(imname, "aol%ld_%s_logbuffinfo0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);

            if(sprintf(imname, "aol%ld_%s_logbuffinfo1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
                printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
            AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo1 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);



            AOconf[loop].RTSLOGarray[rtlindex].buffindex   = 0;
            AOconf[loop].RTSLOGarray[rtlindex].frameindex  = 0;
            AOconf[loop].RTSLOGarray[rtlindex].destptr     = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
            AOconf[loop].RTSLOGarray[rtlindex].IDbuff      = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
            AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo  = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;


            retval = 1;
            AOconf[loop].RTSLOGarray[rtlindex].INIT = 1;

            free(imsize);

        }
    }


    return(0);
}



//
// write single entry in log buffer
//
void AOloopControl_RTstreamLOG_update(long loop, long rtlindex, struct timespec tnow)
{
	
	
	if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE==1) && (AOconf[loop].RTSLOGarray[rtlindex].ON==1) && (AOconf[loop].RTSLOGarray[rtlindex].INIT = 1))
	{
		char *dataptr;
		dataptr = AOconf[loop].RTSLOGarray[rtlindex].destptr + AOconf[loop].RTSLOGarray[rtlindex].memsize * AOconf[loop].RTSLOGarray[rtlindex].frameindex;

/*
		printf("===== STEP ==== %s   %d\n", __FILE__, __LINE__);//TEST
		list_image_ID();
		printf("rtlindex = %ld\n", rtlindex);
		printf("AOconf[loop].RTSLOGarray[rtlindex].destptr    = %p\n", (void*) AOconf[loop].RTSLOGarray[rtlindex].destptr);
		printf("AOconf[loop].RTSLOGarray[rtlindex].frameindex = %ld\n", AOconf[loop].RTSLOGarray[rtlindex].frameindex);
		printf("AOconf[loop].RTSLOGarray[rtlindex].memsize    = %ld\n", (long) AOconf[loop].RTSLOGarray[rtlindex].memsize);
		fflush(stdout);		

sleep(1000);*/


		memcpy((void*) dataptr, 
		(void*) AOconf[loop].RTSLOGarray[rtlindex].srcptr, 
		AOconf[loop].RTSLOGarray[rtlindex].memsize);

/*		printf("===== STEP ==== %s   %d\n", __FILE__, __LINE__);//TEST
		fflush(stdout);		*/

		long IDinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo;
		
		
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5  ] = AOconf[loop].LOOPiteration;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+1] = (long) tnow.tv_sec;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+2] = (long) tnow.tv_nsec;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+3] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt0;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+4] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt1;

		
		
		AOconf[loop].RTSLOGarray[rtlindex].frameindex++;
		if(AOconf[loop].RTSLOGarray[rtlindex].frameindex == AOconf[loop].RTSLOGarray[rtlindex].SIZE)
		{
			AOconf[loop].RTSLOGarray[rtlindex].frameindex = 0;
		
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].cnt0++;
			data.image[IDinfo].md[0].cnt0++;
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].write = 0;
			data.image[IDinfo].md[0].write = 0;
			COREMOD_MEMORY_image_set_sempost_byID(AOconf[loop].RTSLOGarray[rtlindex].IDbuff, -1);
			COREMOD_MEMORY_image_set_sempost_byID(IDinfo, -1);
		
			if(AOconf[loop].RTSLOGarray[rtlindex].buffindex==0)
			{
				AOconf[loop].RTSLOGarray[rtlindex].buffindex = 1;
				AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr1;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff1;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo1;
				if(AOconf[loop].RTSLOGarray[rtlindex].save==1)
					AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 1;
			}
			else
			{
				AOconf[loop].RTSLOGarray[rtlindex].buffindex = 0;
				AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
				if(AOconf[loop].RTSLOGarray[rtlindex].save==1)
					AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 2;
			}
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].write = 1;
			data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo].md[0].write = 1;
		}
		
	}
	
}






int AOloopControl_RTstreamLOG_printstatus(int loop)
{
	long NBstreams = 0;
	
	char ENstring[20];
	char ONstring[20];
	char INstring[20];
	char SAstring[20];
	int i;
	
	
	printf("INITIALIZING MEMORY\n");
    fflush(stdout);
	
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    printf("MEMORY HAS BEEN INITIALIZED\n");
    fflush(stdout);
    
    
	printf("\n");
	printf("RTLOGsize = %ld\n", AOconf[loop].RTLOGsize);
	printf("%2s  %20s  %3s %3s %3s %6s %4s %10s %10s  %5s\n", "id", "streamname", "ENA", " ON", "INI", "SAVE", "buff", "frame", "memsize", "size");
	printf("---------------------------------------------------------------------------\n");
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
	{
		if(AOconf[loop].RTSLOGarray[i].active == 1)
		{
			
			if(AOconf[loop].RTSLOGarray[i].ENABLE == 1)
				sprintf(ENstring, "\033[1;32m ON\033[0m");
			else
				sprintf(ENstring, "OFF");

			if(AOconf[loop].RTSLOGarray[i].ON == 1)
				sprintf(ONstring, "\033[1;32m ON\033[0m");
			else
				sprintf(ONstring, "OFF");
			
			if(AOconf[loop].RTSLOGarray[i].INIT == 1)
				sprintf(INstring, "\033[1;32m ON\033[0m");
			else
				sprintf(INstring, "OFF");

			if(AOconf[loop].RTSLOGarray[i].save == 1)
			{
				if(AOconf[loop].RTSLOGarray[i].saveToggle!=0)
					sprintf(SAstring, "\033[1;31m ON[%1d]\033[0m", AOconf[loop].RTSLOGarray[i].saveToggle);
				else
					sprintf(SAstring, "\033[1;32m ON[%1d]\033[0m", AOconf[loop].RTSLOGarray[i].saveToggle);
			}
			else
				sprintf(SAstring, "OFF   ");						
  
  
			printf("%2d  %20s  %3s %3s %3s %6s %4d %10ld %10ld  %5d\n", i,  
			AOconf[loop].RTSLOGarray[i].name, 
			ENstring, 
			ONstring, 
			INstring, 
			SAstring,			
			AOconf[loop].RTSLOGarray[i].buffindex,
			AOconf[loop].RTSLOGarray[i].frameindex,
			AOconf[loop].RTSLOGarray[i].memsize,
			AOconf[loop].RTSLOGarray[i].SIZE
			);
			NBstreams++;			
		}
	}
	printf("---------------------------------------------------------------------------\n");
	printf("%ld RTstreamLOGs active\n", NBstreams);
	
	return NBstreams;
}











int print_header(const char *str, char c, int wcol)
{
    long n;
    long i;

    attron(A_BOLD);
    n = strlen(str);
    for(i=0; i<(wcol-n)/2; i++)
        printw("%c",c);
    printw("%s", str);
    for(i=0; i<(wcol-n)/2-1; i++)
        printw("%c",c);
    printw("\n");
    attroff(A_BOLD);

    return(0);
}





int AOloopControl_RTstreamLOG_GUI(int loop)
{
    int wrow, wcol;
    float frequ = 10.0;
    int ch;

	long NBstreams = 0;
	
	char ENstring[20];
	char ONstring[20];
	char INstring[20];
	char SAstring[20];
	int i, j;
	
	int selected_entry = 0;
	int ENAstream[MAX_NUMBER_RTLOGSTREAM];
	int SaveSet[MAX_NUMBER_RTLOGSTREAM];
	
	
	printf("INITIALIZING MEMORY\n");
    fflush(stdout);
	
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

    printf("MEMORY HAS BEEN INITIALIZED\n");
    fflush(stdout);
    

	

    /*  Initialize ncurses  */
    if ( initscr() == NULL ) {
        fprintf(stderr, "Error initialising ncurses.\n");
        exit(EXIT_FAILURE);
    }
    getmaxyx(stdscr, wrow, wcol);		/* get the number of rows and columns */
    cbreak();
    keypad(stdscr, TRUE);		/* We get F1, F2 etc..		*/
    nodelay(stdscr, TRUE);
    curs_set(0);
    noecho();			/* Don't echo() while we do getch */



	start_color();
    init_pair(1, COLOR_BLACK, COLOR_WHITE);
    init_pair(2, COLOR_BLACK, COLOR_RED);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_RED,   COLOR_BLACK);

	for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
		SaveSet[i] = 0;

	int loopOK = 1;
	int NB_ENA_streams = 0;
    while ( loopOK == 1 )
    {
		NBstreams = 0;

        usleep((long) (1000000.0/frequ));
        ch = getch();
        clear();
        attron(A_BOLD);
        print_header(" PRESS x TO STOP MONITOR ", '-', wcol);
        printw("  s: Save single    t: Add/remove to/from set   S: Save set ON   U: Save set OFF\n");
        printw("  o: ON/OFF single  O: Set ON   F: Set OFF      Z: Zero index set\n");
        attroff(A_BOLD);

        printw("\n");
        printw("RTLOGsize = %ld\n", AOconf[loop].RTLOGsize);
        printw("%2s  %20s  %3s %3s   %3s %6s %8s %6s %10s  %5s\n", 
			"id", 
			"streamname", 
			"ENA", 
			" ON", 
			"INI", 
			"SAVE", 
			"buff", 
			"frame", 
			"memsize", 
			"size");
		
	//	printw("KEY = %d\n", ch);
		

		
		switch (ch)
		{
			case KEY_DOWN:
			selected_entry++;
			break;
		
			case KEY_UP:
			selected_entry--;
			break;
			
			
			case 's': 
			j = ENAstream[selected_entry];
			if(AOconf[loop].RTSLOGarray[j].save == 1)
				AOconf[loop].RTSLOGarray[j].save = 0;
			else
				AOconf[loop].RTSLOGarray[j].save = 1;
			break;

			case 'o': 
			j = ENAstream[selected_entry];
			if(AOconf[loop].RTSLOGarray[j].ON == 1)
				AOconf[loop].RTSLOGarray[j].ON = 0;
			else
				AOconf[loop].RTSLOGarray[j].ON = 1;
			break;
			
			
			
			case 'S': 
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
					AOconf[loop].RTSLOGarray[i].save = 1;
			}
			break;
		
			case 'U': 
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
					AOconf[loop].RTSLOGarray[i].save = 0;
			}
			break;		
			
			
			case 'O': 
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
					AOconf[loop].RTSLOGarray[i].ON = 1;
			}
			break;
		
			case 'F': 
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
					AOconf[loop].RTSLOGarray[i].ON = 0;
			}
			break;		
			
			
			
			case 'Z':  // Zero indices of set - useful for synchronization
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
					AOconf[loop].RTSLOGarray[i].frameindex = 0;
			}
			break;
			
			
			case 'n':  // Synchro and Save only one cube for set
			for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
			{
				if(SaveSet[i]==1)
				{				
					AOconf[loop].RTSLOGarray[i].frameindex = 0;
					AOconf[loop].RTSLOGarray[i].save = 1;
					AOconf[loop].RTSLOGarray[i].NBcubeSaved = 1;
				}
				
			}
			break;
						
			
			case 't': 
			j = ENAstream[selected_entry];
			if(SaveSet[j] == 1)
				SaveSet[j] = 0;
			else
				SaveSet[j] = 1;
			break;
			
			
			case 'x': // exit
			loopOK = 0;
			break;
			
		}
		
		if (selected_entry<0)
			selected_entry = 0;
		if (selected_entry>NB_ENA_streams-1)
			selected_entry = NB_ENA_streams-1;
		
		
        printw("---------------------------------------------------------------------------\n");
        
        
        NB_ENA_streams = 0;
        for(i=0; i<MAX_NUMBER_RTLOGSTREAM; i++)
        {
			
            if(AOconf[loop].RTSLOGarray[i].active == 1)
            {
				if(i==ENAstream[selected_entry])
				{
					attron(A_REVERSE|A_BOLD);
					printw("%2d  %20s", i, AOconf[loop].RTSLOGarray[i].name);
					attroff(A_REVERSE|A_BOLD);
				}
				else
				{
					printw("%2d  %20s", i, AOconf[loop].RTSLOGarray[i].name);
				}

                if(AOconf[loop].RTSLOGarray[i].ENABLE == 1){
					ENAstream[NB_ENA_streams] = i;
					NB_ENA_streams++;
					
                    attron(COLOR_PAIR(3)|A_BOLD);
                    printw("   ON");
                    attroff(COLOR_PAIR(3)|A_BOLD);
				}
                else
                    printw("  OFF");

                if(AOconf[loop].RTSLOGarray[i].ON == 1)
                {
					attron(COLOR_PAIR(3)|A_BOLD);
                    printw("   ON");
                    attroff(COLOR_PAIR(3)|A_BOLD);
                }
                else
                    printw("  OFF");

                if(AOconf[loop].RTSLOGarray[i].INIT == 1)
                {
					attron(COLOR_PAIR(3)|A_BOLD);
                    printw("   ON");
                    attroff(COLOR_PAIR(3)|A_BOLD);
                }
                else
                    printw("  OFF");
                    
                
                if(SaveSet[i] == 1)
                {
					attron(A_REVERSE);
					printw("   S");
					attroff(A_REVERSE);
				}
				else
					printw("   -");

                if(AOconf[loop].RTSLOGarray[i].save == 1)
                {
                    if(AOconf[loop].RTSLOGarray[i].saveToggle!=0)
                    {
						attron(COLOR_PAIR(2)|A_BOLD);
                        printw(" ON[%1d]", AOconf[loop].RTSLOGarray[i].saveToggle);
                        attroff(COLOR_PAIR(2)|A_BOLD);
                    }
                    else
                    {
						attron(COLOR_PAIR(3)|A_BOLD);
                        printw(" ON[%1d]", AOconf[loop].RTSLOGarray[i].saveToggle);
                        attroff(COLOR_PAIR(3)|A_BOLD);
					}
                }
                else
                    printw(" OFF  ");


                printw(" %4d %10ld %10ld  %5d\n",
                       AOconf[loop].RTSLOGarray[i].buffindex,
                       AOconf[loop].RTSLOGarray[i].frameindex,
                       AOconf[loop].RTSLOGarray[i].memsize,
                       AOconf[loop].RTSLOGarray[i].SIZE
                      );
                NBstreams++;
            }
        }
        printw("---------------------------------------------------------------------------\n");
        printw("%ld RTstreamLOGs active\n", NBstreams);

		
        refresh();
    }
    


    refresh();
    endwin();


    return 0;
}




int AOloopControl_RTstreamLOG_set_saveON(int loop, int rtlindex)
{
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);
    
    if(rtlindex<MAX_NUMBER_RTLOGSTREAM)
		AOconf[loop].RTSLOGarray[rtlindex].save = 1;
	
	AOloopControl_RTstreamLOG_printstatus(loop);
	
	return 0;
}





int AOloopControl_RTstreamLOG_set_saveOFF(int loop, int rtlindex)
{
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);
    
    if(rtlindex<MAX_NUMBER_RTLOGSTREAM)
		AOconf[loop].RTSLOGarray[rtlindex].save = 0;

	AOloopControl_RTstreamLOG_printstatus(loop);

	return 0;
}


int AOloopControl_RTstreamLOG_set_ON(int loop, int rtlindex)
{
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);
    
    if(rtlindex<MAX_NUMBER_RTLOGSTREAM)
		AOconf[loop].RTSLOGarray[rtlindex].ON = 1;
	
	AOloopControl_RTstreamLOG_printstatus(loop);
	
	return 0;
}


int AOloopControl_RTstreamLOG_set_OFF(int loop, int rtlindex)
{
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);
    
    if(rtlindex<MAX_NUMBER_RTLOGSTREAM)
		AOconf[loop].RTSLOGarray[rtlindex].ON = 0;

	AOloopControl_RTstreamLOG_printstatus(loop);

	return 0;
}







int AOloopControl_RTstreamLOG_saveloop(int loop, char *dirname)
{
    int rtlindex;
    int cntsave = 0;

    int sleeptimeus = 10000;
    long sleepcnt = 0;

    tzset();

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    printf("\n");

    for(;;)
    {
        cntsave = 0;
        for(rtlindex=0; rtlindex<MAX_NUMBER_RTLOGSTREAM; rtlindex++)
        {
            if(AOconf[loop].RTSLOGarray[rtlindex].save == 1)
            {
				int buff;
				int SAVEfile = 0; // toggles to 1 if file needs to be saved
				
                // support for saving partial cube if loop turns off
                long NBframe;
                uint32_t ID;
                uint32_t zsizesave;

				if(AOconf[loop].RTSLOGarray[rtlindex].saveToggle!=0) // FULL CUBE
				{
					SAVEfile = 1;
					NBframe = AOconf[loop].RTSLOGarray[rtlindex].SIZE;
					buff = AOconf[loop].RTSLOGarray[rtlindex].saveToggle - 1; // buffindex to save
					
					// in case a finite number of full cubes is to be saved
					if(AOconf[loop].RTSLOGarray[rtlindex].NBcubeSaved >= 0)
						{
							AOconf[loop].RTSLOGarray[rtlindex].NBcubeSaved --;
						}
					if(AOconf[loop].RTSLOGarray[rtlindex].NBcubeSaved == 0)
						AOconf[loop].RTSLOGarray[rtlindex].save = 0;
				}
				else // TEST if loop is off and partial buffer needs to be saved
				{
					if((AOconf[loop].on == 0)&&(AOconf[loop].RTSLOGarray[rtlindex].frameindex>0))
					{
						SAVEfile = 1;
						NBframe = AOconf[loop].RTSLOGarray[rtlindex].frameindex;
						buff = AOconf[loop].RTSLOGarray[rtlindex].buffindex;
						AOconf[loop].RTSLOGarray[rtlindex].frameindex = 0;
						printf("------- LOOP OFF -> SAVING PARTIAL CUBE\n");
					}
					
				}



                if(SAVEfile == 1)
                {
                    char shmimname[200];
                    char shmimnameinfo[200];

                    char fname[500];
                    char fnameinfo[500];

                    long IDin, IDininfo;
                    time_t TSsec;
                    long TSnsec;

                    struct tm *uttime;
                    char timestring[100];
                    char fulldir0[500];
                    char fulldir1[500];
                    char fulldir2[500];

                    FILE *fp;
                    long i;


                    printf("\n\n   SAVING \033[1;32m%s\033[0m buffer (%d)\n", AOconf[loop].RTSLOGarray[rtlindex].name, rtlindex);

                    if(sprintf(shmimname, "aol%d_%s_logbuff%d", loop, AOconf[loop].RTSLOGarray[rtlindex].name, buff) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
                    if(sprintf(shmimnameinfo, "aol%d_%s_logbuffinfo%d", loop, AOconf[loop].RTSLOGarray[rtlindex].name, buff) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");





                    if((IDin = image_ID(shmimname))==-1)
                        IDin = read_sharedmem_image(shmimname);

                    if((IDininfo = image_ID(shmimnameinfo))==-1)
                        IDininfo = read_sharedmem_image(shmimnameinfo);

                    // reading first frame timestamp
                    TSsec  = (time_t) data.image[IDininfo].array.UI64[1];
                    TSnsec = data.image[IDininfo].array.UI64[2];
                    uttime = gmtime(&TSsec);



                    sprintf(timestring, "%02d:%02d:%02d.%09ld", uttime->tm_hour, uttime->tm_min,  uttime->tm_sec, TSnsec);

                    sprintf(fulldir0, "%s", dirname);
                    sprintf(fulldir1, "%s/%04d%02d%02d", dirname, 1900+uttime->tm_year, 1+uttime->tm_mon, uttime->tm_mday);
                    sprintf(fulldir2, "%s/aol%d_%s", fulldir1, loop, AOconf[loop].RTSLOGarray[rtlindex].name);


                    struct stat st = {0};

                    if (stat(fulldir0, &st) == -1) {
                        printf("\033[1;31m CREATING DIRECTORY %s \033[0m\n", fulldir0);
                        mkdir(fulldir0, 0777);
                    }
                    if (stat(fulldir1, &st) == -1) {
                        printf("\033[1;31m CREATING DIRECTORY %s \033[0m\n", fulldir1);
                        mkdir(fulldir1, 0777);
                    }
                    if (stat(fulldir2, &st) == -1) {
                        printf("\033[1;31m CREATING DIRECTORY %s \033[0m\n", fulldir2);
                        mkdir(fulldir2, 0777);
                    }


                    if(sprintf(fnameinfo, "%s/aol%d_%s.%s.dat", fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name, timestring) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    if(sprintf(fname, "%s/aol%d_%s.%s.fits", fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name, timestring) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");




                    printf(" TIME STAMP :  %9ld.%09ld  -> %s\n", (long) TSsec, TSnsec, timestring);
                    printf("       %s -> %s\n", shmimname    , fname);
                    printf("       %s -> %s\n", shmimnameinfo, fnameinfo);


                    ID = image_ID(shmimname);
                    zsizesave = data.image[ID].md[0].size[2];
                    data.image[ID].md[0].size[2] = NBframe;
                    save_fits(shmimname, fname);
                    data.image[ID].md[0].size[2] = zsizesave;


                    long double t0 = data.image[IDininfo].array.UI64[1] + 1.0e-9*data.image[IDininfo].array.UI64[2];
                    fp = fopen(fnameinfo, "w");
                    for(i=0; i<NBframe; i++)
                    {
                        long double t1 = data.image[IDininfo].array.UI64[i*5+1] + 1.0e-9*data.image[IDininfo].array.UI64[i*5+2];
                        fprintf(fp, "%10ld  %10ld  %15.9lf   %010ld.%09ld  %10ld   %10ld\n", i, data.image[IDininfo].array.UI64[i*5], (double) (t1-t0), data.image[IDininfo].array.UI64[i*5+1], data.image[IDininfo].array.UI64[i*5+2], data.image[IDininfo].array.UI64[i*5+3], data.image[IDininfo].array.UI64[i*5+4]);
                    }
                    fclose(fp);


                    AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 0;
                    cntsave++;
                }
            }
        }
        if(cntsave>0)
        {
            printf("%d buffer(s) saved\n", cntsave);
            printf("\n");
            sleepcnt = 0;
        }
        else
        {
            printf("waiting for buffers ready to save %10.3f sec \r", 1.0e-6*sleepcnt*sleeptimeus);
            fflush(stdout);
            sleepcnt ++;
        }

        usleep(sleeptimeus);
    }



    return 0;
}


