/**
 * @file    AOloopControl_RTstreamLOG.c 
 * @brief   Logging of real-time streams
 * 
 * Real-time stream logging
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
#include <pthread.h>
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
	long SIZEwfsim = 1000;
	long SIZEdm = 1000;
	
	// default
	AOconf[loop].RTLOGsize = 1000;
	
	
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
        {
			AOconf[loop].RTSLOGarray[i].active = 0;
			strcpy(AOconf[loop].RTSLOGarray[i].name, "NULL");
			AOconf[loop].RTSLOGarray[i].ENABLE = 1;
			AOconf[loop].RTSLOGarray[i].INIT = 0;
			AOconf[loop].RTSLOGarray[i].SIZE = AOconf[loop].RTLOGsize;
			AOconf[loop].RTSLOGarray[i].ON = 0;
			AOconf[loop].RTSLOGarray[i].save = 0;
			AOconf[loop].RTSLOGarray[i].memcpToggle = 0;
			AOconf[loop].RTSLOGarray[i].NBcubeSaved = -1;
			
			AOconf[loop].RTSLOGarray[i].NBFileBuffer = 30; // number of buffers combined to create large buffer = file to disk
			AOconf[loop].RTSLOGarray[i].FileBuffer   = 0;
			
			AOconf[loop].RTSLOGarray[i].tActive = 0;
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
            AOconf[loop].RTSLOGarray[rtlindex].FileBuffer  = 0;
            AOconf[loop].RTSLOGarray[rtlindex].destptr     = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
            AOconf[loop].RTSLOGarray[rtlindex].IDbuff      = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
            AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo  = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;


            retval = 1;
            AOconf[loop].RTSLOGarray[rtlindex].INIT = 1;

            free(imsize);

        }

    return(0);
}





/**
 * ## Purpose
 * 
 * Write single entry in log buffer
 * 
 * This function is called by the process writing to the stream
 * 
 */
void AOloopControl_RTstreamLOG_update(long loop, long rtlindex, struct timespec tnow)
{
	
    if(aoloopcontrol_var.RTSLOGarrayInitFlag[rtlindex] == 1) // ensure local ownership
    {
        if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE==1) && (AOconf[loop].RTSLOGarray[rtlindex].ON==1) && (AOconf[loop].RTSLOGarray[rtlindex].INIT = 1))
        {	
            char *dataptr;
            dataptr = AOconf[loop].RTSLOGarray[rtlindex].destptr + AOconf[loop].RTSLOGarray[rtlindex].memsize * AOconf[loop].RTSLOGarray[rtlindex].frameindex;
            
            memcpy((void*) dataptr,
                   (void*) AOconf[loop].RTSLOGarray[rtlindex].srcptr,
                   AOconf[loop].RTSLOGarray[rtlindex].memsize);

            long IDinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo;


            data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5  ] = AOconf[loop].aorun.LOOPiteration;
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
                        AOconf[loop].RTSLOGarray[rtlindex].memcpToggle = 1;
                }
                else
                {
                    AOconf[loop].RTSLOGarray[rtlindex].buffindex = 0;
                    AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
                    AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
                    AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
                    
                    if(AOconf[loop].RTSLOGarray[rtlindex].save==1)
                        AOconf[loop].RTSLOGarray[rtlindex].memcpToggle = 2;
                }
                data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff].md[0].write = 1;
                data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo].md[0].write = 1;
            }

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
				if(AOconf[loop].RTSLOGarray[i].memcpToggle!=0)
					sprintf(SAstring, "\033[1;31m ON[%1d]\033[0m", AOconf[loop].RTSLOGarray[i].memcpToggle);
				else
					sprintf(SAstring, "\033[1;32m ON[%1d]\033[0m", AOconf[loop].RTSLOGarray[i].memcpToggle);
			}
			else
				sprintf(SAstring, "OFF   ");						
  
  
			printf("%2d  %20s  %3s %3s %3s %6s %2d %8ld %8ld %5d x %d\n", i,  
			AOconf[loop].RTSLOGarray[i].name, 
			ENstring, 
			ONstring, 
			INstring, 
			SAstring,			
			AOconf[loop].RTSLOGarray[i].buffindex,
			AOconf[loop].RTSLOGarray[i].frameindex,
			AOconf[loop].RTSLOGarray[i].memsize,
			AOconf[loop].RTSLOGarray[i].SIZE,
			AOconf[loop].RTSLOGarray[i].NBFileBuffer
			);
			NBstreams++;			
		}
	}
	printf("---------------------------------------------------------------------------\n");
	printf("%ld RTstreamLOGs active\n", NBstreams);
	
	return NBstreams;
}











static int print_header_line(const char *str, char c, int wcol)
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
        fprintf(stderr, "Error initializing ncurses.\n");
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
	clear();
    while ( loopOK == 1 )
    {
		char title[200];
		
		NBstreams = 0;

        usleep((long) (1000000.0/frequ));
        ch = getch();
        erase();
        attron(A_BOLD);
        sprintf(title, "LOOP %d  REAL-TIME STREAMS MONITOR      PRESS x TO STOP MONITOR");
        print_header_line(" PRESS x TO STOP MONITOR ", '-', wcol);
        printw("  s: Save single    t: Add/remove to/from set   S: Save set ON   U: Save set OFF\n");
        printw("  o: ON/OFF single  O: Set ON   F: Set OFF      Z: Zero index set\n");
        attroff(A_BOLD);

        printw("\n");
        printw("RTLOGsize = %ld\n", AOconf[loop].RTLOGsize);
        printw("%2s  %20s  %3s %3s   %3s %6s %8s %6s x %4s %10s  %5s\n", 
			"id", 
			"streamname", 
			"ENA", 
			" ON", 
			"INI", 
			"SAVE", 
			"buff", 
			"frame",
			"buff", 
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
				//if(SaveSet[i]==1){
					AOconf[loop].RTSLOGarray[i].FileBuffer = 0;
					AOconf[loop].RTSLOGarray[i].frameindex = 0;
				//}
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
		
		
        printw("-------------------------------------------------------------------------------------\n");
        
        
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
                    if(AOconf[loop].RTSLOGarray[i].tActive!=0)
                    {
						attron(COLOR_PAIR(2)|A_BOLD);
                        printw(" ON[%1d]", AOconf[loop].RTSLOGarray[i].memcpToggle);
                        attroff(COLOR_PAIR(2)|A_BOLD);
                    }
                    else
                    {
						attron(COLOR_PAIR(3)|A_BOLD);
                        printw(" ON[%1d]", AOconf[loop].RTSLOGarray[i].memcpToggle);
                        attroff(COLOR_PAIR(3)|A_BOLD);
					}
                }
                else
                    printw(" OFF  ");


                printw(" %2d %8ld x %2d  %8ld  %5d (x %d)\n",
                       AOconf[loop].RTSLOGarray[i].buffindex,
                       AOconf[loop].RTSLOGarray[i].frameindex,
                       AOconf[loop].RTSLOGarray[i].FileBuffer,
                       AOconf[loop].RTSLOGarray[i].memsize,
                       AOconf[loop].RTSLOGarray[i].SIZE,
                       AOconf[loop].RTSLOGarray[i].NBFileBuffer
                      );
                NBstreams++;
            }
        }
        printw("-------------------------------------------------------------------------------------\n");
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











/**
 * ## Purpose
 *
 * Monitors multiple real-time AO loop buffers and saves them to disk
 *
 * ## Arguments
 *
 * Loop index and save directory on disk
 *
 * ## Use
 *
 * This function runs in a dedicated in a dedicated tmux session.\n
 * It is controlled by a ncurses-based GUI.\n
 *
 * ## Details
 *
 * Each real-time process maintains, for each streams, small alternating buffers where data
 * is written at high speed. Timing/indexing buffers are also written.
 *
 * This function monitors the small buffers. When a small buffer is ready/filled, it is included
 * in a larger buffer which can be saved to disk.
 *
 *
 * The routine is designed to only occupy two threads: one to monitor, one to save
 *
 * Takes timing data from the processes that create the data to achieve high performance timing
 */

int AOloopControl_RTstreamLOG_saveloop(
    int loop,
    char *dirname
)
{
    int VERBOSE = 1;
    int NBthreads = 0;
    int NBthreadsActive = 0;
    int rtlindex;
    int cntsave = 0;

    int sleeptimeus = 1000; // 1 ms
    long sleepcnt = 0;



    /*
    	pthread_t thread_savefits;
    	int tOK = 0;
        int iret_savefits;
        //	char tmessage[500];
        struct savethreadmsg *tmsg = malloc(sizeof(struct savethreadmsg));

    */

    /**
     * ### Initialization
     *
     * Intialize :
     * - time conversion information
     * - AOloopControl memory
     * - save thread message array
     * - processinfo
     *
     */

    tzset();

    if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);


    pthread_t thread_savefits[MAX_NUMBER_RTLOGSTREAM];
    int tOK[MAX_NUMBER_RTLOGSTREAM];          // is thread alive ?

    int iret_savefits[MAX_NUMBER_RTLOGSTREAM];
    STREAMSAVE_THREAD_MESSAGE *savethreadmsg_array;
    savethreadmsg_array = (STREAMSAVE_THREAD_MESSAGE*) malloc(sizeof(STREAMSAVE_THREAD_MESSAGE)*MAX_NUMBER_RTLOGSTREAM);

    int thd;
    for(thd=0; thd<MAX_NUMBER_RTLOGSTREAM; thd++)
    {
        tOK[thd] = 0;
        AOconf[loop].RTSLOGarray[thd].tActive = 0;
    }


    PROCESSINFO *processinfo;
    if(data.processinfo==1)
    {
        // CREATE PROCESSINFO ENTRY
        // see processtools.c in module CommandLineInterface for details
        //
        char pinfoname[200];
        sprintf(pinfoname, "aol%d-RealTimeTelemetrySave", loop);
        processinfo = processinfo_shm_create(pinfoname, 0);
        processinfo->loopstat = 0; // loop initialization

        strcpy(processinfo->source_FUNCTION, __FUNCTION__);
        strcpy(processinfo->source_FILE,     __FILE__);
        processinfo->source_LINE = __LINE__;

        char msgstring[200];
        sprintf(msgstring, "Real-time telemetry, loop %d", loop);
        processinfo_WriteMessage(processinfo, msgstring);
    }



    // Catch signals

    if (sigaction(SIGTERM, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGTERM\n");

    if (sigaction(SIGINT, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGINT\n");

    if (sigaction(SIGABRT, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGABRT\n");

    if (sigaction(SIGBUS, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGBUS\n");

    if (sigaction(SIGSEGV, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGSEGV\n");

    if (sigaction(SIGHUP, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGHUP\n");

    if (sigaction(SIGPIPE, &data.sigact, NULL) == -1)
        printf("\ncan't catch SIGPIPE\n");




    for(rtlindex=0; rtlindex<MAX_NUMBER_RTLOGSTREAM; rtlindex++)
    {
        AOconf[loop].RTSLOGarray[rtlindex].save = 0;
        AOconf[loop].RTSLOGarray[rtlindex].memcpToggle = 0;
        AOconf[loop].RTSLOGarray[rtlindex].NBcubeSaved = -1;

        AOconf[loop].RTSLOGarray[rtlindex].frameindex = 0;
        AOconf[loop].RTSLOGarray[rtlindex].FileBuffer   = 0;

        AOconf[loop].RTSLOGarray[rtlindex].tActive = 0;
    }


    printf("\n");


    long double t0; // time reference for differential timer

    if(data.processinfo == 1)
        processinfo->loopstat = 1;

    int loopOK = 1;
    long loopcnt = 0;


    while(loopOK == 1)
    {
        if(data.processinfo==1)
        {
            while(processinfo->CTRLval == 1)  // pause
                usleep(50);

            if(processinfo->CTRLval == 2) // single iteration
                processinfo->CTRLval = 1;

            if(processinfo->CTRLval == 3) // exit loop
            {
                loopOK = 0;
            }
        }

        if((data.processinfo==1)&&(processinfo->MeasureTiming==1))
            processinfo_exec_start(processinfo);

        NBthreadsActive = 0;
        for(rtlindex=0; rtlindex<MAX_NUMBER_RTLOGSTREAM; rtlindex++)
            if((tOK[rtlindex]==1)&&(AOconf[loop].RTSLOGarray[rtlindex].tActive==1))
                NBthreadsActive++;
        if(data.processinfo==1)
        {
            char msgstring[200];
            sprintf(msgstring, "%02d/%02d save threads", NBthreadsActive, NBthreads);
            processinfo_WriteMessage(processinfo, msgstring);
        }


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



                // check thread activity
                if((tOK[rtlindex]==1)&&(AOconf[loop].RTSLOGarray[rtlindex].tActive==1))
                {
                    if(pthread_tryjoin_np(thread_savefits[rtlindex], NULL) == 0)
                        AOconf[loop].RTSLOGarray[rtlindex].tActive = 0;
                }



                if(AOconf[loop].RTSLOGarray[rtlindex].memcpToggle!=0) // FULL CUBE READY
                {
                    SAVEfile = 1;
                    NBframe = AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                    buff = AOconf[loop].RTSLOGarray[rtlindex].memcpToggle - 1; // buffindex to save

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
                    if((AOconf[loop].aorun.on == 0)&&(AOconf[loop].RTSLOGarray[rtlindex].frameindex>0))
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

                    char fnameFITS[500];
                    char fnameinfo[500];

                    long IDin, IDininfo;
                    time_t TSsec;
                    long TSnsec;

                    struct tm *uttime;
                    // char timestring[100];
                    char fulldir0[500];
                    char fulldir1[500];
                    char fulldir2[500];

                    FILE *fp;
                    long i;


                    // printf("\n\n   SAVING \033[1;32m%s\033[0m buffer (%d)\n", AOconf[loop].RTSLOGarray[rtlindex].name, rtlindex);

                    if(sprintf(shmimname, "aol%d_%s_logbuff%d", loop, AOconf[loop].RTSLOGarray[rtlindex].name, buff) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
                    if(sprintf(shmimnameinfo, "aol%d_%s_logbuffinfo%d", loop, AOconf[loop].RTSLOGarray[rtlindex].name, buff) < 1)
                        printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");



                    if((IDin = image_ID(shmimname))==-1)
                    {
                        printf("IMPORTING stream %s buffer %d :  %s\n", AOconf[loop].RTSLOGarray[rtlindex].name, rtlindex, shmimname);
                        IDin = read_sharedmem_image(shmimname);
                    }

                    if((IDininfo = image_ID(shmimnameinfo))==-1)
                    {
                        printf("IMPORTING stream %s buffer %d : buffer %s\n", AOconf[loop].RTSLOGarray[rtlindex].name, rtlindex, shmimnameinfo);
                        IDininfo = read_sharedmem_image(shmimnameinfo);
                    }

                    // reading first frame timestamp
                    TSsec  = (time_t) data.image[IDininfo].array.UI64[1];
                    TSnsec = data.image[IDininfo].array.UI64[2];
                    uttime = gmtime(&TSsec);


                    sprintf(AOconf[loop].RTSLOGarray[rtlindex].timestring, "%02d:%02d:%02d.%09ld", uttime->tm_hour, uttime->tm_min,  uttime->tm_sec, TSnsec);

                    if(AOconf[loop].RTSLOGarray[rtlindex].FileBuffer == 0)
                        sprintf(AOconf[loop].RTSLOGarray[rtlindex].timestring0, "%02d:%02d:%02d.%09ld", uttime->tm_hour, uttime->tm_min,  uttime->tm_sec, TSnsec);

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



                    if(AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer>1)
                    {
                        if(sprintf(fnameinfo, "%s/aol%d_%s.%s.dat.%03d",
                                   fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name,
                                   AOconf[loop].RTSLOGarray[rtlindex].timestring, AOconf[loop].RTSLOGarray[rtlindex].FileBuffer) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        if(sprintf(fnameFITS, "%s/aol%d_%s.%s.fits",
                                   fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name,
                                   AOconf[loop].RTSLOGarray[rtlindex].timestring0) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                    }
                    else
                    {
                        if(sprintf(fnameinfo, "%s/aol%d_%s.%s.dat",
                                   fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name, AOconf[loop].RTSLOGarray[rtlindex].timestring) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");

                        if(sprintf(fnameFITS, "%s/aol%d_%s.%s.fits",
                                   fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name, AOconf[loop].RTSLOGarray[rtlindex].timestring) < 1)
                            printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
                    }


                    /*
                                        printf(" TIME STAMP :  %9ld.%09ld  -> %s\n", (long) TSsec, TSnsec, AOconf[loop].RTSLOGarray[rtlindex].timestring);
                                        printf("       %s -> %s\n", shmimname    , fnameFITS);
                                        printf("       %s -> %s\n", shmimnameinfo, fnameinfo);
                    */



                    if(AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer==1)
                    {
                        // If file size = buffer size, then just save the buffer to file
                        //
                        ID = image_ID(shmimname);
                        zsizesave = data.image[ID].md[0].size[2];
                        data.image[ID].md[0].size[2] = NBframe;

                        save_fits(shmimname, fnameFITS);

                        data.image[ID].md[0].size[2] = zsizesave;
                    }
                    else
                    {
                        // Otherwise, copy buffer into large buffer
                        //

                        long IDout;
                        char OutBuffIm[200];
                        char *destptrBuff;

                        ID = image_ID(shmimname);
                        zsizesave = data.image[ID].md[0].size[2];
                        if(zsizesave>AOconf[loop].RTSLOGarray[rtlindex].SIZE)
                        {
                            printf("[%s][%d] ERROR: zsizesave>AOconf[loop].RTSLOGarray[rtlindex].SIZE\n", __FILE__, __LINE__);
                            printf("     zsizesave                                = %d\n", zsizesave);
                            printf("     AOconf[loop].RTSLOGarray[rtlindex].SIZE  = %d\n", AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            exit(0);
                        }
                        if( AOconf[loop].RTSLOGarray[rtlindex].FileBuffer > AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer )
                        {
                            printf("[%s][%d] ERROR: AOconf[loop].RTSLOGarray[rtlindex].FileBuffer>AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer\n", __FILE__, __LINE__);
                            exit(0);
                        }
                        sprintf(OutBuffIm, "aol%d_%s_outbuff", loop, AOconf[loop].RTSLOGarray[rtlindex].name);
                        IDout = image_ID(OutBuffIm);
                        if(IDout == -1) // create data stream large buffer
                        {
                            uint32_t *imsize;
                            uint8_t atype;
                            int SHARED = 0;

                            imsize = (uint32_t*) malloc(sizeof(uint32_t)*3);
                            imsize[0] = data.image[ID].md[0].size[0];
                            imsize[1] = data.image[ID].md[0].size[1];
                            imsize[2] = AOconf[loop].RTSLOGarray[rtlindex].SIZE*AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer;
                            atype = data.image[ID].md[0].atype;
                            IDout = create_image_ID(OutBuffIm, 3, imsize, atype, SHARED, 0);
                            free(imsize);
                        }


                        switch (data.image[IDout].md[0].atype)
                        {
                        case _DATATYPE_INT8:
                            destptrBuff = (char*) data.image[IDout].array.SI8 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.SI8,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_UINT8:
                            destptrBuff = (char*) data.image[IDout].array.UI8 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.UI8,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;


                        case _DATATYPE_INT16:
                            destptrBuff = (char*) data.image[IDout].array.SI16 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.SI16,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_UINT16:
                            destptrBuff = (char*) data.image[IDout].array.UI16 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.UI16,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_INT32:
                            destptrBuff = (char*) data.image[IDout].array.SI32 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.SI32,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_UINT32:
                            destptrBuff = (char*) data.image[IDout].array.UI32 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.UI32,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_INT64:
                            destptrBuff = (char*) data.image[IDout].array.SI64 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.SI64,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_UINT64:
                            destptrBuff = (char*) data.image[IDout].array.UI64 + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.UI64,
                                   AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_FLOAT:
                            destptrBuff = (char*) data.image[IDout].array.F + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.F, AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        case _DATATYPE_DOUBLE:
                            destptrBuff = (char*) data.image[IDout].array.D + AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            memcpy((void*) destptrBuff, (void*) data.image[ID].array.D, AOconf[loop].RTSLOGarray[rtlindex].memsize*AOconf[loop].RTSLOGarray[rtlindex].SIZE);
                            break;

                        }
                    }

                    if(AOconf[loop].RTSLOGarray[rtlindex].FileBuffer == 0)
                        t0 = data.image[IDininfo].array.UI64[1] + 1.0e-9*data.image[IDininfo].array.UI64[2];


                    fp = fopen(fnameinfo, "w");


                    for(i=0; i<NBframe; i++)
                    {
                        long double t1 = data.image[IDininfo].array.UI64[i*5+1] + 1.0e-9*data.image[IDininfo].array.UI64[i*5+2];
                        fprintf(fp, "%10ld  %10ld  %15.9lf   %010ld.%09ld  %10ld   %10ld\n",
                                i+NBframe*AOconf[loop].RTSLOGarray[rtlindex].FileBuffer,
                                data.image[IDininfo].array.UI64[i*5],
                                (double) (t1-t0),
                                data.image[IDininfo].array.UI64[i*5+1], data.image[IDininfo].array.UI64[i*5+2],
                                data.image[IDininfo].array.UI64[i*5+3],
                                data.image[IDininfo].array.UI64[i*5+4]
                               );
                    }
                    fclose(fp);


                    AOconf[loop].RTSLOGarray[rtlindex].FileBuffer++;

                    if(AOconf[loop].RTSLOGarray[rtlindex].FileBuffer == AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer)
                    {
                        // Save large buffer

                        if(AOconf[loop].RTSLOGarray[rtlindex].NBFileBuffer>1)
                        {
                            // save large buffer to file

                            char command[1000];
                            // merge buffer files
                            sprintf(command, "( cat %s/aol%d_%s.*.dat.0* > %s/aol%d_%s.%s.dat; rm %s/aol%d_%s.*.dat.0* ) &",
                                    fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name,
                                    fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name, AOconf[loop].RTSLOGarray[rtlindex].timestring0,
                                    fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name
                                   );
                            if(system(command) != 0)
                                printERROR(__FILE__,__func__,__LINE__, "system() returns non-zero value");

                            //   sprintf(command, "rm %s/aol%d_%s.*.dat.0*", fulldir2, loop, AOconf[loop].RTSLOGarray[rtlindex].name);
                            //   system(command);

                            char OutBuffIm[200];
                            sprintf(OutBuffIm, "aol%d_%s_outbuff", loop, AOconf[loop].RTSLOGarray[rtlindex].name);


                            strcpy(savethreadmsg_array[rtlindex].iname, OutBuffIm);
                            strcpy(savethreadmsg_array[rtlindex].fname, fnameFITS);
                            savethreadmsg_array[rtlindex].partial = 0; // full cube
                            savethreadmsg_array[rtlindex].cubesize = AOconf[loop].RTSLOGarray[rtlindex].FileBuffer*AOconf[loop].RTSLOGarray[rtlindex].SIZE;
                            savethreadmsg_array[rtlindex].saveascii = 0; // just save FITS, dat file handled separately

                            // Wait for save thread to complete to launch next one
                            if(tOK[rtlindex] == 1)
                            {
                                printf("\n Wait start-----------------------\n");
                                fflush(stdout);

                                if(pthread_tryjoin_np(thread_savefits[rtlindex], NULL) == EBUSY)
                                {
                                    if(VERBOSE > 0)
                                    {
                                        printf("%5d  PREVIOUS SAVE THREAD NOT TERMINATED -> waiting\n", __LINE__);
                                        fflush(stdout);
                                    }
                                    pthread_join(thread_savefits[rtlindex], NULL);
                                    if(VERBOSE > 0)
                                    {
                                        printf("%5d  PREVIOUS SAVE THREAD NOW COMPLETED -> continuing\n", __LINE__);
                                        fflush(stdout);
                                    }
                                }
                                else if(VERBOSE > 0)
                                {
                                    printf("%5d  PREVIOUS SAVE THREAD ALREADY COMPLETED -> OK\n", __LINE__);
                                    fflush(stdout);
                                }
                                NBthreads--;
                                printf("\n Wait end  -----------------------\n");
                                fflush(stdout);
                            }

                            iret_savefits[rtlindex] = pthread_create( &thread_savefits[rtlindex], NULL, save_fits_function, &savethreadmsg_array[rtlindex]);
                            AOconf[loop].RTSLOGarray[rtlindex].tActive = 1;
                            NBthreads++;



                            tOK[rtlindex] = 1;  // next time, we'll wait for thread to be done
                            if(iret_savefits[rtlindex])
                            {
                                fprintf(stderr, "Error - pthread_create() return code: %d\n", iret_savefits[rtlindex]);
                                exit(EXIT_FAILURE);
                            }

                            //                         save_fits(OutBuffIm, fnameFITS);

                        }

                        AOconf[loop].RTSLOGarray[rtlindex].FileBuffer = 0;
                    }

                    AOconf[loop].RTSLOGarray[rtlindex].memcpToggle = 0;
                    cntsave++;
                }
            }
            else
            {
                AOconf[loop].RTSLOGarray[rtlindex].FileBuffer = 0; // Ensure we are at buffer start when starting to save
            }

        }
        
        if(cntsave>0)
        {
            //  printf("%d buffer(s) saved\n", cntsave);
            printf("\n");
            sleepcnt = 0;
        }
        else
        {
            printf("waiting for buffers ready to save %10.3f sec \r", 1.0e-6*sleepcnt*sleeptimeus);
            fflush(stdout);
            sleepcnt ++;
        }

        if((data.processinfo==1)&&(processinfo->MeasureTiming==1))
            processinfo_exec_end(processinfo);


        if(data.signal_TERM == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGTERM);
        }


        if(data.signal_INT == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGINT);
        }

        if(data.signal_ABRT == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGABRT);
        }

        if(data.signal_BUS == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGBUS);
        }

        if(data.signal_SEGV == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGSEGV);
        }

        if(data.signal_HUP == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGHUP);
        }

        if(data.signal_PIPE == 1) {
            loopOK = 0;
            if(data.processinfo==1)
                processinfo_SIGexit(processinfo, SIGPIPE);
        }

        usleep(sleeptimeus);

        loopcnt++;
        if(data.processinfo==1)
            processinfo->loopcnt = loopcnt;

    }

    if(data.processinfo==1)
        processinfo_cleanExit(processinfo);

    free(savethreadmsg_array);

    return 0;
}


