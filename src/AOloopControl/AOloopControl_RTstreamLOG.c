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
	
	for(i=0;i<MAX_NUMBER_RTLOGSTREAM;i++)
        {
			AOconf[loop].RTSLOGarray[i].active = 0;
			strcpy(AOconf[loop].RTSLOGarray[i].name, "NULL");
			AOconf[loop].RTSLOGarray[i].ENABLE = 0;
			AOconf[loop].RTSLOGarray[i].INIT = 0;
			AOconf[loop].RTSLOGarray[i].ON = 0;
			AOconf[loop].RTSLOGarray[i].save = 0;
		 AOconf[loop].RTSLOGarray[i].saveToggle = 0;
        }
	
	    i = RTSLOGindex_wfsim;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "wfsim"); //   U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
     
        i = RTSLOGindex_imWFS0;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS0");// U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
     
        i = RTSLOGindex_imWFS1;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS1");  // U in Read_cam_frame()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
     
        i = RTSLOGindex_imWFS2;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "imWFS2"); // U in AOcompute()
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
        
        
// managed by AOloopControl_ComputeOpenLoopModes()

        i = RTSLOGindex_modeval;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval"); // U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
                
        i = RTSLOGindex_modeval_ol;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_ol");// U 
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
        AOconf[loop].RTSLOGarray[i].save = 1;
        AOconf[loop].RTSLOGarray[i].ON = 1;                

        i = RTSLOGindex_modeval_dm;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
                
        i = RTSLOGindex_modeval_dm_corr;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_corr");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
                        
        i = RTSLOGindex_modeval_dm_now;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
                                
        i = RTSLOGindex_modeval_dm_now_filt;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modeval_dm_now_filt");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;

        i = RTSLOGindex_modevalPF;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPF");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;
                                        
        i = RTSLOGindex_modevalPFsync;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFsync");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;        
                
        i = RTSLOGindex_modevalPFres;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "modevalPFres");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;        

        i = RTSLOGindex_dmC;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "dmC");// U
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;                        

        i = RTSLOGindex_dmdisp;
        AOconf[loop].RTSLOGarray[i].active = 1;
        strcpy(AOconf[loop].RTSLOGarray[i].name, "dmdisp");// 
        AOconf[loop].RTSLOGarray[i].ENABLE = 0;                        
	
	
	return 0;
}





int AOloopControl_RTstreamLOG_setup(long loop, long rtlindex, char *streamname)
{
	long IDstream;
	uint32_t *imsize;
	int retval = 0;
	char imname[500];
	uint64_t nelement;
	long infosize = 5;
	uint8_t atype;
	
	IDstream = image_ID(streamname);

	imsize = (uint32_t*) malloc(sizeof(uint32_t)*3);
	imsize[0] = data.image[IDstream].md[0].size[0];
	imsize[1] = data.image[IDstream].md[0].size[1];
	imsize[2] = AOconf[loop].RTLOGsize;

	atype = data.image[IDstream].md[0].atype;
	
	
	if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE == 1)&&(AOconf[loop].RTSLOGarray[rtlindex].INIT == 0))
	{	
		
		if(sprintf(imname, "aol%ld_%s_logbuff0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff0 = create_image_ID(imname, 3, imsize, atype, 1, 0);

		if(sprintf(imname, "aol%ld_%s_logbuff1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff1 = create_image_ID(imname, 3, imsize, atype, 1, 0);

		
		nelement = (uint64_t) imsize[0];
		nelement *= imsize[1];
		nelement *= imsize[2];
		
		switch(atype)
		{
			case _DATATYPE_UINT8 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT8*nelement);
			break;

			case _DATATYPE_UINT16 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT16*nelement);
			break;

			case _DATATYPE_UINT32 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT32*nelement);
			break;

			case _DATATYPE_UINT64 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_UINT64*nelement);
			break;

			case _DATATYPE_INT8 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT8*nelement);
			break;

			case _DATATYPE_INT16 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT16*nelement);
			break;

			case _DATATYPE_INT32 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT32*nelement);
			break;

			case _DATATYPE_INT64 :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_INT64*nelement);
			break;

			case _DATATYPE_FLOAT :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_FLOAT*nelement);
			break;

			case _DATATYPE_DOUBLE :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_DOUBLE*nelement);
			break;

			case _DATATYPE_COMPLEX_FLOAT :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_COMPLEX_FLOAT*nelement);
			break;

			case _DATATYPE_COMPLEX_DOUBLE :
			AOconf[loop].RTSLOGarray[rtlindex].memsize = (size_t) (SIZEOF_DATATYPE_COMPLEX_DOUBLE*nelement);
			break;
			
			default :
			printf("Unknown data type\n");
			exit(0);
			break;
		}



		imsize[0] = infosize;
		imsize[1] = AOconf[loop].RTLOGsize;		
		imsize[2] = 1;
			
		if(sprintf(imname, "aol%ld_%s_logbuffinfo0", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);

		if(sprintf(imname, "aol%ld_%s_logbuffinfo1", loop, AOconf[loop].RTSLOGarray[rtlindex].name) < 1)
			printERROR(__FILE__, __func__, __LINE__, "sprintf wrote <1 char");
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo1 = create_image_ID(imname, 2, imsize, _DATATYPE_UINT64, 1, 0);

		
		AOconf[loop].RTSLOGarray[rtlindex].srcptr      = data.image[IDstream].array.F;
		AOconf[loop].RTSLOGarray[rtlindex].destptr0    = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff0].array.F;
		AOconf[loop].RTSLOGarray[rtlindex].destptr1    = (char*) data.image[AOconf[loop].RTSLOGarray[rtlindex].IDbuff1].array.F;
		

		AOconf[loop].RTSLOGarray[rtlindex].buffindex   = 0;
		AOconf[loop].RTSLOGarray[rtlindex].frameindex  = 0;
		AOconf[loop].RTSLOGarray[rtlindex].destptr     = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
		AOconf[loop].RTSLOGarray[rtlindex].IDbuff      = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
		AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo  = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
		
		
		retval = 1;
		AOconf[loop].RTSLOGarray[rtlindex].INIT = 1;
	}
	
	
	free(imsize);
	
	return(0);
}






void AOloopControl_RTstreamLOG_update(long loop, long rtlindex, struct timespec tnow)
{
	if((AOconf[loop].RTSLOGarray[rtlindex].ENABLE==1) && (AOconf[loop].RTSLOGarray[rtlindex].ON==1))
	{
		char *dataptr;
		dataptr = AOconf[loop].RTSLOGarray[rtlindex].destptr + AOconf[loop].RTSLOGarray[rtlindex].memsize * AOconf[loop].RTSLOGarray[rtlindex].frameindex;


		
		memcpy((void*) dataptr, 
		(void*) AOconf[loop].RTSLOGarray[rtlindex].srcptr, 
		AOconf[loop].RTSLOGarray[rtlindex].memsize);



		long IDinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo;
		
		
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5  ] = AOconf[loop].LOOPiteration;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+1] = (long) tnow.tv_sec;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+2] = (long) tnow.tv_nsec;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+3] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt0;
		data.image[IDinfo].array.UI64[AOconf[loop].RTSLOGarray[rtlindex].frameindex*5+4] = data.image[AOconf[loop].RTSLOGarray[rtlindex].IDsrc].md[0].cnt1;


		AOconf[loop].RTSLOGarray[rtlindex].frameindex++;
		if(AOconf[loop].RTSLOGarray[rtlindex].frameindex == AOconf[loop].RTLOGsize)
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
				AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 1;
			}
			else
			{
				AOconf[loop].RTSLOGarray[rtlindex].buffindex = 0;
				AOconf[loop].RTSLOGarray[rtlindex].destptr = AOconf[loop].RTSLOGarray[rtlindex].destptr0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuff = AOconf[loop].RTSLOGarray[rtlindex].IDbuff0;
				AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo = AOconf[loop].RTSLOGarray[rtlindex].IDbuffinfo0;
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
	printf("%2s  %20s  %3s %3s %3s %6s %4s %10s %10s\n", "id", "streamname", "ENA", " ON", "INI", "SAVE", "buff", "frame", "memsize");
	printf("----------------------------------------------------------------------\n");
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
  
  
			printf("%2d  %20s  %3s %3s %3s %6s %4d %10ld %10ld\n", i,  
			AOconf[loop].RTSLOGarray[i].name, 
			ENstring, 
			ONstring, 
			INstring, 
			SAstring,			
			AOconf[loop].RTSLOGarray[i].buffindex,
			AOconf[loop].RTSLOGarray[i].frameindex,
			AOconf[loop].RTSLOGarray[i].memsize
			);
			NBstreams++;			
		}
	}
	printf("----------------------------------------------------------------------\n");
	printf("%ld RTstreamLOGs active\n", NBstreams);
	
	return NBstreams;
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
	float sleeptime = 1.0;

	tzset();
	
	if(aoloopcontrol_var.AOloopcontrol_meminit==0)
        AOloopControl_InitializeMemory(1);

	
    printf("\n");
    
    for(;;)
    {
		cntsave = 0;
	for(rtlindex=0;rtlindex<MAX_NUMBER_RTLOGSTREAM;rtlindex++)
	{
		if((AOconf[loop].RTSLOGarray[rtlindex].save == 1)&&(AOconf[loop].RTSLOGarray[rtlindex].saveToggle!=0))
		{
			int buff;
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
			
			// buffindex to save	
			buff = AOconf[loop].RTSLOGarray[rtlindex].saveToggle-1;
			
			printf("\n   SAVING \033[1;32m%s\033[0m buffer (%d)\n", AOconf[loop].RTSLOGarray[rtlindex].name, rtlindex);
			
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

			save_fits(shmimname, fname);
			
			long double t0 = data.image[IDininfo].array.UI64[1] + 1.0e-9*data.image[IDininfo].array.UI64[2];
			fp = fopen(fnameinfo, "w");
			for(i=0;i<AOconf[loop].RTLOGsize;i++) //TO BE CLIPPED
			{
				long double t1 = data.image[IDininfo].array.UI64[i*5+1] + 1.0e-9*data.image[IDininfo].array.UI64[i*5+2];
				fprintf(fp, "%10ld  %10ld  %15.9lf   %010ld.%010ld  %10ld   %10ld\n", i, data.image[IDininfo].array.UI64[i*5], (double) (t1-t0), data.image[IDininfo].array.UI64[i*5+1], data.image[IDininfo].array.UI64[i*5+2], data.image[IDininfo].array.UI64[i*5+3], data.image[IDininfo].array.UI64[i*5+4]);
			}
			fclose(fp);
			

			AOconf[loop].RTSLOGarray[rtlindex].saveToggle = 0;
			cntsave++;
		}
	}
	if(cntsave>0)
		{
			printf("%d buffer(s) saved\n", cntsave);
			printf("\n");
		}
	else
	{
		printf(".");
		fflush(stdout);
	}
		
	sleep(sleeptime);
	}
    
    
    
	return 0;
}


