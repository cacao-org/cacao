/**
 * @file    AOloopControl_read_param.c 
 * @brief   AO loop control : read parameters value - float, char, int 
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>



#define AOconfname "/tmp/AOconf.shm"
extern AOLOOPCONTROL_CONF *AOconf; // configuration - this can be an array
extern AOloopControl_var aoloopcontrol_var;
extern DATA data;
#define NB_AOloopcontrol 10 // max number of loops





/* =============================================================================================== */
/** @brief Read parameter value - float, char or int                                               */
/* =============================================================================================== */

/**
 * ## Purpose
 * 
 * Read parameter value (float) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	FLOAT
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
float AOloopControl_readParam_float(char *paramname, float defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	float value;
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        value = defaultValue;
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%50f", &value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			value = defaultValue;
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%f", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %f\n", paramname, value);
    
	
	return value;
}



/**
 * ## Purpose
 * 
 * Read parameter value (int) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	INT
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
int AOloopControl_readParam_int(char *paramname, int defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	int value;
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        value = defaultValue;
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%50d", &value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			value = defaultValue;
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%d", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %d\n", paramname, value);
    
	
	return value;
}



/**
 * ## Purpose
 * 
 * Read parameter value (char*) from file 
 * 
 * ## Arguments
 * 
 * @param[in]
 * paramname	CHAR*
 * 				parameter name
 * 
 * @param[in]
 * defaultValue	CHAR*
 * 				default value if file conf/param_paramname.txt not found
 *
 * @param[in]
 * fplog		FILE*
 * 				log file. If NULL, do not log
 *  
 */
char* AOloopControl_readParam_string(char *paramname, char* defaultValue, FILE *fplog)
{
	FILE *fp;
	char fname[200];
	char* value = " ";
	int wParamFile = 0;
	
	sprintf(fname, "./conf/param_%s.txt", paramname);
	if((fp=fopen(fname, "r"))==NULL)
    {
        printf("WARNING: file %s missing\n", fname);
        strcpy(value, defaultValue);
        wParamFile = 1;
    }
    else
    {
        if(fscanf(fp, "%200s", value) != 1){
            printERROR(__FILE__,__func__,__LINE__, "Cannot read parameter for file");
			strcpy(value, defaultValue);
			wParamFile = 1;
		}
        fclose(fp);
	}

	if(wParamFile == 1) // write file
	{
		fp = fopen(fname, "w");
		fprintf(fp, "%s", value);
		fclose(fp);
	}


    if(fplog!=NULL)
		fprintf(fplog, "parameter %20s = %s\n", paramname, value);
    
	
	return value;
}
