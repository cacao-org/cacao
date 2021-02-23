/**
 * @file    milk_module_example.c
 * @brief   example milk module
 *
 * milk module source code template\n
 * Demonstates how to add modules to milk and connect functions.\n
 *
 *  File list :
 * - milk_module_example.c  : module main C file, includes binding code to milk
 * - milk_module_example.h  : function prototypes to be included by other modules
 * - create_example_image.c : source code, .c file
 * - create_example_image.h : source code, .h file
 * - CMakeLists.txt         : cmake input file for module
 */



/* ================================================================== */
/* ================================================================== */
/*            MODULE INFO                                             */
/* ================================================================== */
/* ================================================================== */

// module default short name
// all CLI calls to this module functions will be <shortname>.<funcname>
// if set to "", then calls use <funcname>
#define MODULE_SHORTNAME_DEFAULT "modex"

// Module short description
#define MODULE_DESCRIPTION       "Example module: template for creating new modules"

// Application to which module belongs
#define MODULE_APPLICATION       "milk"






/* ================================================================== */
/* ================================================================== */
/*            DEPENDANCIES                                            */
/* ================================================================== */
/* ================================================================== */


#define _GNU_SOURCE
#include "CommandLineInterface/CLIcore.h"



//
// Forward declarations are required to connect CLI calls to functions
// If functions are in separate .c files, include here the corresponding .h files
//
#include "create_example_image.h"






/* ================================================================== */
/* ================================================================== */
/*            INITIALIZE LIBRARY                                      */
/* ================================================================== */
/* ================================================================== */

// Module initialization macro in CLIcore.h
// macro argument defines module name for bindings
//
INIT_MODULE_LIB(milk_module_example)


/* ================================================================== */
/* ================================================================== */
/*            COMMAND LINE INTERFACE (CLI) FUNCTIONS                  */
/* ================================================================== */
/* ================================================================== */


/** @brief Example CLI function
 *
 * Command Line Interface (CLI) wrapper to function\n
 * This is referred to as a "CLI function", written to connect a command
 * on the CLI prompt to a function.\n
 * A CLI function will check arguments entered on the prompt, and pass
 * them to the function.
 *
 * Naming conventions:
 * - CLI function <modulename>__<function>__cli()
 * - Execution function <modulename>__<function>()
 *
 *
 *
 * ### Checking if arguments are valid
 *
 * Each argument is checked by calling CLI_checkarg(i, argtype), which
 * checks if argument number <i> conforms to type <argtype>.\n
 *
 * Types are defined in CLIcore.h. Common types are:
 * - CLIARG_FLOAT             floating point number
 * - CLIARG_LONG              integer (int or long)
 * - CLIARG_STR_NOT_IMG       string, not existing image
 * - CLIARG_IMG               existing image
 * - CLIARG_STR               string
 */
errno_t milk_module_example__create_image_with_value__cli()
{
    if(0
            + CLI_checkarg(1, CLIARG_STR_NOT_IMG)
            + CLI_checkarg(2, CLIARG_FLOAT)
            == 0)
    {
        // If arguments meet requirements, command is executed
        //
        milk_module_example__create_image_with_value(
            data.cmdargtoken[1].val.string,
            data.cmdargtoken[2].val.numf);

        return CLICMD_SUCCESS;
    }
    else
    {
        // If arguments do not pass test, errror code returned
        return CLICMD_INVALID_ARG;
    }
}




/* ================================================================== */
/* ================================================================== */
/*  MODULE CLI INITIALIZATION                                             */
/* ================================================================== */
/* ================================================================== */

/**
 * @brief Initialize module CLI
 *
 * CLI entries are registered: CLI call names are connected to CLI functions.\n
 * Any other initialization is performed\n
 *
 */
static errno_t init_module_CLI()
{

    //CLI_CMD_CONNECT("func1", "create_image_with_value");


    RegisterCLIcommand(
        "func1",                                                                       // function call name from CLI
        __FILE__,                                                                      // this file, used to track where function comes from
        milk_module_example__create_image_with_value__cli,                             // function to call
        "creates image with specified value",                                          // short description
        "<image> <value>",                                                             // arguments
        "mexfunc im1 3.5",                                                             // example use
        "milk_module_example__create_image_with_value(char *imname, double value)");   // source code call


    // optional: add atexit functions here

    return RETURN_SUCCESS;
}







/* ================================================================== */
/* ================================================================== */
/*  FUNCTIONS                                                         */
/* ================================================================== */
/* ================================================================== */

// Functions could be added here, or (better) in separate .c files

// Each .c file should have a corresponding .h file including
// function prototypes







