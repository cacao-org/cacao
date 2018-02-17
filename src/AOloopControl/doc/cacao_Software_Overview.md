# cacao Software Overview {#page_cacao_Software_Overview}


At the low level, most operations are performed by calls to the [main executable](#main-executable) which is precompiled C code. The user interacts with a [high level ASCII-based GUI](#high-level-GUI), which performs calls to individual [scripts](#supporting-scripts) or directly execute instances of the main executable. 

The standard code layers are : [GUI](#high-level-gui) calls [SCRIPT](#supporting-scripts) calls [PRECOMPILED EXECUTABLE](#main-executable)



## High-level GUI


The top level script is `aolconf`. Run it with `-h` option for a quick help

	./aolconf -h

The `aolconf` script starts the main GUI screen from which sub-screens can be accessed. ASCII control GUI scripts are in the `aolconfscripts` directory. 

The scripts are listed below in the order they appear in the GUI menu. Boldface scripts corresponds to GUI screens. Supporting scripts (holding frequently used functions) are boldface italic.

------------------------------------ -----------------------------------------------------------
Script                               Description
------------------------------------ -----------------------------------------------------------
**aolconf_menutop**                  **GUI**: Top level menu

***aolconf_funcs***                  Misc functions

***aolconf_DMfuncs***                DM Functions

***aolconf_readconf***               Configuration read functions

***aolconf_menuview***               **GUI**: Data view menu

***aolconf_template***               Template (use to create additional GUI screens)

----CONFIGURATION------------------- setup configuration, links to hardware

**aolconf_menuconfigureloop**        **GUI**: Configure loop menu. Called from main menu

***aolconf_configureloop_funcs***    Functions used within the configureloop GUI screen

----CONTROL MATRIX------------------ compute control matrix

**aolconf_menucontrolmatrix**        **GUI**: Control matrix menu. Called from main menu

***aolconf_controlmatrix_funcs***    Functions used within the controlmatrix GUI screen

***aolconf_menu_mkFModes***          Make modes

----LOOP CONTROL-------------------- control AO loop

**aolconf_menucontrolloop**          **GUI**: Control loop menu. Called from main menu

***aolconf_controlloop_funcs***      Functions used within the controlloop GUI screen

----PREDICTIVE CONTROL-------------- WFS telemetry prediction and related AO control

***aolconf_menupredictivecontrol***  **GUI**:Predictive control

----TESTING------------------------- AO loop tests

**aolconf_menutestmode**             **GUI**: test mode menu

***aolconf_DMturb***                 DM turbulence functions

----LOGGING------------------------- Log telemetry

**aolconf_menurecord**               **GUI**: Record / log telemetry

***aolconf_logfuncs***               Data and command logging

------------------------------ -----------------------------------------------------------


## Supporting Scripts

Scripts are organized in directories according to their purpose 


------------------------------ -----------------------------------------------------------
Directory                      Description
------------------------------ -----------------------------------------------------------
aolconfscripts                 GUI scripts (see previous section)

aolfuncs                       Frequently used functions

auxscripts                     Auxillary scripts. Essential high level scripts in this directory.

aohardsim                      Hardware simulation

aocscripts                     Custom user-provided scripts to interact with non real-time hardware
------------------------------ -----------------------------------------------------------


The `auxscripts` directory are called by aolconf to perform various tasks. To list all commands, type in the `auxscripts` directory :

	./listcommands
	
For each script, the `-h` option will print help.




## Main executable

The main precompiled executable is `./AOloopControl`, which provides a command line interface (CLI) to all compiled code. Type `AOloopControl -h` for help. You can enter the CLI and list the available libraries (also called modules) that are linked to the CLI. You can also list the functions available within each module (`m? <module.c>`) and help for each function (`cmd? <functionname>`). Type `help` within the CLI for additional directions, and `exit` or `exitCLI` to exit the command line.

~~~
olivier@ubuntu:/data/AOloopControl/AOloop1$ ./AOloopControl 
type "help" for instructions
Running with openMP, max threads = 8  (defined by environment variable OMP_NUM_THREADS)
LOADED: 21 modules, 269 commands
./AOloopControl > exitCLI
Closing PID 5291 (prompt process)
~~~



## Memory storage and Configuration Parameters

AOCCE adopts a common shared memory data format for all data streams. The data structure is defined in file `<srcdir>/src/ImageStruct.h`. 

Configurations parameters are stored in directory `<workdir>/conf` as ASCII files. When AOCCEE is launched, it can load all required parameters and populate required shared memory streams from information contained in the `<workdir>/conf` directory.





