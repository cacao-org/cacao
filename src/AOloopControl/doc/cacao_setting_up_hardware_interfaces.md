# Setting up hardware interfaces {#page_cacao_setting_up_hardware_interfaces}


## Top level script

Start aolconf with loop number and loop name (you can ommit these arguments when launching the script again):

~~~~~
./aolconf -L 3 -N testsim
~~~~~

The loop name (`testsim` in the above example) will both allocate a name for the loop and execute an optional custom setup script. The software package comes with a few such pre-made custom scripts for specific systems / examples. When the `-N` option is specified, the custom setup script `./setup/setup_<name>` is ran. The script may make some of the steps described below optional.

You can check the current loop number and name settings with:

~~~~~
./aolconf -h
~~~~~

The script can also launch a pre-written CPU/OS configuration script named `./aocscripts/cpuconfig_<LOOPNAME>` :

~~~~
./aolconf -C
~~~~



## Setting the DM interface

There are four options for setting up the DM:

- [A] Connect to an existing DM

- [B] Create a new DM and connect to it

- [C] Create a new modal DM, mapped to an existing DM using another loop's control modes

- [D] Create a new modal DM, mapped to an existing DM channel using a custom set of modes

Before choosing an option, select if the DM to be controlled is `MODAL` or `ZONAL`. A zonal DM is one where the DM pixel locations map to physical actuator locations on the DM, allowing spatial filtering when creating control modes. With a zonal DM, each pixel of the DM map corresponds to a wavefront control mode, and spatial filtering functions are turned off. 

Options [C] and [D] are `MODAL` options, as the DM does not represent physical spatial actuators. These options build a virtual DM which controls another DM.




### Mode [A]: Connecting to an existing DM

1. **Set DM number** (`S` command in `Top Menu` screen). You should see its x and y size in the two lines below. If not, the DM does not exist yet (see next section).

2. **autoconfigure DM: main DM (nolink)** (`nolink` in `Top Menu` screen). This command automactically sets up the following symbolic links:
	- dm##disp00 is linked to aol#_dmO      (flat offset channel)
	- dm##disp02 is linked to aol#_dmRM     (response matrix actuation channel)
	- dm##disp03 is linked to aol#_dmC      (loop dm control channel)
	- dm##disp04 is linked to aol#_dmZP0    (zero point offset 0 actuation channel)
	- dm##disp05 is linked to aol#_dmZP1    (zero point offset 1 actuation channel)
	- dm##disp06 is linked to aol#_dmZP2    (zero point offset 2 actuation channel)
	- dm##disp07 is linked to aol#_dmZP3    (zero point offset 3 actuation channel)
	- dm##disp08 is linked to aol#_dmZP4    (zero point offset 4 actuation channel)
	- dm##disp   is linked to aol#_dmdisp   (total dm displacement channel)

3. **load Memory** (`M` in `Top Menu` screen). The dm performs the symbolic links to the DM channels.


### Mode [B]: Creating and Connecting to a DM

1. Set **DM number** (`S` command in `Top Menu` screen). 

2. Enter the desired **DM size** with the `dmxs` and `dmys` commands.

- OPTIONAL: **set DM delay** ('setDMdelayON' and 'setDMdelayval' in `Top Menu` screen)

3. **Create the DM streams** with the `initDM` command in the `Top Menu`. You may need to run the `stopDM` command first.

4. **autoconfigure DM: main DM (nolink)** (`nolink` in `Top Menu` screen). This command automactically sets up the following symbolic links:
	- dm##disp00 is linked to aol#_dmO      (flat offset channel)
	- dm##disp02 is linked to aol#_dmRM     (response matrix actuation channel)
	- dm##disp03 is linked to aol#_dmC      (loop dm control channel)
	- dm##disp04 is linked to aol#_dmZP0    (zero point offset 0 actuation channel)
	- dm##disp05 is linked to aol#_dmZP1    (zero point offset 1 actuation channel)
	- dm##disp06 is linked to aol#_dmZP2    (zero point offset 2 actuation channel)
	- dm##disp07 is linked to aol#_dmZP3    (zero point offset 3 actuation channel)
	- dm##disp08 is linked to aol#_dmZP4    (zero point offset 4 actuation channel)
	- dm##disp   is linked to aol#_dmdisp   (total dm displacement channel)
	
5. **Load Memory** (`M` in `Top Menu` screen). The dm performs the symbolic links to the DM channels.


### Mode [C]: Create a new modal DM, mapped to an existing DM using another loop's control modes

In this mode, the AO loop controls a virtual DM. The virtual actuators are correspond to modes controlling the zero point offset of another loop. In this section, I assume that **loopA** is the main loop (directly controls a physical DM) and that **loopB** is the virtual loop (this is the loop we are setting up).

1. Select **MODAL** DM (`DMmodeZ` in `Top Menu` screen)

2. Set **DM number** (`S` command in `Top Menu` screen). This is the DM index for loopB.

3. Set **DM x size** to the number of modes of loop A to be addressed by loop B's virtual DM

4. Set **DM y size** to 1 

5. **Auto-configure: DM output linked to other loop** (`dmolink` in `Top Menu` screen).
	1. choose loop index from which modes will be extracted (loop A index)
	2. choose offset channel in output loop
	This will set up several key parameters and files:
	- **DM-to-DM** mode will be set to 1, and associated streams:
		- **dm2dmM**    : **loopA** modes controlled by **loopB**
		- **dm2dmO**    : symbolic link to **loopA** DM channel controlled by **loopB**
	- **CPU-based dmcomb output WFS ref** will be set to 1, and associated streams:
		- **dmwrefRM**  : **loopA** WFS response to modes controlled by **loopB**
		- **dmwrefO**   : **loopA** WFS zero point offset

- **OPTIONAL: set DM delay** ('setDMdelayON' and 'setDMdelayval' in `Top Menu` screen)

6. **Create the DM streams** with the `initDM` command in the `Top Menu`.

7. **Load Memory** (`M` in `Top Menu` screen). The dm performs the symbolic links to the DM channels.


### Mode [D]: Create a new modal DM, mapped to an existing DM channel using a custom set of modes

In this mode, the AO loop controls a virtual DM. The virtual actuators correspond to modes controlling another DM stream. In this section, I assume that **loop A** is the main loop (directly controls a physical DM) and that **loop B** is the virtual (higher level) loop.

1. Choose DM index number (`S`) for loop B

2. Select number of loop A modes controlled by loop B. The number is entered as DM x size (`dmxs` in `Top menu`)

3. Enter 1 for DM y size (`dmys` in `Top menu`)

4. Set **DM-to-DM** mode to 1, and associated streams:
	- **dm2dmM**    : loop A modes controlled by loop B
	- **dm2dmO**    : symbolic link to loop A DM channel controlled by loop B

5. Set **CPU-based dmcomb output WFS ref** to 0 (see section below more enabling this option)

6. **(Re)-create DM streams and run DMcomb process** (`initDM`) 

7. **Load Memory** (`M` in `Top Menu` screen). The dm performs the symbolic links to the DM channels.

Commands to the loop B DM should now propagate to modal commands to loop A.


### Option: WFS Zero point offset

It is possible to add a zero point offset to mode D. Every write to the loop B's modal DM then generate both a write to loop A's DM (described above) and a write to the reference of a wavefront sensor (presumably loop A's wavefront sensor). This optional feature is refered to as a CPU-based WFS zero point offset.

To enable this feature, add between steps 4 and 5:

1. set **CPU-based dmcomb output WFS ref** to 1, and associated streams:
	- **dmwrefRM**  : **loopA** WFS response to modes controlled by **loopB**
	- **dmwrefO**   : **loopA** WFS zero point offset
 
 

### Notes
	
You can (Re-)Start DM comb to re-initialize arrays and links ('stopDM' and 'initDM' commands in `Top Menu` screen). The `initDM` command will

- (re-)create shared memory streams dm##disp00 to dm##disp11
- start the dmcomb process, which adds the dm##disp## channels to create the overall dm##disp displacement
- create poke mask and maps





## Setting the camera interface

- **link to WFS camera** (`wfs` to `Loop Configuration` screen). Select the WFS shared memory stream. 


## Setup script

An `aosetup` script may be used to perform all these operations. Inspect the content of directory `aosetup` to see such scripts. You may use or modify as needed. If you use a `aosetup` script, execute it from the working directory, and then start aolconf:

~~~
./aosetup/aosetup_<myLoop>
./aolconf
~~~
