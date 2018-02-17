# STEP-BY-STEP EXAMPLE {#page_cacao_stepbystep_example}


## Starting the linear hardware simulator

**STEP 1**: Create directory `<workdir>`

**STEP 2**: Install scripts into `<workdir>`:

	cd <srcdir>/src/AOloopControl/scripts
	syncscripts -e <workdir>
	cd <workdir>
	./syncscripts

**STEP 3**: Download a calibration, consisting of zonal response matrix and a WFS reference. Place the file in a new directory `<workdir>/simLHS`

**STEP 4**: Launch aolconf, loop number 5, loop name simtest:

	./aolconf -L 5 -N simtest
	
Note that you subsequent calls to aolconf should then be without the -L and -N options.

**STEP 5**: Set DM size to match the calibration. `dmxs` and `dmys` GUI top menu.

**STEP 6**: Autoconfigure DM: `dmnolink` in GUI top menu.

**STEP 7**: Turn off dmvolt: `dmvolt0` in GUI top menu.

**STEP 8**: Set DM averaging mode to 2: `dmcombam` in GUI top menu.

**STEP 9**: Start DMcomb. `initDM` in GUI top menu.

**STEP 10**: Load all memory. `M` in GUI top menu

**STEP 11**: Link LHS files

**STEP 12**: Start the LHS process


## Acquiring calibration, compute control matrix

Under configuration GUI menu :

- Measure hardware timing: `mlat`
- set Hadamard mode to ON: `Hon`
- set RM modal to ON: `RMMon`
- Acquire automatic calibration: `nAUTOc`

Under control matrix GUI menu :

- Select maximum control spatial frequency: `modeCPA`
- Create modes and control matrix: `mkModes0`
- update configuration: `confUp`

Load and manage configuration :

- load configuration
- save configuration


