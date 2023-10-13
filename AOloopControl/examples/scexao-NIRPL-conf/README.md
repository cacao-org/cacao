# Overview

Near-IR Photonic Lantern

cacao-task-manager tasks for this example :

~~~
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
~~~
Subsequent tasks can perform specific parts of the AO loop.


# Running the example

:warning: Check the [instructions](https://github.com/cacao-org/cacao/tree/dev/AOloopControl/examples) before running these steps



## Setting up processes

```bash
# Deploy configuration :
# download from source and start conf processes
cacao-loop-deploy scexao-NIRPL

# Go to rootdir, from which user controls the loop
cd NIRPL-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)
# make sure DMch2disp-00 is running and DMch2disp-00.option.volttype.2  = 2, otherwise DM won't work
```



## Run DM, DM+WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs start
```


## Take full frame dark -> will store it in aolX_wfsdark

Turn off light source, and run:
```bash
cacao-aorun-005-takedark -n 3000
```
Then turn light source back on.

## Find spots

```bash
./scripts/scexao-NIRPL-findspots 1000
```

## Start remapping

```bash
cacao-aorun-003-wfsmapping
```

## Remap dark frame

```bash
./scripts/scexao-NIRPL-mapdark
```


## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS start
```

## Measure DM to WFS latency


```bash
# Measure latency
# first turn off normalization under acquWFS-<LOOP>.comp ; it messes w/ latency measurement becasue the poke changes the total WFS image instensity significantly
cacao-aorun-020-mlat -w
# then turn normalization back on
```

## Acquire Calibration


### Prepare DM poke modes

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes -z <NUM> -c <CPA>
```
The following files are written to ./conf/RMmodesDM/
| File                 | Contents                                            |
| -------------------- | --------------------------------------------------- |
| `DMmask.fits     `   | DM mask                                             |
| `FpokesC.<CPA>.fits` | Fourier modes (where \<CPA> is an integer)          |
| `ZpokesC.<NUM>.fits` | Zernike modes (where \<NUM> is the number of modes) |
| `HpokeC.fits     `   | Hadamard modes                                      |
| `Hmat.fits       `   | Hadamard matrix (to convert Hadamard-zonal)         |
| `Hpixindex.fits  `   | Hadamard pixel index                                |
| `SmodesC.fits    `   | *Simple* (single actuator) pokes                    |



### Run acquisition


```bash
# Acquire response matrix - Hadamard modes
# 4 cycles - default is 10.
cacao-aorun-030-acqlinResp -n 4 HpokeC
```
This could take a while. Check status on milk-procCTRL.
To inspect results, display file conf/RMmodesWFS/HpokeC.WFSresp.fits.

### Decode Hadamard matrix

```bash
cacao-aorun-031-RMHdecode
```
To inspect results, display file conf/RMmodesWFS/zrespM-H.fits.
This should visually look like a zonal response matrix.


### Make DM and WFS masks

```bash
cacao-aorun-032-RMmkmask
```
Check results:
- conf/dmmask.fits
- conf/wfsmask.fits

If needed, rerun command with non-default parameters (see -h for options).
Note: we are not going to apply the masks in this example, so OK if not net properly. The masks are informative here, allowing us to view which DM actuators and WFS pixels have the best response.

## Acquire response matrix (Zernike)

### Run acquisition


```bash
# Acquire response matrix - Zernike modes
cacao-fpsctrl setval measlinresp procinfo.loopcntMax 4
cacao-aorun-030-acqlinResp ZpokesC.<NUM>
```


### Take reference

```bash
# Acquire reference
cacao-aorun-026-takeref -n 3000
```


## Compute control matrix (straight)


Compute control modes, in both WFS and DM spaces.

```bash
cacao-fpsctrl setval compstrCM RMmodesDM "../conf/RMmodesDM/ZpokesC.<NUM>.fits"
cacao-fpsctrl setval compstrCM RMmodesWFS "../conf/RMmodesWFS/ZpokesC.<NUM>.WFSresp.fits"
cacao-fpsctrl setval compstrCM svdlim 0.1
```
Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```


## Running the loop

Configuring to CPU mode
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 99
cacao-fpsctrl setval mvalC2dm GPUindex 99
```


From directory nirpl-rootdir, start 3 processes :

```bash
# start WFS -> mode coefficient values
cacao-aorun-050-wfs2cmval start

# start modal filtering
cacao-aorun-060-mfilt start

# start mode coeff values -> DM
cacao-aorun-070-cmval2dm start

```

Closing the loop and setting loop parameters with mfilt:

```bash
# Set loop gain
cacao-fpsctrl setval mfilt loopgain 0.1

# Set loop mult
cacao-fpsctrl setval mfilt loopmult 0.98

# close loop
cacao-fpsctrl setval mfilt loopON ON

```


THE END


