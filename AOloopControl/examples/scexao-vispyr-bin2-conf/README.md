# Overview

SCExAO system pyramid WFS.
Low-resolution WFS mode (120x120)

This is a (nearly) full-featured example for a single input / single output control loop.


# Running the example

:warning: Check the [instructions](https://github.com/cacao-org/cacao/tree/dev/AOloopControl/examples) before running these steps

## Setting up processes


```bash
# Deploy configuration :
# download from source to current directory
cacao-loop-deploy -c scexao-vispyr-bin2

# OPTIONAL: Change loop number, name, DM index, simulation DM index:
# CACAO_LOOPNUMBER=7 cacao-loop-deploy -c scexao-vispyr-bin2
# CACAO_LOOPNUMBER=7 CACAO_DMINDEX="03" cacao-loop-deploy -c scexao-vispyr-bin2

# OPTIONAL: Edit file scexao-vispyr-bin2-conf/cacaovars.bash as needed
# For example, change loop index, DM index, etc ...

# OPTIONAL: Clean previous deployment :
# rm -rf .vispyr2.cacaotaskmanager-log


# Run deployment (starts conf processes)
cacao-loop-deploy -r scexao-vispyr-bin2

# Note: the copy and run steps can be done at once with :
# cacao-loop-deploy scexao-vispyr-bin2


# Go to rootdir, from which user controls the loop
cd vispyr2-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)
```

## Run DM and WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs -w start
```



## Measure WFS dark


Takes dark, stores it into aolX_wfsdarkraw, with aolX_wfsdark pointing to it.
```bash
cacao-aorun-005-takedark -n 2000
```


## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS -w start
```

```bash
# Acquire WFS reference
cacao-aorun-026-takeref -n 2000
```


## Measure DM to WFS latency

```bash
# Measure latency
cacao-aorun-020-mlat -w
```



## Acquire response matrix


### Prepare DM poke modes

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes -z 5 -c 25
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
# 6 cycles - default is 10.
cacao-aorun-030-acqlinResp -n 6 -w HpokeC
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


### Create synthetic (Fourier) response matrix

```bash
cacao-aorun-033-RM-mksynthetic -c 25
```


## Compute control matrix (straight)

Compute control modes, in both WFS and DM spaces.
Set GPU device (if GPU available).

```bash
cacao-fpsctrl setval compstrCM svdlim 0.01
cacao-fpsctrl setval compstrCM GPUdevice 0
```
Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```



## Running the loop

Select GPUs for the modal decomposition (WFS->modes) and expansion (modes->DM) MVMs
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 0
cacao-fpsctrl setval mvalC2dm GPUindex 0
```


Start the 3 control loop processes :

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


## Testing the loop

### SelfRM

```bash
# Set max number of modes above nbmodes

cacao-fpsctrl runstop mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.zsize 20
cacao-fpsctrl setval mfilt selfRM.NBmode 1000
cacao-fpsctrl runstart mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.enable ON
```

Check result: maps-rundir/selfRM.fits

### Turbulence

```bash
cacao-aorun-100-DMturb start
cacao-aorun-100-DMturb off
cacao-aorun-100-DMturb on
cacao-aorun-100-DMturb stop
```

### Monitoring

```bash
cacao-modalstatsTUI
```


## Predictive Control

Start process mctrlstats to split telemetry into blocks.
Start mkPFXX-Y processes.
Start applyPFXX-Y processes.



# Cleanup

From main directory (upstream of rootdir) :

```bash
cacao-task-manager -C 0 scexao-vispyr-bin2
rm -rf .vispyr2.cacaotaskmanager-log
```



THE END
