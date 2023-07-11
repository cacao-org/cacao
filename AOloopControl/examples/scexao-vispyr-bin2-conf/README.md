# Overview

SCExAO system pyramid WFS.
Low-resolution WFS mode (120x120)

This is a (nearly) full-featured example for a single input / single output control loop.




# Running the example

:warning: Check the [instructions](https://github.com/cacao-org/cacao/tree/dev/AOloopControl/examples) before running these steps

## 1. Setting up processes


```bash
# Deploy configuration :
# download from source to current directory
cacao-loop-deploy -c scexao-vispyr-bin2

# OPTIONAL: Change loop number, name, DM index, simulation DM index:
# CACAO_LOOPNUMBER=7 cacao-loop-deploy -c scexao-vispyr-bin2
# CACAO_LOOPNUMBER=7 CACAO_DMINDEX="03" cacao-loop-deploy -c scexao-vispyr-bin2

# OPTIONAL:
# Edit file scexao-vispyr-bin2-conf/fpstmuxenv to modify local environment for processes


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

### Logging and fpsCTRL start

Deploy logging processes and terminals :
```bash
cacao-msglogCTRL start
cacao-msglogCTRL terms
```

The command is equivalent to running in seprate windows :
```bash
# Start automatic fpsCTRL logging
cacao-fpsctrl-log -r &

# Check output on tmux session
tmux a -t fpsCTRLlog-cacao-vispyr2

# Start interactive logging terminal
cacao-log -k "OPNOTES" -i
```

Start fpsCTRL terminal
```bash
cacao-fpsctrl-TUI
```


## 2. Run DM and WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes
# (skip if in hardware mode)
cacao-aorun-002-simwfs -w start
```



## 3. Measure WFS dark


Takes dark, stores it into aolX_wfsdarkraw, with aolX_wfsdark pointing to it.
```bash
cacao-aorun-005-takedark -n 2000
```


## 4. Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS -w start
```

The acqWFS process performs flux normalization, and at this point assumes all WFS pixels are active (wfsmask set to 1). The mask will be updated later.

```bash
# Acquire WFS reference
cacao-aorun-026-takeref -n 2000
```

The reference is acquired here and immediately applied through the acquWFS process.

## 5. Measure DM to WFS latency

```bash
# Measure latency
cacao-aorun-020-mlat -w
```



## 6. Acquire Calibration


### 6.1. Prepare DM poke modes

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



### 6.2. Run acquisition


```bash
# Acquire response matrix - Hadamard modes
# 4 cycles - default is 10.
cacao-aorun-030-acqlinResp -n 4 -w HpokeC
```
This could take a while. Check status on milk-procCTRL.
To inspect results, display file conf/RMmodesWFS/HpokeC.WFSresp.fits.

### Decode Hadamard matrix

```bash
cacao-aorun-031-RMHdecode
```
To inspect results, display file conf/RMmodesWFS/zrespM-H.fits.
This should visually look like a zonal response matrix.


### 6.3. Make DM and WFS masks

```bash
cacao-aorun-032-RMmkmask
```
Check results:
- conf/dmmask.fits
- conf/wfsmask.fits

If needed, rerun command with non-default parameters (see -h for options).
Note: we are not going to apply the masks in this example, so OK if not net properly. The masks are informative here, allowing us to view which DM actuators and WFS pixels have the best response.


### 6.4. Create synthetic (Fourier) response matrix

```bash
cacao-aorun-033-RM-mksynthetic -c 25 -a 2.0
```


## 7. Compute control matrix (straight)

Compute control modes, in both WFS and DM spaces.
Set GPU device (if GPU available).

```bash
cacao-fpsctrl setval compstrCM svdlim 0.002
cacao-fpsctrl setval compstrCM GPUdevice 0
```
Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```

Check results:
- conf/CMmodesDM/CMmodessDM.fits
- conf/CMmodesWFS/CMmodesWFS.fits


## 8. Running the loop

### 8.1. Core processes

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


python -m pycacao.calib.mkmodes randhad HpokeCrand.fits (edited) 
python -m pycacao.calib.rmdecode


Closing the loop and setting loop parameters with mfilt:

```bash
# Set loop gain
cacao-fpsctrl setval mfilt loopgain 0.1

# set modal gains, mults and limits
cacao-aorun-061-setmgains 0.8 -f 0.05 -t 1.2
cacao-aorun-062-setmgmults 0.05 -f 0.9 -t 1.0
cacao-aorun-063-setmlimits 0.8 -f 0.05 -t 1.0

# Set loop mult
cacao-fpsctrl setval mfilt loopmult 0.98

# close loop
cacao-fpsctrl setval mfilt loopON ON

```

Misc tools:

```bash
# Astrogrid control
cacao-DMastrogrid start
cacao-DMastrogrid stop

# Set WFS reference to flat illumination over wfsmask
cacao-wfsref-setflat
```

scexao-specific tools
```bash

```



### 8.2. Zero Point Offsetting

```bash
cacao-aorun-071-zpo start
```

Select DM channels to be included in zpo.


## 9. Testing the loop

### 9.1. SelfRM

```bash
# Set max number of modes above nbmodes to measure all modes

cacao-fpsctrl runstop mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.zsize 20
cacao-fpsctrl setval mfilt selfRM.NBmode 2000
cacao-fpsctrl runstart mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.enable ON
```

Check result: vispyr2-rundir/selfRM.fits

### 9.2. Turbulence

```bash
cacao-aorun-100-DMturb start
cacao-aorun-100-DMturb off
cacao-aorun-100-DMturb on
cacao-aorun-100-DMturb stop
```

### 9.3. Monitoring

```bash
cacao-modalstatsTUI
```


## 10. Predictive Control

### 10.1. Pseudo-OL reconstruction

OPTIONAL: Tune software latency and WFS factor to ensure exact pseudoOL reconstruction.

```bash
cacao-fpsctrl setval mfilt auxDMmval.enable ON
cacao-fpsctrl setval mfilt auxDMmval.mixfact 1.0
cacao-fpsctrl setval mfilt auxDMmval.modulate OFF

cacao-fpsctrl setval mfilt loopgain 0.03
cacao-fpsctrl setval mfilt loopmult 0.999

cacao-aorun-080-testOL -w 1.0

# repeat multiple times to converge to correct parameters
cacao-aorun-080-testOL -w 0.1
```

Check that probe and psOL reconstruction overlap and have same amplitude:
```bash
gnuplot
plot [0:] "vispyr2-rundir/testOL.log" u 1:2 w l title "probe", "vispyr2-rundir/testOL.log" u ($1):5 title "psOL", "vispyr2-rundir/testOL.log" u ($1):3 title "DM"
quit
```
The x-offset is the total latency (hardw+softw).



### 10.2. Modal control blocks

Start process mctrlstats to split telemetry into blocks.

```bash
cacao-aorun-120-mstat start
```

Start mkPFXX-Y processes.
```bash
cacao-aorun-130-mkPF 0 start
cacao-aorun-130-mkPF 1 start
```

Start applyPFXX-Y processes.
```bash
cacao-aorun-140-applyPF 0 start
cacao-aorun-140-applyPF 1 start
```


# Cleanup

From main directory (upstream of rootdir) :

```bash
cacao-msglogCTRL stop
cacao-task-manager -C 0 scexao-vispyr-bin2
rm -rf .vispyr2.cacaotaskmanager-log
```



THE END
