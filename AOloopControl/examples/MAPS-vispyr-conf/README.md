# Overview

MMT MAPS project.
Visible light Pyramid WFS. Slopes input.



# Running the example

Notes:
- Use commands milk-streamCTRL, milk-fpsCTRL and milk-procCTRL to track status
- Run each command with -h option to get help and more details


## Setting up processes


```bash
# Deploy configuration :
# download from source to current directory
cacao-loop-deploy -c MAPS-vispyr

# OPTIONAL: Change loop number, name, DM index, simulation DM index:
# CACAO_LOOPNUMBER=7 cacao-loop-deploy -c MAPS-vispyr
# CACAO_LOOPNUMBER=7 CACAO_DMINDEX="03" cacao-loop-deploy -c MAPS-vispyr

# OPTIONAL: Edit file MAPS-vispyr-conf/cacaovars.bash as needed
# For example, change loop index, DM index, etc ...

# OPTIONAL: Clean previous deployment :
# rm -rf .maps.cacaotaskmanager-log

# Run deployment (starts conf processes)
cacao-loop-deploy -r MAPS-vispyr

# Note: the copy and run steps can be done at once with :
# cacao-loop-deploy MAPS-vispyr


# Go to rootdir, from which user controls the loop
cd maps-rootdir

# select simulation mode
./scripts/aorun-setmode-sim
# (alternatively, run ./scripts/aorun-setmode-hardw to connect to hardware)
```

## Run DM, DM+WFS simulators

```bash
# Run hardware DM (optional if running in simulation mode)
# cacao-aorun-000-dm start

# Run simulation DM
cacao-aorun-001-dmsim start

# Start simulation processes (skip if in hardware mode) :
# - starts  process simmvmgpu
# - starts process DMstreamDelay
# First, select GPU (99=CPU)
cacao-fpsctrl setval simmvmgpu GPUindex 99
# then, run the process
cacao-aorun-002-simwfs start
```


## Start WFS acquisition

```bash
# Acquire WFS frames
cacao-aorun-025-acqWFS -w start
```



## Measure DM to WFS latency

FOR SIMULATOR ONLY: adjust simulator frequency to computing hardware capabilities :

```bash
# Set simulator speed
cacao-fpsctrl setval simmvmgpu procinfo.triggerdelay 0.01
```

```bash
# Measure latency
# option -w is to wait for completion
cacao-aorun-020-mlat -w
```

Check latency file. If using gnuplot with loop number 2 :
```bash
plot "maps-rundir/fps.mlat-2.datadir/hardwlatency.dat" u 2:3
```
Check that the latency curve is clean and the latency reported matches the curve peak.



## Acquire response matrix


### Prepare DM poke modes

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes
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

Note: With a 1D DM representation, FpokesC and ZpokesC are meaningless.

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



### Alternative: Import arbitrary RM

Instead of acquiring a RM with the above method (Hadamard + decode), an arbitrary RM can be imported and projected to the zonal space.

```bash
cacao
# load custom RM files
loadfits "customRMmodesDM.fits" RMmodesDM
loadfits "customRMmodesWFS.fits" RMmodesWFS
# convert to zonal representation
# last argument is SVD limit - may require tuning
cacaocc.RM2zonal RMmodesDM RMmodesWFS RMmodesDMz RMmodesWFSz 0.01
# save results
saveFITS RMmodesDMz "conf/RMmodesDM/RMmodesDMz.fits"
saveFITS RMmodesDMz "conf/RMmodesWFS/RMmodesWFSz.fits"
exitCLI
```

To adopt this matrix :
```bash
ln -sf ${PWD}/conf/RMmodesDM/RMmodesDMz.fits ./conf/RMmodesDM/RMmodesDM.fits
ln -sf ${PWD}/conf/RMmodesWFS/RMmodesWFSz.fits ./conf/RMmodesWFS/RMmodesWFS.fits
```




### Make DM and WFS masks

```bash
cacao-aorun-032-RMmkmask -dmc0 0.0 -dmc1 0.0
```
Check results:
- conf/dmmask.fits
- conf/wfsmask.fits

If needed, rerun command with non-default parameters (see -h for options).
Note: we are not going to apply the masks in this example, so OK if not net properly. The masks are informative here, allowing us to view which DM actuators and WFS pixels have the best response.




### Create synthetic (Fourier) response matrix

A synthetic RM allows for modes to be weighted by spatial frequency prior to computing the CM. Doing so will ensure that the resulting CM is approximately ordered by spatial frequency.

```bash
cacao-aorun-033-RM-mksynthetic -c 7
```


## Compute control matrix (straight)

Compute control modes, in both WFS and DM spaces.

```bash
cacao-fpsctrl setval compstrCM svdlim 0.001
```

Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```
Inspect result:
- conf/CMmodesDM/CMmodesDM.fits
- conf/CMmodesWFS/CMmodesWFS.fits

Check especially the number of modes controlled.
With svdlim=0.001, there should be 189 control modes.

## Running the loop


Select GPUs for the modal decomposition (WFS->modes) and expansion (modes->DM) MVMs
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 99
cacao-fpsctrl setval mvalC2dm GPUindex 99
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



## Testing the loop (selfRM)


```bash
# Set max number of modes above nbmodes

cacao-fpsctrl runstop mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.zsize 20
cacao-fpsctrl setval mfilt selfRM.NBmode 1000
cacao-fpsctrl runstart mfilt 0 0
cacao-fpsctrl setval mfilt selfRM.enable ON
```

Check result: maps-rundir/selfRM.fits


# Cleanup

From main directory (upstream of rootdir) :

```bash
cacao-task-manager -C 0 MAPS-vispyr
rm -rf .maps.cacaotaskmanager-log
```


THE END
