# Observing Run Notes:

These are notes for running MAPs as of the 2023 May/June run. References to simulations are cut out for ease of use. 

## Streams

Milk commands for keeping tabs on streams:
- `milk-streamCTRL`
    - x => exit the viewer 
- `milk-fpsCTRL` => "Function Parameter monitor" can set params and run proc from here
    - shift + R => starts a process
    - ctrl + R => ends a process
    - enter => changes a value
    - space => to run thing within menues
- `milk-procCTRL` => what processes are actively running

How to check a specific stream?
`milk-shmimmon dm00disp`

## CACAO: Set up at start of night

make a new working directory
<br> `mkdir work-2023-06-##`
<br> `cd work-2023-06-##`

download from source to current directory
<br>`cacao-loop-deploy -c MAPS-vispyr`

run deployment
<br>`cacao-loop-deploy -r MAPS-vispyr`

go to the root directory
<br> `cd maps-rootdir`

connect to hardware
<br> `./scripts/aorun-setmode-hardw`


## CHAI: Setting up 
(not a part  of this package, MAPs specific)

```bash
chai
> ttmod.enable
> ttmod.start 
# these previous two lines have been automated at time of writing
> viswfs.enable # starting the wfs streams
> cacao.enable 
```
Once chai has been started, we can look at wfs streams
``` bash
rtimview viswfs_slopes & 
rtimview viswfs_pupils &
```

## CACAO

### Connecting stream readers

Start hardware DM
<br> `cacao-aorun-000-dm start`

Start WFS fram aquisition
<br> `cacao-aorun-025-acqWFS -w start`

### DM to WFS latency

Running from command line: 
```bash
# option -w is to wait for completion
cacao-aorun-020-mlat -w
```

Running from fpsCTRL:
<br>    shift + R `mlat`

Viewing latency:
``` bash
gnuplot
plot "maps-rundir/fps.mlat-2.datadir/hardwlatency.dat" u 2:3
```
can check `mlat-> out -> latencyfr`
- we expect this to be about 2 frames, around 5 frames we know would be noise

### Response Matrix Aquisition

#### Prepare Poke Modes (do once)

```bash
# Create DM poke mode cubes
cacao-mkDMpokemodes
```

Can add our own poke modes in this folder

#### Run acquisition

```bash
cacao-aorun-030-acqlinResp -n 6 -w HpokeC
```
- Acquires response matrix - Hadamard modes
- -n 6 cycles - default is 10.

If this is a hadamar mode, you'll need to decode:
```bash
cacao-aorun-032-RMmkmask -dmc0 0.0 -dmc1 0.0
```
Check results:
- conf/dmmask.fits
- conf/wfsmask.fits

How to do this in fpsCTRL:
 - shift + R `measlinreps-2`
 - edit `measlinreps -> inmodes` to change poke matrix
 - `measlinresp ->ampl -> 200` for most recent MAPs params

#### Compute control matrix

Compute control modes, in both WFS and DM spaces.

```bash
cacao-fpsctrl setval compstrCM svdlim 0.01
```

Then run the compstrCM process to compute CM and load it to shared memory :
```bash
cacao-aorun-039-compstrCM
```

Inspect result:
- ds9 conf/CMmodesDM/CMmodesDM.fits # these are still in a 1D image
- ds9 conf/CMmodesWFS/CMmodesWFS.fits

## Running the loop


Select GPUs for the modal decomposition (WFS->modes) and expansion (modes->DM) MVMs
```bash
cacao-fpsctrl setval wfs2cmodeval GPUindex 99
cacao-fpsctrl setval mvalC2dm GPUindex 99
```

Start the 3 loop processes
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

Alternatively, on the fpsCTRL
- shift + R `wfs2cmodeval`
- shift + R `mfilt` 
- shift + R `mvalC2dm` 
- edit values in `mfilt`
   - `mfilt -> loopgain -> 0.1`
   - `mfilt -> loopmult -> 0.98`
- spacebar `mfilt -> loopON -> ON`

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
