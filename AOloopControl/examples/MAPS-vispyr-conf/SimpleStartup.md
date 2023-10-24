# Startup Simple
This document assumes you already have the loaded directories etc. Your system is already calibrated. You should already have an interaction matrix.

Starting from nothing, in maps-rootdir
```
cacao-setup maps
```

Past that we run, with no calibration/poke:

```
cacao-aorun-000-dm start
ls -sf /milk/shm/viswfs_slopes.im.shm /milk/shm/aol2_wfsim.im.shm
cacao-aorun-025-acqWFS -w start
```

Now we are picking a calibration
``` 
ls ../maps-calibs
 ```
find the <calib-dir> from that list that you want to use tonight.

```
cacao-calib-apply <calib-dir>
```
^ long term, this needs to shut down all confs and restart them again, this ensures the proper resp mat sizes. 

```
cacao-aorun-050-wfs2cmval start
cacao-aorun-060-mfilt start
cacao-aorun-070-cmval2dm start
```

To start the 2D DM viewing stream:
``` 
tmux new -s remap
./scripts/maps-remapdm10disp 
ctrl-b d
```

