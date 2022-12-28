# Overview

SCExAO system pyramid WFS.
Low-resolution WFS mode (120x120)

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

## Setup and Calibration

    $ cacao-loop-deploy scexao-vispyr-bin2
    $ cd vispyr2-rootdir
    $ ./aorun-setmode-sim
    $ cacao-aorun-000-dm start
    $ cacao-aorun-001-dmsim start
    $ cacao-aorun-002-simwfs start
    $ cacao-aorun-005-takedark
    $ cacao-aorun-020-mlat -w
    $ cacao-aorun-025-acqWFS start
    $ cacao-aorun-030-acqzRM start
    $ cacao-aorun-035-acqloRM start
    $ cacao-aorun-040-compfCM
    $ mkdir -p ../AOcalibs
    $ cd ..; ln -s $(pwd)/AOcalibs $(pwd)/vispyr2-rootdir/AOcalibs; cd -
    $ cacao-calib-archive cal000
    $ cacao-calib-apply cal000

## Running the loop

From directory vispyr-rootdir :







THE END
