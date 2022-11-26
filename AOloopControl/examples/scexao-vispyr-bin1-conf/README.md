# Overview

SCExAO system pyramid WFS.
Full-resolution WFS (240x240)



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

To run the example :

    $ rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/cacaoloop-scexaovispyr-conf cacaoloop-scexaovispyr-rootdir
    $ cd cacaoloop-scexaovispyr-rootdir
    $ cacao-task-manager -X 3 cacaoloop-scexaovispyr
    $ cp cacaoloop-scexaovispyr-conf/aorunscript .
    $ ./aorunscript

THE END
