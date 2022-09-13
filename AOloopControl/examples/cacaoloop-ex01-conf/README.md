# 1. Overview



Copy this directory to the root work directory :

    $ rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/cacaoloop-ex01-conf <rootworkdir>



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

To run tasks 0, 1, 2 and 3 :

    $ rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/cacaoloop-ex01-conf cacaoloop=ex01
    # cd cacaoloop=ex01
    $ cacao-task-manager -X 3 cacaoloop-ex01
    $ cp cacaoloop-ex01-conf/aorunscript .
    $ ./aorunscript
