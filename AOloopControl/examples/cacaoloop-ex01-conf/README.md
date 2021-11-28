# 1. Overview

This directory contains cacao configuration files. It should be placed in the root work directory for use by cacao-task-manager script to deploy and manage a cacao loop instance.

## 1.1. Example directory layout and naming

We define **rootworkdir** as the root work directory. All directory paths are relative to **rootworkdir**, which will be ommitted for convenience. User should have read and write permission in **roorworkdir**.

Directory and file names are constructed from the following three variables:

- **CONFNAME**: The configuration name. The present directory is named **CONFNAME**-conf
- **WORKDIR**: Where cacao will install files and run
- **LOOPNAME**: The loop name, which will be the basis for process names, tmux sessions and various files

Note that **WORKDIR** and **LOOPNAME** are specified in a configuration (see section below).

For now, only the **CONFNAME** directory (here) exists. The directory structure downstream of **rootworkdir** will ultimately be:

~~~
├── <CONFNAME>-conf                    -> configuration directory (this directory)
│   └── simLHS                         -> Linear Hardware Simulation files (optional)
├── <WORKDIR>
│   ├── <LOOPNAME>dir                  -> conf and run processes launched from here
│   ├── datalogdir
│   ├── log                            -> cacao-setup log
│   └── simLHS                         -> Linear Hardware Simulator files
├── .<LOOPNAME>-cacaotaskmanager-log   -> cacao-task-manager status and log

~~~

Note that **WORKDIR** and **CONFNAME**, and **LOOPNAME** may or may not be the same. You can set them to be identical if a single configuration will run a single loop in a single directory. For testing purposes, it may be useful to deploy multiple versions of the same loop in different directories, and/or to maintain multiple configurations for the same loop: to manage these cases, the three names will be different.


## 1.2. Description of files

- **LOOPNAME**: Name of cacao loop.
- **WORKDIR**: Directory in which configuration will run.
- **tasklist.txt**: List of tasks that will be managed by cacao-task-manager.
- **cacaovars.LOOPNAME.bash**: Variables (cacaovars) that define the configuration. Defines which processes will be setup by cacao-setup.
- **fpssetup.setval.LOOPNAME.conf**: Initialization read by milk-fpsCTRL after lauch.

---

# 3. Deploying a cacao instance


Copy this directory to the root work directory :

    $ rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/cacaoloop-ex01-conf <rootworkdir>


## 3.1. Running Setup Tasks

cacao-task-manager is a high level script that runs a sequence of tasks, reading configuration files, and deploying necessary processes and tmux sessions.

To get more info about the task manager script, run it with help option:

    $ cacao-task-manager -h <CONFNAME>


To list the tasks and their status, run the command as follows:

    $ cacao-task-manager <CONFNAME>

The following tasks should be listed :

~~~
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
~~~
Subsequent tasks perform specific parts of the AO loop.

The INITSETUP task creates the **WORKDIR** directory.

The GETSIMCONFFILES task downloads calibration file to simulate an AO system for test purposes.

The TESTCONFIG task performs tests.

The CACAOSETUP task runs cacao-setup within **WORKDIR**, which :

- Reads cacaovars.LOOPNAME.bash to collect information about main loop parameters and which processes should be run.
- Prepares tmux sessions and windows
- Launches all conf processes
- Launches milk-fpsCTRL instance in tmux session. This will be used to manage and communicate with processes.


To run tasks 0, 1, 2 and 3 :

    $ cacao-task-manager -X 3 <CONFNAME>


## 3.2. cacao-setup

cacao-setup is the main setup script, which calls other scripts and sets parameters for processes. The help option lists the main operations performed by cacao-setup :

    $ cacao-setup -h


## 3.3. Configuring and controlling processes through milk-fpsCTRL fifo: aorunscript

The cacao-setup task (task 3 above) will start an instance of milk-fpsCTRL within a dedicated tmux session. This instance is processing commands sent to a fifo named **/milk/shm/cacaoloop01_fpsCTRL.fifo**.

From this point on, scripts can send commands to the fifo to change parameters, and run/stop processes. Alternatively, users can also use milk-fpsCTRL as an interactive GUI to perform these operations.

The script **aorunscript** performs these steps.

    $ cp cacaoloop-ex01-conf/aorunscript .
    $ ./aorunscript

Each time aorunscript runs, it performs one step. Run it several times until done. Users are encouraged to read the script content as a template for writing custom scripts.



---

# 4. Managing data products and configurations


## 4.1.Directory, files, scripts and conventions

Each process managed by the function parameter structure (FPS) framework uses the following standard directories:

- **fps._fpsname_.data** : Work directory for the FPS processes. This is where results are written by the run process. This directory may contain both temporary files which do not need to be archived, and files/output that should be saved and archived.
- **fps._fpsname_.conf** : Configuration directory for the FPS processes. This is mostly an input for the FPS.
- **fps._fpsname_.archive**: Archive directory. Note this is usually a sym link to another directory.

The directories are managed by the following scripts:

- **fpsconf-adopt**: Copy configuration files/parameters from fps._fpsname_.data to fps._fpsname_.conf
- **fpsconf-sync**: Import parameters from fps._fpsname_.conf to the active FPS conf process.
- **fpsconf-archive**: Copy configuration to an archive directory, attaching timestamp and label.
- **fpsconf-load**: Load from fps._fpsname_.archive into fps._fpsname_.conf


## 4.2. What is considered part of a configuration ?

Not all files in fps._fpsname_.data should be saved to configuration of archived.

- fps._fpsname_.dat : values of all fields in the FPS
- blahblah


## Communication between FPSs

Tools are provided to manage dependencies between different FPSs.

