# Running cacao examples - Overview

Each example directory contains the cacao configuration files required to setup and run an example. Directories are named :

	CONFNAME-conf

Where CONFNAME is, unsurprisingly, the example name.

Directory and file names for each example are constructed from the following three variables:

- **CONFNAME**: The configuration name (as described above)
- **WORKDIR**: Directory where cacao will install files and run
- **LOOPNAME**: The loop name, which will be the basis for process names, tmux sessions and various files

:warning: Make sure you understand the role of these three variables before proceeding. Confusingly, **CONFNAME**, **WORKDIR** and **LOOPNAME** may or may not be the same. You can set them to be identical if a single configuration will run a single loop in a single directory. For testing purposes, it may be useful to deploy multiple versions of the same loop in different directories, and/or to maintain multiple configurations for the same loop: to manage these cases, the three names can be different.

---

# 1. Description of files in example directory

Content of directory CONFNAME-conf

~~~
├── <CONFNAME>-conf                    -> configuration directory (where configuration files are stored)
│   ├── LOOPNAME                       -> Name of cacao loop
│   ├── WORKDIR                        -> Directory name in which cacao will run
│   ├── tasklist.txt                   -> List of tasks that will be managed by cacao-task-manager
│   ├── cacaovars.LOOPNAME.bash        -> Variables defining the configuration: lists processes to be setup by cacao-setup
│   ├── fpssetup.setval.LOOPNAME.conf  -> (optional) Initialization read by milk-fpsCTRL after launch
│   ├── aorunscript                    -> (optional) custom user script
│   └── simLHS                         -> (optional) Linear Hardware Simulation files
~~~


---

# 2. Selecting a cacao example

We define **rootworkdir** as the work directory under which the cacao example is run. All directory paths are relative to **rootworkdir**, which will be omitted for convenience.

:warning: You need to have read and write permission in **rootworkdir**.

First, copy the example configuration directory to the work directory :

    $ rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/CONFNAME-conf <rootworkdir>
    $ cd <rootworkdir>


---

# 3. Running Setup Tasks

cacao-task-manager is a high level wrapper script that runs a sequence of tasks, reading configuration files, and deploying necessary processes and tmux sessions.

To get more info about the task manager script, run it with help option:

    $ cacao-task-manager -h <CONFNAME>


To list the tasks and their status, run the command as follows:

    $ cacao-task-manager <CONFNAME>

For example, the following tasks could be listed :

~~~
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
~~~


:warning: Instruction steps below depend on the tasks. For example, tasks GETSIMCONFFILES and TESTCONFIG may not exit... in which case you can skip the reading the corresponding sections. Note also that the task numbering may change: if GETSIMCONFFILES and TESTCONFIG don't exist, then CACAOSETUP will be task #1. The example may also include additional setup tasks.

## 3.1. INITSETUP: Setting up WORKDIR

The INITSETUP task creates the **WORKDIR** directory :

        $ cacao-task-manager -X 0 <CONFNAME>

The WORKDIR content will ultimately be as follows :

~~~
├── <WORKDIR>/
│   ├── cacaovars.<LOOPNAME>.bash       -> cacao variables (see cacao-setup step)
│   ├── fpssetup.setval.<LOOPNAME>.conf -> custom FPS setup values (see cacao-setup step)
│   ├── <LOOPNAME>dir/                  -> conf and run processes launched from here
│   ├── datalogdir/
│   ├── log/                            -> cacao-setup log
│   └── simLHS/                         -> Linear Hardware Simulator files (optional)
├── .<LOOPNAME>-cacaotaskmanager-log/   -> cacao-task-manager status and log
~~~

All cacao processes will be running from the WORKDIR/LOOPNAMEdir directory.


## 3.2. GETSIMCONFFILES: Uploading external file(s)

Some of the examples include downloading external files.
The GETSIMCONFFILES task downloads calibration file to simulate an AO system for test purposes.

To run this step:

    $ cacao-task-manager -X 1 <CONFNAME>

## 3.3. TESTCONFIG: Testing the configuration

The TESTCONFIG task performs tests.

To run this step:

    $ cacao-task-manager -X 2 <CONFNAME>

## 3.4. CACAOSETUP: Running cacao-setup

The CACAOSETUP task runs cacao-setup within **WORKDIR**, which :

- Reads cacaovars.LOOPNAME.bash to collect information about main loop parameters and which processes should be run.
- Prepares tmux sessions and windows
- Launches all conf processes
- Launches milk-fpsCTRL instance in tmux session. This will be used to manage and communicate with processes.


cacao-setup is the main setup script, which calls other scripts and sets parameters for processes. The help option lists the main operations performed by cacao-setup :

    $ cacao-setup -h

To run this step using cacao-task-manager:

    $ cacao-task-manager -X 3 <CONFNAME>

:bulb: To run tasks 0, 1, 2 and 3 (inclusive) :

    $ cacao-task-manager -X 3 <CONFNAME>


---


# 4. Configuring and controlling processes through milk-fpsCTRL fifo: aorunscript

The cacao-setup task (task 3.4. above) will start an instance of milk-fpsCTRL within a dedicated tmux session. This instance is processing commands sent to a fifo named **/milk/shm/LOOPNAME_fpsCTRL.fifo**.

From this point on, scripts can send commands to the fifo to change parameters, and run/stop processes. Alternatively, users can also use milk-fpsCTRL as an interactive GUI to perform these operations.

The script **aorunscript** included in some of the examples performs these steps.

    $ cp CONFNAME-conf/aorunscript .
    $ ./aorunscript

Each time aorunscript runs, it performs one step. Run it several times until done. Users are encouraged to read the script content as a template for writing custom scripts.



---

# 5. Managing data products and configurations


## 5.1.Directory, files, scripts and conventions

Each process managed by the function parameter structure (FPS) framework uses the following standard directories:

- **fps._fpsname_.data** : Work directory for the FPS processes. This is where results are written by the run process. This directory may contain both temporary files which do not need to be archived, and files/output that should be saved and archived.
- **fps._fpsname_.conf** : Configuration directory for the FPS processes. This is mostly an input for the FPS.
- **fps._fpsname_.archive**: Archive directory. Note this is usually a sym link to another directory.

The directories are managed by the following scripts:

- **fpsconf-adopt**: Copy configuration files/parameters from fps._fpsname_.data to fps._fpsname_.conf
- **fpsconf-sync**: Import parameters from fps._fpsname_.conf to the active FPS conf process.
- **fpsconf-archive**: Copy configuration to an archive directory, attaching timestamp and label.
- **fpsconf-load**: Load from fps._fpsname_.archive into fps._fpsname_.conf


<!--
# Notes - to be done


:construction: Everything below this point is under construction

Three directories are associated to each fps:

- datadir: where files are written, acts as a staging/working directory
- confdir: where configuration is stored. This can be automatically read/imported by other FPSs
- archivedir
These directories are set in <FPSname>.conf.datadir, <FPSname>.conf.confdir, and <FPSname>.conf.archivedir


FPS actions include:

- ADOPT   : copy from datadir to confdir
- SYNC    : load/update from confdir to FPS memory content
- ARCHIVE : copy from datadir to archive directory
- PULL    : copy from archive to datadir


Communication between FPSs can be achieved in multiple ways:

- A FPS entry can point to another FPS. This allows real-time exchange of parameter values.



After each step, to archive results:

cacao-fpsarchive

To adopt results :

cacao-fpsconfadopt
-->

---

THE END
