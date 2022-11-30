# List of examples

Each directory is named **CONFNAME**-conf.

Name                       |  Description
---------------------------|------------------------------------------------------------
scexao-vispyr-bin2-conf    | SCExAO visible pyramid WFS loop, 2x2 binned WFS (this is the recommended example)
scexao-vispyr-bin1-conf    | SCExAO visible pyramid WFS loop, unbinned WFS
KalAO-dmloop-conf          | KalAO system, SHWFS


---

# Deploying and running cacao examples - Quickstart


To deploy a cacao loop from the example configuration :


```bash
# from work directory (workdir)
# workdir is the current directory, where the configuration will be deployed
cacao-loop-deploy <CONFNAME>
```

For example :

```bash
# from work directory (workdir)
cacao-loop-deploy scexao-vispyr-bin2
```

The cacao-loop-deploy script will copy the configuration from the source to the current directory and run it.

Then, run the following tools to control and monitor processes and streams:

```bash
# from any directory
milk-fpsCTRL     # interact with function parameters, run/stop processes
milk-streamCTRL  # monitor streams
milk-procCTRL    # monitor processes
```

Each example comes with a set of user scripts, following the naming convention aorun-XXX-yyyyyy, where XXX is an integer representing in which order scripts should be exectuted, and yyyyyy is a descriptive name. For example:

```bash
# from subdirectory workdir/$LOOPROOTDIR
./aorun-000-simstart  # start hardware simulator
```

Use the -h option to get more details.

---

# Running cacao examples - Details


## 0. Conventions and naming

Each example directory contains the cacao configuration files required to setup and run an example. Directories are named :

	CONFNAME-conf

Where CONFNAME is, unsurprisingly, the example name.

Directory and file names for each example are constructed from the following three variables:

- **CONFNAME**: The configuration name - this is the name of the directory containing the configuration (dirname = **CONFNAME**-conf).
- **LOOPNAME**: The loop name, which will be the basis for process names, tmux sessions and various files.
- **LOOPROOTDIR**: Directory where cacao will install files, and from which most scripts should be launched.
- **LOOPRUNDIR**: Subdirectory of LOOPROOTDIR from which executables are running

:warning: Make sure you understand the role of these four variables before proceeding. Confusingly, **CONFNAME**, **LOOPNAME**, **LOOPROOTDIR** and **LOOPRUNDIR** could be the same string. You can set them to be identical if a single configuration will run a single loop in a single directory. For testing purposes, it may be useful to deploy multiple versions of the same loop in different directories, and/or to maintain multiple configurations for the same loop: to manage these cases, the four names can be different.

Name                  |  Where is it set ?
----------------------|------------------------------------------------------------
**CONFNAME**          | This is a directory name: **CONFNAME**-conf contains configuration files
**LOOPNAME**          | Variable CACAO_LOOPNAME defined in file cacaovars.bash
**LOOPROOTDIR**       | Variable CACAO_LOOPROOTDIR defined in file cacaovars.bash
**LOOPROOTDIR**       | Variable CACAO_LOOPRUNDIR defined in file cacaovars.bash


---

# 1. Description of files in each configuration directory

Content of directory CONFNAME-conf

~~~
├── <CONFNAME>-conf                     -> configuration directory (where configuration files are stored)
│   ├── tasklist.txt                    -> List of tasks that will be managed by cacao-task-manager
│   ├── cacaovars.bash                  -> Variables defining the configuration: lists processes to be setup by cacao-setup
│   ├── fpssetup.setval.conf            -> (optional) Initialization read by milk-fpsCTRL after launch
│   ├── aorun-XXX-yyyyyy                -> (optional) custom user script. XXX=index, yyyyy=description
│   └── simLHS                          -> (optional) Linear Hardware Simulation files
~~~


---

# 2. Selecting a cacao example

:warning: You need to have read and write permission in the current directory.

First, copy the example configuration directory to the current (work) directory :

```bash
# from workdir
rsync -au --progress $MILK_ROOT/plugins/cacao-src/AOloopControl/examples/CONFNAME-conf .
```

---

# 3. Running Setup Tasks

cacao-task-manager is a high level wrapper script that runs a sequence of tasks, reading configuration files, and deploying necessary processes and tmux sessions.

To get more info about the task manager script, run it with help option:

```bash
# from workdir
cacao-task-manager -h <CONFNAME>
```

To list the tasks and their status, run the command as follows:

```bash
# from workdir
cacao-task-manager <CONFNAME>
```

For example, the following tasks could be listed :

~~~
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
~~~


:warning: Instruction steps below depend on the tasks. For example, tasks GETSIMCONFFILES and TESTCONFIG may not exit... in which case you can skip the reading the corresponding sections. Note also that the task numbering may change: if GETSIMCONFFILES and TESTCONFIG don't exist, then CACAOSETUP will be task #1. The example may also include additional setup tasks.

## 3.1. Setting up LOOPROOTDIR

The INITSETUP task creates the **LOOPROOTDIR** directory :

```bash
# from workdir
cacao-task-manager -X 0 <CONFNAME>
```

The LOOPROOTDIR content is as follows :

~~~
├── <LOOPROOTDIR>
│   ├── aorun-XXX-yyyyyy files
│   ├── cacaovars.LOOPNAME.bash
│   └── fpssetup.setval.LOOPNAME.conf
├── .cacaotaskmanager-log
~~~

The number of aorun scripts depends on the configuration.



## 3.2. Uploading external file(s)

Some of the examples include downloading external files.
The GETSIMCONFFILES task downloads calibration file to simulate an AO system for test purposes.

To run this step:

```bash
# from workdir
cacao-task-manager -X 1 <CONFNAME>
```


## 3.3. Testing the configuration

The TESTCONFIG task performs tests.

To run this step:

```bash
# from workdir
cacao-task-manager -X 2 <CONFNAME>
```


## 3.4. Running cacao-setup

The CACAOSETUP task runs cacao-setup within **LOOPROOTDIR**, which :

- Reads cacaovars.LOOPNAME.bash to collect information about main loop parameters and which processes should be run.
- Prepares tmux sessions and windows
- Launches all conf processes
- Launches milk-fpsCTRL instance in tmux session. This will be used to manage and communicate with processes.


cacao-setup is the main setup script, which calls other scripts and sets parameters for processes. The help option lists the main operations performed by cacao-setup :

```bash
# from anywhere
cacao-setup -h
```


To run this step using cacao-task-manager:

```bash
# from workdir
cacao-task-manager -X 3 <CONFNAME>
```

:bulb: To run tasks 0, 1, 2 and 3 (inclusive), you can skip the X=0,1,2 commands, and simply run the X=3 command.

---


# 4. Configuring and controlling processes through milk-fpsCTRL fifo: aorunscript

The cacao-setup task (task 3 above) will start an instance of milk-fpsCTRL within a dedicated tmux session. This instance is processing commands sent to a fifo named **/milk/shm/LOOPNAME_fpsCTRL.fifo**.

From this point on, scripts can send commands to the fifo to change parameters, and run/stop processes. Alternatively, users can also use milk-fpsCTRL as an interactive GUI to perform these operations.

Scripts **aorun-XXX-yyyyy** included in some of the examples perform these steps.

```bash
# from workdir/$LOOPROOTDIR
./aorun-000-simstart
```

Users are encouraged to read the script content as a template for writing custom scripts.



---

# 5. Managing data products and configurations


## 5.1.Directory, files, scripts and conventions

Each process managed by the function parameter structure (FPS) framework uses the following standard directories:

- **LOOPROOTDIR/LOOPRUNDIR/fps._fpsname_.data** : Work directory for the FPS processes. This is where results are written by the run process. This directory may contain both temporary files which do not need to be archived, and files/output that should be saved and archived.
- **LOOPROOTDIR/LOOPRUNDIR/fps._fpsname_.conf** : Configuration directory for the FPS processes. This is mostly an input for the FPS.
- **LOOPROOTDIR/LOOPRUNDIR/fps._fpsname_.archive**: Archive directory. Note this is usually a sym link to another directory.



<!--
The directories are managed by the following scripts:

- **fpsconf-adopt**: Copy configuration files/parameters from fps._fpsname_.data to fps._fpsname_.conf
- **fpsconf-sync**: Import parameters from fps._fpsname_.conf to the active FPS conf process.
- **fpsconf-archive**: Copy configuration to an archive directory, attaching timestamp and label.
- **fpsconf-load**: Load from fps._fpsname_.archive into fps._fpsname_.conf
-->



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
