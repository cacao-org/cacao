# Overview

This directory contains cacao configuration files. It should be placed in the root work directory for use by cacao-task-manager script to deploy and manage a cacao loop instance.

## Example directory layout and naming

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


---

# Description of files

## LOOPNAME

Name of cacao loop.

## WORKDIR

Directory in which configuration will run.

## tasklist.txt

List of tasks that will be managed by cacao-task-manager.

## cacaovars.LOOPNAME.bash

Variables (cacaovars) that define the configuration.
Defines which processes will be setup by cacao-setup.

## fpssetup.setval.LOOPNAME.conf

Initialization read by milk-fpsCTRL after lauch.

---

# Deploying a cacao instance


## Copy this directory to the root work directory

    cp cacaoloop-ex01-conf <rootworkdir>

## Run deployment script

    cacao-task-manager -X 3 <CONFNAME>

The cacao-task-manager script reads the configuration files, and deploys necessary processes and tmux sessions.

The main opertion performed are:

- create **WORKDIR** directory
- Prepare tmux sessions and windows
- Launch all conf processes
- Launch milk-fpsCTRL instance in tmux session. This will be used to manage processes.

To get more info about the task manager script, run it with help option:

    cacao-task-manager -h

## Run execution script

    ./aorunscript


---

# Managing data products and configurations


## Directory, files, scripts and conventions

Each process managed by the function parameter structure (FPS) framework uses the following standard directories:

- **fps._fpsname_.data** : Work directory for the FPS processes. This is where results are written by the run process. This directory may contain both temporary files which do not need to be archived, and files/output that should be saved and archived.
- **fps._fpsname_.conf** : Configuration directory for the FPS processes. This is mostly an input for the FPS.
- **fps._fpsname_.archive**: Archive directory. Note this is usually a sym link to another directory.

The directories are managed by the following scripts:

- **fpsconf-adopt**: Copy configuration files/parameters from fps._fpsname_.data to fps._fpsname_.conf
- **fpsconf-sync**: Import parameters from fps._fpsname_.conf to the active FPS conf process.
- **fpsconf-archive**: Copy configuration to an archive directory, attaching timestamp and label.
- **fpsconf-load**: Load from fps._fpsname_.archive into fps._fpsname_.conf


## What is considered part of a configuration ?

Not all files in fps._fpsname_.data should be saved to configuration of archived.

- fps._fpsname_.dat : values of all fields in the FPS
- d


## Communication between FPSs

Tools are provided to manage dependencies between different FPSs.

