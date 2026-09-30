# Running a loop from an example

cacao ships example loop configurations in `AOloopControl/examples/`. Each
example is a directory named `<CONFNAME>-conf`. Deploying an example is the
recommended way to start a new loop: copy the closest example, then edit it.

## Available examples

The recommended starting point is `scexao-vispyr-bin2`. Its `README.md` walks
through a full simulated loop, from DM setup to closing the loop.

| CONFNAME | System | LOOPNAME |
|---|---|---|
| `scexao-vispyr-bin2` | SCExAO visible pyramid WFS, 2x2 binned (**recommended**) | `vispyr2` |
| `scexao-vispyr-bin1` | SCExAO visible pyramid WFS, unbinned | `vispyrhr` |
| `scexao-LDFC` | SCExAO | `nirLDFC` |
| `scexao-NIRPL` | SCExAO NIR photonic lantern | `NIRPL` |
| `scexao-llowfs` | SCExAO | `llowfs` |
| `KalAO-dmloop` | KalAO, Shack-Hartmann WFS and main DM | `KalAO-dmloop` |
| `KalAO-ttmloop` | KalAO, tip-tilt offload | `KalAO-ttmloop` |
| `MAPS-vispyr` | MMT MAPS, visible pyramid WFS | `maps` |
| `ao3k-apd188`, `ao3k-apd3k` | Subaru AO3k | `apd188`, `apd3k` |
| `ao3k-bimdm3kpt` | Subaru AO3k | `bimdm3kpt` |
| `ao3k-lowfs188`, `ao3k-lowfs3k` | Subaru AO3k | `lowfs188`, `lowfs3k` |
| `ao3k-nirpyr188`, `ao3k-nirpyr3k` | Subaru AO3k NIR pyramid WFS | `nir188`, `nir3k` |
| `ao3k-ttoff188`, `ao3k-ttoff3k` | Subaru AO3k tip-tilt offload | `ttoff188`, `ttoff3k` |

The `LOOPNAME` column is the default set in each example's `cacaovars.bash`.

## Quickstart

From a work directory where you have read and write permission:

```bash
# Copy the configuration from the cacao source tree into the work directory
cacao-loop-deploy -c scexao-vispyr-bin2

# Optionally edit scexao-vispyr-bin2-conf/cacaovars.bash, then run the setup
cacao-loop-deploy -r scexao-vispyr-bin2
```

`cacao-loop-deploy <CONFNAME>`, without an option, copies and runs in one step.
`cacao-loop-deploy -h` lists all options and the available configurations. The
environment variables `CACAO_LOOPNUMBER`, `CACAO_LOOPNAME`, `CACAO_DMINDEX` and
`CACAO_DMSIMINDEX` override the corresponding values in `cacaovars.bash`:

```bash
CACAO_LOOPNUMBER=7 CACAO_DMINDEX="03" cacao-loop-deploy -c scexao-vispyr-bin2
```

Then use milk's tools to control and monitor the loop:

```bash
milk-fpsCTRL     # view and set function parameters, start/stop processes
milk-streamCTRL  # monitor streams
milk-procCTRL    # monitor processes
```

Loop operations are run from `LOOPROOTDIR` with the `cacao-aorun-XXX-yyyy`
commands. `XXX` gives the usual order of execution and `yyyy` describes the
step. For example:

```bash
cacao-aorun-001-dmsim start     # start the DM simulator
cacao-aorun-005-takedark -n 2000
cacao-aorun-030-acqlinResp -n 4 HpokeC
```

Each command takes `-h`. The example's `README.md` lists the full sequence.

## Naming conventions

Four names identify a deployed loop:

CONFNAME
: The configuration name. The configuration lives in the directory
  `<CONFNAME>-conf`.

LOOPNAME
: The loop name, used for process names, tmux sessions and file names. Set by
  `CACAO_LOOPNAME` in `cacaovars.bash`.

LOOPROOTDIR
: The directory where cacao installs the loop's files, and where most commands
  are run from. Set by `CACAO_LOOPROOTDIR` in `cacaovars.bash`.

LOOPRUNDIR
: The subdirectory of `LOOPROOTDIR` that the processes run in. Set by
  `CACAO_LOOPRUNDIR` in `cacaovars.bash`.

These can all be the same string when one configuration runs one loop in one
directory. They differ when you deploy several copies of a loop, or keep
several configurations for the same loop.

## Configuration directory contents

```text
<CONFNAME>-conf/
├── tasklist.txt           tasks run by cacao-task-manager
├── cacaovars.bash         loop variables and the processes cacao-setup starts
├── fpssetup.setval.conf   (optional) parameter values applied by milk-fpsCTRL at launch
├── fpstmuxenv             (optional) environment for processes in tmux
├── scripts/               (optional) loop-specific scripts
└── simLHS/                (optional) linear hardware simulation files
```

## Setup tasks

`cacao-loop-deploy -r` runs the setup through `cacao-task-manager`, which
executes the tasks listed in `tasklist.txt` in order. Each task code `XXX`
corresponds to an executable `cacaotask-XXX` on the `PATH`.

```bash
cacao-task-manager -h <CONFNAME>   # help
cacao-task-manager <CONFNAME>      # list tasks and their status
cacao-task-manager -X 3 <CONFNAME> # run tasks up to and including task 3
```

A typical task list:

```text
 0           INITSETUP             DONE        READY   Initial setup:
 1     GETSIMCONFFILES             DONE        READY   Get simulation files:
 2          TESTCONFIG             DONE        READY   Test configuration:
 3          CACAOSETUP             DONE        READY   Run cacao-setup:
```

Task numbers depend on the example: if an example has no `GETSIMCONFFILES` or
`TESTCONFIG`, `CACAOSETUP` is task 1.

INITSETUP
: Creates `LOOPROOTDIR`, containing `cacaovars.LOOPNAME.bash` and
  `fpssetup.setval.LOOPNAME.conf`.

GETSIMCONFFILES
: Downloads calibration files used to simulate the AO system.

TESTCONFIG
: Checks the configuration.

CACAOSETUP
: Runs `cacao-setup` in `LOOPROOTDIR`. It reads `cacaovars.LOOPNAME.bash`,
  creates the tmux sessions, launches the configuration processes, and starts a
  `milk-fpsCTRL` instance in its own tmux session. See `cacao-setup -h`.

## Controlling processes through fpsCTRL

The `milk-fpsCTRL` instance started by `CACAOSETUP` reads commands from the
fifo `<LOOPNAME>_fpsCTRL.fifo` in the shared-memory directory
(`/milk/shm/<LOOPNAME>_fpsCTRL.fifo` by default). Scripts, including the
`cacao-aorun-*` commands, write to this fifo to set parameters and to start and
stop processes. You can do the same operations interactively in the
`milk-fpsCTRL` interface.

## FPS directories

Each process managed by the function parameter structure (FPS) framework uses
these directories under `LOOPROOTDIR/LOOPRUNDIR/`:

`fps.<fpsname>.data`
: Working directory. Results are written here, both temporary files and output
  to keep.

`fps.<fpsname>.conf`
: Configuration directory, mostly input to the FPS.

`fps.<fpsname>.archive`
: Archive directory, usually a symlink to another location.
