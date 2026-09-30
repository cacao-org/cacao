# Modules

cacao is split into modules. Each module is a directory at the top of the
repository, and each one builds a shared library that milk loads at runtime.
Within the milk CLI, a module's commands are prefixed by its shortname.

| Directory | Library | CLI shortname | Description |
|---|---|---|---|
| `AOloopControl` | `cacaoAOloopControl` | `cacao` | AO loop control engine: modal and zonal filtering, modal statistics. Also installs the cacao setup, task manager, FPS and `cacao-aorun-*` scripts. |
| `AOloopControl_DM` | `cacaoAOloopControlDM` | `cacaodm` | DM operation: DM channel combination, simulated turbulence, astrogrid |
| `AOloopControl_IOtools` | `cacaoAOloopControlIOtools` | `cacaoio` | I/O tools: WFS image acquisition, WFS camera simulation, spot finding |
| `AOloopControl_acquireCalib` | `cacaoAOloopControlacquireCalib` | `cacaoac` | Acquire calibration: linear response measurement |
| `computeCalib` | `cacaocomputeCalib` | `cacaocc` | Compute calibration: control modes, control matrices, Hadamard decoding, masks |
| `AOloopControl_PredictiveControl` | `cacaoAOloopControlPredictiveControl` | `cacaopc` | Predictive control filters |
| `AOloopControl_compTools` | `cacaoAOloopControlcompTools` | `cacaoct` | Miscellaneous computation tools |
| `AOloopControl_perfTest` | `cacaoAOloopControlperfTest` | `cacaopt` | Performance monitoring and testing, including latency measurement |
| `pyramidWFStools` | `cacaopyramidWFStools` | `cacaopyrwfs` | Pyramid WFS tools |
| `pycacao` | Python package | | Python AO tools (calibration, DM tools) |

## Dependencies

All modules link milk's `CLIcore`, and all modules except `AOloopControl`
link `cacaoAOloopControl`. Linear algebra comes from OpenBLAS with LAPACKE, or
from Intel MKL when found. `computeCalib` uses CUDA and MAGMA when milk is
configured with `-DUSE_CUDA=ON` / `-DUSE_MAGMA=ON`.

Modules also use headers from milk modules they don't link, and
`AOloopControl_acquireCalib` calls functions in `computeCalib` without linking
it. These symbols are resolved at runtime because milk loads all modules into
one process. So a module can't be used on its own without milk.

## Loading a module

```text
milk > mload cacaoAOloopControl
milk > m?
```
