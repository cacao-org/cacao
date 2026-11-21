# Module: AOloopControl_acquireCalib

Module responsible for `AOloopControl_acquireCalib` functionality.

## Source Files

| File                    | Description                                 |
| ----------------------- | ------------------------------------------- |
| `acquireWFSlincalib.c`  | Acquire linear WFS response                 |
| `measure_linear_resp.c` | Measure linear resp module                          |
| `measure_dyn_resp.c`    | Acquire raw 3D spatiotemporal response cube         |
| `factor_dyn_resp.c`     | Factor raw 3D RM into static M_cal and H_eff[l]     |

## Standalone Executables

| Executable                          | Source File             | Description                                         |
| ----------------------------------- | ----------------------- | --------------------------------------------------- |
| `cacao-fpsexec-cacaoac-measlinresp` | `measure_linear_resp.c` | Measure linear resp module                          |
| `cacao-fpsexec-cacaoac-measdynresp` | `measure_dyn_resp.c`    | Acquire raw 3D spatiotemporal response cube         |
| `cacao-fpsexec-cacaoac-factordynrm` | `factor_dyn_resp.c`     | Factor raw 3D RM into static M_cal and H_eff[l]     |

## Dependencies

- Implicit standard: `milkdata`, `ImageStreamIO`
- Optional: `LAPACKE`, `CBLAS` (via `MILK_CMAKE_REQUEST_LAPACK` / `MILK_CMAKE_REQUEST_BLAS` in `factor_dyn_resp.c`)
