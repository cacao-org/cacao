# Module: pyramidWFStools

Pyramid wavefront sensor calibration, packed-zone flux budgeting, edge-based 4-pupil alignment
tracking, and live/offline telemetry tools for `cacao`.

## Architecture (C Core + `@calc` / Bash Outer Shell)

| Layer | File(s) | Purpose |
| :--- | :--- | :--- |
| Pure-C core | `pyrwfs_tlm_core.h` | Zero-milk-dependency single-pass packed-zone flux budget, single-frame & temporal-variance 4-pupil geometry detector (`pyrwfs_detect_pupils_cube`), soft-Sobel 6-DOF pupil registration, bilinear 4-pupil gather (`Isum`, `Sx`, `Sy`), scintillation, curl/div, LWE |
| C FPS unit | `pyWFStlm.c` (`cacao-fpsexec-pyrwfstlm`) | Live 2D stream & offline 3D cube telemetry engine publishing `<p>_tlm`, `<p>_pupgeom`, `<p>_pupdof`, and registered 2D maps |
| C FPS unit | `pyWFSgridmatch.c` (`cacao-fpsexec-pywfsgridmatch`) | Exact weighted linear least-squares 4-pupil + DM affine grid fitter on `zrespM` |
| Tier 2 `@calc` ext | `calc/pyrwfs_calc_ext.c` (`lib/milk/calc.d/libcacaopyrwfs_calc.so`) | Auto-loaded `@calc` ops: `pyrwfs_detect`, `pyrwfs_pupreg`, `pyrwfs_extract4`, `pyrwfs_metrics` |
| Tier 0 `.calc` units | `units/pyrwfs_pupsteer.calc`, `units/pyrwfs_vibmon.calc` | Live-tunable JIT slow pupil-steering offload & `hist()`/`fft()` vibration/modulator-beat monitor |
| Bash / CLI | `commands.d/pyrwfs.bash`, `scripts/cacao-pyrwfs-*` | `@calc` mask generator (`pyrwfs.mkzones`), calibration (`pyrwfs.calib`), live unit management, `mlt.windlayers` bridge (`pyrwfs.windattach`), offline cube inspector (`pyrwfs.inspect`) |

## Packed Zone-Label Encoding (`uint8_t`)

- Low nibble (`val & 0x0F`, mutually exclusive): `1..4` = pupils (BL, TL, BR, TR), `5` = central apex diamond, `6` = horizontal inter-pupil bridges, `7` = vertical inter-pupil bridges, `8` = outer diffraction halo + obscuration shadows, `9` = far-corner background.
- High nibble (`val & 0xF0`, overlay flags on pupil pixels): `16` = eroded inner pupil, `32` = spider-flanking band (LWE/petaling), `64` = inner diffraction rim.

## Telemetry Vector Columns (`<prefix>_tlm`, 29 float32 values)

`[0] ts`, `[1] ftot`, `[2] bg_mean`, `[3] bg_std`, `[4] sat_hi_frac`, `[5] sat_dark_frac`,
`[6] f_pup`, `[7] f_diam`, `[8] f_bH`, `[9] f_bV`, `[10] f_out`, `[11] df_pup`, `[12] aniso`,
`[13] nl_ratio`, `[14] qx_mean`, `[15] qy_mean`, `[16] qx_rms`, `[17] qy_rms`, `[18] qdiag`,
`[19] scint_idx`, `[20] curl_ratio`, `[21] lwe_metric`, `[22] pup_dx`, `[23] pup_dy`,
`[24] pup_ds`, `[25] pup_dast`, `[26] pup_rot [mrad]`, `[27] pup_corr`, `[28] flux_ok`.
