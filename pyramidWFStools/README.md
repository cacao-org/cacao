# pyramidWFStools

Pyramid wavefront sensor processing tools for cacao.

## Purpose

Provides algorithms for pyramid WFS image decomposition,
pupil extraction, and modal reconstruction.

## Dependencies

- `CLIcore` — CLI integration
- `milkinfo` — Stream monitoring
- `cacaoAOloopControl` — AO loop framework
- `lapacke` (optional) — Linear algebra

## Key Files

| File | Purpose |
|------|---------|
| `pyramidWFStools.c` | Module init and registration |
| `pyramidWFStools.h` | Public API |
