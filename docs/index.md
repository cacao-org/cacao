# CACAO Documentation

**Compute And Control for Adaptive Optics (CACAO)** is a high-performance computation engine for adaptive optics control, built as a powerful plugin on top of the `milk` framework.

## 🚀 Getting Started
If you are new to CACAO, start here:
- **[Installation and Setup](../README.md)**
- **[Compute Performance Benchmarks](https://github.com/cacao-org/cacao/wiki/Compute-Performance-Benchmarks)**

## 🧠 Core Architecture
CACAO leverages the architecture provided by `milk`, specifically utilizing its zero-copy shared memory (`ImageStreamIO`) and parameter control framework (`FPS`).
- **[Shared Memory Streams (milk)](https://github.com/milk-org/milk/blob/master/docs/streams.md)**
- **[Function Processing System (milk)](https://github.com/milk-org/milk/blob/master/docs/fps.md)**

## 🛠️ Developer Guides
Because CACAO is a `milk` plugin, most of the developer guidelines for writing modules and interacting with the CLI apply directly.
- **[Coding Standards](https://github.com/milk-org/milk/blob/master/docs/developer/coding_standards.md)**
- **[Module Loading & Writing](https://github.com/milk-org/milk/blob/master/docs/developer/plugins.md)**
- **[CACAOCore API Reference](https://cacao-org.github.io/cacao/)**

## 📚 Standardized Module Reference
All computational modules within CACAO now contain standardized `README.md` files mapping their source code to the standalone Executables they generate:
- `AOloopControl`
- `AOloopControl_DM`
- `AOloopControl_IOtools`
- `AOloopControl_acquireCalib`
- `AOloopControl_perfTest`
- `computeCalib`
- `pyramidWFStools`
