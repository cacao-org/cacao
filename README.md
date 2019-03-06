[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/8fc93c97bde340078b02340e71b10580)](https://www.codacy.com/app/oguyon/CACAO?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=CACAO-org/CACAO&amp;utm_campaign=Badge_Grade)





Master branch v0.1.01
[![Build Status](https://travis-ci.org/cacao-org/cacao.svg?branch=master)](https://travis-ci.org/cacao-org/cacao)
![lastcommit](https://img.shields.io/github/last-commit/cacao-org/cacao/master.svg)


Development branch v0.1.02
[![Build Status dev](https://travis-ci.org/cacao-org/cacao.svg?branch=dev)](https://travis-ci.org/cacao-org/cacao) 
![lastcommit](https://img.shields.io/github/last-commit/cacao-org/cacao/dev.svg)



[![Gitter](https://badges.gitter.im/cacao-org/community.svg)](https://gitter.im/cacao-org/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)




---

NOTE: cacao uses git submodules. Use `git clone --recursive` (see Downloading and Installing section)

---


# cacao : Compute And Control for Adaptive Optics


<img align="left" src="cacao-logo-250pix.png">

cacao is a computation engine for adaptive optics control.

**Compute Performance**: Uses multi-core CPUs and GPGPUs for [high computing throughput](https://github.com/cacao-org/cacao/wiki/Compute-Performance-Benchmarks). Written in C, optimized for performance. Holds images in RAM, with image stream support (shared memory with low-latency IPC support). cacao uses [milk](https://github.com/milk-org/milk). 


**User input**: Executable launches a [command line interface (CLI)](https://cacao-org.github.io/cacao/page_userinput.html) from which functions are accessible. Type "help" in the CLI to get started.


**Modular**, [easy to add functions](https://cacao-org.github.io/cacao/page_LoadingModules.html), loaded at runtime as shared objects.



---


## Downloading and installing cacao


```bash
git clone --recursive https://github.com/cacao-org/cacao cacao
```


```bash
cd cacao
mkdir _build
cd _build
cmake ..
```


### Post-installation 

You may need to add /usr/local/lib to LD_LIBRARY_PATH environment variable:
```bash
echo "/usr/local/lib" > usrlocal.conf
sudo mv usrlocal.conf /etc/ld.so.conf.d/
sudo ldconfig -v
```

Unless you have a separate install of milk on your system, a symbolic link to milk is required for milk scripts that are included in cacao:

```bash
sudo ln -s /usr/local/bin/cacao /usr/local/bin/milk
```

Add milk executable scripts to PATH environment variable. Add this line to the .bashrc file (change source code location as needed):
```bash
export PATH=$PATH:/home/myname/src/cacao/src/CommandLineInterface/scripts
```


---

## Shared memory data streams

Both cacao and milk use a common shared memory data stream format. See [ImageStreamIO module](https://github.com/milk-org/ImageStreamIO) for details.

### Viewing real-time data streams

Additional software is required to view real-time data streams. Several options exist:

  * [shmimviewGTK](https://github.com/milk-org/shmimviewGTK), a lightweight efficient GTK-based viewer.
  * [milk2ds9](https://github.com/jaredmales/milk2ds9) uses ds9 to view data streams (convenient for ds9 users)
  * [rtimv](https://github.com/jaredmales/rtimv), a qt-based image viewer, higher performance than ds9 option
  * [shmimviewqt](https://github.com/milk-org/shmimviewqt), another qt-based option (less polished than rtimv)
  * [xaosim](https://github.com/fmartinache/xaosim)'s shmview image viewer, using python qt interface
  
### Python interface to data streams

Python users can read/write milk/cacao's data streams using additional packages:

  * [pyImageStreamIO](https://github.com/milk-org/pyImageStreamIO) provides an interface to data streams.
  * [xaosim](https://github.com/fmartinache/xaosim) includes a python interface to data streams.. and much more



---


## Documentation

[Online documentation]( http://CACAO-org.github.io/cacao/index.html ) 

Report bugs and issues on [this page]( https://github.com/cacao-org/cacao/issues )

Contributing to project : See [coding standards]( http://CACAO-org.github.io/cacao/page_coding_standards.html ) 




## Libraries

The following libraries are used:

- readline, for reading the command line input
- ncurses-dev
- flex, for parsing the command line input
- bison, to interpret the command line input
- fftw, for performing Fourier Transforms
- gsl, for math functions and tools
- fitsio, for reading and writing FITS image files
- CUDA, CuBLAS, MAGMA for GPU acceleration (optional)

If you use NVIDIA GPUs, install cuda and magma libraries, and add "--enable-cuda and --enable-magma" options to the configure command.


---


## Getting Started

All functions are accessible from the command line interface (CLI). Enter the CLI and type "help" for instructions.

```bash
cacao
```
---


## LICENCE

[GNU General Public License v3.0]( https://github.com/cacao-org/cacao/blob/master/LICENCE.txt )
