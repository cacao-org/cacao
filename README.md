[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/8fc93c97bde340078b02340e71b10580)](https://www.codacy.com/app/oguyon/CACAO?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=CACAO-org/CACAO&amp;utm_campaign=Badge_Grade)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;dev branch: [![Build Status dev](https://travis-ci.org/cacao-org/cacao.svg?branch=dev)](https://travis-ci.org/cacao-org/cacao)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;master branch: [![Build Status](https://travis-ci.org/cacao-org/cacao.svg?branch=master)](https://travis-ci.org/cacao-org/cacao)



CACAO version 0.1.01

---

IMPORTANT NOTE: cacao uses git submodules. Use `git clone --recursive` (see Downloading and Installing section)

---


# cacao : Compute And Control for Adaptive Optics


<img align="left" src="cacao-logo-250pix.png">

Computation engine for adaptive optics control; functions accessible through a command line interface (CLI). Holds images in RAM, with image stream support (shared memory with low-latency IPC support).


Written in C, optimized for performance.
Executable launches a command line interface (CLI). Type "help" in the CLI to get started.


Uses multi-core CPUs and GPGPUs to high computing throughput.


Modular, easy to add functions, loaded at runtime as shared objects.

cacao uses [milk](https://github.com/milk-org/milk). 

---


## Downloading and installing cacao

### Downloading 

The CACAO package follows the standard git clone steps :

```bash
git clone --recursive https://github.com/cacao-org/cacao cacao
```


### Compiling


```bash
cd cacao
mkdir _build
cd _build
cmake ..
```

On CentOS, the cmake command could be cmake3.

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

  * [milk2ds9](https://github.com/jaredmales/milk2ds9) uses ds9 to view data streams (convenient for ds9 users)
  * [rtimv](https://github.com/jaredmales/rtimv), a qt-based image viewer, higher performance than ds9 option
  * [shmimviewqt](https://github.com/milk-org/shmimviewqt), another qt-based option (less polished than rtimv)
  * [xaosim](https://github.com/fmartinache/xaosim)'s shmview image viewer, using python qt interface
  
### Python interface to data streams

Python users can read/write milk/cacao's data streams using additional packages:

  * [pyImageStreamIO](https://github.com/milk-org/pyImageStreamIO) provides an interface to data streams.
  * [xaosim](https://github.com/fmartinache/xaosim) includes a python interface to data streams.. and much more


---

# Post-intallation configuration

Add these lines to your .bashrc (edit first line as needed)
```bash
export CACAO_DIR=/home/cacaouser/src/cacao
export LIBRARY_PATH=$LIBRARY_PATH:$CACAO_DIR/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CACAO_DIR/lib

```


---

## Reporting bugs, issues

Report bugs and issues on [this page]( https://github.com/cacao-org/cacao/issues )


---


## Contributing to project

See [coding standards]( http://CACAO-org.github.io/cacao/page_coding_standards.html ) 


---


## Documentation

[Online documentation]( http://CACAO-org.github.io/cacao/index.html ) 


## Libraries

The following libraries are used:

- libtool
- automake
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
./bin/cacao
```
---


## LICENCE

[GNU General Public License v3.0]( https://github.com/cacao-org/cacao/blob/master/LICENCE.txt )
