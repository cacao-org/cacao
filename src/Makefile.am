SUBDIRS = src
ACLOCAL_AMFLAGS = -I m4


#SUFFIXES = .c .h .y .l

bin_PROGRAMS = CACAO
CACAO_SOURCES = CLImain.c CommandLineInterface/CLIcore.c CommandLineInterface/CLIcore.h CommandLineInterface/calc_bison.y CommandLineInterface/calc_flex.l CommandLineInterface/calc.h
CACAO_CPPFLAGS = -I@abs_top_srcdir@/src -DCONFIGDIR=\"@abs_top_srcdir@/config\" -DSOURCEDIR=\"@abs_top_srcdir@/src\" -DABSSRCTOPDIR=\"@abs_top_srcdir@\" -fopenmp -ldl

ACLOCAL_AMFLAGS = -I m4

## For Bison and FLEX
AM_YFLAGS = -d
AM_LFLAGS = -i 

## The contents of this variable are passed to every compilation that invokes the C preprocessor IF target_CPPFLAGS does not exit; it is a list of arguments to the preprocessor. For instance, -I and -D options should be listed here.
## AM_CPPFLAGS = -DCONFIGDIR=\"@abs_top_srcdir@/config\"

noinst_HEADERS = CommandLineInterface/calc.h

BUILT_SOURCES = CommandLineInterface/calc_bison.h


SUBDIRS = 
CACAO_LDADD = 


## Modules are built as libraries of convenience
## Linking order matters

## 
## AO modules
##

SUBDIRS += AOloopControl
CACAO_LDADD += AOloopControl/libaoloopcontrol.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl

SUBDIRS += AOloopControl_acquireCalib
CACAO_LDADD += AOloopControl_acquireCalib/libaoloopcontrolacquirecalib.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_acquireCalib

SUBDIRS += AOloopControl_compTools
CACAO_LDADD += AOloopControl_compTools/libaoloopcontrolcomptools.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_compTools

SUBDIRS += AOloopControl_computeCalib
CACAO_LDADD += AOloopControl_computeCalib/libaoloopcontrolcomputecalib.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_computeCalib

SUBDIRS += AOloopControl_DM
CACAO_LDADD += AOloopControl_DM/libaoloopcontroldm.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_DM

SUBDIRS += AOloopControl_IOtools
CACAO_LDADD += AOloopControl_IOtools/libaoloopcontroliotools.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_IOtools

SUBDIRS += AOloopControl_perfTest
CACAO_LDADD += AOloopControl_perfTest/libaoloopcontrolperftest.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_perfTest

SUBDIRS += AOloopControl_PredictiveControl
CACAO_LDADD += AOloopControl_PredictiveControl/libaoloopcontrolpredictivecontrol.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_PreditiveControl

SUBDIRS += FPAOloopControl
CACAO_LDADD += FPAOloopControl/libfpaoloopcontrol.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/FPAOloopControl


## Core modules last, higher level modules at the top


SUBDIRS += cudacomp
CACAO_LDADD += cudacomp/libcudacomp.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/cudacomp

SUBDIRS += psf
CACAO_LDADD += psf/libpsf.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/psf

SUBDIRS += img_reduce
CACAO_LDADD += img_reduce/libimgreduce.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/img_reduce

SUBDIRS += image_format
CACAO_LDADD += image_format/libimageformat.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/image_format

SUBDIRS += ZernikePolyn
CACAO_LDADD += ZernikePolyn/libzernikepolyn.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/ZernikePolyn

SUBDIRS += image_basic
CACAO_LDADD += image_basic/libimagebasic.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/image_basic

SUBDIRS += image_filter
CACAO_LDADD += image_filter/libimagefilter.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/image_filter

SUBDIRS += image_gen
CACAO_LDADD += image_gen/libimagegen.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/image_gen

SUBDIRS += linopt_imtools
CACAO_LDADD += linopt_imtools/liblinoptimtools.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/linopt_imtools

SUBDIRS += statistic
CACAO_LDADD += statistic/libstatistic.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/statistic

SUBDIRS += info
CACAO_LDADD += info/libinfo.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/info

SUBDIRS += fft
CACAO_LDADD += fft/libfft.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/fft


## 
## CORE modules, should always be included
##



SUBDIRS += COREMOD_arith
CACAO_LDADD += COREMOD_arith/libcoremodarith.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_arith

SUBDIRS += COREMOD_memory
CACAO_LDADD += COREMOD_memory/libcoremodmemory.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_memory

SUBDIRS += COREMOD_iofits
CACAO_LDADD += COREMOD_iofits/libcoremodiofits.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_iofits

SUBDIRS += COREMOD_tools
CACAO_LDADD += COREMOD_tools/libcoremodtools.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_tools

SUBDIRS += 00CORE
CACAO_LDADD += 00CORE/lib00core.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/00CORE

SUBDIRS += ImageStreamIO
CACAO_LDADD += ImageStreamIO/libimagestreamio.la
CACAO_CPPFLAGS += -I@abs_top_srcdir@/src/ImageStreamIO
