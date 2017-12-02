#SUFFIXES = .c .h .y .l

bin_PROGRAMS = cacao
cacao_SOURCES = CLImain.c CommandLineInterface/CLIcore.c CommandLineInterface/CLIcore.h CommandLineInterface/calc_bison.y CommandLineInterface/calc_flex.l CommandLineInterface/calc.h
cacao_CPPFLAGS = -I@abs_top_srcdir@/src -DCONFIGDIR=\"@abs_top_srcdir@/config\" -DSOURCEDIR=\"@abs_top_srcdir@/src\" -DABSSRCTOPDIR=\"@abs_top_srcdir@\" -fopenmp -ldl

ACLOCAL_AMFLAGS = -I m4

## For Bison and FLEX
AM_YFLAGS = -d
AM_LFLAGS = -i 

## The contents of this variable are passed to every compilation that invokes the C preprocessor IF target_CPPFLAGS does not exit; it is a list of arguments to the preprocessor. For instance, -I and -D options should be listed here.
## AM_CPPFLAGS = -DCONFIGDIR=\"@abs_top_srcdir@/config\"

noinst_HEADERS = CommandLineInterface/calc.h

BUILT_SOURCES = CommandLineInterface/calc_bison.h


cacao_LDADD = 
SUBDIRS =


## Modules are built as libraries of convenience
## Linking order matters

## 
## AO modules
##

SUBDIRS += AOloopControl
cacao_LDADD += AOloopControl/libaoloopcontrol.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl

SUBDIRS += AOloopControl_acquireCalib
cacao_LDADD += AOloopControl_acquireCalib/libaoloopcontrolacquirecalib.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_acquireCalib

SUBDIRS += AOloopControl_compTools
cacao_LDADD += AOloopControl_compTools/libaoloopcontrolcomptools.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_compTools

SUBDIRS += AOloopControl_computeCalib
cacao_LDADD += AOloopControl_computeCalib/libaoloopcontrolcomputecalib.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_computeCalib

SUBDIRS += AOloopControl_DM
cacao_LDADD += AOloopControl_DM/libaoloopcontroldm.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_DM

SUBDIRS += AOloopControl_IOtools
cacao_LDADD += AOloopControl_IOtools/libaoloopcontroliotools.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_IOtools

SUBDIRS += AOloopControl_perfTest
cacao_LDADD += AOloopControl_perfTest/libaoloopcontrolperftest.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_perfTest

SUBDIRS += AOloopControl_PredictiveControl
cacao_LDADD += AOloopControl_PredictiveControl/libaoloopcontrolpredictivecontrol.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/AOloopControl_PreditiveControl

SUBDIRS += FPAOloopControl
cacao_LDADD += FPAOloopControl/libfpaoloopcontrol.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/FPAOloopControl


## Core modules last, higher level modules at the top


SUBDIRS += cudacomp
cacao_LDADD += cudacomp/libcudacomp.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/cudacomp

SUBDIRS += psf
cacao_LDADD += psf/libpsf.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/psf

SUBDIRS += img_reduce
cacao_LDADD += img_reduce/libimgreduce.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/img_reduce

SUBDIRS += image_format
cacao_LDADD += image_format/libimageformat.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/image_format

SUBDIRS += ZernikePolyn
cacao_LDADD += ZernikePolyn/libzernikepolyn.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/ZernikePolyn

SUBDIRS += image_basic
cacao_LDADD += image_basic/libimagebasic.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/image_basic

SUBDIRS += image_filter
cacao_LDADD += image_filter/libimagefilter.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/image_filter

SUBDIRS += image_gen
cacao_LDADD += image_gen/libimagegen.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/image_gen

SUBDIRS += linopt_imtools
cacao_LDADD += linopt_imtools/liblinoptimtools.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/linopt_imtools

SUBDIRS += statistic
cacao_LDADD += statistic/libstatistic.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/statistic

SUBDIRS += info
cacao_LDADD += info/libinfo.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/info

SUBDIRS += fft
cacao_LDADD += fft/libfft.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/fft

SUBDIRS += kdtree
cacao_LDADD += kdtree/libkdtree.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/kdtree


## 
## CORE modules, should always be included
##



SUBDIRS += COREMOD_arith
cacao_LDADD += COREMOD_arith/libcoremodarith.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_arith

SUBDIRS += COREMOD_memory
cacao_LDADD += COREMOD_memory/libcoremodmemory.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_memory

SUBDIRS += COREMOD_iofits
cacao_LDADD += COREMOD_iofits/libcoremodiofits.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_iofits

SUBDIRS += COREMOD_tools
cacao_LDADD += COREMOD_tools/libcoremodtools.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/COREMOD_tools

SUBDIRS += 00CORE
cacao_LDADD += 00CORE/lib00core.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/00CORE

SUBDIRS += ImageStreamIO
cacao_LDADD += ImageStreamIO/libimagestreamio.la
cacao_CPPFLAGS += -I@abs_top_srcdir@/src/ImageStreamIO
