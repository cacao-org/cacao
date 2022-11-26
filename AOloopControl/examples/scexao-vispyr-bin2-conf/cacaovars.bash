#!/usr/bin/env bash
# This file will be sourced by cacao-setup


export CACAO_LOOPNAME="vispyr2"
export CACAO_LOOPNUMBER="1"

# ====== DEFORMABLE MIRROR ==========

# Deformable mirror (DM) size
# If DM is single dimension, enter "1" for DMsize
#
export CACAO_DMINDEX="00"
export CACAO_DMxsize="50"
export CACAO_DMysize="50"

# 1 if DM actuators are on a coordinate grid
# This informs processes if a spatial relationship exists
# between DM actuators
export CACAO_DMSPATIAL="1"



# ====== DIRECTORIES ================

# Root directory
export CACAO_LOOPROOTDIR="${CACAO_LOOPNAME}-rootdir"

# Run directory. This is a subdirectory of rootdir
# processes run in CACAO_LOOPROOTDIR/CACAO_LOOPRUNDIR
export CACAO_LOOPRUNDIR="${CACAO_LOOPNAME}-rundir"


# input WFS stream
export CACAO_WFSSTREAM="ocam2d"

export CACAO_LOOPDATALOGDIR="$(pwd)/datalogdir"



# ========================================
#       FPS processes to be set up
# ========================================

# DM combination
# Manages mutipe DM channels
#
export CACAO_FPSPROC_DMCH2DISP="ON"




# Delay stream: emulates time lag in hardware
# Used to simulate a time lag
#
export CACAO_FPSPROC_STREAMDELAY="ON"

# MVM lop on GPU: used to simulate hardware
#
export CACAO_FPSPROC_SIMMVMGPU="ON"

# Measure hardware latency
#
export CACAO_FPSPROC_MLAT="ON"

# Acquire WFS stream
#
export CACAO_FPSPROC_ACQUWFS="ON"

# Acquire linear RM (zonal)
#
export CACAO_FPSPROC_ACQLINZRM="ON"
#export CACAO_FPSPROC_ACQWFSLINCALZ="ON"

# Acquire low-order modal RM
#
export CACAO_FPSPROC_ACQLINLORM="ON"


# Compute control matrix - Fourier
#
export CACAO_FPSPROC_COMPFCM="ON"

# Compute control matrix - straight
#
#export CACAO_FPSPROC_COMPSCM="ON"


export CACAO_FPSPROC_COMPCTRLMODES="ON"

# Extract control modes
#
export CACAO_FPSPROC_MODESEXTRACTWFSGPU="ON"

# Control loop
#
export CACAO_FPSPROC_AOLOOP_RUN="ON"

# Extract control modes from WFS using MVM
#
export CACAO_FPSPROC_MVMGPU_WFS2CMODEVAL="ON"

# Modal control filtering
export CACAO_FPSPROC_MODALFILTERING="ON"

# Compute DM command from control mode values
export CACAO_FPSPROC_MVMGPU_CMODEVAL2DM="ON"



# Zonal control
#export CACAO_FPSPROC_AOLOOP_RUN="ON"




# Zero Point Offset from DM to WFS
export CACAO_FPSPROC_MVMGPU_ZPO="ON"

# Modal control statistics
export CACAO_FPSPROC_MODALCTRL_STATS="ON"

# Reconstruct DM shape from OL mode values
export CACAO_FPSPROC_MVMGPU_OLMODEVAL2DM="ON"

# Reconstruct DM shape from OL mode values
export CACAO_FPSPROC_MVMGPU_WFSMODEVAL2DM="ON"


# Modal control DM comb
export CACAO_FPSPROC_CMDMCOMB="ON"

# Modal response matrix using control modes
#export CACAO_FPSPROC_ACQLINCMRM="ON"

# Predictive control - build filters
export CACAO_FPSPROC_MKPF00="ON"
export CACAO_FPSPROC_MKPF01="ON"
export CACAO_FPSPROC_MKPF02="ON"
export CACAO_FPSPROC_MKPF03="ON"
export CACAO_FPSPROC_MKPF04="ON"
export CACAO_FPSPROC_MKPF05="ON"


export CACAO_FPSPROC_APPLYPF00="ON"
export CACAO_FPSPROC_APPLYPF01="ON"
export CACAO_FPSPROC_APPLYPF02="ON"
export CACAO_FPSPROC_APPLYPF03="ON"
export CACAO_FPSPROC_APPLYPF04="ON"
export CACAO_FPSPROC_APPLYPF05="ON"
