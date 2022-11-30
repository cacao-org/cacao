# cacao environment variables for setup
# This file will be sourced by cacao-setup


export CACAO_LOOPNAME="ttmloop"
export CACAO_LOOPNUMBER="2"



# ====== DEFORMABLE MIRROR ==========

# Deformable mirror (DM) size
# If DM is single dimension, enter "1" for DMsize
#
export CACAO_DMINDEX="02"
export CACAO_DMxsize="2"
export CACAO_DMysize="1"

# 1 if DM actuators are on a coordinate grid
# This informs processes if a spatial relationship exists
# between DM actuators
export CACAO_DMSPATIAL="0"



# ====== DIRECTORIES ================

# Optional. If not defined, will take value "LOOPNAME-rootdir"
export CACAO_LOOPROOTDIR="ttmloop-rootdir"

# Optional. If not defined, will take value "LOOPNAME-rundir"
export CACAO_LOOPRUNDIR="ttmloop-rundir"





# input WFS stream
export CACAO_WFSSTREAM="dm01disp"

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
# should not be needed
export CACAO_FPSPROC_STREAMDELAY="ON"

# MVM lop on GPU: used to simulate hardware
# should not be needed
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

# Acquire low-order modal RM
#
export CACAO_FPSPROC_ACQLINLORM="ON"


# Compute control matrix - Fourier
#
export CACAO_FPSPROC_COMPFCM="ON"

# Compute control matrix - straight
#
export CACAO_FPSPROC_COMPSCM="ON"


# Extract control modes
#
export CACAO_FPSPROC_MODESEXTRACTWFSGPU="ON"

# Control loop
#
export CACAO_FPSPROC_AOLOOP_RUN="ON"

# Extract control modes from WFS using MVM
export CACAO_FPSPROC_MVMGPU_WFS2CMODEVAL="ON"

# Modal control filtering
export CACAO_FPSPROC_MODALFILTERING="ON"

# Compute DM command from control mode values
export CACAO_FPSPROC_MVMGPU_CMODEVAL2DM="ON"
