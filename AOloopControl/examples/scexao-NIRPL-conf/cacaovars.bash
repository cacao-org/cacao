# cacao environment variables for setup
# This file will be sourced by cacao-setup

export CACAO_LOOPNAME="NIRPL"
export CACAO_LOOPNUMBER="6"



# ====== DEFORMABLE MIRROR ==========

# Deformable mirror (DM) size
# If DM is single dimension, enter "1" for DMsize
#
export CACAO_DMINDEX="00"     # Hardware DM - connected to physical DM
export CACAO_DMSIMINDEX="10"  # Simulation DM
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
export CACAO_WFSSTREAM="apapane_windowed"     # Hardware stream, connected to physical camera. currently configured to accept a custom shm that is (presumably) a windowed version of apapane
export CACAO_WFSSTREAMSIM="apapanesim"

# Specify that WFS stream is not raw image, but processed WFS signal
# If set to ON, this turns off intensity scaling
export CACAO_WFSSTREAM_PROCESSED="OFF"

export CACAO_LOOPDATALOGDIR="$(pwd)/datalogdir"


# ========================================
#       FPS processes to be set up
# ========================================

# DM combination
# Manages mutipe DM channels
#
export CACAO_FPSPROC_DMCH2DISP="ON"
export CACAO_FPSPROC_DMCH2DISPSIM="ON"


# Delay stream: emulates time lag in hardware
# Used to simulate a time lag
#
export CACAO_FPSPROC_DMSIMDELAY="ON"

# MVM lop on GPU: used to simulate hardware
#
export CACAO_FPSPROC_SIMMVMGPU="ON"

# Camera simulator
#
export CACAO_FPSPROC_WFSCAMSIM="ON"




# Measure hardware latency
#
export CACAO_FPSPROC_MLAT="ON"


# Remap WFS input
#
export CACAO_FPSPROC_MAPWFS="ON"

# Acquire WFS stream
#
export CACAO_FPSPROC_ACQUWFS="ON"

# Acquire linear RM 
#
export CACAO_FPSPROC_MEASURELINRESP="ON"


# Compute control matrix - straight
#
export CACAO_FPSPROC_COMPSTRCM="ON"


# Extract control modes
#
export CACAO_FPSPROC_MVMGPU_WFS2CMODEVAL="ON"

# Modal control filtering
#
export CACAO_FPSPROC_MODALFILTERING="ON"

# Compute DM command from control mode values
#
export CACAO_FPSPROC_MVMGPU_CMODEVAL2DM="ON"
