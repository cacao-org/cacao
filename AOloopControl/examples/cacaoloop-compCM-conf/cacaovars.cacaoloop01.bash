# cacao environment variables for setup
# This file will be sourced by cacao-setup

# 1 if DM actuators are on a coordinate grid
# This informs processes if a spatial relationship exists
# between DM actuators
export CACAO_DMSPATIAL="1"

# Deformable mirror (DM) size
# If DM is single dimension, enter "1" for DMsize
#
export CACAO_DMxsize="12"
export CACAO_DMysize="12"

export CACAO_LOOPWORKDIR="cacaoloop01"
#export CACAO_DMINDEX="01"
export CACAO_LOOPNUMBER="2"

# input WFS stream
#export CACAO_WFSSTREAM="aol2_linsimWFS"

#export CACAO_LOOPDATALOGDIR="$(pwd)/datalogdir"

# ========================================
#       FPS processes to be set up
#       Comment processes not used
# ========================================




# Compute control matrix - Fourier
#
export CACAO_FPSPROC_COMPFCM="ON"
export CACAO_FPSPROC_COMPCTRLMODES="ON"


# Compute control matrix - straight
#
export CACAO_FPSPROC_COMPSCM="ON"
