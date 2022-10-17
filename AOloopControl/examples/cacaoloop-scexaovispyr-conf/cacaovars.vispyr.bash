#!/usr/bin/env bash

# cacao environment variables for setup
# source this file to take into effect

export CACAO_LOOPNAME="vispyr"
export CACAO_LOOPWORKDIR="vispyr"

export CACAO_DMxsize="50"
export CACAO_DMysize="50"
export CACAO_WORKDIR="vispyr"
export CACAO_DMINDEX="00"
export CACAO_LOOPNUMBER="0"
export CACAO_WFSSTREAM="ocam2d"


# FPS processes to be set up

#export CACAO_FPSPROC_STREAMDELAY="ON"
#export CACAO_FPSPROC_SIMMVMGPU="ON"

export CACAO_FPSPROC_DMCH2DISP="ON"
#export CACAO_FPSPROC_DMCOMB="ON"
##export CACAO_FPSPROC_DMCOMB_TEST="ON"

export CACAO_FPSPROC_MLAT="ON"

export CACAO_FPSPROC_ACQUWFS="ON"

export CACAO_FPSPROC_ACQLINZRM="ON"
#export CACAO_FPSPROC_ACQWFSLINCALZ="ON"

export CACAO_FPSPROC_ACQLINLORM="ON"

export CACAO_FPSPROC_COMPFCM="ON"
#export CACAO_FPSPROC_COMPSCM="ON"

export CACAO_FPSPROC_COMPCTRLMODES="ON"


# Zonal control
#export CACAO_FPSPROC_AOLOOP_RUN="ON"


# Extract control modes from WFS using MVM
#
export CACAO_FPSPROC_MVMGPU_WFS2CMODEVAL="ON"

# Compute DM command from control mode values
export CACAO_FPSPROC_MVMGPU_CMODEVAL2DM="ON"


# Modal control filtering
export CACAO_FPSPROC_MODALFILTERING="ON"

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
