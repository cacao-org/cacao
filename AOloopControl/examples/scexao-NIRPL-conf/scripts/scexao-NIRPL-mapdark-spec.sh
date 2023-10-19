#!/usr/bin/env bash

# This script uses milk-argparse
# See template milk-scriptexample in module milk_module_example for template and instructions

# script 1-line description
MSdescr="remap WFS dark frame - spectral version"

MSextdescr="Remaps dark according to mapping file"

source milk-script-std-config
source cacao-check-cacaovars

source milk-argparse




# average #NBAVE frames, write result to conf/wfsimave.fits

rm -f ${MILK_SHM_DIR}/aol${CACAO_LOOPNUMBER}_wfsdark.im.shm
cacao << EOF
loadfits "conf/wfsdarkraw.fits" darkraw
readshmim wfsspecmask
cacaoio.extract_spectra darkraw wfsspecmask darkm
saveFITS darkm "conf/wfsdark.fits"
imcpshm darkm aol${CACAO_LOOPNUMBER}_wfsdark
exitCLI
EOF

