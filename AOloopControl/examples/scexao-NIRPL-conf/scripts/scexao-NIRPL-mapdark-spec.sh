#!/usr/bin/env bash

# This script uses milk-argparse
# See template milk-scriptexample in module milk_module_example for template and instructions

# script 1-line description
MSdescr="remap WFS dark frame - spectral version"

MSextdescr="Remaps dark according to mapping file"

source milk-script-std-config
source cacao-check-cacaovars

source milk-argparse
source ./scripts/fps-utility

sendFPScmd "waitonrunON"
sendFPScmd "waitonconfON"

sendFPScmd "setval acquire_spectra-${CACAO_LOOPNUMBER}.procinfo.loopcntMax 1"
sendFPScmd "setval acquire_spectra-${CACAO_LOOPNUMBER}.wfsin aol${CACAO_LOOPNUMBER}_wfsdarkraw"
sendFPScmd "confwupdate acquire_spectra-${CACAO_LOOPNUMBER}"
sendFPScmd "runstart acquire_spectra-${CACAO_LOOPNUMBER}"
sendFPScmd "runstop acquire_spectra-${CACAO_LOOPNUMBER}"

cacao << EOF
readshmim aol${CACAO_LOOPNUMBER}_imWFS2
readshmim aol${CACAO_LOOPNUMBER}_wfsdark
imcpshm aol${CACAO_LOOPNUMBER}_imWFS2 aol${CACAO_LOOPNUMBER}_wfsdark
exitCLI
EOF

sendFPScmd "setval acquire_spectra-${CACAO_LOOPNUMBER}.procinfo.loopcntMax -1"
sendFPScmd "setval acquire_spectra-${CACAO_LOOPNUMBER}.wfsin ${CACAO_WFSSTREAMSIM}"