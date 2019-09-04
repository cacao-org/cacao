#!/bin/bash

# TEMPLATE SCRIPT FOR ADDING CACAO FPS-ENABLED PROCESSES 

# For each FPS-enabled process:
# - entry is added to fpslist.txt file
# - parameters setup added to file fpssetup.setval.conf


FPSCONFFILE="fpssetup.setval.conf"

touch fpslist.txt
touch ${FPSCONFFILE}




# User is expected to set FPS processes to ON or OFF before running this script
# Examples:

#export CACAO_FPSPROC_DMCOMB="ON"
#export CACAO_FPSPROC_STREAMDELAY="ON"
#export CACAO_FPSPROC_SIMMVMGPU="ON"
#export CACAO_FPSPROC_MLAT="ON"
#export CACAO_FPSPROC_ACQLINRM="ON"




if [ "${CACAO_FPSPROC_DMCOMB}" = "ON" ]; then
# ==============================================================================
# ==========  DM combination ===================================================
# ==============================================================================

# FPS name
fpsname="DMcomb" 
fpsarg0="${CACAO_DMINDEX}"

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}      aolcontrolDMcomb       ${CACAO_DMINDEX}"  >> fpslist.txt

echo "setval ${fpsfname}.DMxsize ${CACAO_DMxsize}" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.DMysize ${CACAO_DMxsize}" >> ${FPSCONFFILE}
fi

fi







if [ "${CACAO_FPSPROC_STREAMDELAY}" = "ON" ]; then
# ==============================================================================
# ==========  streamdelay for simulation mode ==================================
# ==============================================================================

# FPS name
fpsname="streamDelay" 
fpsarg0="${CACAO_DMINDEX}"

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 


if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}    streamdelay         ${fpsarg0}"  >> fpslist.txt

echo "setval ${fpsfname}.in_name aol${CACAO_LOOPNUMBER}_dmdisp" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.out_name aol${CACAO_LOOPNUMBER}_dmdispD" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.delayus 20000" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.timeavemode 1" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.avedt 0.005" >> ${FPSCONFFILE}
fi

fi






if [ "${CACAO_FPSPROC_SIMMVMGPU}" = "ON" ]; then
# ==============================================================================
# ========== GPU-based Linear Simulator ========================================
# ==============================================================================

# FPS name
fpsname="simmvmgpu" 
fpsarg0="${CACAO_LOOPNUMBER}"

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 


if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}      cudaextrmodes       ${fpsarg0}"  >> fpslist.txt

# time delay [us]
LINSIMDT="10000" 

# copy calibration files
mkdir ${CACAO_WORKDIR}/simLHS
cp simLHS/respM.fits ./${CACAO_WORKDIR}/simLHS/simLHS_respM.fits
cp simLHS/wfsref.fits ./${CACAO_WORKDIR}/simLHS/simLHS_wfsref.fits

cp ./${CACAO_WORKDIR}/simLHS/simLHS_respM.fits ./${CACAO_WORKDIR}/conf/shmim.aolsimLHSrespM.fits
cp ./${CACAO_WORKDIR}/simLHS/simLHS_wfsref.fits ./${CACAO_WORKDIR}/conf/shmim.aolsimLHSwfsref.fits

echo "setval ${fpsfname}.sname_in aol${CACAO_LOOPNUMBER}_dmdispD" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.sname_modes aolsimLHSrespM" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.sname_outmodesval aol${CACAO_LOOPNUMBER}_linsimWFS" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.outinit ON" >> ${FPSCONFFILE}

# run simulator at finite frame rate
echo "setval ${fpsfname}.option.twait ${LINSIMDT}" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.sname_refout aolsimLHSwfsref" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.insem 6" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.axmode 1" >> ${FPSCONFFILE}
fi

fi






if [ "${CACAO_FPSPROC_MLAT}" = "ON" ]; then
# ==============================================================================
# ========== Measure Latency ===================================================
# ==============================================================================

# FPS name
fpsname="mlat" 
fpsarg0="${CACAO_LOOPNUMBER}"

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}           aoltestlat     ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.NBiter 20" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.sn_dm aol${CACAO_LOOPNUMBER}_dmRM" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.sn_wfs aol${CACAO_LOOPNUMBER}_wfsim" >> ${FPSCONFFILE}
fi

fi




if [ "${CACAO_FPSPROC_ACQUWFS}" = "ON" ]; then
# ==============================================================================
# ========== WFS acquire frames ================================================
# ==============================================================================

# FPS name
fpsname="acquWFS" 
fpsarg0="${CACAO_LOOPNUMBER}"

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}           aolacquireWFSloop     ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.WFSnormalize ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.comp.darksub ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.comp.imtotal ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.comp.normwfsim ON" >> ${FPSCONFFILE}

fi

fi







if [ "${CACAO_FPSPROC_ACQLINZRM}" = "ON" ]; then
# ==============================================================================
# ========== Acquire Linear Response Matrix ====================================
# ==============================================================================

# FPS name
fpsname="acqlin_zRM" 
fpsarg0="${CACAO_LOOPNUMBER}"
outdir="conf-zRM-staged"
mkdir -p ${CACAO_WORKDIR}/${outdir}

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}           aolmeaslWFSrespC     ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.FPS_mlat mlat-${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}" >> ${FPSCONFFILE}

# default startup values
echo "setval ${fpsfname}.normalize ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.Hpoke ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.autoTiming ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.compDMmask ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.compMpoke ON" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.NBave 1" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBcycle 2" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBinnerCycle 1" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBexcl 0" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.MaskMode ON" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.exec.RMdecode cacaobin/cacao-RMdecode" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkDMWFSmasks cacaobin/cacao-mkDMWFSmasks" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkDMslaveact cacaobin/cacao-mkDMslaveActprox" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkLODMmodes cacaobin/cacao-mkLODMmodes" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.out.dir ${outdir}" >> ${FPSCONFFILE}

fi
fi





if [ "${CACAO_FPSPROC_ACQLINLORM}" = "ON" ]; then
# ==============================================================================
# ========== Acquire Linear Response Matrix ====================================
# ==============================================================================

# FPS name
fpsname="acqlin_loRM" 
fpsarg0="${CACAO_LOOPNUMBER}"
outdir="conf-loRM-staged"
mkdir -p ${CACAO_WORKDIR}/${outdir}

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}           aolmeaslWFSrespC     ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.FPS_mlat mlat-${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}" >> ${FPSCONFFILE}

# default startup values
echo "setval ${fpsfname}.normalize ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.Hpoke OFF" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.autoTiming ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.compDMmask OFF" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.compMpoke OFF" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.NBave 1" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBcycle 4" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBinnerCycle 1" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.NBexcl 0" >> ${FPSCONFFILE}


echo "setval ${fpsfname}.fn_pokeC conf/respM_LOmodes.fits" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.MaskMode OFF" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.exec.RMdecode cacaobin/cacao-NULL" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkDMWFSmasks cacaobin/cacao-NULL" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkDMslaveact cacaobin/cacao-NULL" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.exec.mkLODMmodes cacaobin/cacao-NULL" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.out.dir ${outdir}" >> ${FPSCONFFILE}

fi
fi





if [ "${CACAO_FPSPROC_COMPCM}" = "ON" ]; then
# ==============================================================================
# ========== Compute Control Matrix ============================================
# ==============================================================================

# FPS name
fpsname="compCM" 
fpsarg0="${CACAO_LOOPNUMBER}"
outdir="conf-CM-staged"
mkdir -p ${CACAO_WORKDIR}/${outdir}

mkdir -p ${CACAO_WORKDIR}/conf_staged
mkdir -p ${CACAO_WORKDIR}/mkmodestmp

# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}         aolcomputeCM       ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.FPS_zRMacqu acqlin_zRM-${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.FPS_loRMacqu acqlin_loRM-${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}" >> ${FPSCONFFILE}

fi
fi






