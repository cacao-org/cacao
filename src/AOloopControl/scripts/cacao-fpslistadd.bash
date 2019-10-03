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
echo "setval ${fpsfname}.DMysize ${CACAO_DMysize}" >> ${FPSCONFFILE}
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
cp simLHS/${CACAO_LOOPNAME}_respM.fits ./${CACAO_WORKDIR}/simLHS/simLHS_respM.fits
cp simLHS/${CACAO_LOOPNAME}_wfsref.fits ./${CACAO_WORKDIR}/simLHS/simLHS_wfsref.fits

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
# ========== Acquire Linear Response Matrix (ZONAL) ============================
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
# ========== Acquire Linear Response Matrix (Low Orders - MODAL) ===============
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





if [ "${CACAO_FPSPROC_COMPFCM}" = "ON" ]; then
# ==============================================================================
# ========== Compute Control Matrix using Fourier modes (spatial frequency) ====
# ==============================================================================

# FPS name
fpsname="compfCM" 
fpsarg0="${CACAO_LOOPNUMBER}"
outdir="conf-fCM-staged"
mkdir -p ${CACAO_WORKDIR}/${outdir}

mkdir -p ${CACAO_WORKDIR}/conf_staged
mkdir -p ${CACAO_WORKDIR}/mkmodestmp

mkdir -p ${CACAO_WORKDIR}/DMmodes
mkdir -p ${CACAO_WORKDIR}/respM
mkdir -p ${CACAO_WORKDIR}/contrM
mkdir -p ${CACAO_WORKDIR}/contrMc
mkdir -p ${CACAO_WORKDIR}/contrMcact




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





if [ "${CACAO_FPSPROC_COMPSCM}" = "ON" ]; then
# ==============================================================================
# ========== Compute Control Matrix (straight, no modal decomposition) =========
# ==============================================================================

# FPS name
fpsname="compsCM" 
fpsarg0="${CACAO_LOOPNUMBER}"
outdir="conf-sCM-staged"
mkdir -p ${CACAO_WORKDIR}/${outdir}

mkdir -p ${CACAO_WORKDIR}/sCM



# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}         aolRM2CM       ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.fname_respM conf-zRM-staged/zrespMn.fits" >> ${FPSCONFFILE}

fi
fi










if [ "${CACAO_FPSPROC_MODESEXTRACTWFSGPU}" = "ON" ]; then
# ==============================================================================
# ========== GPU-based MVM extract modes ============================================
# ==============================================================================

# FPS name
fpsname="modesextractWFSGPU" 
fpsarg0="${CACAO_LOOPNUMBER}"



# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}         cudaextrmodes       ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}" >> ${FPSCONFFILE}

# input frame
echo "setval ${fpsfname}.sname_in aol${CACAO_LOOPNUMBER}_imWFS0" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.sname_intot aol${CACAO_LOOPNUMBER}_imWFS0tot" >> ${FPSCONFFILE}

# we use respM for direct MVM as it is already orthonormal set of modes
echo "setval ${fpsfname}.sname_modes aol${CACAO_LOOPNUMBER}_modesWFS" >> ${FPSCONFFILE}
# TBD: load in shm

echo "setval ${fpsfname}.option.sname_refin aol${CACAO_LOOPNUMBER}_wfsref" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.sname_outmodesval aol${CACAO_LOOPNUMBER}_modeval" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.outinit ON" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.GPindex 0" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.PROCESS ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.TRACEMODE ON" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.option.MODENORM ON" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.insem 2" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.axmode 0" >> ${FPSCONFFILE}

# run GPU full speed
echo "setval ${fpsfname}.option.twait 0" >> ${FPSCONFFILE}

echo "setval ${fpsfname}.option.semwarn ON" >> ${FPSCONFFILE}


fi
fi






if [ "${CACAO_FPSPROC_AOLOOP_RUN}" = "ON" ]; then
# ==============================================================================
# ========== Run control loop =========
# ==============================================================================

# FPS name
fpsname="loopRUN" 
fpsarg0="${CACAO_LOOPNUMBER}"


# FPS full name
fpsfname="${fpsname}-${fpsarg0}" 

if grep -q "${fpsname}" fpslist.txt
then
echo "Process ${fpsname} already registered - skipping"
else
echo "Adding process ${fpsname}"
echo "${fpsname}         aolrun       ${fpsarg0}" >> fpslist.txt

echo "setval ${fpsfname}.loopgain 0.02" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.loopmult 0.99" >> ${FPSCONFFILE}
echo "setval ${fpsfname}.loopON OFF" >> ${FPSCONFFILE}

fi
fi







