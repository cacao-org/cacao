#!/bin/bash

# TEMPLATE SCRIPT FOR ADDING CACAO FPS-ENABLED PROCESSES 

# For each FPS-enabled process:
# - entry is added to fpslist.txt file
# - parameters setup added to file fpssetup.setval.conf


FPSCONFFILE="fpssetup.setval.conf"

touch fpslist.txt
touch ${FPSCONFFILE}




# log messages


LOGFILEDIR="$PWD/log"
mkdir $LOGFILEDIR
LOGFILENAME="${LOGFILEDIR}/$(basename $0).log"
rm $LOGFILENAME

function echolog {
	echo $1
	echo "$(date -u +"%Y%m%dT%H%M%S.%N %s.%N") $1" >> $LOGFILENAME
}

function runstrcmd {
	echolog "RUNNING CMD : $1"
	eval "$1"
}

function fpscmd {
	echolog "FPS CMD TO ${FPSCONFFILE} : $1"
	echo "$1" >> ${FPSCONFFILE}
}



echolog "START"


# User is expected to set FPS processes to ON or OFF before running this script
# Examples:

#export CACAO_FPSPROC_DMCOMB="ON"
#export CACAO_FPSPROC_STREAMDELAY="ON"
#export CACAO_FPSPROC_SIMMVMGPU="ON"
#export CACAO_FPSPROC_MLAT="ON"
#export CACAO_FPSPROC_ACQLINRM="ON"




if [ "${CACAO_FPSPROC_DMCOMB}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_DMCOMB"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}      aolcontrolDMcomb       ${CACAO_DMINDEX}"  >> fpslist.txt

fpscmd "setval ${fpsfname}.DMxsize ${CACAO_DMxsize}"
fpscmd "setval ${fpsfname}.DMysize ${CACAO_DMysize}"
fi
else
echolog "CACAO_FPSPROC_DMCOMB = OFF"
fi







if [ "${CACAO_FPSPROC_STREAMDELAY}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_STREAMDELAY"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}    streamdelay         ${fpsarg0}"  >> fpslist.txt

fpscmd "setval ${fpsfname}.in_name aol${CACAO_LOOPNUMBER}_dmdisp"
fpscmd "setval ${fpsfname}.out_name aol${CACAO_LOOPNUMBER}_dmdispD"
fpscmd "setval ${fpsfname}.delayus 20000"
fpscmd "setval ${fpsfname}.option.timeavemode 1"
fpscmd "setval ${fpsfname}.option.avedt 0.005"
fi
else
echolog "OFF CACAO_FPSPROC_STREAMDELAY"
fi






if [ "${CACAO_FPSPROC_SIMMVMGPU}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_SIMMVMGPU"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}      cudaextrmodes       ${fpsarg0}"  >> fpslist.txt

# time delay [us]
LINSIMDT="10000" 

# copy calibration files

runstrcmd "mkdir -p ${CACAO_WORKDIR}/simLHS"
runstrcmd "cp simLHS/${CACAO_LOOPNAME}.respM.fits ./${CACAO_WORKDIR}/simLHS/simLHS.respM.fits"
runstrcmd "cp simLHS/${CACAO_LOOPNAME}.wfsref.fits ./${CACAO_WORKDIR}/simLHS/simLHS.wfsref.fits"

runstrcmd "cp ./${CACAO_WORKDIR}/simLHS/simLHS.respM.fits ./${CACAO_WORKDIR}/conf/shmim.aolsimLHSrespM.fits"
runstrcmd "cp ./${CACAO_WORKDIR}/simLHS/simLHS.wfsref.fits ./${CACAO_WORKDIR}/conf/shmim.aolsimLHSwfsref.fits"



fpscmd "setval ${fpsfname}.sname_in aol${CACAO_LOOPNUMBER}_dmdispD"

fpscmd "setval ${fpsfname}.sname_modes aolsimLHSrespM"

fpscmd "setval ${fpsfname}.sname_outmodesval aol${CACAO_LOOPNUMBER}_linsimWFS"

fpscmd "setval ${fpsfname}.outinit ON"

# run simulator at finite frame rate
fpscmd "setval ${fpsfname}.option.twait ${LINSIMDT}"

fpscmd "setval ${fpsfname}.option.sname_refout aolsimLHSwfsref"

fpscmd "setval ${fpsfname}.option.insem 6"
fpscmd "setval ${fpsfname}.option.axmode 1"
fi
else
echolog "CACAO_FPSPROC_SIMMVMGPU = OFF"
fi






if [ "${CACAO_FPSPROC_MLAT}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_MLAT"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}           aoltestlat     ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.NBiter 20"
fpscmd "setval ${fpsfname}.sn_dm aol${CACAO_LOOPNUMBER}_dmRM"
fpscmd "setval ${fpsfname}.sn_wfs aol${CACAO_LOOPNUMBER}_wfsim"
fi
else
echolog "CACAO_FPSPROC_MLAT = OFF"
fi




if [ "${CACAO_FPSPROC_ACQUWFS}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_ACQUWFS"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}           aolacquireWFSloop     ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"

fpscmd "setval ${fpsfname}.WFSnormalize ON"
fpscmd "setval ${fpsfname}.comp.darksub ON"
fpscmd "setval ${fpsfname}.comp.imtotal ON"
fpscmd "setval ${fpsfname}.comp.normwfsim ON"
fi
else
echolog "OFF CACAO_FPSPROC_ACQUWFS"
fi







if [ "${CACAO_FPSPROC_ACQLINZRM}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_ACQLINZRM"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}           aolmeaslWFSrespC     ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"

fpscmd "setval ${fpsfname}.FPS_mlat mlat-${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}"

# default startup values
fpscmd "setval ${fpsfname}.normalize ON"
fpscmd "setval ${fpsfname}.Hpoke ON"
fpscmd "setval ${fpsfname}.autoTiming ON"
fpscmd "setval ${fpsfname}.compDMmask ON"
fpscmd "setval ${fpsfname}.compMpoke ON"

fpscmd "setval ${fpsfname}.NBave 1"
fpscmd "setval ${fpsfname}.NBcycle 2"
fpscmd "setval ${fpsfname}.NBinnerCycle 1"
fpscmd "setval ${fpsfname}.NBexcl 0"

fpscmd "setval ${fpsfname}.MaskMode ON"
fpscmd "setval ${fpsfname}.exec.RMdecode cacaobin/cacao-RMdecode"
fpscmd "setval ${fpsfname}.exec.mkDMWFSmasks cacaobin/cacao-mkDMWFSmasks"
fpscmd "setval ${fpsfname}.exec.mkDMslaveact cacaobin/cacao-mkDMslaveActprox"
fpscmd "setval ${fpsfname}.exec.mkLODMmodes cacaobin/cacao-mkLODMmodes"
fpscmd "setval ${fpsfname}.out.dir ${outdir}"

fi
else
echolog "OFF CACAO_FPSPROC_ACQLINZRM"
fi





if [ "${CACAO_FPSPROC_ACQLINLORM}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_ACQLINLORM"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}           aolmeaslWFSrespC     ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"

fpscmd "setval ${fpsfname}.FPS_mlat mlat-${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}"


# default startup values
fpscmd "setval ${fpsfname}.normalize ON"
fpscmd "setval ${fpsfname}.Hpoke OFF"
fpscmd "setval ${fpsfname}.autoTiming ON"
fpscmd "setval ${fpsfname}.compDMmask OFF"
fpscmd "setval ${fpsfname}.compMpoke OFF"


#echo "setval ${fpsfname}.NBave 1" >> ${FPSCONFFILE}
fpscmd "setval ${fpsfname}.NBave 1"
fpscmd "setval ${fpsfname}.NBcycle 4"
fpscmd "setval ${fpsfname}.NBinnerCycle 1"
fpscmd "setval ${fpsfname}.NBexcl 0"


fpscmd "setval ${fpsfname}.fn_pokeC conf/respM_LOmodes.fits"

fpscmd "setval ${fpsfname}.MaskMode OFF"

fpscmd "setval ${fpsfname}.exec.RMdecode cacaobin/cacao-NULL"
fpscmd  "setval ${fpsfname}.exec.mkDMWFSmasks cacaobin/cacao-NULL"
fpscmd "setval ${fpsfname}.exec.mkDMslaveact cacaobin/cacao-NULL"
fpscmd "setval ${fpsfname}.exec.mkLODMmodes cacaobin/cacao-NULL"


#echo "setval ${fpsfname}.out.dir ${outdir}" >> ${FPSCONFFILE}
fpscmd "setval ${fpsfname}.out.dir ${outdir}"

fi
else
echolog "OFF CACAO_FPSPROC_ACQLINLORM"
fi





if [ "${CACAO_FPSPROC_COMPFCM}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_COMPFCM"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}         aolcomputeCM       ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"

fpscmd "setval ${fpsfname}.FPS_zRMacqu acqlin_zRM-${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.FPS_loRMacqu acqlin_loRM-${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.FPS_DMcomb DMcomb-${CACAO_DMINDEX}"

fi
else
echolog "OFF CACAO_FPSPROC_COMPFCM"
fi





if [ "${CACAO_FPSPROC_COMPSCM}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_COMPSCM"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}         aolRM2CM       ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.fname_respM conf-zRM-staged/zrespMn.fits"

fi
else
echolog "OFF CACAO_FPSPROC_COMPSCM"
fi










if [ "${CACAO_FPSPROC_MODESEXTRACTWFSGPU}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_MODESEXTRACTWFSGPU"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}         cudaextrmodes       ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"

# input frame
fpscmd "setval ${fpsfname}.sname_in aol${CACAO_LOOPNUMBER}_imWFS0"
fpscmd "setval ${fpsfname}.option.sname_intot aol${CACAO_LOOPNUMBER}_imWFS0tot"

# we use respM for direct MVM as it is already orthonormal set of modes
fpscmd "setval ${fpsfname}.sname_modes aol${CACAO_LOOPNUMBER}_modesWFS"
# TBD: load in shm

fpscmd "setval ${fpsfname}.option.sname_refin aol${CACAO_LOOPNUMBER}_wfsref"

fpscmd "setval ${fpsfname}.sname_outmodesval aol${CACAO_LOOPNUMBER}_modeval"
fpscmd "setval ${fpsfname}.outinit ON"

fpscmd "setval ${fpsfname}.option.GPindex 0"
fpscmd "setval ${fpsfname}.option.PROCESS ON"
fpscmd "setval ${fpsfname}.option.TRACEMODE ON"
fpscmd "setval ${fpsfname}.option.MODENORM ON"


fpscmd "setval ${fpsfname}.option.insem 2"

fpscmd "setval ${fpsfname}.option.axmode 0"

# run GPU full speed
fpscmd "setval ${fpsfname}.option.twait 0"

fpscmd "setval ${fpsfname}.option.semwarn ON"

fi
else
echolog "OFF CACAO_FPSPROC_MODESEXTRACTWFSGPU"
fi






if [ "${CACAO_FPSPROC_AOLOOP_RUN}" = "ON" ]; then
echolog "ON  CACAO_FPSPROC_AOLOOP_RUN"
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
echolog "Process ${fpsname} already registered - skipping"
else
echolog "Adding process ${fpsname}"
echo "${fpsname}         aolrun       ${fpsarg0}" >> fpslist.txt

fpscmd "setval ${fpsfname}.loop ${CACAO_LOOPNUMBER}"
fpscmd "setval ${fpsfname}.sn_wfs aol${CACAO_LOOPNUMBER}_imWFS1"

fpscmd "setval ${fpsfname}.loopgain 0.02"
fpscmd "setval ${fpsfname}.loopmult 0.99"
fpscmd "setval ${fpsfname}.loopON OFF"

fpscmd "setval ${fpsfname}.sn_cmat aol${CACAO_LOOPNUMBER}_CMat"
fpscmd "setval ${fpsfname}.sn_DMout aol${CACAO_LOOPNUMBER}_dmC"

fi
else
echolog "OFF CACAO_FPSPROC_AOLOOP_RUN"
fi


echolog "END"





