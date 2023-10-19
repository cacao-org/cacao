#!/usr/bin/env bash

# add column_sum function to fpsCTRL

source cacao-check-cacaovars
source ./scripts/fps-utility.sh

# name of function to setup
fname = "column_sum"

# add to fpsCTRL, start tmux, start conf, turn on procinfo
setupfps "${fname}"

# procinfo specific stuff
sendFPScmd "setval ${fname}-${CACAO-LOOPNUMBER}.procinfo.triggermode 3"
sendFPScmd "setval ${fname}-${CACAO-LOOPNUMBER}.procinfo.loopcntMax -1"

if ${CACAO_MODE} == "HARDW"
then
  sendFPScmd "setval ${fname}-${CACAO-LOOPNUMBER}.procinfo.triggersname ${CACAO_WFSSTREAM}"
elif ${CACAO_MODE} == "SIM"
then
  sendFPScmd "setcal ${fname}-${CACAO-LOOPNUMBER}.procinfo.triggersname ${CACAO_WFSSTREAMSIM}"
fi

