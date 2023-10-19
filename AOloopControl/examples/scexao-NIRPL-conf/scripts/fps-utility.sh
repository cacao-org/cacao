#!/usr/bin/env bash

# utility functions for configuring custom functions in fpsCTRL
# run in rootdir

source cacao-check-cacaovars

function sendFPScmd {
  echo "SENDING: $1"
  echo "$1" >> ${MILK_SHM_DIR}/${CACAO_LOOPNAME}_fpsCTRL.fifo
}

function setupfps {
	echo "CONFIGURING: $1"
	echo "ADDING TO FPSCTRL AND STARTING TMUX"
	# first, init fps and start tmux
	cacao <<- EOF
	cacaoio.$1 _FPSINIT_ "${CACAO_LOOPNUMBER}"
	cacaoio.$1 _TMUXSTART_ "${CACAO_LOOPNUMBER}"
	exitCLI
	EOF
	echo "STARTING CONF PROCESS"
	sendFPScmd "confstart $1-${CACAO_LOOPNUMBER}"
	echo "STARTING PROCINFO"
	cacao <<- EOF
	cacaoio.$1 ..procinfo 1
	exitCLI
	EOF
}

