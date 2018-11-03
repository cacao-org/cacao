#!/bin/bash

# ==============================================================================
# START cacao ASCII GUI
#
# GUI starts in tmux session (fixed size) to be opened in a xterm terminal
#
# WINDOW 0
#
#  pane 0    aolconf control
#
#  pane 2    instrument alignment
#  pane 3    Hardware control
#  pane 4    
#
#  pane 5    log input
#  pane 6    log output
#  pane 7
#  pane 8    loop monitor (aolmon)
#  pane 7    Data logging RT streams
#  pane 10   process control
#
# ==============================================================================


NBARGS=0

wXsize=500
wYsize=150

if [ -f LOOPNUMBER ]; then
LOOPNUMBER=$( head -1 LOOPNUMBER )
else
LOOPNUMBER="7"
fi

SESSION="aoloop${LOOPNUMBER}-GUI"



# ======================= PROCESS NAME =================================
pname=`echo "$0" | sed "s/\.\///g"`

printHELP ()
{
echo
echo "------------------------------------------------------------------------"
echo "$(tput bold) $pname :  Set up cacao control screen $(tput sgr0)"
echo "------------------------------------------------------------------------"
echo " "
echo "Starts AO control screen in tmux session $SESSION"
echo "Connects to session"
echo "If session already exists, do not re-create"
echo " "
echo " $(tput bold)USAGE:$(tput sgr0)"
echo "     $0 [options]"
echo ""
echo " $(tput bold)OPTIONS:$(tput sgr0)"
echo "  Performs task and exit: "
echo "   --help -h             print (h)elp and exit"
echo "   --help1               print 1 line summary"
echo "   -s                    simple control, no hardware control"
echo "   -x                    screen X size - width  (default $wXsize)"
echo "   -y                    screen Y size - height (default $wYsize)"
echo "   -q                    quits/close existing session"
echo "   -n                    Loop number"    
echo "------------------------------------------------------------------------"
}



printHELP1 ()
{
	printf "%20s       Start AO control screen\n" "$0" 
}

EXITSTATUS=0


function cmdexists()
{
  command -v "$1" >/dev/null 2>&1
}


function checkCommand {
if cmdexists $1; then
    echo "[$(tput setaf 2)$(tput bold)   OK   $(tput sgr0)] Command $(tput bold)$1$(tput sgr0) found"
else
    echo "[$(tput setaf 1)$(tput bold) FAILED $(tput sgr0)] Command $(tput bold)$1$(tput sgr0) not installed.  Aborting."; EXITSTATUS=1;
fi

}


# Transform long options to short ones
singlelinehelp=0
for arg in "$@"; do
  shift
  case "$arg" in
    "--help") set -- "$@" "-h" ;;
    "--help1") 
set -- "$@" "-h" 
singlelinehelp=1;
;;
    *)        set -- "$@" "$arg"
  esac
done



SIMPLECONTROL=0

### Start getopts code ###
while getopts :hsx:y:n:lq FLAG; do
  case $FLAG in
    h)  #show help
	if [ "$singlelinehelp" -eq "0" ]; then
      printHELP
      else
      printHELP1
      fi
      exit
      ;;
    s)
    SIMPLECONTROL=1
    wXsize="360"
    wYsize="90"
    ;;
    x)  #  set X size
      wXsize=$OPTARG
      ;;
    y)  #  set Y size
      wYsize=$OPTARG
      ;;
    n)
    LOOPNUMBER=$OPTARG
    SESSION="aoloop${LOOPNUMBER}-GUI"
    ;;
    q)
    tmux kill-session -t $SESSION
    exit
    ;;
    l)
    tmux ls| grep -E "\-GUI"
    exit
    ;;
    \?) #unrecognized option - show help
      echo -e \\n"Option -${BOLD}$OPTARG${NORM} not allowed."
      printHELP
      ;;
  esac
done

shift $((OPTIND-1)) 

### End getopts code ###






if [ "$1" = "help" ] || [ "$#" -ne $NBARGS ]; then
if [ "$#" -ne $NBARGS ]; then
    echo -e \\n"Illegal number of parameters ($NBARGS params required, $# entered)"\\n
fi
printHELP
        exit
fi


checkCommand tmux


if [ $EXITSTATUS -eq 1 ]; then
echo ""
echo "$(tput setaf 1)$(tput bold) REQUIRED FILES, COMMANDS NOT FOUND: EXITING $(tput sgr0)"
echo ""
exit
else
echo ""
fi









STARTSESSION="1"
STARTPROC="1"



datestr=`date -u +%Y%m%d`






# TEST if session exists
if tmux ls | grep -q "$SESSION"; then
	echo "Session $SESSION exists -> reconnecting"
	STARTSESSION="0"
	STARTPROC="0"
else
	echo "Session $SESSION does not exist -> creating"
	STARTSESSION="1"
	STARTPROC="1"
fi







if [ "$STARTSESSION" = "1" ]; then

# open session, assume 256 colors (-2)
tmux -2 new-session -d -s $SESSION
tmux setw force-width $wXsize
tmux setw force-height $wYsize


# Setup additional window for logs
tmux new-window -t $SESSION:1 -n logs


# partition window 0
tmux select-window -t $SESSION:0

fi




xterm -geometry ${wXsize}x${wYsize} -e "tmux a -t $SESSION" &



if [ "$STARTSESSION" = "1" ]; then
tdelay="0.5"

if [ "$SIMPLECONTROL" = "0" ]; then
sleep $tdelay
tmux split-window -h -p 70 -t $SESSION:0
sleep $tdelay
tmux split-window -h -p 62 -t $SESSION:0
RIGHTSCREEN="5"
else
sleep $tdelay
tmux split-window -h -p 60 -t $SESSION:0
RIGHTSCREEN="2"
fi


# pane 0 1 (left column)
sleep $tdelay
tmux select-pane -t 0
sleep $tdelay
tmux split-window -v -p 20 


if [ "$SIMPLECONTROL" = "0" ]; then
# pane 2 3 4 (center column)
sleep $tdelay
tmux select-pane -t 2
sleep $tdelay
tmux split-window -v -p 50 
sleep $tdelay
tmux split-window -v -p 50 
fi



if [ "$SIMPLECONTROL" = "0" ]; then
# pane 5 6 7 8 9 10 (right column)
sleep $tdelay
tmux select-pane -t $RIGHTSCREEN
sleep $tdelay
tmux split-window -v -p 95 # 0.0500
sleep $tdelay
tmux split-window -v -p 90 # 0.1425
sleep $tdelay
tmux split-window -v -p 85 # 0.1615
sleep $tdelay
tmux split-window -v -p 76 # 0.1744
sleep $tdelay
tmux split-window -v -p 70 # 0.1651 - 0.3065
else
# pane 5 6 7 (right column)
sleep $tdelay
tmux select-pane -t $RIGHTSCREEN
sleep $tdelay
tmux split-window -v -p 76 
sleep $tdelay
tmux split-window -v -p 65
fi


# Allow TMUX access within TMUX
sleep $tdelay
tmux send-keys -t $SESSION:0.0 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.1 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.2 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.3 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.4 "TMUX=" C-m
sleep $tdelay
if [ "$SIMPLECONTROL" = "0" ]; then
tmux send-keys -t $SESSION:0.5 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.6 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.7 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.8 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.9 "TMUX=" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.10 "TMUX=" C-m
fi

fi


tdelay="1.0"




if [ "$STARTPROC" = "1" ]; then
# START aolconf
tmux send-keys -t $SESSION:0.0 "./aolconf" C-m
sleep $tdelay


if [ "$SIMPLECONTROL" = "0" ]; then
# START Alignment screen
tmux send-keys -t $SESSION:0.2 "./aolconf -n" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.2 "A" C-m
sleep $tdelay

# START Hardware control screen
tmux send-keys -t $SESSION:0.3 "./aolconf -n" C-m
sleep $tdelay
tmux send-keys -t $SESSION:0.3 "H" C-m
sleep $tdelay

SCREEN_LOG1="5"
SCREEN_LOG2="6"
SCREEN_MONITOR="8"
SCREEN_RTLOGGING="9"
SCREEN_PROCCTRL="10"



# START log input
tmux send-keys -t $SESSION:0.$SCREEN_LOG1 "./aolconfscripts/aollog -ie RTC-MISC NULL" C-m
sleep $tdelay


# START log output
touch logdir/$datestr/logging/RTC-MISC.log
tmux send-keys -t $SESSION:0.$SCREEN_LOG2 "tail -f aolconf.log &" C-m
tmux send-keys -t $SESSION:0.$SCREEN_LOG2 "tail -f logdir/$datestr/logging/RTC-MISC.log" C-m
sleep $tdelay


else

SCREEN_MONITOR="2"
SCREEN_RTLOGGING="3"
SCREEN_PROCCTRL="4"

fi






# START monitor
tmux send-keys -t $SESSION:0.$SCREEN_MONITOR "cacao" C-m
tmux send-keys -t $SESSION:0.$SCREEN_MONITOR "aolmon" C-m
sleep $tdelay


# START RT logging
tmux send-keys -t $SESSION:0.$SCREEN_RTLOGGING "cacao" C-m
tmux send-keys -t $SESSION:0.$SCREEN_RTLOGGING "aolrtlogGUI" C-m
sleep $tdelay

# START process control
tmux send-keys -t $SESSION:0.$SCREEN_PROCCTRL "cacao" C-m
tmux send-keys -t $SESSION:0.$SCREEN_PROCCTRL "procCTRL" C-m
sleep $tdelay


fi


