#!/usr/bin/env bash


MSdescr="cacao control TUI within tmux"


MSextdescr="
START cacao ASCII GUI

Starts AO control screen in tmux session
Connects to session
If session already exists, do not re-create

GUI starts in tmux session (fixed size) to be opened in a xterm terminal

WINDOW 0

  pane 0    aolconf control

  pane 2    instrument alignment
  pane 3    Hardware control
  pane 4

  pane 5    log input
  pane 6    log output
  pane 7
  pane 8    loop monitor (aolmon)
  pane 7    Data logging RT streams
  pane 10   process control
"


source milk-script-std-config
source cacao-check-cacaovars
SESSION="aoloop${CACAO_LOOPNUMBER}-GUI"

SIMPLECONTROL=0
MSopt+=( "s:simple:set_simple::simple control, no hardware control" )
function set_simple() {
	SIMPLECONTROL=1
}


# window size in characters
wXsize=550
wYsize=150

MSopt+=( "x:xsize:set_xsize:xsize[int]:screen X size - width  (default $wXsize)" )
function set_xsize() {
	wXsize="$1"
}

MSopt+=( "y:ysize:set_ysize:ysize[int]:screen Y size - height  (default $wYsize)" )
function set_ysize() {
	wYsize="$1"
}

MSopt+=( "q:quit:quitfunc::quits/close existing session" )
function quitfunc() {
	tmux kill-session -t $SESSION
	exit 0
}

RequiredCommands=(tmux xterm)



source milk-argparse















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
    tmux setw -g mouse on
    #tmux setw force-width $wXsize
    #tmux setw force-height $wYsize

    # Setup additional window for logs
    tmux new-window -t $SESSION:1 -n logs


    # partition window 0
    tmux select-window -t $SESSION:0

fi



# -fa 'Monospace' -fs 14
# 3x4 6x8 9x12 12x16
xterm -fn 12x16 -fg white -bg black -geometry ${wXsize}x${wYsize} -xrm 'XTerm.vt100.allowTitleOps: false' -T "cacao CTRL screen - loop ${CACAO_LOOPNAME}" -e "tmux a -t $SESSION" &



if [ "$STARTSESSION" = "1" ]; then
    tdelay="0.5"

    if [ "$SIMPLECONTROL" = "0" ]; then
        sleep $tdelay
        tmux split-window -h -p 72 -t $SESSION:0
        sleep $tdelay
        tmux split-window -h -p 65 -t $SESSION:0
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
    tmux send-keys -t $SESSION:0.0 "cacao-fpsCTRL-TUI" C-m
    sleep $tdelay


    if [ "$SIMPLECONTROL" = "0" ]; then
        # START Alignment screen
        #tmux send-keys -t $SESSION:0.2 "./aolconf -n" C-m
        sleep $tdelay
        #tmux send-keys -t $SESSION:0.2 "A" C-m
        sleep $tdelay

        # START Hardware control screen
        #tmux send-keys -t $SESSION:0.3 "./aolconf -n" C-m
        sleep $tdelay
        #tmux send-keys -t $SESSION:0.3 "H" C-m
        sleep $tdelay

        SCREEN_LOG1="5"
        SCREEN_LOG2="6"
        SCREEN_MONITOR="8"
        SCREEN_RTLOGGING="9"
        SCREEN_PROCCTRL="10"



        # START log input
        #tmux send-keys -t $SESSION:0.$SCREEN_LOG1 "./aolconfscripts/aollog -ie RTC-MISC NULL" C-m
        sleep $tdelay


        # START log output
        #touch logdir/$datestr/logging/RTC-MISC.log
        #tmux send-keys -t $SESSION:0.$SCREEN_LOG2 "tail -f aolconf.log &" C-m
        #tmux send-keys -t $SESSION:0.$SCREEN_LOG2 "tail -f logdir/$datestr/logging/RTC-MISC.log" C-m
        sleep $tdelay


    else

        SCREEN_MONITOR="2"
        SCREEN_RTLOGGING="3"
        SCREEN_PROCCTRL="4"

    fi






    # START monitor
    #tmux send-keys -t $SESSION:0.$SCREEN_MONITOR "cacao" C-m
    #tmux send-keys -t $SESSION:0.$SCREEN_MONITOR "aolmon" C-m
    sleep $tdelay


    # START RT logging
    #tmux send-keys -t $SESSION:0.$SCREEN_RTLOGGING "cacao" C-m
    #tmux send-keys -t $SESSION:0.$SCREEN_RTLOGGING "aolrtlogGUI" C-m
    sleep $tdelay

    # START process control
    #tmux send-keys -t $SESSION:0.$SCREEN_PROCCTRL "cacao" C-m
    tmux send-keys -t $SESSION:0.$SCREEN_PROCCTRL "milk-procCTRL" C-m
    sleep $tdelay


fi
