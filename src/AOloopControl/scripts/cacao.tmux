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

STARTSESSION="1"
STARTPROC="1"
wXsize=500
wYsize=150


datestr=`date -u +%Y%m%d`


DIR="/data0/dataWORK/AOloop"

if [ -f LOOPNUMBER ]; then
LOOPNUMBER=$( head -1 LOOPNUMBER )
else
LOOPNUMBER="7"
fi

SESSION="aoloop${LOOPNUMBER}-GUI"



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
cd $DIR
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
sleep 0.1

tmux split-window -h -p 70 -t $SESSION:0
tmux split-window -h -p 57 -t $SESSION:0


# pane 0 1 (left column)
tmux select-pane -t 0
tmux split-window -v -p 20 


# pane 2 3 4 (center column)
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux split-window -v -p 50 

# pane 5 6 7 8 (right column)
tmux select-pane -t 5
tmux split-window -v -p 95
tmux split-window -v -p 85
tmux split-window -v -p 80
tmux split-window -v -p 73
tmux split-window -v -p 65


sleep 0.5
# Allow TMUX access within TMUX
tmux send-keys -t $SESSION:0.0 "TMUX=" C-m
tmux send-keys -t $SESSION:0.1 "TMUX=" C-m
tmux send-keys -t $SESSION:0.2 "TMUX=" C-m
tmux send-keys -t $SESSION:0.3 "TMUX=" C-m
tmux send-keys -t $SESSION:0.4 "TMUX=" C-m
tmux send-keys -t $SESSION:0.5 "TMUX=" C-m
tmux send-keys -t $SESSION:0.6 "TMUX=" C-m
tmux send-keys -t $SESSION:0.7 "TMUX=" C-m
tmux send-keys -t $SESSION:0.8 "TMUX=" C-m
tmux send-keys -t $SESSION:0.9 "TMUX=" C-m
tmux send-keys -t $SESSION:0.10 "TMUX=" C-m
fi






if [ "$STARTPROC" = "1" ]; then
# START aolconf
tmux send-keys -t $SESSION:0.0 "./aolconf" C-m
sleep 0.5

# START Alignment screen
tmux send-keys -t $SESSION:0.2 "./aolconf -n" C-m
sleep 0.5
tmux send-keys -t $SESSION:0.2 "A" C-m
sleep 0.1

# START Hardware control screen
tmux send-keys -t $SESSION:0.3 "./aolconf -n" C-m
sleep 0.5
tmux send-keys -t $SESSION:0.3 "H" C-m
sleep 0.1

# START log input
tmux send-keys -t $SESSION:0.5 "./aolconfscripts/aollog -ie RTC-MISC NULL" C-m
sleep 0.1


# START log output
touch logdir/$datestr/logging/RTC-MISC.log
tmux send-keys -t $SESSION:0.6 "tail -f aolconf.log &" C-m
tmux send-keys -t $SESSION:0.6 "tail -f logdir/$datestr/logging/RTC-MISC.log" C-m
sleep 0.1

# START process control
tmux send-keys -t $SESSION:0.8 "cacao" C-m
tmux send-keys -t $SESSION:0.8 "aolmon" C-m
sleep 0.1


# START process control
tmux send-keys -t $SESSION:0.9 "cacao" C-m
tmux send-keys -t $SESSION:0.9 "aolrtlogGUI" C-m
sleep 0.1

# START process control
tmux send-keys -t $SESSION:0.10 "cacao" C-m
tmux send-keys -t $SESSION:0.10 "procCTRL" C-m
sleep 0.1


fi


