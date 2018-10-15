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
#  pane 5    log
#  pane 6    Data logging RT streams
#  pane 7    process control
#
# ==============================================================================

STARTSESSION="1"
STARTPROC="1"
wXsize=500
wYsize=160


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

tmux split-window -h -p 65 -t $SESSION:0
tmux split-window -h -p 50 -t $SESSION:0


# pane 0 1 (left column)
tmux select-pane -t 0
tmux split-window -v -p 20 


# pane 2 3 4 (center column)
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux split-window -v -p 50 

# pane 5 6 7 (right column)
tmux select-pane -t 5
tmux split-window -v -p 65
tmux split-window -v -p 50


sleep 1
tmux send-keys -t $SESSION:0.0 "# pane 0: " C-m
tmux send-keys -t $SESSION:0.1 "# pane 1: " C-m
tmux send-keys -t $SESSION:0.2 "# pane 2: " C-m
tmux send-keys -t $SESSION:0.3 "# pane 3: " C-m
tmux send-keys -t $SESSION:0.4 "# pane 4: " C-m
tmux send-keys -t $SESSION:0.5 "# pane 5: " C-m
tmux send-keys -t $SESSION:0.6 "# pane 6: " C-m
tmux send-keys -t $SESSION:0.7 "# pane 7: " C-m

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

# START process control
tmux send-keys -t $SESSION:0.6 "cacao" C-m
tmux send-keys -t $SESSION:0.6 "aolrtlogGUI" C-m
sleep 0.1

# START process control
tmux send-keys -t $SESSION:0.7 "cacao" C-m
tmux send-keys -t $SESSION:0.7 "procCTRL" C-m
sleep 0.1


fi


