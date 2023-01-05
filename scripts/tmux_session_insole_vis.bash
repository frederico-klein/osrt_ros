#!/usr/bin/env bash
set -e

tmux new-session -s insole_vis -d 
tmux set-option -s -t insole_vis default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t insole_vis:0.0 "roscore" C-m
sleep 2
tmux split-window -h 
tmux split-window -h 
tmux split-window -h 
tmux select-layout even-horizontal
tmux select-pane -t 3
tmux split-window -v -p 50 
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux select-pane -t 1 
tmux split-window -v -p 50 
tmux select-pane -t 0
tmux split-window -v -p 50 

tmux new-window
tmux select-window -t 1
tmux split-window -h 
tmux split-window -h 
tmux split-window -h 
tmux select-layout even-horizontal
tmux select-pane -t 3
tmux split-window -v -p 50 
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux select-pane -t 1 
tmux split-window -v -p 50 
tmux select-pane -t 0
tmux split-window -v -p 50 
#tmux select-layout tiled
#tmux select-pane -t 0

#sends keys to first and second terminals
#tmux send -t insole_vis:0.0 "" C-m #NOPE roscore is running here!
## spawn graphs here
tmux send -t insole_vis:0.1 "rqt_plot /left/force/data /right/force/data" C-m
## the times for the diffs are in ms
tmux send -t insole_vis:0.2 "rqt_plot /insoles/data /left/time_diff/data /right/time_diff/data" C-m
#tmux send -t insole_vis:0.3 "rosrun rqt_graph rqt_graph" C-m
#tmux send -t insole_vis:0.4 "roslaunch moticon_insoles see_wrench.launch" C-m
#tmux send -t insole_vis:0.5 "roslaunch osrt_ros ik_lowerbody.launch" C-m
#tmux send -t insole_vis:0.6 "roslaunch ximu3_ros ximu_lower.launch" C-m
#tmux send -t insole_vis:0.7 "roslaunch moticon_insoles onfeet_wrench.launch" C-m

tmux send -t insole_vis:1.0 "rostopic hz /right/force" C-m
tmux send -t insole_vis:1.1 "rostopic hz /left/force" C-m
tmux send -t insole_vis:1.2 "rostopic hz /right/cop" C-m
tmux send -t insole_vis:1.3 "rostopic hz /left/cop" C-m
tmux send -t insole_vis:1.4 "rostopic echo /right/time_diff" C-m
# this is the latency of the loop that needs to get data from each insole, so it has to have the period of the input divided by 2, one for each insole since they are separate proto msgs
tmux send -t insole_vis:1.5 "rostopic echo /insoles" C-m
#tmux send -t insole_vis:1.6 "roslaunch ximu3_ros ximu_lower.launch" C-m
tmux send -t insole_vis:1.7 "roslaunch moticon_insoles onfeet_wrench.launch" C-m
#tmux setw synchronize-panes on

tmux -2 a -t insole_vis
