#!/usr/bin/env bash

tmux new-session -s mysession -d 
tmux set-option -s -t mysession default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t mysession:0.0 "roscore" C-m
sleep 2

tmux new-window
tmux select-window -t 1
tmux split-window -h 
tmux split-window -h 
tmux select-layout even-horizontal
tmux select-pane -t 2
tmux split-window -v -p 50 
tmux select-pane -t 1 
tmux split-window -v -p 50 
tmux select-pane -t 0
tmux split-window -v -p 50 

tmux send -t mysession:1.0 "roslaunch osrt_ros ik_lowerbody_inverted_pelvis.launch" C-m
tmux send -t mysession:1.1 "roslaunch ximu3_ros ximu_as_pelvis.launch" C-m
tmux send -t mysession:1.2 "roslaunch osrt_ros lower_body_tfs_pelvis.launch" C-m
tmux send -t mysession:1.3 "roslaunch gait1992_description human_control.launch" C-m
tmux send -t mysession:1.4 "roslaunch osrt_ros custom.launch" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession
