#!/usr/bin/env bash

tmux new-session -s mysession -d 
tmux set-option -s -t mysession default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t mysession:0.0 "roscore" C-m
sleep 2

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
tmux send -t mysession:1.0 "rostopic echo /inverse_kinematics_from_file/r_data" C-m
tmux send -t mysession:1.1 "rostopic echo /cgrf_node/output" C-m
tmux send -t mysession:1.3 "roslaunch osrt_ros cgrfm.launch" C-m
tmux send -t mysession:1.4 "roslaunch opensimrt_bridge ik_acceleration_prediction_gfrm.launch" C-m
tmux send -t mysession:1.5 "rosrun rqt_graph rqt_graph" C-m
tmux send -t mysession:1.6 "rosservice call /inverse_kinematics_from_file/start" C-m


#tmux send -t mysession:1.6 "ls -la" C-m
#tmux send -t mysession:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession

