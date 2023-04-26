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
tmux send -t mysession:1.0 "sleep 2; rosservice call /ik/outlabels" C-m
tmux send -t mysession:1.1 "rostopic echo /model_generic/joint_states" C-m
tmux send -t mysession:1.2 "roslaunch osrt_ros cgrfm_as_grf.launch model_file:=/srv/data/gait1992/residual_reduction_algorithm/model_adjusted.osim" C-m

tmux send -t mysession:1.3 "roslaunch gait1992_description human_control.launch" C-m
tmux send -t mysession:1.4 "roslaunch osrt_ros ik_bare_1992.launch" C-m
## this can also play ik with size 23 for a very similar lowerbody model. scaling is not done, so distances will be off.
#tmux send -t mysession:1.4 "roslaunch osrt_ros ik_bare_2392.launch" C-m
tmux send -t mysession:1.5 "rosrun rqt_graph rqt_graph" C-m
tmux send -t mysession:1.6 "roslaunch osrt_ros custom.launch" C-m
tmux send -t mysession:1.7 "sleep 2.2; rosservice call /inverse_kinematics_from_file/start" C-m
#tmux send -t mysession:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession

