#!/usr/bin/env bash
set -e

tmux new-session -s insole_everything -d 
tmux set-option -s -t insole_everything default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t insole_everything:0.0 "roscore" C-m
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
#tmux send -t insole_everything:0.0 "" C-m #NOPE roscore is running here!
tmux send -t insole_everything:0.1 "roslaunch osrt_ros custom.launch" C-m
tmux send -t insole_everything:0.2 "ROS_NAMESPACE=usb_cam_acer rosrun image_proc image_proc" C-m
tmux send -t insole_everything:0.3 "rosrun rqt_graph rqt_graph" C-m
#tmux send -t insole_everything:0.4 "roslaunch moticon_insoles see_wrench.launch" C-m
#tmux send -t insole_everything:0.5 "roslaunch osrt_ros ik_lowerbody.launch" C-m
#tmux send -t insole_everything:0.6 "roslaunch ximu3_ros ximu_lower.launch" C-m
#tmux send -t insole_everything:0.7 "roslaunch moticon_insoles onfeet_wrench.launch" C-m

tmux send -t insole_everything:1.0 "roslaunch gait1992_description human_control.launch" C-m
tmux send -t insole_everything:1.1 "roslaunch ar_test ar_cube_acer.launch" C-m
tmux send -t insole_everything:1.2 "roslaunch ar_test acer_video_stream.launch" C-m
#tmux send -t insole_everything:1.3 "" C-m
tmux send -t insole_everything:1.4 "roslaunch osrt_ros agrfm_as_grf.launch" C-m
tmux send -t insole_everything:1.5 "roslaunch osrt_ros ik_lowerbody.launch" C-m
tmux send -t insole_everything:1.6 "roslaunch ximu3_ros ximu_lower.launch" C-m
tmux send -t insole_everything:1.7 "roslaunch moticon_insoles onfeet_wrench.launch" C-m
#tmux setw synchronize-panes on

tmux -2 a -t insole_everything
