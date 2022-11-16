#!/usr/bin/env bash
set -e

tmux new-session -s ar_acer_session -d 
tmux set-option -s -t ar_acer_session default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t ar_acer_session:0.0 "roscore" C-m
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
tmux send -t ar_acer_session:1.0 "rviz -d /catkin_ws/_cam_acer_tf.rviz" C-m
 #rviz -d ./_default.rviz
tmux send -t ar_acer_session:1.1 "roslaunch ar_test ar_cube_acer.launch" C-m
tmux send -t ar_acer_session:1.2 "roslaunch ar_test acer_video_stream.launch" C-m
tmux send -t ar_acer_session:1.3 "ROS_NAMESPACE=usb_cam_acer rosrun image_proc image_proc" C-m
tmux send -t ar_acer_session:1.4 "roslaunch moticon_insoles see_wrench.launch" C-m
#tmux send -t ar_acer_session:1.5 "roslaunch opensimrt	id.launch" C-m

#tmux send -t ar_acer_session:1.6 "ls -la" C-m
#tmux send -t ar_acer_session:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t ar_acer_session
