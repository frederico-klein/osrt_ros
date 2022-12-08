#!/usr/bin/env bash
set -e

tmux new-session -s fix_quaternions -d 
tmux set-option -s -t fix_quaternions default-command "bash --rcfile ~/.bashrc_ws.sh"
#tmux send -t fix_quaternions:0.0 "roscore" C-m
#sleep 2

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
#tmux send -t fix_quaternions:1.0 "rviz -d /catkin_ws/_cam_tf.rviz" C-m
 #rviz -d ./_default.rviz
tmux send -t fix_quaternions:1.1 "rosrun ar_test fixquaternions.py" C-m
tmux send -t fix_quaternions:1.2 "rosrun ar_test fixquaternions_server.py" C-m
tmux send -t fix_quaternions:1.3 "rosrun rqt_reconfigure rqt_reconfigure" C-m
#tmux send -t fix_quaternions:1.4 "rosservice call /inverse_kinematics_from_file/start" C-m
#tmux send -t fix_quaternions:1.5 "roslaunch opensimrt	id.launch" C-m

#tmux send -t fix_quaternions:1.6 "ls -la" C-m
#tmux send -t fix_quaternions:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t fix_quaternions
