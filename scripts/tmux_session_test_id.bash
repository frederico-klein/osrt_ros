#!/usr/bin/env bash

tmux new-session -s mysession -d 
tmux set-option -s -t mysession default-command "bash --rcfile ~/.bashrc_ws.sh"
tmux send -t mysession:0.0 "roscore" C-m
sleep 2

tmux new-window
tmux new-window
tmux split-window -h 
tmux new-window
tmux split-window -h 
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
tmux select-pane -t 7
tmux split-window -h 
tmux select-pane -t 5
tmux split-window -v -p 50 


#sends keys to first and second terminals
tmux send -t mysession:1.0 "rostopic echo /ik/output" C-m
tmux send -t mysession:1.1 "rostopic echo /grf_node/output" C-m
tmux send -t mysession:1.2 "roslaunch osrt_ros id.launch" C-m
tmux send -t mysession:1.3 "roslaunch opensimrt_bridge grf.launch" C-m
tmux send -t mysession:1.4 "roslaunch osrt_ros ik_bare.launch" C-m
tmux send -t mysession:1.5 "rosrun rqt_graph rqt_graph" C-m
tmux send -t mysession:1.6 "rosrun osrt_ros graph_grfs.bash" C-m
tmux send -t mysession:1.7 "sleep 2; rosservice call /inverse_kinematics_from_file/start" C-m
tmux send -t mysession:1.8 "rostopic echo /grf_left/wrench" C-m
tmux send -t mysession:1.9 "rostopic echo /grf_right/wrench" C-m

#tmux send -t mysession:1.7 "rosrun rviz rviz -d _insole.rviz" C-m
tmux send -t mysession:2.0 "rosrun rviz rviz -d _insole_robot.rviz" C-m
tmux send -t mysession:2.1 "roslaunch osrt_ros custom.launch" C-m

#tmux send -t mysession:2.0 "cd /catkin_opensim/src/opensimrt_core/OpenSimRT/Pipeline; nv" C-m
#tmux send -t mysession:3.0 "cd /catkin_opensim/src/opensimrt_core/launch; nv" C-m
tmux send -t mysession:3.0 "roslaunch gait1992_description human_control_no_rviz.launch" C-m

#tmux send -t mysession:1.6 "ls -la" C-m
#tmux send -t mysession:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession
