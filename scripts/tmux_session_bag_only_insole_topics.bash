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
tmux split-window -h -p 50 
tmux select-pane -t 5
tmux split-window -v -p 50 


#sends keys to first and second terminals
tmux send -t mysession:1.0 "rosparam set use_sim_time true" C-m
tmux send -t mysession:1.0 "rostopic echo /ik/output" C-m
tmux send -t mysession:1.1 "rostopic echo /grf_node/output" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 25 --duration 10 --loop" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 35 --duration 8 --loop" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 43 --duration 13 --loop" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 56 --duration 12 --loop" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 68 --duration 14 --loop" C-m
#tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 82 --duration 12 --loop" C-m
tmux send -t mysession:1.2 "rosbag play /catkin_ws/2022-11-17-14-35-31.bag --clock --start 84 --duration 8 --loop --topics /left/wrench /right/wrench /left/cop /right/cop" C-m
tmux send -t mysession:1.3 "roslaunch opensimrt_bridge grf_fk.launch" C-m
tmux send -t mysession:1.4 "roslaunch opensimrt_bridge ik_bare.launch" C-m
tmux send -t mysession:1.5 "rosrun rqt_graph rqt_graph" C-m
#tmux send -t mysession:1.6 "rosrun osrt_ros graph_grfs.bash" C-m
tmux send -t mysession:1.6 "roslaunch osrt_ros bag_tfs.launch" C-m
tmux send -t mysession:1.7 "sleep 1; rosservice call /inverse_kinematics_from_file/start_at" C-m
tmux send -t mysession:1.8 "rostopic echo /left/wrench" C-m
tmux send -t mysession:1.9 "rostopic echo /right/wrench" C-m

#tmux send -t mysession:1.7 "rosrun rviz rviz -d _insole.rviz" C-m
tmux send -t mysession:2.0 "rosrun rviz rviz -d /catkin_ws/_insole_robot_bag_combined.rviz" C-m
tmux send -t mysession:2.1 "roslaunch osrt_ros custom.launch" C-m

#tmux send -t mysession:2.0 "cd /catkin_opensim/src/opensimrt_core/OpenSimRT/Pipeline; nv" C-m
#tmux send -t mysession:3.0 "cd /catkin_opensim/src/opensimrt_core/launch; nv" C-m
tmux send -t mysession:3.0 "roslaunch gait1992_description human_control_no_rviz.launch" C-m
tmux send -t mysession:3.1 "rosrun osrt_ros graph_bag_grfs_ground_plate_insoles.bash" C-m

#tmux send -t mysession:1.6 "ls -la" C-m
#tmux send -t mysession:1.7 "ls -la" C-m
#tmux setw synchronize-panes on

tmux -2 a -t mysession
