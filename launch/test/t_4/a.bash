#!/usr/bin/env bash
SESSION_NAME=test_heading
a_file=$1
angle=$2

source "`rospack find tmux_session_core`/common_functions.bash"
ros_core_tmux "$SESSION_NAME"
#ros_core_tmux_clock "$SESSION_NAME"

tmux set -g pane-border-status top
    
W2=(
"rosrun tf view_frames"
#"roslaunch osrt_ros t.launch sto_file:=$a_file"
#"roslaunch osrt_ros t3.launch sto_file:=$a_file"
"roslaunch osrt_ros t41.launch sto_file:=$a_file"
"roslaunch osrt_ros t42.launch run_as_service:=true parent_frame:=map"
"roslaunch osrt_ros t43.launch "
#### These to show the skeleton
#"roslaunch osrt_ros t44.launch base_parent:=opensim_default_frame"
"roslaunch osrt_ros t44.launch"
"roslaunch osrt_ros t45.launch "
"sleep 1 ;roslaunch osrt_ros t46.launch bypass_heading_computation:=true heading_debug:=0 heading_offset:=$angle visualise:=false wait_to_start:=false"

##"roslaunch custom_clock simpler_clock.launch clock_step_microsseconds:=1000 slowdown_rate:=1" 
"roslaunch osrt_ros vis_ik.launch"
#"rqt_graph"
#"/catkin_ws/src/osrt_ros/avg.py"
)

create_tmux_window "$SESSION_NAME" "sync" "${W2[@]}"

tmux -2 a -t $SESSION_NAME

