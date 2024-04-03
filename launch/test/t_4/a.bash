#!/usr/bin/env bash
SESSION_NAME=test_heading
a_file=$1

source "`rospack find tmux_session_core`/common_functions.bash"
ros_core_tmux "$SESSION_NAME"
#ros_core_tmux_clock "$SESSION_NAME"

tmux set -g pane-border-status top
    
W2=(
#"roslaunch osrt_ros t.launch sto_file:=$a_file"
#"roslaunch osrt_ros t3.launch sto_file:=$a_file"
"roslaunch osrt_ros t41.launch sto_file:=$a_file"
"roslaunch osrt_ros t42.launch sto_file:=$a_file run_as_service:=false"
"roslaunch osrt_ros t43.launch sto_file:=$a_file"
"roslaunch osrt_ros t44.launch sto_file:=$a_file"
"roslaunch osrt_ros t45.launch sto_file:=$a_file"
"roslaunch osrt_ros t46.launch sto_file:=$a_file bypass_heading_computation:=true heading_debug:=90 visualise:=false"
#"roslaunch custom_clock simpler_clock.launch clock_step_microsseconds:=1000 slowdown_rate:=1" 
"roslaunch osrt_ros vis_ik.launch"
"rqt_graph"
"/catkin_ws/src/osrt_ros/avg.py"
)

create_tmux_window "$SESSION_NAME" "sync" "${W2[@]}"

tmux -2 a -t $SESSION_NAME

