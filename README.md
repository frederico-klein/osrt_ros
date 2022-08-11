# osrt\_ros

Ros wrapper interface for the [OpenSimRT](https://github.com/mitkof6/OpenSimRT) project. 

This is the ros nodes that should be able to run with vanilla OpenSimRT at some point. This was done to allow more easily for updates on OpenSimRT to be used in the ros wrapper version. 

## Getting started:

You should start with the docker https://github.com/frederico-klein/docker-opensimrt

Branch ros-full

You probably also want to be placed in your catkin\_ws/src directory:

    - https://github.com/mysablehats/opensimrt_bridge
    - https://github.com/mysablehats/opensimrt_msgs
    - https://github.com/mysablehats/ximu3_ros
    - https://github.com/mysablehats/OpenSimRT_data
    - https://github.com/mysablehats/cometa_bridge
    - https://github.com/frederico-klein/gait1992_description


## TODO:

- Make sure it compiles with vanilla opensimrt
- Remove extra unnecessary files:
	- Make UIMU derived from IMU
	- Standardize OnlineUpperLimb and OnlineLowerLimb to use TFs only
- add parameters to nodes to remove dependency on INI file
