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


## Notes

### Common node:

The current architecture uses a common node class interface which deals with setting up the label order for CommonTimed messages, which are single lines of a OpenSim TimeSeriesTable object. They also need the labels and first we went with a structure like the cameraInfo messages, with another publisher for that and using latching messages, but later we considered that it made more sense to make that a service to ask what are the labels of each message and not needing to transfer the header for each individual message (thus reducing bandwidth a little bit). 

The structure here is minimal, each common node is a pipe, which can have either its output or input disabled to generate sinks and sources. 

### IK node:

The IK node reads the TFs from the ximu driver with the name of the tf frame being the name that is listed under the observation\_order parameter of the ik node. In short, to get something publishing IKs you need the ximu3\_ros node driver publishing the TFs (or some static publisher doing the same thing) and the node that reads those values and converts it to joint angles (currently either ik\_lowebody.launch of ik\_upper.launch). 
It is technically a source node (input CommonTimed subscriber and service is disabled), because the TFs are obtained through a threaded loop which will get the latest TFs available, no matter what the TF publishing rate is. This loops will have its independent publishing frequency rate, which can be set as a parameter. 

The default publishing is to output both, but it is showing on the screen the filtered values. Outputs are /output and /output\_filtered which need to be properly remapped, together with the services to be used by subsequent nodes. 

The sequence in which values are published is dictaded by the observation\_order parameter which is the same used to get the TFs, so this shouldn't be a concern.

### Pipe nodes

### GRF nodes (currently agrfm\*.launch and cgrfm\*.launch)

Currently the grf nodes accept 2 types of input. Filtered IK and Unfiltered IK. I did this because I was worried that the message size would be too large, but this seems not to be an issue. 

Depending on which node you send the messages to, you will get the correct output, so if runnign filtered, remap the filtered topics accordingly. This has the advantage of making the GRFM nodes having a faster loop time, which will add up down the line if you need to filter IKs twice.

To call the correct filtered or unfiltered node, you just need to setup the correct topic remaps. It will execute both if both are remapped, which is unnecessary, since they are both in the end filtered (it just changes if it is inside the node or outside). This might have uses, because the filter characteristics can be different, but probably is a mistake. 

### Dual input, single output nodes

The dual input nodes have some extra complexity now because not only they need to use message filters to get appropriately timed messages, but the also need to deal with either filtered or unfiltered input namely:

either:

- PosVelAccTimed ik\_filtered, CommonTimed GRF

or

- CommonTimed ik, CommonTimed GRF

We implement a dual callback, like the one used for GRFs, which calls then calls the same main function which can output results accordingly.


## TODO:

- Make sure it compiles with vanilla opensimrt
- Remove extra unnecessary files:
	- Make UIMU derived from IMU
	- Standardize OnlineUpperLimb and OnlineLowerLimb to use TFs only
- add parameters to nodes to remove dependency on INI file
- The SO implementation is slow. One idea is to multithread it with a round-robin scheduler and then put the messages in order by timestamp with a [TimeSequencer from message\_filters] (http://wiki.ros.org/message_filters). The round-robin was not too hard to build, I wrote [an example here](https://github.com/frederico-klein/ros_tutorials/blob/noetic-devel/roscpp_tutorials/listener_long_processing_time/listener_long_processing_time.cpp)
