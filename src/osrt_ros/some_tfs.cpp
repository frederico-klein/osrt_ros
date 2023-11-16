/**
 * @author      : $USER ($USER@dbefbc0c19ed)
 * @file        : some_tfs
 * @created     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "some_tfs.h" // if I include this everything fails. figure it out.
#include <OpenSim/Simulation/Model/Model.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	ros::NodeHandle nh("~");

	//something like imuBodiesObservation, but not really
	
	Osim_tf_publisher op;
	op.init();	//
	/*std::function<void(sensor_msgs::JointStateConstPtr)> callback = [&op](sensor_msgs::JointStateConstPtr msg) {
        op.other_callback(msg);
    };
    */
	//op.sync_input_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, callback);



	ros::spin();
	return 0;
}

