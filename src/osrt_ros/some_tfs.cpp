/**
 * @author      : $USER ($USER@dbefbc0c19ed)
 * @file        : some_tfs
 * @created     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include "ros/ros.h"
#include <Simulation/Model/Model.h>
//#include "some_tfs.h" // if I include this everything fails. figure it out.

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	ros::NodeHandle nh("~");
	std::string modelFile;
	nh.param<std::string>("model_file", modelFile, "");
	ROS_INFO_STREAM("modelFile" << modelFile);
	auto model = OpenSim::Model(modelFile);
	auto state = model.initSystem(); //this crashes idk why, probably model is broken or it's looking for things it cant find
			    //

	//something like imuBodiesObservation, but not really
	
	//Osim_tf_publisher op(modelFile);
	//op.init();	//

	ros::spin();
	return 0;
}

