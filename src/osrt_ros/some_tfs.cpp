/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : some_tfs.cpp
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
//#include "some_tfs.h" // if I include this everything fails. figure it out.
#include <OpenSim/Simulation/Model/Model.h>

class Vic
{
	public:
	Vic(const OpenSim::Model& otherModel)
		: model(*otherModel.clone())
	{
	state = model.initSystem();
	}
	void update(const SimTK::Vector& q)
	{
		state.updQ() = q;
		model.realizePosition(state);
	}
	private:
	OpenSim::Model model;
	SimTK::State state;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	ros::NodeHandle nh("~");
	std::string modelFile;
	nh.param<std::string>("model_file", modelFile, "");
	ROS_INFO_STREAM("==== modelFile: " << modelFile);
	OpenSim::Model model = OpenSim::Model(modelFile);
	//something like imuBodiesObservation, but not really

	//Osim_tf_publisher op(model, state);
	//op.init();	//
	//
	std::vector<double> qq{0, 0, 0, 0, 1, 0, 0.785398, 0, 0, -2e-08, -2e-8};
	SimTK::Vector fixed_q(11);
	for (size_t i=0;i<qq.size();i++)
		fixed_q[i] = qq[i];
	const SimTK::Vector* q = &fixed_q;
	Vic vic(model);
	vic.update(*q);
	/*std::function<void(sensor_msgs::JointStateConstPtr)> callback = [&op](sensor_msgs::JointStateConstPtr msg) {
	  op.other_callback(msg);
	  };
	  */
	//op.sync_input_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, callback);
	//	



	ros::spin();
	return 0;
}

