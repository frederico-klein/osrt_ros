#include "osrt_ros/Visualizers/dualsink_vis.h"
#include <SimTKcommon/internal/BigMatrix.h>

void Visualizers::DualSinkVis::callback(const opensimrt_msgs::DualConstPtr &message)
{
	ROS_INFO_STREAM("callback dialvissink reached received message:" << message);
	//initialize q
	SimTK::Vector q(message->q.data.size()); //TODO: its not tau the name of the message is unfortunate change it, so that this reads better
	for (size_t i=0;i<message->q.data.size(); i++)
	{
		q[i] = message->q.data[i];
	}
	try {

		visualizer->update(q);
		after_callback();
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}
}
void Visualizers::DualSinkVis::callback_filtered(const opensimrt_msgs::DualPosConstPtr &message)
{
	ROS_INFO_STREAM("callback dialvissink filtered reached received message:" << message);
	
	//initialize q
	SimTK::Vector q(message->qqq.d0_data.size()); //TODO: its not tau the name of the message is unfortunate change it, so that this reads better
	for (size_t i=0;i<message->qqq.d0_data.size(); i++)
	{
		q[i] = message->qqq.d0_data[i];
	}
	try {

		visualizer->update(q);
		after_callback();
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}
}
