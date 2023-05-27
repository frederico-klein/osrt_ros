#include "osrt_ros/Visualizers/ik_vis.h"
#include "opensimrt_msgs/PosVelAccTimed.h"

void Visualizers::IkVis::callback(const opensimrt_msgs::CommonTimedConstPtr& msg)
{
	ROS_ERROR_STREAM("from the dynamic_cast part, this shouldnt be reached.");
	VisualizerCommon::callback(msg);
}
void Visualizers::IkVis::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& msg)
{
	ROS_ERROR_STREAM("from the dynamic_cast part, this shouldnt be reached, filtered.");
	VisualizerCommon::callback_filtered(msg);
}
