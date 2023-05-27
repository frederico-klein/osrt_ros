#include "osrt_ros/Visualizers/ik_vis.h"
#include "opensimrt_msgs/PosVelAccTimed.h"

void Visualizers::IkVis::callback(const opensimrt_msgs::CommonTimedConstPtr& msg)
{
	VisualizerCommon::callback(msg);
}
void Visualizers::IkVis::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& msg)
{
	VisualizerCommon::callback_filtered(msg);
}
