/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : utils
 * @created     : Saturday May 27, 2023 19:28:28 CEST
 */
#include "ros/ros.h"
#include "osrt_ros/utils.h"
#include "opensimrt_msgs/CommonTimed.h"
opensimrt_msgs::PosVelAccTimed get_as_ik_filtered_msg(std_msgs::Header h, double t, SimTK::Vector q, SimTK::Vector qDot, SimTK::Vector qDDot)
{
	opensimrt_msgs::PosVelAccTimed msg_filtered;
	msg_filtered.header = h;
	msg_filtered.time = t;
	//for loop to fill the data appropriately:
	for (int i=0;i<q.size();i++)
	{
		msg_filtered.d0_data.push_back(q[i]);
		msg_filtered.d1_data.push_back(qDot[i]);
		msg_filtered.d2_data.push_back(qDDot[i]);
	}
	return msg_filtered;
}		

opensimrt_msgs::CommonTimed get_as_ik_msg(std_msgs::Header h, double t, SimTK::Vector q)
{
	opensimrt_msgs::CommonTimed msg;
	msg.header = h;
	msg.time = t;

	for (double joint_angle:q)
	{
		ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
		msg.data.push_back(joint_angle);
	}
	return msg;
}
void update_pose(opensimrt_msgs::CommonTimed& msg, double t, SimTK::Vector q)
{
	msg.time = t;

	for (double joint_angle:q)
	{
		ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
		msg.data.push_back(joint_angle);
	}
}


