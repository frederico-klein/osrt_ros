/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : conversions.h
 * @created     : Friday Mar 24, 2023 16:04:47 CET
 */

#ifndef CONVERSIONS_H_FBK24032023

#define CONVERSIONS_H_FBK24032023

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Wrench.h"
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Transform.h"
#include "InverseDynamics.h"

namespace Convert
{
struct ROS_WrenchBuilder
{
	geometry_msgs::WrenchPtr wrench;
	geometry_msgs::TransformPtr transform;
};

ROS_WrenchBuilder get_as(OpenSimRT::ExternalWrench::Input o_w)
{
	ROS_WrenchBuilder wb;
	geometry_msgs::WrenchPtr w;
	geometry_msgs::TransformPtr t;
	w->force.x = o_w.force[0];
	w->force.y = o_w.force[1];
	w->force.z = o_w.force[2];
	w->torque.x = o_w.torque[0];
	w->torque.y = o_w.torque[1];
	w->torque.z = o_w.torque[2];
	//possibly i need to get the reference frame and resolve the tf here instead of just setting it
	
	t->translation.x = o_w.point[0];
	t->translation.y = o_w.point[1];
	t->translation.z = o_w.point[2];
	wb.transform = t;
	wb.wrench = w;
	return wb;


}


}


#endif /* end of include guard CONVERSIONS_H_FBK24032023 */


