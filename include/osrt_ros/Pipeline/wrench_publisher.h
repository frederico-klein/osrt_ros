#ifndef PIPELINE_WRENCH_HEADER_FBK_03052023
#define PIPELINE_WRENCH_HEADER_FBK_03052023

#include "geometry_msgs/WrenchStamped.h"
#include "InverseDynamics.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <tf/transform_broadcaster.h>

class WrenchPub
{

	public: 
		WrenchPub();
		WrenchPub(ros::NodeHandle nh, std::string wrench_frame_name);
		ros::NodeHandle n;
		void onInit();
		std::string wrench_frame = "wrench_frame";
		ros::Publisher wren;
		tf::TransformBroadcaster tb;
		void publish(std_msgs::Header h, OpenSimRT::ExternalWrench::Input w);

};

#endif

