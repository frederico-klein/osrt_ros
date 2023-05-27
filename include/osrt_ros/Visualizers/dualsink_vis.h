#ifndef VISUALIZER_DUALSINK_HEADER_FBK_26052023
#define VISUALIZER_DUALSINK_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "osrt_ros/Visualizers/visualizer_common.h"
#include "ros/node_handle.h"

namespace Visualizers
{

	class DualSinkVis:public Visualizers::VisualizerCommon
	{
		public:
			DualSinkVis() {}
			~DualSinkVis() {}
			void callback(const opensimrt_msgs::DualConstPtr& message);
			void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 

			virtual void before_vis()
			{
				ROS_INFO_STREAM("adding subscribers");
				ros::NodeHandle nh("~");
				sub = nh.subscribe("input",1, &Visualizers::DualSinkVis::callback, this);

				sub_filtered = nh.subscribe("input_filtered",1, &Visualizers::DualSinkVis::callback_filtered, this);
				ROS_INFO_STREAM("added subscribers ok.");
				
			}
			virtual void after_callback() {};
	};

}
#endif

