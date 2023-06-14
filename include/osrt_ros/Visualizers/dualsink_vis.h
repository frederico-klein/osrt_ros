#ifndef VISUALIZER_DUALSINK_HEADER_FBK_26052023
#define VISUALIZER_DUALSINK_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "osrt_ros/Visualizers/visualizer_common.h"
#include "ros/node_handle.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_sequencer.h"

namespace Visualizers
{

	class DualSinkVis:public Visualizers::VisualizerCommon
	{
		public:
			DualSinkVis(): seq(sub_multi, ros::Duration(0.1), ros::Duration(0.005), 1000) {}
			virtual ~DualSinkVis() {}
			message_filters::Subscriber<opensimrt_msgs::MultiMessage> sub_multi;
			message_filters::TimeSequencer<opensimrt_msgs::MultiMessage> seq;
			virtual void callback_multi(const opensimrt_msgs::MultiMessageConstPtr& message) {};
			virtual void callback(const opensimrt_msgs::DualConstPtr& message);
			virtual void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 

			virtual void before_vis()
			{
				ROS_INFO_STREAM("adding subscribers");
				ros::NodeHandle nh("~");
				sub = nh.subscribe("dual_input",1, &Visualizers::DualSinkVis::callback, this);

				sub_filtered = nh.subscribe("dual_input_filtered",1, &Visualizers::DualSinkVis::callback_filtered, this);
				sub_multi.subscribe(nh, "multi_input", 1);
				seq.registerCallback(&Visualizers::DualSinkVis::callback_multi, this);
				ROS_INFO_STREAM("added subscribers ok.");
				
			}
			virtual void after_callback() {};
	};

}
#endif

