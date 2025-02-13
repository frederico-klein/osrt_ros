#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "Ros/include/common_node.h"
#include "message_filters/synchronizer.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "opensimrt_msgs/MultiMessagePosVelAcc.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service.h"
#include "ros/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#define INPUT_LENGTH 10 
#define FILTER_LENGTH 3 
Pipeline::DualSink::DualSink(bool debug): Ros::CommonNode::CommonNode(debug), sync(sub,sub2,FILTER_LENGTH), sync_filtered(sub_filtered,sub2,FILTER_LENGTH)
{
	if(debug)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
			ros::console::notifyLoggerLevelsChanged();
		}
	}
	else
	{
		ROS_INFO_STREAM("Debug flag not set.");
	}

} 

Pipeline::DualSink::~DualSink()
{
	ROS_INFO_STREAM("Closing DualSink!");
}

void Pipeline::DualSink::onInit()
{
	ROS_INFO_STREAM("called onInit from DualSink");
	Ros::CommonNode::onInit(2);

	sub.subscribe(nh, "input",INPUT_LENGTH);
	sub_filtered.subscribe(nh, "input_filtered",INPUT_LENGTH);

	//bypasses all of this if a receiving from an upstream node that did the synchronization already. I still need to know the labels, so there is that.
	sync_input_sub = nh.subscribe("sync_input", INPUT_LENGTH, &DualSink::sync_callback, this);
	sync_input_filtered_sub = nh.subscribe("sync_filtered_input", INPUT_LENGTH, &DualSink::sync_callback_filtered, this);

	ros::Rate r(1);
	opensimrt_msgs::LabelsSrv l;
	//while(!ros::service::call("in_labels", l))
	if (get_second_label)
	{
		sub2.subscribe(nh, "input2",INPUT_LENGTH);
		while(ros::ok())
		{
			if(ros::service::call("in_labels2", l))
			{
				input2_labels = l.response.data;
				ROS_INFO_STREAM("got input 2 labels!");
				// I need the second input label if I want to register the dual callback. Withoit it cannot really read the second message, so in the case I don't get the second label (in other types of dualSink, then I don't want to register this callback either

				//typedef message_filters::sync_policies::ExactTime<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> mEsp;
				//message_filters::Synchronizer<mEsp> sync(mEsp(10), sub, sub2);

				//typedef message_filters::sync_policies::ApproximateTime<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> mAsp;
				//message_filters::Synchronizer<mAsp> sync(mAsp(10), sub, sub2);

				sync.connectInput(sub, sub2);
				//sync.registerCallback(&DualSink::callback, this);
				sync.registerCallback(boost::bind(&DualSink::callback, this, _1, _2));


				//Now the callback for the filtered ik version:
				sync_filtered.connectInput(sub_filtered, sub2);
				//sync_filtered.registerCallback(&DualSink::callback_filtered, this);
				sync_filtered.registerCallback(boost::bind(&DualSink::callback_filtered, this, _1, _2));
				//sub2.registerCallback(&DualSink::callback2,this);
				ROS_INFO_STREAM("registered the dual callback just fine");
				break;
			}
			ros::spinOnce();
			ROS_INFO_STREAM("Waiting to read input2 labels."); 
			r.sleep();
		}
	}
	else
	{
		ROS_WARN_STREAM("Second label not being read. Callbacks for DualSink with CommonTimed messages will not be used.");
	}
	//uncomment to test individual callbacks
	//sub.registerCallback(&DualSink::callback1,this);
	//sub2.registerCallback(&DualSink::callback2,this);

	ROS_INFO_STREAM("Setting up synchronized output for visualizers");
	ROS_WARN_STREAM("Outlabels are not implemented for these outputs, if there is reshuffling, they will not look correct.");
	sync_output = nh.advertise<opensimrt_msgs::Dual>("output_combined", 1);
	sync_output_filtered = nh.advertise<opensimrt_msgs::DualPos>("output_combined_filtered", 1);
	sync_output_multi = nh.advertise<opensimrt_msgs::MultiMessage>("output_multi", 1);
	sync_output_multi_filtered = nh.advertise<opensimrt_msgs::MultiMessagePosVelAcc>("output_multi_filtered", 1);

	ROS_INFO_STREAM("Finished onInit from DualSink");
}
