#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "Ros/include/common_node.h"
#include "message_filters/synchronizer.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service.h"
#include "ros/subscriber.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

Pipeline::DualSink::DualSink(bool debug): Ros::CommonNode::CommonNode(debug), sync(sub,sub2,10), sync_filtered(sub_filtered,sub2,10)
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

	sub.subscribe(nh, "input",100);
	sub_filtered.subscribe(nh, "input_filtered",100);

	ros::Rate r(1);
	opensimrt_msgs::LabelsSrv l;
	//while(!ros::service::call("in_labels", l))
	if (get_second_label)
	{
		sub2.subscribe(nh, "input2",100);
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


	ROS_INFO_STREAM("Finished onInit from DualSink");
}
