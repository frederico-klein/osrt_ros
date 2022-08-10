#include "Pipeline/include/dualsink_pipe.h"
#include "common_node.h"
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

Pipeline::DualSink::DualSink(): sync(sub,sub2,10)
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

} //constructor
Pipeline::DualSink::~DualSink()
{
	ROS_INFO_STREAM("Closing DualSink!");
}

void Pipeline::DualSink::onInit()
{
	ROS_INFO_STREAM("called onInit from DualSink");
	Pipeline::CommonNode::onInit(2);

	sub2.subscribe(nh, "input2",100);
    	sub.subscribe(nh, "input",100);

	ros::Rate r(1);
	opensimrt_msgs::LabelsSrv l;
	//while(!ros::service::call("in_labels", l))
	while(ros::ok())
	{
		if(ros::service::call("in_labels2", l))
		{
		    input2_labels = l.response.data;
		    ROS_INFO_STREAM("got input 2 labels!");
		    break;
		}
		ros::spinOnce();
		ROS_INFO_STREAM("Waiting to read input2 labels."); 
		r.sleep();
	}


	//typedef message_filters::sync_policies::ExactTime<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> mEsp;
	//message_filters::Synchronizer<mEsp> sync(mEsp(10), sub, sub2);

	//typedef message_filters::sync_policies::ApproximateTime<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> mAsp;
	//message_filters::Synchronizer<mAsp> sync(mAsp(10), sub, sub2);

	sync.connectInput(sub, sub2);
	//sync.registerCallback(&DualSink::callback, this);
	sync.registerCallback(boost::bind(&DualSink::callback, this, _1, _2));
	//sub2.registerCallback(&DualSink::callback2,this);
	//sub2.registerCallback(&DualSink::callback2,this);
	ROS_INFO_STREAM("registered the dual callback just fine");
	ROS_INFO_STREAM("Finished onInit from DualSink");
}
