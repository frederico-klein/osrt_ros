#ifndef PIPELINE_DUALSINK_HEADER_FBK_21072022
#define PIPELINE_DUALSINK_HEADER_FBK_21072022

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/common_node.h"
#include "opensimrt_msgs/PosVelAccTimed.h"

namespace Pipeline
{
	class DualSink:public Pipeline::CommonNode
	{
		public:

			DualSink();
			~DualSink();

			//this is a sink, I think. should be made into a class, but I think it will be complicated, so I didnt do it.
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub2; //input2
										       //ros::Subscriber sub2; //input2
			message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync;
			message_filters::TimeSynchronizer<opensimrt_msgs::PosVelAccTimed, opensimrt_msgs::CommonTimed> sync_filtered;
			std::vector<std::string> input2_labels;
			void onInit();
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message, const opensimrt_msgs::CommonTimedConstPtr& message2) 
			{
				ROS_ERROR_STREAM("dual message callback not implemented!");
			}
			virtual void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message, const opensimrt_msgs::CommonTimedConstPtr& message2) 
			{
				ROS_ERROR_STREAM("dual message callback_filtered not implemented!");
			}
	};
}
#endif

