#ifndef PIPELINE_ID_ASYNC_HEADER_FBK_27072022
#define PIPELINE_ID_ASYNC_HEADER_FBK_27072022

#include "message_filters/subscriber.h"
#include "changeable_time_sequencer.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "osrt_ros/Pipeline/id_common.h"
#include "Ros/include/common_node.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "ros/duration.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorBase.h>
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_ros/transform_listener.h"
#include "osrt_ros/events.h"
#include <memory>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "opensimrt_bridge/conversions/message_convs.h"
#include <osrt_ros/delayConfig.h>

namespace Pipeline
{
	class WrenchSubscriber
	{
		public:
			//NOW get real ros wrenches:
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener;
			ros::NodeHandle nh{"~"};

			std::string grf_reference_frame, calcn_frame, foot_tf_name;

			std::string wrench_name_prefix;

			//double add_additional_wrench_delay;//TODO:This should be done at wrench creation or republishing, I think, to make more sense
			//do I need this?
			bool use_grfm_filter;
			bool no_rotation, get_external_orientation;
			int memory, delay, splineOrder;
			double cutoffFreq;
    			ros::ServiceClient ground_orientation_client;

			int max_buffer_length = 1000;
			std::deque<geometry_msgs::WrenchStamped> wrenchBuffer;

			ros::Subscriber sub; 
			//message_filters::Subscriber<geometry_msgs::WrenchStamped> sub; 
			virtual void callback(const geometry_msgs::WrenchStampedConstPtr& msg);
			ros::Publisher pub_grf, pub_cop; //crazy debug going nuts
			const geometry_msgs::WrenchStamped find_wrench_in_buffer(const std_msgs::Header::_stamp_type timestamp);
			bool get_wrench(const std_msgs::Header::_stamp_type time_stamp, OpenSimRT::ExternalWrench::Input* wO );
			void onInit();
			WrenchSubscriber(std::string wrench_name_prefix_, std::string calcn_frame);
			void pub(std_msgs::Header h); //it will just publish the wrench it gets from this timestamp.
	};


	class IdAsync:public Pipeline::IdCommon::IdCommon
	{
		public:
			IdAsync();
			IdAsync(std::shared_ptr<ros::Duration> delay__);
			~IdAsync();
			std::shared_ptr<ros::Duration> ik_delay;
			std_msgs::Header::_stamp_type last_received_ik_stamp;
			std::string left_foot_tf_name, right_foot_tf_name, grf_reference_frame;
			//I need my very super slow delayed subscribers for IK.
			//
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub__;
			message_filters::ChangeableTimeSequencer<opensimrt_msgs::CommonTimed> seq__;
			message_filters::Subscriber<opensimrt_msgs::PosVelAccTimed> sub_filtered__;
			message_filters::ChangeableTimeSequencer<opensimrt_msgs::PosVelAccTimed> seq_filtered__;
			//TODO: we get the grfs from the buffer in this version of the id, so the call is actually correct to have only IK. I think maybe I should just change the name of this function call.
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik) override;

			void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) override; //ik, grf are received at the same time
			void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) override; // thi is the standard grf message which may have many grfs.
												//
			//for the normal grf message we will have a buffer of size 1 and assume we dont have a lot of delay?
			//actually this will not work, not sure if it is worth implementing at all

			WrenchSubscriber wsL, wsR;
			
			std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const std_msgs::Header::_stamp_type timestamp);
			ros::Time convert_time_stamp_to_the_past(const ros::Time ik_t);
			void onInit();

			// now all the stuff I need to save between inInit and the callbacks
			void publish_additional_topics(std_msgs::Header h, SimTK::Vector q, std::vector<OpenSimRT::ExternalWrench::Input> wV) override
			{ 
				//the idea here is to create like a list of named commontimed messages that we can publish, i think, not sure if it is worth implementing now
				ROS_DEBUG_STREAM("publish additional topics not implemented");}

			
			ros::Publisher pub_ik;

			void reconfigure_delay_callback(osrt_ros::delayConfig &config, uint32_t level);

	};

}
#endif
