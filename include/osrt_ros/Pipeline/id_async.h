#ifndef PIPELINE_ID_ASYNC_HEADER_FBK_27072022
#define PIPELINE_ID_ASYNC_HEADER_FBK_27072022

#include "osrt_ros/Pipeline/id_common.h"
#include "Ros/include/common_node.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/VectorBase.h>
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_ros/transform_listener.h"
#include "osrt_ros/events.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Pipeline
{
	class WrenchSubscriber
	{
		public:
			//NOW get real ros wrenches:
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener;
			ros::NodeHandle nh{"~"};

			std::string grf_reference_frame, ref_frame, calcn_frame, foot_tf_name;

			std::string wrench_name_prefix;
			//do I need this?
			bool use_grfm_filter;
			int memory, delay, splineOrder;
			double cutoffFreq;

			std::vector<geometry_msgs::WrenchStampedConstPtr> wrenchBuffer;

			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub; 
			virtual void callback(const geometry_msgs::WrenchStampedConstPtr& msg);
			ros::Publisher pub_grf, pub_cop; //crazy debug going nuts
			const geometry_msgs::WrenchStampedConstPtr find_wrench_in_buffer(const std_msgs::Header::_stamp_type timestamp);
			bool get_wrench(const std_msgs::Header::_stamp_type time_stamp, OpenSimRT::ExternalWrench::Input* wO );
			void onInit();
			WrenchSubscriber(std::string wrench_name_prefix_, std::string calcn_frame);
			void pub(std_msgs::Header h); //it will just publish the wrench it gets from this timestamp.
	};


	class IdAsync:public Pipeline::IdCommon::IdCommon
	{
		public:
			IdAsync();
			~IdAsync();
			std_msgs::Header::_stamp_type last_received_ik_stamp;
			std::string left_foot_tf_name, right_foot_tf_name, grf_reference_frame;
			// now since I have 2 sinks I will need message_filters
			
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik);

			void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik); //ik, grf are received at the same time
			void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf); // thi is the standard grf message which may have many grfs.
												//
			//for the normal grf message we will have a buffer of size 1 and assume we dont have a lot of delay?
			//actually this will not work, not sure if it is worth implementing at all

			WrenchSubscriber wsL, wsR;
			
			std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const std_msgs::Header::_stamp_type timestamp);

			void onInit();

			// now all the stuff I need to save between inInit and the callbacks

			
			ros::Publisher pub_ik;
			//other stuff
	};

}
#endif
