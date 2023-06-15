#ifndef PIPELINE_ID_HEADER_FBK_27072022
#define PIPELINE_ID_HEADER_FBK_27072022

#include "osrt_ros/Pipeline/id_common.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "message_filters/sync_policies/exact_time.h"
#include "ros/message_traits.h"
#include "std_srvs/Empty.h"
#include "tf2_ros/transform_listener.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef message_filters::sync_policies::ExactTime<opensimrt_msgs::CommonTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped > grf_exact_wrench_policy;
typedef message_filters::sync_policies::ExactTime<opensimrt_msgs::PosVelAccTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped > grf_exact_wrench_filtered_policy;

typedef message_filters::sync_policies::ApproximateTime<opensimrt_msgs::CommonTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped > grf_approx_wrench_policy;
typedef message_filters::sync_policies::ApproximateTime<opensimrt_msgs::PosVelAccTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped > grf_approx_wrench_filtered_policy;

namespace Pipeline
{

	class Id:public Pipeline::IdCommon::IdCommon
	{
		public:
			Id();
			~Id();
			std_msgs::Header::_stamp_type last_received_ik_stamp;
			std::string left_foot_tf_name, right_foot_tf_name, grf_reference_frame;
			// now since I have 2 sinks I will need message_filters
			
			//NOW get real ros wrenches:
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener;
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wl; //input3
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wr; //input4
			
			virtual void callback_wl(const geometry_msgs::WrenchConstPtr& wl_msg)
			{ ROS_ERROR_STREAM("callback for wl shouldn't be registerd. getting message though.");};
			virtual void callback_wr(const geometry_msgs::WrenchConstPtr& wr_msg)
			{ ROS_ERROR_STREAM("callback for wr shouldn't be registerd. getting message though.");};

			//a parameter should set this to either exact or approximate time
			bool use_exact_sync = true;
			message_filters::Synchronizer<grf_exact_wrench_policy> sync_real_wrenches_exact;
			message_filters::Synchronizer<grf_approx_wrench_policy> sync_real_wrenches_aprox;
			//message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_real_wrenches;
			//message_filters::TimeSynchronizer<opensimrt_msgs::PosVelAccTimed, geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_filtered_real_wrenches;
			message_filters::Synchronizer<grf_approx_wrench_filtered_policy> sync_filtered_real_wrenches_aprox;
			message_filters::Synchronizer<grf_exact_wrench_filtered_policy> sync_filtered_real_wrenches_exact;

			void callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, 		const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr);
			
			void callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, 	const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr);
			
			void onInit();

			// now all the stuff I need to save between inInit and the callbacks


			ros::Publisher pub_grf_left, pub_grf_right, pub_ik, pub_cop_left, pub_cop_right; //crazy debug going nuts

			opensimrt_msgs::CommonTimed conv_ik_to_msg(std_msgs::Header h, SimTK::Vector ik);
			opensimrt_msgs::CommonTimed conv_grf_to_msg(std_msgs::Header h, OpenSimRT::ExternalWrench::Input ow);
	std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr, std::string right_foot_tf_name, std::string left_foot_tf_name, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame);

			bool parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frme, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame, OpenSimRT::ExternalWrench::Input* wO );


	};

}
#endif
