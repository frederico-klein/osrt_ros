#ifndef PIPELINE_SO_RR_HEADER_FBK_26052023
#define PIPELINE_SO_RR_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/MultiMessagePosVelAcc.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "ros/ros.h"
#include "so.h"
#include <boost/thread.hpp>
#include "osrt_ros/Pipeline/so_bare.h"

namespace Pipeline
{
	class SoRR:public Pipeline::DualSink
	{
		public:
			ros::NodeHandle node_handle_;
			ros::V_Subscriber subs_;
			ros::V_Subscriber subs_filtered;
			ros::Subscriber main_subs_;
			std::vector<ros::Publisher> pubs_;
			std::vector<ros::Publisher> pubs_filtered;
			//hack
			std::vector<Pipeline::SoBare> sos;
			ros::Publisher outcome_pub, outcome_multi_pub;
			int counter = 0;
			int num_processes_;
			SoRR(const ros::NodeHandle& node_handle, const int num_processes);
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) ;
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) ;
			void so_rrCallback(const opensimrt_msgs::DualConstPtr& d, int process);
			void so_rr_filteredCallback(const opensimrt_msgs::DualPosConstPtr& d, int process);
			void sync_callback(const opensimrt_msgs::MultiMessageConstPtr &message);
			void sync_callback_filtered(const opensimrt_msgs::MultiMessagePosVelAccConstPtr &message);

			void init();
			
			//the normal callbacks will be replaced by the roundrobin callback
			//void roundRobin(const std_msgs::String::ConstPtr& msg);
			
			void runRR(const std_msgs::Header h, double t, SimTK::Vector q, const std::vector<double> tau, opensimrt_msgs::Events e , int process);

	};

}
#endif
