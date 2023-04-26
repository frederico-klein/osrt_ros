#ifndef PIPELINE_ID_HEADER_FBK_27072022
#define PIPELINE_ID_HEADER_FBK_27072022

#include "InverseDynamics.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/message_traits.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/VectorBase.h>
#include "geometry_msgs/WrenchStamped.h"
#include "tf2_ros/transform_listener.h"
#include "osrt_ros/events.h"
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<opensimrt_msgs::CommonTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped > grf_approx_wrench_policy;

namespace Pipeline
{
	static const geometry_msgs::Quaternion TO_OPENSIM()
	{
		geometry_msgs::Quaternion	q;
		q.x = -0.5;
		q.y = -0.5;
		q.z = -0.5;
		q.w = 0.5;
		return q;
	};
	

	class Id:public Pipeline::DualSink
	{
		public:
			Id();
			~Id();
			std::string left_foot_tf_name, right_foot_tf_name;
			// now since I have 2 sinks I will need message_filters
			void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik); //ik, grf are received at the same time
			void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			
			//NOW get real ros wrenches:
			tf2_ros::Buffer tfBuffer;
			tf2_ros::TransformListener tfListener;
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wl; //input3
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wr; //input4
			
			virtual void callback_wl(const geometry_msgs::WrenchConstPtr& wl_msg)
			{ ROS_ERROR_STREAM("callback for wl shouldn't be registerd. getting message though.");};
			virtual void callback_wr(const geometry_msgs::WrenchConstPtr& wr_msg)
			{ ROS_ERROR_STREAM("callback for wr shouldn't be registerd. getting message though.");};

			message_filters::Synchronizer<grf_approx_wrench_policy> sync_real_wrenches;
			//message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed   , geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_real_wrenches;
			message_filters::TimeSynchronizer<opensimrt_msgs::PosVelAccTimed, geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_filtered_real_wrenches;
			
			void callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, 		const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr);
			
			void callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, 	const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr);
			
			std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr);

			//rest of class:
			std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf);
			virtual void run(const std_msgs::Header h, double t, std::vector<SimTK::Vector> iks, std::vector<OpenSimRT::ExternalWrench::Input> ,  opensimrt_msgs::Events e);

			void onInit();

			// now all the stuff I need to save between inInit and the callbacks

			std::string subjectDir;
			double sumDelayMS, sumDelayMSCounter;
			double previousTime, previousTimeDifference;
			//loggers
			//
			OpenSim::TimeSeriesTable* tauLogger;
			OpenSim::TimeSeriesTable* grfRightLogger; 
			OpenSim::TimeSeriesTable* grfLeftLogger;
			OpenSim::TimeSeriesTable* qLogger;
			OpenSim::TimeSeriesTable* qDotLogger; 
			OpenSim::TimeSeriesTable* qDDotLogger;
			
			//other stuff
			OpenSimRT::InverseDynamics* id;
			OpenSimRT::ForceDecorator *rightGRFDecorator,*leftGRFDecorator;
			OpenSim::Model* model;
			OpenSimRT::BasicModelVisualizer* visualizer;
			OpenSimRT::LowPassSmoothFilter* ikfilter, *grfRightFilter, *grfLeftFilter;
			std::vector<std::string> grfRightLabels, grfLeftLabels;
			
			boost::array<int,9> generateIndexes(std::vector<std::string> pick, std::vector<std::string> whole); 
			boost::array<int,9> grfLeftIndexes, grfRightIndexes;
			void print_wrench(OpenSimRT::ExternalWrench::Input w);

			//do I need this?
			bool use_grfm_filter;
			int counter;
			int memory, delay, splineOrder;
			double cutoffFreq;

			void finish();
			bool see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

			void write_();
			OpenSimRT::ExternalWrench::Input parse_message(const opensimrt_msgs::CommonTimedConstPtr& msg_grf, boost::array<int,9> grfIndexes);
			OpenSimRT::ExternalWrench::Input parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frme);
			std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::CommonTimedConstPtr& message_ik, double* filtered_t);
			std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik);
			virtual bool usesVisualizarFromId() { return true;}

			void print_vec(std::vector<std::string> vs);
	};

}
#endif
