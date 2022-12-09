#ifndef PIPELINE_ID_HEADER_FBK_27072022
#define PIPELINE_ID_HEADER_FBK_27072022

#include "InverseDynamics.h"
#include "geometry_msgs/Wrench.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/VectorBase.h>
#include "geometry_msgs/WrenchStamped.h"

namespace Pipeline
{

	class Id:public Pipeline::DualSink
	{
		public:
			Id();
			~Id();
			// now since I have 2 sinks I will need message_filters
			void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik); //ik, grf are received at the same time
			void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time
			
			//NOW get real ros wrenches:
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wl; //input3
			message_filters::Subscriber<geometry_msgs::WrenchStamped> sub_wr; //input4
			
			virtual void callback_wl(const geometry_msgs::WrenchConstPtr& wl_msg)
			{ ROS_ERROR_STREAM("callback for wl shouldn't be registerd. getting message though.");};
			virtual void callback_wr(const geometry_msgs::WrenchConstPtr& wr_msg)
			{ ROS_ERROR_STREAM("callback for wr shouldn't be registerd. getting message though.");};

			message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_real_wrenches;
			message_filters::TimeSynchronizer<opensimrt_msgs::PosVelAccTimed, geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_filtered_real_wrenches;
			
			void callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedPtr& wl, const geometry_msgs::WrenchConstPtr& wr);
			
			void callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedPtr& wl, const geometry_msgs::WrenchConstPtr& wr);

			//rest of class:
			std::vector<OpenSimRT::ExternalWrench::Input> get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf);
			virtual void run(double t, std::vector<SimTK::Vector> iks, std::vector<OpenSimRT::ExternalWrench::Input>  );

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
			int counter;
			int memory, delay, splineOrder;
			double cutoffFreq;

			void finish();
			bool see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

			void write_();
			OpenSimRT::ExternalWrench::Input parse_message(const opensimrt_msgs::CommonTimedConstPtr& msg_grf, boost::array<int,9> grfIndexes);
			OpenSimRT::ExternalWrench::Input parse_message(const geometry_msgs::WrenchStampedConstPtr& w);
			std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::CommonTimedConstPtr& message_ik, double* filtered_t);
			std::vector<SimTK::Vector> parse_ik_message(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik);
			virtual bool usesVisualizarFromId() { return true;}
			
	};

}
#endif
