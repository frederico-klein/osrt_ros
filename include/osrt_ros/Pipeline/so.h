#ifndef PIPELINE_SO_HEADER_FBK_26052023
#define PIPELINE_SO_HEADER_FBK_26052023

#include "opensimrt_msgs/Events.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <vector>
#include "MuscleOptimization.h"
#include "osrt_ros/events.h"
#include "dualsink_pipe.h"

namespace Pipeline
{

	class So:public Pipeline::DualSink
	{
		public:
			So();
			~So();
			// now since I have 2 sinks I will need message_filters
			//void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_id); //ik, id are received at the same time
			//void old_callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_id); //ik, id are received at the same time
			void callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik); //ik, id are received at the same time
			void callback1(const opensimrt_msgs::CommonTimedConstPtr& message_id); //ik, id are received at the same time
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_id); //ik, id are received at the same time
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_id); //ik, id are received at the same time
			void run(const std_msgs::Header h, double t, SimTK::Vector q, const std::vector<double> tau, opensimrt_msgs::Events e );

			void onInit();
			void finish();
			//SO portion:
			OpenSimRT::MuscleOptimization* so;
			OpenSim::Model* model;
			OpenSimRT::BasicModelVisualizer* visualizer;
			OpenSimRT::MomentArmFunctionT calcMomentArm; 
			OpenSim::TimeSeriesTable fmLogger;
			OpenSim::TimeSeriesTable amLogger;

	};

}
#endif

