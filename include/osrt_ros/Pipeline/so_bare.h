#ifndef PIPELINE_SO_BARE_HEADER_FBK_26052023
#define PIPELINE_SO_BARE_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
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

namespace Pipeline
{

	class SoBare
	{
		public:
			SoBare();
			~SoBare();
			ros::NodeHandle nh{"~"};
			opensimrt_msgs::Dual run(const std_msgs::Header h, double t, SimTK::Vector q, const std::vector<double> tau, opensimrt_msgs::Events e );

			void onInit();
			void finish();
			//SO portion:
			OpenSimRT::MuscleOptimization* so;
			OpenSimRT::MuscleOptimization::OptimizationParameters optimizationParameters;
			OpenSim::Model* model;
			OpenSimRT::MomentArmFunctionT calcMomentArm; 

	};

}
#endif

