#ifndef PIPELINE_GRFM_HEADER_FBK_01062022
#define PIPELINE_GRFM_HEADER_FBK_01062022

#include "opensimrt_msgs/CommonTimed.h"
#include "experimental/GRFMPrediction.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include <Common/TimeSeriesTable.h>
#include "ros/service_server.h"
#include "std_srvs/Empty.h"
#include "Pipeline/include/common_node.h"

namespace Pipeline
{
	class Grf:public Pipeline::CommonNode
	{
		public:
		//TimeSeriesTable qTable;
		//int simulationLoops;
		//int loopCounter;
		OpenSimRT::GaitPhaseDetector* detector;
		OpenSimRT::GRFMPrediction* grfm;
		double sumDelayMS;
		int sumDelayMSCounter;
		//int i;
		OpenSimRT::InverseDynamics* id;
		OpenSimRT::BasicModelVisualizer* visualizer;
		OpenSimRT::ForceDecorator* rightGRFDecorator;
		OpenSimRT::ForceDecorator* leftGRFDecorator;
		OpenSim::TimeSeriesTable* grfRightLogger;
		OpenSim::TimeSeriesTable* grfLeftLogger;
		OpenSim::TimeSeriesTable* tauLogger;
		SimTK::Vec3 grfOrigin;
		int counter;
		double previousTime, previousTimeDifference;

			Grf();
			~Grf();
			void onInit(); 
		
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message) ;
			void finish(); 
			bool see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			void write_();
			OpenSimRT::LowPassSmoothFilter* filter;
			std::string subjectDir;
	
			//virtual void detectorFunction();	
	};
}
#endif

