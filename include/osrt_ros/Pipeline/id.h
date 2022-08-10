#ifndef PIPELINE_ID_HEADER_FBK_27072022
#define PIPELINE_ID_HEADER_FBK_27072022

#include "InverseDynamics.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>

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
			OpenSimRT::ForceDecorator* rightGRFDecorator;
			OpenSimRT::ForceDecorator* leftGRFDecorator;
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
			virtual bool usesVisualizarFromId() { return true;}
	};

}
#endif
