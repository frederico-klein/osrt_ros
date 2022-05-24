//#include "ros/ros.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GaitPhaseDetector.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include <Common/TimeSeriesTable.h>

#ifndef GRFMROS_H
#define GRFMROS_H

class Gfrm
{
	//TimeSeriesTable qTable;
	//int simulationLoops;
	//int loopCounter;
	OpenSimRT::LowPassSmoothFilter* filter;
	
	OpenSimRT::GaitPhaseDetector* detector;
	
	OpenSimRT::GRFMPrediction* grfm;
	double sumDelayMS;
	int sumDelayMSCounter;
	//int i;
	OpenSimRT::InverseDynamics* id;

	OpenSimRT::BasicModelVisualizer* visualizer;
	OpenSimRT::ForceDecorator* rightGRFDecorator;
	OpenSimRT::ForceDecorator* leftGRFDecorator;
	OpenSim::TimeSeriesTable grfRightLogger;
	OpenSim::TimeSeriesTable grfLeftLogger;
	OpenSim::TimeSeriesTable tauLogger;
	SimTK::Vec3 grfOrigin;
	std::string subjectDir;

	double previousTime, previousTimeDifference;

	public:
		Gfrm();
		void operator() (const opensimrt_msgs::CommonTimedConstPtr& message);	
		void finish() ;
};

class Fc : public Gfrm 
{
	OpenSimRT::ContactForceBasedPhaseDetector* detector;
};

class Acc : public Gfrm 
{
	OpenSimRT::AccelerationBasedPhaseDetector* detector;
};



#endif
