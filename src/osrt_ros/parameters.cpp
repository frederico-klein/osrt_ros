/**
 * @author      : $USER ($USER@cbe0486182c4)
 * @file        : parameters
 * @created     : Wednesday Jul 05, 2023 14:50:53 UTC
 */

#include "osrt_ros/parameters.h"
#include "InverseDynamics.h"
#include "MuscleOptimization.h"
#include "SignalProcessing.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "ros/node_handle.h"
#include <SimTKcommon/SmallMatrix.h>
#include <vector>

OpenSimRT::MuscleOptimization::OptimizationParameters pars::getparamSO(ros::NodeHandle nh)
{
	auto convergenceTolerance 	= nh.param<double>	("convergence_tolerance",0.);
	auto memoryHistory 		= nh.param<int>		("memory_history"	,0);
	auto maximumIterations 		= nh.param<int>		("maximum_iterations"	,0);
	auto objectiveExponent		= nh.param<int>		("objective_exponent"	,0);
	auto o_par = OpenSimRT::MuscleOptimization::OptimizationParameters();
	o_par.convergenceTolerance = convergenceTolerance;
	o_par.maximumIterations = maximumIterations;
	o_par.memoryHistory = memoryHistory;
	o_par.objectiveExponent = objectiveExponent;
	return o_par;
}
//TODO: consider using the same filter ger param for the IK filter or the GRFM or 2 functions
OpenSimRT::LowPassSmoothFilter::Parameters pars::getparamFilterIK(ros::NodeHandle nh, int num_signals)
{
	int memory;
	nh.param<int>("memory", memory, 0);
	double cutoffFreq; 
	nh.param<double>("cutoff_freq", cutoffFreq, 0);
	int delay;
	nh.param<int>("delay", delay, 0);
	int splineOrder;
	nh.param<int>("spline_order", splineOrder, 0);
	OpenSimRT::LowPassSmoothFilter::Parameters ikFilterParam;

	ROS_INFO_STREAM("FILTER PARAMS:\nmemory: " << memory << "cutoffFreq: " << cutoffFreq << "delay: " << delay << "splineOrder: " << splineOrder);
	//ikFilterParam.numSignals = model.getNumCoordinates();
	ikFilterParam.numSignals = num_signals;
	ikFilterParam.memory = memory;
	ikFilterParam.delay = delay;
	ikFilterParam.cutoffFrequency = cutoffFreq;
	ikFilterParam.splineOrder = splineOrder;
	ikFilterParam.calculateDerivatives = true;
	return ikFilterParam;
};
OpenSimRT::LowPassSmoothFilter::Parameters pars::getparamFilterGRFM(ros::NodeHandle nh)
{
	//Params for GFRM filter
	int memory;
	nh.param<int>("memory", memory, 0);
	double cutoffFreq; 
	nh.param<double>("cutoff_freq", cutoffFreq, 0);
	int delay;
	nh.param<int>("delay", delay, 0);
	int splineOrder;
	nh.param<int>("spline_order", splineOrder, 0);
	OpenSimRT::LowPassSmoothFilter::Parameters grfFilterParam;
	grfFilterParam.numSignals = 9;
	grfFilterParam.memory = memory;
	grfFilterParam.delay = delay;
	grfFilterParam.cutoffFrequency = cutoffFreq;
	grfFilterParam.splineOrder = splineOrder;
	grfFilterParam.calculateDerivatives = false;
	return grfFilterParam;
}

std::vector<std::string> pars::getparamGRFMLabels(ros::NodeHandle nh, std::string grf_name_prefix)
{
	//TODO: the grf_right should be replaced by a string so we can have multiple GRFM inputs and reuse this
	///These only make sense for reading a file, I think so maybe they should be in bridge?
	//GFRM params
	std::string grfPointIdentifier;
	nh.param<std::string>(grf_name_prefix + "_point_identifier", grfPointIdentifier, "");
	std::string grfForceIdentifier;
	nh.param<std::string>(grf_name_prefix + "_force_identifier", grfForceIdentifier, "");
	std::string grfTorqueIdentifier;
	nh.param<std::string>(grf_name_prefix + "_torque_identifier", grfTorqueIdentifier, "");
	return OpenSimRT::ExternalWrench::createGRFLabelsFromIdentifiers(grfPointIdentifier, grfForceIdentifier,grfTorqueIdentifier);
}
OpenSimRT::ExternalWrench::Parameters pars::getparamWrench(ros::NodeHandle nh, std::string grf_name_prefix)
{
	//TODO: the grf_right should be replaced by a string so we can have multiple GRFM inputs and reuse this
	///These only make sense for reading a file, I think so maybe they should be in bridge?
	//GFRM params
	std::string grfApplyBody;
	nh.param<std::string>(grf_name_prefix + "_apply_to_body", grfApplyBody, "");
	std::string grfForceExpressed;
	nh.param<std::string>(grf_name_prefix + "_force_expressed_in_body", grfForceExpressed, "");
	std::string grfPointExpressed;
	nh.param<std::string>(grf_name_prefix + "_point_expressed_in_body", grfPointExpressed, "");
	return OpenSimRT::ExternalWrench::Parameters{grfApplyBody, grfForceExpressed,  grfPointExpressed};
}

OpenSimRT::AccelerationBasedPhaseDetector::Parameters pars::getparamAccPhaseDetector(ros::NodeHandle nh) { 
	//TODO: write down all the get param statements here
	//TODO: replace wherever this appears
	// acceleration-based event detector
	/*
	   AccelerationBasedPhaseDetector::Parameters detectorParameters;
	   detectorParameters.heelAccThreshold = heelAccThreshold;
	   detectorParameters.toeAccThreshold = toeAccThreshold;
	   detectorParameters.windowSize = windowSize;
	   detectorParameters.rFootBodyName = rFootBodyName;
	   detectorParameters.lFootBodyName = lFootBodyName;
	   detectorParameters.rHeelLocationInFoot = rHeelLocation;
	   detectorParameters.lHeelLocationInFoot = lHeelLocation;
	   detectorParameters.rToeLocationInFoot = rToeLocation;
	   detectorParameters.lToeLocationInFoot = lToeLocation;
	   detectorParameters.samplingFrequency = 1 / 0.01;
	   detectorParameters.accLPFilterFreq = accLPFilterFreq;
	   detectorParameters.velLPFilterFreq = velLPFilterFreq;
	   detectorParameters.posLPFilterFreq = posLPFilterFreq;
	   detectorParameters.accLPFilterOrder = accLPFilterOrder;
	   detectorParameters.velLPFilterOrder = velLPFilterOrder;
	   detectorParameters.posLPFilterOrder = posLPFilterOrder;
	   detectorParameters.posDiffOrder = posDiffOrder;
	   detectorParameters.velDiffOrder = velDiffOrder;
	   */
	return OpenSimRT::AccelerationBasedPhaseDetector::Parameters();
};

OpenSimRT::ContactForceBasedPhaseDetector::Parameters pars::getparamConPhaseDetector(ros::NodeHandle nh) {
	//TODO:: implement!
	/*
	   ContactForceBasedPhaseDetector::Parameters detectorParameters;
	   detectorParameters.threshold = threshold;
	   detectorParameters.windowSize = windowSize;
	   detectorParameters.plane_origin = Vec3(0.0, platform_offset, 0.0);
	   detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
	   detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
	   detectorParameters.rToeSphereLocation = rToeSphereLocation;
	   detectorParameters.lToeSphereLocation = lToeSphereLocation;
	   detectorParameters.sphereRadius = contactSphereRadius;
	   detectorParameters.rFootBodyName = rFootBodyName;
	   detectorParameters.lFootBodyName = lFootBodyName;
	 * 
	 */
	return OpenSimRT::ContactForceBasedPhaseDetector::Parameters();

}

void pars::getSimtkVec(ros::NodeHandle nh, std::string name, SimTK::Vec3 &vec){

	std::vector<double> tempVec;
	nh.getParam(name, tempVec);
	if (tempVec.size()!=3)
		ROS_ERROR_STREAM("Cannot assign vecs of different sizes! 3 !="<< tempVec.size() );
	vec[0] = tempVec[0];
	vec[1] = tempVec[1];
	vec[2] = tempVec[2];
}

OpenSimRT::GRFMPrediction::Parameters pars::getparamGRFM(ros::NodeHandle nh)
{

	auto grfmMethod = nh.param<std::string>("method","");
	auto  pelvisBodyName   		= nh.param<std::string>	("pelvis_body_name"	,"");
	auto  rFootBodyName    		= nh.param<std::string>	("r_foot_body_name"	,"");
	auto  lFootBodyName    		= nh.param<std::string>	("l_foot_body_name"	,"");

	SimTK::Vec3 rHeelCoPLocation,lHeelCoPLocation, rToeCoPLocation, lToeCoPLocation;
	getSimtkVec(	nh, "r_heel_cop_location"	,	  rHeelCoPLocation );
	getSimtkVec(	nh, "l_heel_cop_location"	,	  lHeelCoPLocation );
	getSimtkVec(	nh, "r_toe_cop_location"	,	  rToeCoPLocation  );
	getSimtkVec(	nh, "l_toe_cop_location"	,	  lToeCoPLocation  );
	auto  directionWindowSize 	= nh.param<int>		("direction_window_size",0);

	OpenSimRT::GRFMPrediction::Parameters grfmParameters;
	grfmParameters.method = OpenSimRT::GRFMPrediction::selectMethod(grfmMethod);
	grfmParameters.pelvisBodyName = pelvisBodyName;
	grfmParameters.rStationBodyName = rFootBodyName;
	grfmParameters.lStationBodyName = lFootBodyName;
	grfmParameters.rHeelStationLocation = rHeelCoPLocation;
	grfmParameters.lHeelStationLocation = lHeelCoPLocation;
	grfmParameters.rToeStationLocation = rToeCoPLocation;
	grfmParameters.lToeStationLocation = lToeCoPLocation;
	grfmParameters.directionWindowSize = directionWindowSize;
	return grfmParameters;

}
