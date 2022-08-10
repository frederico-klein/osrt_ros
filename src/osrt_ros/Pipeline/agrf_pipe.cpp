#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/agrf_pipe.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::Acc::Acc()
{
	// subject data
	INIReader ini(INI_FILE);
	subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
	auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
	//auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

	auto grfRightApplyBody =
		ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
	auto grfRightForceExpressed =
		ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
	auto grfRightPointExpressed =
		ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
	auto grfRightPointIdentifier =
		ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
	auto grfRightForceIdentifier =
		ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
	auto grfRightTorqueIdentifier =
		ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

	auto grfLeftApplyBody =
		ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
	auto grfLeftForceExpressed =
		ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
	auto grfLeftPointExpressed =
		ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
	auto grfLeftPointIdentifier =
		ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
	auto grfLeftForceIdentifier =
		ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
	auto grfLeftTorqueIdentifier =
		ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");
	grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", Vec3(0));

	// repeat cyclic motion X times
	//simulationLoops = ini.getInteger(section, "SIMULATION_LOOPS", 0);
	// remove last N samples in motion for smooth transition between loops
	auto removeNLastRows =
		ini.getInteger(section, "REMOVE_N_LAST_TABLE_ROWS", 0);

	// filter
	auto memory = ini.getInteger(section, "MEMORY", 0);
	auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
	auto delay = ini.getInteger(section, "DELAY", 0);
	auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

	// acceleration-based detector parameters
	auto heelAccThreshold = ini.getReal(section, "HEEL_ACC_THRESHOLD", 0);
	auto toeAccThreshold = ini.getReal(section, "TOE_ACC_THRESHOLD", 0);
	auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
	auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
	auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
	auto rHeelLocation =
		ini.getSimtkVec(section, "RIGHT_HEEL_LOCATION_IN_FOOT", Vec3(0));
	auto lHeelLocation =
		ini.getSimtkVec(section, "LEFT_HEEL_LOCATION_IN_FOOT", Vec3(0));
	auto rToeLocation =
		ini.getSimtkVec(section, "RIGHT_TOE_LOCATION_IN_FOOT", Vec3(0));
	auto lToeLocation =
		ini.getSimtkVec(section, "LEFT_TOE_LOCATION_IN_FOOT", Vec3(0));
	auto accLPFilterFreq = ini.getInteger(section, "ACC_LP_FILTER_FREQ", 0);
	auto velLPFilterFreq = ini.getInteger(section, "VEL_LP_FILTER_FREQ", 0);
	auto posLPFilterFreq = ini.getInteger(section, "POS_LP_FILTER_FREQ", 0);
	auto accLPFilterOrder = ini.getInteger(section, "ACC_LP_FILTER_ORDER", 0);
	auto velLPFilterOrder = ini.getInteger(section, "VEL_LP_FILTER_ORDER", 0);
	auto posLPFilterOrder = ini.getInteger(section, "POS_LP_FILTER_ORDER", 0);
	auto posDiffOrder = ini.getInteger(section, "POS_DIFF_ORDER", 0);
	auto velDiffOrder = ini.getInteger(section, "VEL_DIFF_ORDER", 0);

	// grfm parameters
	auto grfmMethod = ini.getString(section, "METHOD", "");
	auto pelvisBodyName = ini.getString(section, "PELVIS_BODY_NAME", "");
	auto rHeelCoPLocation =
		ini.getSimtkVec(section, "RIGHT_HEEL_STATION_LOCATION", Vec3(0));
	auto lHeelCoPLocation =
		ini.getSimtkVec(section, "LEFT_HEEL_STATION_LOCATION", Vec3(0));
	auto rToeCoPLocation =
		ini.getSimtkVec(section, "RIGHT_TOE_STATION_LOCATION", Vec3(0));
	auto lToeCoPLocation =
		ini.getSimtkVec(section, "LEFT_TOE_STATION_LOCATION", Vec3(0));
	auto directionWindowSize =
		ini.getInteger(section, "DIRECTION_WINDOW_SIZE", 0);

	// setup model
	Object::RegisterType(Thelen2003Muscle());
	Model model(modelFile);
	model.initSystem();

	// setup external forces parameters
	ExternalWrench::Parameters grfRightFootPar{
		grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
	auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
			grfRightPointIdentifier, grfRightForceIdentifier,
			grfRightTorqueIdentifier);
	auto grfRightLoggerTemp = ExternalWrench::initializeLogger();
	grfRightLogger = &grfRightLoggerTemp;

	ExternalWrench::Parameters grfLeftFootPar{
		grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
	auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
			grfLeftPointIdentifier, grfLeftForceIdentifier,
			grfLeftTorqueIdentifier);
	auto grfLeftLoggerTemp = ExternalWrench::initializeLogger();
	grfLeftLogger = &grfLeftLoggerTemp;

	output_labels.insert(output_labels.end(),grfRightLabels.begin(),grfRightLabels.end());
	output_labels.insert(output_labels.end(),grfLeftLabels.begin(),grfLeftLabels.end());


	vector<ExternalWrench::Parameters> wrenchParameters;
	wrenchParameters.push_back(grfRightFootPar);
	wrenchParameters.push_back(grfLeftFootPar);

	// get kinematics as a table with ordered coordinates
	//qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
	//		model, ikFile, 0.01);

	// remove last rows in qTable
	//for (int i = 0; i < removeNLastRows; ++i)
	//	qTable.removeRow(qTable.getIndependentColumn().back());

	// setup filters
	LowPassSmoothFilter::Parameters filterParam;

	filterParam.numSignals = model.getNumCoordinates();
	filterParam.memory = memory;
	filterParam.delay = delay;
	filterParam.cutoffFrequency = cutoffFreq;
	filterParam.splineOrder = splineOrder;
	filterParam.calculateDerivatives = true;
	filter = new LowPassSmoothFilter(filterParam);

	// acceleration-based event detector
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
	detector = new AccelerationBasedPhaseDetector(model, detectorParameters);

	// grfm prediction
	GRFMPrediction::Parameters grfmParameters;
	grfmParameters.method = GRFMPrediction::selectMethod(grfmMethod);
	grfmParameters.pelvisBodyName = pelvisBodyName;
	grfmParameters.rStationBodyName = rFootBodyName;
	grfmParameters.lStationBodyName = lFootBodyName;
	grfmParameters.rHeelStationLocation = rHeelCoPLocation;
	grfmParameters.lHeelStationLocation = lHeelCoPLocation;
	grfmParameters.rToeStationLocation = rToeCoPLocation;
	grfmParameters.lToeStationLocation = lToeCoPLocation;
	grfmParameters.directionWindowSize = directionWindowSize;
	
	grfm = new GRFMPrediction(model, grfmParameters, detector);

	// id
	id = new InverseDynamics(model, wrenchParameters);
	auto tauLoggerTemp = id->initializeLogger();
	tauLogger = &tauLoggerTemp;

	// visualizer
	visualizer = new BasicModelVisualizer(model);
	rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
	visualizer->addDecorationGenerator(rightGRFDecorator);
	leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
	visualizer->addDecorationGenerator(leftGRFDecorator);

	// mean delay
	sumDelayMS = 0;
	sumDelayMSCounter = 0;

	//loopCounter = 0;
	//i = 0;
	counter = 0;
}

Pipeline::Acc::~Acc()
{
	ROS_INFO_STREAM("Shutting down Acc");
}


