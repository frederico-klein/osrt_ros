#include "grfm_ros.h"
#include "opensimrt_msgs/CommonTimed.h"
//#include "Settings.h"
#include "INIReader.h"
#include "SignalProcessing.h"
#include <Actuators/Thelen2003Muscle.h>

Gfrm::Gfrm(){


};

void Gfrm::init(){
//so the order is as such that this needs to be called after I defined the section variable, so it looks weird. 

	// subject data
	//INIReader ini(INI_FILE);
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
	grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", SimTK::Vec3(0));

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
	
	// grfm parameters
	auto grfmMethod = ini.getString(section, "METHOD", "");
	auto pelvisBodyName = ini.getString(section, "PELVIS_BODY_NAME", "");
	auto rHeelCoPLocation =
		ini.getSimtkVec(section, "RIGHT_HEEL_STATION_LOCATION", SimTK::Vec3(0));
	auto lHeelCoPLocation =
		ini.getSimtkVec(section, "LEFT_HEEL_STATION_LOCATION", SimTK::Vec3(0));
	auto rToeCoPLocation =
		ini.getSimtkVec(section, "RIGHT_TOE_STATION_LOCATION", SimTK::Vec3(0));
	auto lToeCoPLocation =
		ini.getSimtkVec(section, "LEFT_TOE_STATION_LOCATION", SimTK::Vec3(0));
	auto directionWindowSize =
		ini.getInteger(section, "DIRECTION_WINDOW_SIZE", 0);

	// setup model
	OpenSim::Object::RegisterType(OpenSim::Thelen2003Muscle());

	OpenSim::Model model(modelFile);
	model.initSystem();

	// setup external forces parameters
	OpenSimRT::ExternalWrench::Parameters grfRightFootPar{
		grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
	auto grfRightLabels = OpenSimRT::ExternalWrench::createGRFLabelsFromIdentifiers(
			grfRightPointIdentifier, grfRightForceIdentifier,
			grfRightTorqueIdentifier);
	auto grfRightLogger = OpenSimRT::ExternalWrench::initializeLogger();

	OpenSimRT::ExternalWrench::Parameters grfLeftFootPar{
		grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
	auto grfLeftLabels = OpenSimRT::ExternalWrench::createGRFLabelsFromIdentifiers(
			grfLeftPointIdentifier, grfLeftForceIdentifier,
			grfLeftTorqueIdentifier);
	auto grfLeftLogger = OpenSimRT::ExternalWrench::initializeLogger();

	std::vector<OpenSimRT::ExternalWrench::Parameters> wrenchParameters;
	wrenchParameters.push_back(grfRightFootPar);
	wrenchParameters.push_back(grfLeftFootPar);

	// get kinematics as a table with ordered coordinates
	//qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
	//		model, ikFile, 0.01);

	// remove last rows in qTable
	//for (int i = 0; i < removeNLastRows; ++i)
	//	qTable.removeRow(qTable.getIndependentColumn().back());

	// setup filters
	OpenSimRT::LowPassSmoothFilter::Parameters filterParam;
	filterParam.numSignals = model.getNumCoordinates();
	filterParam.memory = memory;
	filterParam.delay = delay;
	filterParam.cutoffFrequency = cutoffFreq;
	filterParam.splineOrder = splineOrder;
	filterParam.calculateDerivatives = true;
	filter = new OpenSimRT::LowPassSmoothFilter(filterParam);

	auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
	auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
	auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
	
	// grfm prediction
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
	
	grfm = new OpenSimRT::GRFMPrediction(model, grfmParameters, detector);

	// id
	id = new OpenSimRT::InverseDynamics(model, wrenchParameters);
	tauLogger = id->initializeLogger();

	// visualizer
	visualizer = new OpenSimRT::BasicModelVisualizer(model);
	rightGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(rightGRFDecorator);
	leftGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(leftGRFDecorator);

	// mean delay
	sumDelayMS = 0;
	sumDelayMSCounter = 0;

	//loopCounter = 0;
	//i = 0;
	previousTime = ros::Time::now().toSec();
	previousTimeDifference = 0;

}; 

void Gfrm::finish(){};

void Gfrm::operator() (const opensimrt_msgs::CommonTimedConstPtr&) {};

