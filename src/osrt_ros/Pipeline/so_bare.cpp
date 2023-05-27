#include "opensimrt_msgs/CommonTimed.h"
#include "ros/ros.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "osrt_ros/Pipeline/so_bare.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::SoBare::SoBare()  
{
	ROS_DEBUG_STREAM("constructor of SoBare");
	cout << "Warning" << endl
		<< "This test might fail on different machines. "
		<< "The performance of the optimization depends on the underlying OS. "
		<< "We think it has to do with how threads are scheduled by the OS. "
		<< "We did not observed this behavior with OpenSim v3.3." << endl
		<< endl;

	// subject data
	INIReader ini(INI_FILE);
	auto section = "TEST_SO_FROM_FILE";
	auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
	//auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
	std::string modelFile = "";
	nh.param<std::string>("model_file",modelFile,"");
	

	//auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
	//auto idFile = subjectDir + ini.getString(section, "ID_FILE", "");

	// Windows places executables in different folders. When ctest is
	// called on a Linux machine it runs the test from different
	// folders and thus the dynamic library might not be found
	// properly.
	//#ifndef WIN32
	auto momentArmLibraryPath =
		LIBRARY_OUTPUT_PATH + "/" +
		ini.getString(section, "MOMENT_ARM_LIBRARY", "");
	ROS_DEBUG_STREAM("momentArmLibraryPath:" << momentArmLibraryPath);
	/*#else
	  auto momentArmLibraryPath =
	  ini.getString(section, "MOMENT_ARM_LIBRARY", "");
#endif*/

	//auto memory = ini.getInteger(section, "MEMORY", 0);
	//auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
	//auto delay = ini.getInteger(section, "DELAY", 0);
	//auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

	auto convergenceTolerance =
		ini.getReal(section, "CONVERGENCE_TOLERANCE", 0);
	auto memoryHistory = ini.getReal(section, "MEMORY_HISTORY", 0);
	auto maximumIterations = ini.getInteger(section, "MAXIMUM_ITERATIONS", 0);
	auto objectiveExponent = ini.getInteger(section, "OBJECTIVE_EXPONENT", 0);

	Object::RegisterType(Thelen2003Muscle());
	model = new Model(modelFile);
	model->initSystem();

	ROS_DEBUG_STREAM("registered model okay.");
	// load and verify moment arm function
	auto calcMomentArmTemp = OpenSimUtils::getMomentArmFromDynamicLibrary(
			*model, momentArmLibraryPath);
	ROS_DEBUG_STREAM("initialized calcMomentArmTemp from dynamic library ok.");

	calcMomentArm = calcMomentArmTemp;

	ROS_DEBUG_STREAM("initialized MomentArm from dynamic library ok.");
	// get kinematics as a table with ordered coordinates
	//auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
	//        model, ikFile, 0.01);

	// read external forces
	//auto tauTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
	//        model, idFile, 0.01);

	/*if (tauTable.getNumRows() != qTable.getNumRows()) {
	  THROW_EXCEPTION("ik and id storages of different size " +
	  toString(qTable.getNumRows()) +
	  " != " + toString(tauTable.getNumRows()));
	  }*/

	// initialize so
	//    MuscleOptimization::OptimizationParameters optimizationParameters;
	optimizationParameters.convergenceTolerance = convergenceTolerance;
	optimizationParameters.memoryHistory = memoryHistory;
	optimizationParameters.maximumIterations = maximumIterations;
	optimizationParameters.objectiveExponent = objectiveExponent;
	ROS_DEBUG_STREAM("set parameter for optimizer okay.");
	// auto tauResLogger = so.initializeResidualLogger();
	// mean delay
	//int sumDelayMS = 0;

	so = new MuscleOptimization(*model, optimizationParameters, calcMomentArm);
	ROS_DEBUG_STREAM("initialized MuscleOptimization okay.");
	//so = &so_temp;
	ROS_DEBUG_STREAM("SO fake constructor ran ok.");
}
Pipeline::SoBare::~SoBare()
{

	ROS_INFO_STREAM("Shutting down SoBare");
}

void Pipeline::SoBare::onInit() {
	ROS_DEBUG_STREAM("onInitSoBare bare");

	visualizer = new BasicModelVisualizer(*model);
	//these need to be shared with the rest:
	fmLogger = so->initializeMuscleLogger();
	amLogger = so->initializeMuscleLogger();
}

opensimrt_msgs::CommonTimed Pipeline::SoBare::run(const std_msgs::Header h, double t, SimTK::Vector q, std::vector<double> Rtau, opensimrt_msgs::Events e )
{
	ROS_DEBUG_STREAM("Received run call. Running SoBare loop"); 
	opensimrt_msgs::CommonTimed msg_out;
	msg_out.header = h;
	//unpacking tau
 	SimTK::Vector tau(Rtau.size());
	for (int j=0; j<tau.size();j++)
	{
		tau[j] = Rtau[j];
	}


	ROS_DEBUG_STREAM("attempting to call SO.");
	if (!so)
	{
		ROS_ERROR_STREAM("so not initialized!");
		return msg_out;
	}
	ROS_DEBUG_STREAM("attempting to run SO.");
	//ROS_DEBUG_STREAM("t: ["<< t << "] q: [" << q << "] tau: [" << tau << "]");
	auto soOutput = so->solve({t, q, tau});
	addEvent("id_combined after so",e);
	chrono::high_resolution_clock::time_point t2;
	t2 = chrono::high_resolution_clock::now();

	//TODO: actually capture return params from soOutput!!!
	msg_out.events = e;
	//pub.publish(msg_out);
	return msg_out;
	try {

		visualizer->update(q, soOutput.am);
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}

	try{

		// log data (use filter time to align with delay)
		if(false)
		{
			ROS_WARN_STREAM("THIS SHOULDNT BE RUN.");
			// loggers from SO
			// log data (use filter time to align with delay)
			fmLogger.appendRow(t, ~soOutput.fm);
			amLogger.appendRow(t, ~soOutput.am);

		}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}

}


