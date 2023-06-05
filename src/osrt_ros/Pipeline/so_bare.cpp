#include "opensimrt_bridge/conversions/message_convs.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/MultiMessage.h"
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

	// subject data
	INIReader ini(INI_FILE);
	auto section = "TEST_SO_FROM_FILE";
	auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
	//auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
	std::string modelFile = "";
	nh.param<std::string>("model_file",modelFile,"");
	
	auto momentArmLibraryPath =
		LIBRARY_OUTPUT_PATH + "/" +
		ini.getString(section, "MOMENT_ARM_LIBRARY", "");
	ROS_DEBUG_STREAM("momentArmLibraryPath:" << momentArmLibraryPath);

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
	
	optimizationParameters.convergenceTolerance = convergenceTolerance;
	optimizationParameters.memoryHistory = memoryHistory;
	optimizationParameters.maximumIterations = maximumIterations;
	optimizationParameters.objectiveExponent = objectiveExponent;
	ROS_DEBUG_STREAM("set parameter for optimizer okay.");
	// auto tauResLogger = so.initializeResidualLogger();
	// mean delay

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
}

opensimrt_msgs::Dual Pipeline::SoBare::run(const std_msgs::Header h, double t, SimTK::Vector q, std::vector<double> Rtau, opensimrt_msgs::Events e )
{
	ROS_DEBUG_STREAM("Received run call. Running SoBare loop"); 
	//unpacking tau
 	SimTK::Vector tau(Rtau.size());
	for (int j=0; j<tau.size();j++)
	{
		tau[j] = Rtau[j];
	}


	ROS_DEBUG_STREAM("attempting to call SO.");
	if (!so)
	{
		opensimrt_msgs::Dual msg_out;
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
	//msg_out.events = e;
	opensimrt_msgs::Dual msg_out = Osb::get_SO_as_Dual(h,t,q,soOutput);
	//pub.publish(msg_out);
	return msg_out;
}

opensimrt_msgs::MultiMessage Pipeline::SoBare::run2(const std_msgs::Header h, double t, SimTK::Vector q, std::vector<double> Rtau, opensimrt_msgs::Events e )
{
	ROS_DEBUG_STREAM("Received run call. Running SoBare loop"); 
	//unpacking tau
 	SimTK::Vector tau(Rtau.size());
	for (int j=0; j<tau.size();j++)
	{
		tau[j] = Rtau[j];
	}


	ROS_DEBUG_STREAM("attempting to call SO.");
	if (!so)
	{
		opensimrt_msgs::MultiMessage msg_out;
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
	//msg_out.events = e;
	opensimrt_msgs::MultiMessage msg_out = Osb::get_SO_as_Multi(h,t,q,soOutput);
	//pub.publish(msg_out);
	return msg_out;
}


