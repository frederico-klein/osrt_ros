#include "ros/ros.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
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
#include "osrt_ros/Pipeline/so.h"
#include "opensimrt_bridge/conversions/message_convs.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::So::So(): Pipeline::DualSink::DualSink(false)  
{
	ROS_DEBUG_STREAM("fake constructor of SO");
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
Pipeline::So::~So()
{

	ROS_INFO_STREAM("Shutting down So");
}

void Pipeline::So::onInit() {
	nh.getParam("get_second_label", get_second_label);
	//get_second_label = false;
	Pipeline::DualSink::onInit();

	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	sub0.registerCallback(&Pipeline::So::callback0,this);
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	sub1.registerCallback(&Pipeline::So::callback1,this);


	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	// TODO: set the column labels, or it will break when you try to use them!
	ROS_INFO_STREAM("Attempting to set loggers.");
	//initializeLoggers("grfRight",grfRightLogger);
	//initializeLoggers("grfLeft", grfLeftLogger);

	message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync(sub, sub2, 500);
	sync.registerCallback(std::bind(&Pipeline::So::callback, this, std::placeholders::_1, std::placeholders::_2));
	sync.registerCallback(&Pipeline::So::callback, this);

	visualizer = new BasicModelVisualizer(*model);
	ROS_DEBUG_STREAM("onInitSo");
	//these need to be shared with the rest:
	fmLogger = so->initializeMuscleLogger();
	amLogger = so->initializeMuscleLogger();
}

void Pipeline::So::run(const std_msgs::Header h, double t, SimTK::Vector q, std::vector<double> Rtau, opensimrt_msgs::Events e )
{
	ROS_DEBUG_STREAM("Received run call. Running So loop"); 
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
		return;
	}
	ROS_DEBUG_STREAM("attempting to run SO.");
	//ROS_DEBUG_STREAM("t: ["<< t << "] q: [" << q << "] tau: [" << tau << "]");
	auto soOutput = so->solve({t, q, tau});
	addEvent("id_combined after so",e);
	chrono::high_resolution_clock::time_point t2;
	t2 = chrono::high_resolution_clock::now();

	msg_out.events = e;
	pub.publish(msg_out);
	
	opensimrt_msgs::Dual msg_dual = Osb::get_SO_as_Dual(h,t,q,soOutput);
	sync_output.publish(msg_dual);
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


void Pipeline::So::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) {
	ROS_INFO_STREAM("callback ik called");
}
void Pipeline::So::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_id) {
	ROS_INFO_STREAM("callback tau called");
}

void Pipeline::So::finish() {

	//finish for other parts

}

void Pipeline::So::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
{
	auto bothEvents = combineEvents(message_ik, message_tau);
	addEvent("so received ik & tau",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running So loop callback."); 
	SimTK::Vector qRaw(message_ik->data.size()); //cant find the right copy constructor syntax. will for loop it
	for (int j = 0;j < qRaw.size();j++)
	{
		qRaw[j] = message_ik->data[j];
	}

	double t = message_ik->time;
	//	I dont need to filter things, but I need to get either q if unfiltered or q[0] if filtered and then run stuff
	run(message_ik->header, message_ik->time, qRaw, message_tau->data, bothEvents);	
}

void Pipeline::So::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
{
	auto bothEvents = combineEvents(message_ik, message_tau);
	addEvent("so received ik & tau",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running So filtered loop"); 
	SimTK::Vector q(message_ik->d0_data.size()); 
	for (int j = 0;j < q.size();j++)
	{
		q[j] = message_ik->d0_data[j];
	}

	//Construct qVec :should be same as above
	run(message_ik->header, message_ik->time, q, message_tau->data, bothEvents);

}	

