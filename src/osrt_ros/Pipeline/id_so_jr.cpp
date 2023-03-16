#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
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
#include "osrt_ros/Pipeline/id_so_jr.h"
#include "MuscleOptimization.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::IdSoJr::IdSoJr() // : Pipeline::Id() //this is done automatically I think, so I dont need to call it here
{
	//construct of So
	So();
	//construct of Jr
	Jr();

}

void Pipeline::IdSoJr::So()
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
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
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
void Pipeline::IdSoJr::Jr()
{
	ROS_DEBUG_STREAM("fake constructor of Jr");
}
Pipeline::IdSoJr::~IdSoJr()
{

	ROS_INFO_STREAM("mean delay:" << sumDelayMS/ sumDelayMSCounter << " ms");

	ROS_INFO_STREAM("Shutting down Id_So_Jr");
}

void Pipeline::IdSoJr::onInit() {
	Pipeline::Id::onInit();
	onInitSo();
	onInitJr();
    // visualizer
    //BasicModelVisualizer visualizer(model);
	//for(;;)
	//{
	//	ROS_DEBUG_THROTTLE(60,"iiiiiiii");
	//}
    visualizer = new BasicModelVisualizer(*model);

	
}

void Pipeline::IdSoJr::onInitSo()
{
	ROS_DEBUG_STREAM("onInitSo");
    //these need to be shared with the rest:
    fmLogger = so->initializeMuscleLogger();
    amLogger = so->initializeMuscleLogger();
}

void Pipeline::IdSoJr::onInitJr()
{
	ROS_DEBUG_STREAM("onInitJr");
}

void Pipeline::IdSoJr::run(const std_msgs::Header h, double t, std::vector<SimTK::Vector> iks, std::vector<OpenSimRT::ExternalWrench::Input> grfs )
{
	ROS_DEBUG_STREAM("Received run call. Running IdSoJr loop"); 
	ROS_ERROR_STREAM("not implemented!");

	if(iks.size() == 0)
	{
		ROS_ERROR_STREAM("THERE ARE NO IKS!");
		return;
	}
	auto q = iks[0];
	auto qDot = iks[1];
	auto qDDot = iks[2];
	//unpacks wrenches:

	if(grfs.size() == 0)
	{
		ROS_ERROR_STREAM("THERE ARE NO GRFS!");
		return;
	}
	auto grfLeftWrench = grfs[0]; 
	auto grfRightWrench = grfs[1]; 

	//filter wrench!
	//
	auto grfRightFiltered =
		grfRightFilter->filter({t, grfRightWrench.toVector()});
	grfRightWrench.fromVector(grfRightFiltered.x);
	auto grfLeftFiltered =
		grfLeftFilter->filter({t, grfLeftWrench.toVector()});
	grfLeftWrench.fromVector(grfLeftFiltered.x);

	if (!grfRightFiltered.isValid ||
			!grfLeftFiltered.isValid) {
	return;
	}
	//
	// perform id
	chrono::high_resolution_clock::time_point t1;
	t1 = chrono::high_resolution_clock::now();
	auto idOutput = id->solve(
			{t, q, qDot, qDDot,
			vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});
	auto tau = idOutput.tau;

	
	ROS_DEBUG_STREAM("inverse dynamics ran ok");


	ROS_DEBUG_STREAM("attempting to call SO.");
	if (!so)
	{
		ROS_ERROR_STREAM("so not initialized!");
		return;
	}
	ROS_DEBUG_STREAM("attempting to run SO.");
	ROS_DEBUG_STREAM("t: ["<< t << "] q: [" << q << "] tau: [" << tau << "]");
        auto soOutput = so->solve({t, q, tau});
	chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        double dddd = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        sumDelayMS += dddd;
	//cout << "freq:" << 1000/dddd << endl;
	ROS_DEBUG_STREAM_THROTTLE(2,"Average delay:"<< sumDelayMS/sumDelayMSCounter << "ms. Fps:" << 1000/dddd );
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
		tauLogger->appendRow(t, ~idOutput.tau);
		grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
		grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
		qLogger->appendRow(t, ~q);
		qDotLogger->appendRow(t, ~qDot);
		qDDotLogger->appendRow(t, ~qDDot);
	// loggers from SO
		// log data (use filter time to align with delay)
        	fmLogger.appendRow(t, ~soOutput.fm);
        	amLogger.appendRow(t, ~soOutput.am);

		ROS_INFO_STREAM("Added data to loggers. "<< sumDelayMSCounter);
	}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}

}

void Pipeline::IdSoJr::run(double t, SimTK::Vector q,SimTK::Vector qDot, SimTK::Vector qDDot, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	ROS_DEBUG_STREAM("Received run call. Running IdSoJr loop"); 
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();
	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
        auto grfRightFiltered =
                grfRightFilter->filter({t, grfRightWrench.toVector()});
        grfRightWrench.fromVector(grfRightFiltered.x);
        auto grfLeftFiltered =
                grfLeftFilter->filter({t, grfLeftWrench.toVector()});
        grfLeftWrench.fromVector(grfLeftFiltered.x);

        if (!grfRightFiltered.isValid ||
            !grfLeftFiltered.isValid) {
		ROS_DEBUG_STREAM("filter results are NOT valid");
            return;
        }

        // perform id
        auto idOutput = id->solve(
                {t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});


	sumDelayMSCounter++;
	auto tau = idOutput.tau;
	ROS_DEBUG_STREAM("attempting to call SO.");
	if (!so)
	{
		ROS_ERROR_STREAM("so not initialized!");
		return;
	}
        auto soOutput = so->solve({t, q, tau});
	chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        double dddd = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        sumDelayMS += dddd;
	//cout << "freq:" << 1000/dddd << endl;
	ROS_DEBUG_STREAM_THROTTLE(2,"Average delay:"<< sumDelayMS/sumDelayMSCounter << "ms. Fps:" << 1000/dddd );
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
		tauLogger->appendRow(t, ~idOutput.tau);
		grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
		grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
		qLogger->appendRow(t, ~q);
		qDotLogger->appendRow(t, ~qDot);
		qDDotLogger->appendRow(t, ~qDDot);
	// loggers from SO
		// log data (use filter time to align with delay)
        	fmLogger.appendRow(t, ~soOutput.fm);
        	amLogger.appendRow(t, ~soOutput.am);

		ROS_INFO_STREAM("Added data to loggers. "<< sumDelayMSCounter);
	}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
}	
void Pipeline::IdSoJr::finish() {
	Pipeline::Id::finish();

	//finish for other parts

}
