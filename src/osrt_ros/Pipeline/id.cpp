#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "message_filters/time_synchronizer.h"
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
#include "osrt_ros/Pipeline/id.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::Id::Id()
{
    
	// subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_ID_FROM_FILE";
    subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

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

    memory = ini.getInteger(section, "MEMORY", 0);
    cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    delay = ini.getInteger(section, "DELAY", 0);
    splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    model = new Model(modelFile);
    OpenSimUtils::removeActuators(*model);
    model->initSystem();

    // setup external forces
    //Storage grfMotion(grfMotFile);

    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    auto grfRightLoggerTemp = ExternalWrench::initializeLogger();
    grfRightLogger = &grfRightLoggerTemp; 

    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    auto grfLeftLoggerTemp = ExternalWrench::initializeLogger();
    grfLeftLogger = &grfLeftLoggerTemp;

    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // get kinematics as a table with ordered coordinates
    /*auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);
	    */

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model->getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    ikfilter = new LowPassSmoothFilter(ikFilterParam);

    LowPassSmoothFilter::Parameters grfFilterParam;
    grfFilterParam.numSignals = 9;
    grfFilterParam.memory = memory;
    grfFilterParam.delay = delay;
    grfFilterParam.cutoffFrequency = cutoffFreq;
    grfFilterParam.splineOrder = splineOrder;
    grfFilterParam.calculateDerivatives = false;
    grfRightFilter = new LowPassSmoothFilter(grfFilterParam);
    grfLeftFilter = new LowPassSmoothFilter(grfFilterParam);

    // test with state space filter
    // StateSpaceFilter ikFilter({model.getNumCoordinates(), cutoffFreq});
    // StateSpaceFilter grfRightFilter({9, cutoffFreq}), grfLeftFilter({9,
    // cutoffFreq});

    // initialize id and logger
    id = new InverseDynamics(*model, wrenchParameters);
    auto tauLoggerTemp = id->initializeLogger();
    tauLogger = &tauLoggerTemp;
    auto qLoggerTemp = id->initializeLogger();
    qLogger = &qLoggerTemp;
    auto qDotLoggerTemp = id->initializeLogger();
    qDotLogger = &qDotLoggerTemp;
    auto qDDotLoggerTemp = id->initializeLogger();
    qDDotLogger = &qDDotLoggerTemp;

    // mean delay
    sumDelayMS = 0;
    sumDelayMSCounter = 0;

    counter = 0;
}

Pipeline::Id::~Id()
{
	ROS_INFO_STREAM("Shutting down Id");
}

void Pipeline::Id::onInit() {
	Pipeline::DualSink::onInit();
	
	previousTime = ros::Time::now().toSec();
	previousTimeDifference = 0;
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
			sub0.registerCallback(&Pipeline::Id::callback0,this);
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
			sub1.registerCallback(&Pipeline::Id::callback1,this);
	
	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	initializeLoggers("grfRight",grfRightLogger);
	initializeLoggers("grfLeft", grfLeftLogger);
	initializeLoggers("tau",tauLogger);
	message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync(sub, sub2, 500);
	sync.registerCallback(std::bind(&Pipeline::Id::callback, this, std::placeholders::_1, std::placeholders::_2));
	//sync.registerCallback(&Pipeline::Id::callback, this);

	//i should have the input labels from grf already
	
	ROS_INFO_STREAM("left");
	grfLeftIndexes = generateIndexes(grfLeftLabels,input2_labels);
	ROS_INFO_STREAM("right");
	grfRightIndexes = generateIndexes(grfRightLabels, input2_labels);
    // visualizer
    if (usesVisualizarFromId())
    {
	    ROS_WARN_STREAM("CREATING VISUALIZER FROM ID!");
	    visualizer = new BasicModelVisualizer(*model);
	    rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
	    visualizer->addDecorationGenerator(rightGRFDecorator);
	    leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
	    visualizer->addDecorationGenerator(leftGRFDecorator);
    }

}


boost::array<int,9> Pipeline::Id::generateIndexes(std::vector<std::string> pick, std::vector<std::string> whole) // point,force, torque
{
	boost::array<int,9> grfIndexes;
	for (int i= 0; i<9 ; i++)
	{
		//im assuming this is in order. if it isnt this will break
		int j = 0;
		for (auto label:whole)
		{
			if(label.compare(pick[i])==0)
			{
				// the pick_label and the label are the same, so the index i is what we want
				grfIndexes[i] = j; 
			}
			j++;
		}
	}
	for (auto ind: grfIndexes)
		ROS_WARN_STREAM(ind);

	return grfIndexes;
}


ExternalWrench::Input Pipeline::Id::parse_message(const opensimrt_msgs::CommonTimedConstPtr & msg_grf, boost::array<int,9> grfIndexes)
{
	//ROS_DEBUG_STREAM("Parsing wrench from message");
	ExternalWrench::Input a;
	//for (auto ind:grfIndexes)
	//	ROS_INFO_STREAM("index:" << ind);
	a.point  = SimTK::Vec3(msg_grf->data[grfIndexes[0]],msg_grf->data[grfIndexes[1]],msg_grf->data[grfIndexes[2]]);
	a.force  = SimTK::Vec3(msg_grf->data[grfIndexes[3]],msg_grf->data[grfIndexes[4]],msg_grf->data[grfIndexes[5]]);
	a.torque = SimTK::Vec3(msg_grf->data[grfIndexes[6]],msg_grf->data[grfIndexes[7]],msg_grf->data[grfIndexes[8]]);

	//ROS_DEBUG_STREAM("wrench i got:");
	//print_wrench(a);

	return a;
}
void Pipeline::Id::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) {
	ROS_INFO_STREAM("callback ik called");
}
void Pipeline::Id::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_INFO_STREAM("callback grf called");
}

void Pipeline::Id::print_wrench(ExternalWrench::Input w)
{
	ROS_INFO_STREAM("POINT" << w.point[0] << ","<< w.point[1] << "," << w.point[2] );
	ROS_INFO_STREAM("FORCE" << w.force[0] << ","<< w.force[1] << "," << w.force[2] );
	ROS_INFO_STREAM("TORQUE" << w.torque[0] << ","<< w.torque[1] << "," << w.torque[2] );


}
void Pipeline::Id::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	ROS_DEBUG_STREAM("Received message. Running Id loop"); 
	counter++;
	SimTK::Vector qRaw(19); //cant find the right copy constructor syntax. will for loop it
	for (int j = 0;j < qRaw.size();j++)
	{
		qRaw[j] = message_ik->data[j];
	}
	
	double t = message_ik->time;
	// filter
	auto ikFiltered = ikfilter->filter({t, qRaw});
	auto q = ikFiltered.x;
	auto qDot = ikFiltered.xDot;
	auto qDDot = ikFiltered.xDDot;
	ROS_DEBUG_STREAM("Filter ran ok");
	if (!ikFiltered.isValid) {
		ROS_DEBUG_STREAM("filter results are NOT valid");
		return; }
	ROS_DEBUG_STREAM("Filter results are valid");

	run(ikFiltered.t, q, qDot, qDDot, message_grf);	
}	
void Pipeline::Id::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop"); 
	counter++;
	//cant find the right copy constructor syntax. will for loop it
	SimTK::Vector q(19),qDot(19),qDDot(19); 
	for (int j = 0;j < q.size();j++)
	{
		q[j] = message_ik->d0_data[j];
		qDot[j] = message_ik->d1_data[j];
		qDDot[j] = message_ik->d2_data[j];
	}
	run(message_ik->time, q, qDot, qDDot,message_grf);

}	

void Pipeline::Id::run(double t, SimTK::Vector q,SimTK::Vector qDot, SimTK::Vector qDDot, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 

{
	ROS_ERROR_STREAM("Received run call. Running Id loop"); 
	counter++;

	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-5 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_DEBUG_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_DEBUG_STREAM("DeltaT :"<< std::setprecision (15) << t);

	// TODO: get wrench message!!!!!!!!!!

	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
	//cout << "left wrench.";
	ROS_INFO_STREAM("rw");
	print_wrench(grfRightWrench);
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
	ROS_INFO_STREAM("lw");
	print_wrench(grfLeftWrench);
//	return;

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

        // perform id
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();
        auto idOutput = id->solve(
                {t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

	sumDelayMSCounter++;

	ROS_DEBUG_STREAM("inverse dynamics ran ok");

	// visualization
	try {
	visualizer->update(q);
	rightGRFDecorator->update(grfRightWrench.point,
				  grfRightWrench.force);
	leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}

	try{

	// log data (use filter time to align with delay)
	if(false)
	{
		ROS_WARN_STREAM("THIS SHOULDNT BE RUNNING");

		tauLogger->appendRow(t, ~idOutput.tau);
		grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
		grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
		qLogger->appendRow(t, ~q);
		qDotLogger->appendRow(t, ~qDot);
		qDDotLogger->appendRow(t, ~qDDot);

		ROS_INFO_STREAM("Added data to loggers. "<< counter);
	}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
	//if (counter > 720)
	//	write_();
	//}
}	
void Pipeline::Id::finish() {

    cout << "Mean delay: " << (double) sumDelayMS / sumDelayMSCounter << " ms"
         << endl;

    // Compare results with reference tables. Make sure that M, D,
    // spline order, fc are the same as the test.
    SimTK_ASSERT_ALWAYS(memory == 35,
                        "ensure that MEMORY = 35 in setup.ini for testing");
    SimTK_ASSERT_ALWAYS(delay == 14,
                        "ensure that DELAY = 35 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(cutoffFreq == 6,
                        "ensure that CUTOFF_FREQ = 6 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(splineOrder == 3,
                        "ensure that SPLINE_ORDER = 3 setup.ini for testing");
    OpenSimUtils::compareTables(
            *tauLogger,
            TimeSeriesTable(subjectDir + "real_time/inverse_dynamics/tau.sto"));
    OpenSimUtils::compareTables(
            *grfLeftLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/wrench_left.sto"));
    OpenSimUtils::compareTables(
            *grfRightLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/wrench_right.sto"));
    OpenSimUtils::compareTables(
            *qLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/q_filtered.sto"));
    OpenSimUtils::compareTables(
            *qDotLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/qDot_filtered.sto"));
    OpenSimUtils::compareTables(
            *qDDotLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/qDDot_filtered.sto"));
}
bool Pipeline::Id::see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
	std::copy(grfRightLogger->getTableMetaData().getKeys().begin(), grfRightLogger->getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
	ss << std::endl;
	ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());
	return true;
}
void Pipeline::Id::write_() {
//			std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
//		  	std::copy(grfRightLogger.getTableMetaData().getKeys().begin(), grfRightLogger.getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
//		  	ss << std::endl;
//			ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());


/*	ROS_INFO_STREAM("I TRY WRITE sto");
	STOFileAdapter::write(grfRightLogger,"grfRight.sto");
	STOFileAdapter::write(grfLeftLogger,"grfLeft.sto");
	STOFileAdapter::write(tauLogger,"tau.sto");
	ROS_INFO_STREAM("I TRY WRITE csv");
	CSVFileAdapter::write(grfRightLogger,"grfRight.csv");
	CSVFileAdapter::write(grfLeftLogger,"grfLeft.csv");
	CSVFileAdapter::write(tauLogger,"tau.csv");
*/
	saveCsvs();
	saveStos();
	ROS_INFO_STREAM("i write");
}

