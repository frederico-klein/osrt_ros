#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "message_filters/time_synchronizer.h"
#include "ros/exception.h"
#include "ros/message_traits.h"
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
#include <SimTKcommon/internal/Vector_.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"

#include "osrt_ros/Pipeline/id.h"
#include "opensimrt_bridge/conversions/message_convs.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::Id::Id(): Pipeline::DualSink::DualSink(false),
	sync_real_wrenches(grf_approx_wrench_policy(10),sub,sub_wl,sub_wr), 
	//sync_filtered_real_wrenches(sub_filtered,sub_wl,sub_wr,10),
	sync_filtered_real_wrenches(grf_approx_wrench_filtered_policy(10),sub_filtered,sub_wl,sub_wr),
	tfListener(tfBuffer)
{
	//TODO: this needs to be abstracted. I have this copied over and over again. maybe that should be done before standardizing
	// subject data
	INIReader ini(INI_FILE);
	auto section = "TEST_ID_FROM_FILE";
	subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
	//auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
	std::string modelFile = "";
	nh.param<std::string>("model_file",modelFile,"");
	nh.param<bool>("use_grfm_filter", use_grfm_filter, false);
	nh.param<std::string>("left_foot_tf_name", left_foot_tf_name, "left_foot_forceplate");
	nh.param<std::string>("right_foot_tf_name", right_foot_tf_name, "right_foot_forceplate");
	nh.param<std::string>("grf_reference_frame", grf_reference_frame, "map");
	//auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");
	//auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
	//TODO:params!!!! copy from bridge
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
	ROS_INFO_STREAM("Using model: " << modelFile);
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
	if (false)
	{ //very wrong
		auto grfRightLoggerTemp = ExternalWrench::initializeLogger();
		grfRightLogger = &grfRightLoggerTemp; 
	}
	ExternalWrench::Parameters grfLeftFootPar{
		grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
	grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
			grfLeftPointIdentifier, grfLeftForceIdentifier,
			grfLeftTorqueIdentifier);
	if (false)
	{ //very wrong
		auto grfLeftLoggerTemp = ExternalWrench::initializeLogger();
		grfLeftLogger = &grfLeftLoggerTemp;
	}
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
	ROS_INFO_STREAM("Setting loggers,");
	OpenSim::TimeSeriesTable tauLoggerTemp = id->initializeLogger();
	tauLogger = new TimeSeriesTable();
	output_labels = tauLoggerTemp.getColumnLabels();
	tauLogger->setColumnLabels(tauLoggerTemp.getColumnLabels());
	if (false)
	{ //this is super wrong
		auto qLoggerTemp = id->initializeLogger();
		qLogger = &qLoggerTemp;
		auto qDotLoggerTemp = id->initializeLogger();
		qDotLogger = &qDotLoggerTemp;
		auto qDDotLoggerTemp = id->initializeLogger();
		qDDotLogger = &qDDotLoggerTemp;
	}
	ROS_INFO_STREAM("loggers set!");

	// mean delay
	sumDelayMS = 0;
	sumDelayMSCounter = 0;

	counter = 0;
}

Pipeline::Id::~Id()
{
	ROS_INFO_STREAM("Shutting down Id");
}

void Pipeline::Id::print_vec(std::vector<std::string> vs)
{
	std::string s="[";
	for (auto vsi:vs)
		s+= vsi+", ";
	s+= "]";
	ROS_INFO_STREAM(s);
}

void Pipeline::Id::onInit() {
	//TODO: this is technically wrong. if I am subscribing to the version with CommonTimed version, then I definetely want the second label as well, but it will fail if I am not subscribing to this, so this flag needs to be set only in that case
	nh.getParam("get_second_label", get_second_label);
	//get_second_label = false;
	Pipeline::DualSink::onInit();

	previousTime = ros::Time::now().toSec();
	previousTimeDifference = 0;
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	sub0.registerCallback(&Pipeline::Id::callback0,this);
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	sub1.registerCallback(&Pipeline::Id::callback1,this);


	//the message filters for wrenches	
	sub_wl.subscribe(nh,"left_wrench",100);
	sub_wr.subscribe(nh,"right_wrench",100);
	sync_real_wrenches.connectInput(sub,sub_wl,sub_wr);
	sync_real_wrenches.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches,this, _1,_2,_3));
	sync_filtered_real_wrenches.connectInput(sub_filtered,sub_wl,sub_wr);
	sync_filtered_real_wrenches.registerCallback(boost::bind(&Pipeline::Id::callback_real_wrenches_filtered,this, _1,_2,_3));

	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	// TODO: set the column labels, or it will break when you try to use them!
	ROS_INFO_STREAM("Attempting to set loggers.");
	//initializeLoggers("grfRight",grfRightLogger);
	//initializeLoggers("grfLeft", grfLeftLogger);

	print_vec(tauLogger->getColumnLabels());

	initializeLoggers("tau",tauLogger);
	message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync(sub, sub2, 500);
	sync.registerCallback(std::bind(&Pipeline::Id::callback, this, std::placeholders::_1, std::placeholders::_2));
	sync.registerCallback(&Pipeline::Id::callback, this);

	//i should have the input labels from grf already
	if(get_second_label)
	{
		//TODO:: this looks like a reimplementation of the Reshuffler idea. check and fix!
		ROS_INFO_STREAM("left");
		grfLeftIndexes = Osb::generateIndexes(grfLeftLabels,input2_labels);
		ROS_INFO_STREAM("right");
		grfRightIndexes = Osb::generateIndexes(grfRightLabels, input2_labels);
	}
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




void Pipeline::Id::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) {
	ROS_INFO_STREAM("callback ik called");
}
void Pipeline::Id::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_INFO_STREAM("callback grf called");
}

void Pipeline::Id::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback."); 
	counter++;
	double filtered_t;
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	//ROS_DEBUG_STREAM("message_grf\n" << *message_grf);
	auto grfs = Osb::get_wrench(message_grf, grfRightIndexes, grfLeftIndexes);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	run(message_ik->header, filtered_t, iks, grfs, bothEvents);	
}



void Pipeline::Id::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop"); 
	counter++;
	//cant find the right copy constructor syntax. will for loop it
	auto iks = Osb::parse_ik_message(message_ik);
	auto grfs = Osb::get_wrench(message_grf, grfRightIndexes, grfLeftIndexes);
	run(message_ik->header, message_ik->time, iks,grfs, bothEvents);

}	


void Pipeline::Id::run(const std_msgs::Header h , double t, std::vector<SimTK::Vector> iks, std::vector<ExternalWrench::Input> grfs, opensimrt_msgs::Events e ) 

{
	ROS_DEBUG_STREAM("Received run call. Running Id run loop.");	    
	counter++;

	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-5 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_DEBUG_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_DEBUG_STREAM("DeltaT :"<< std::setprecision (15) << t);

	//unpacks wrenches:

	if(grfs.size() == 0)
	{
		ROS_WARN_STREAM("THERE ARE NO GRFS!");
		return;
	}
	auto grfLeftWrench = grfs[0]; 
	auto grfRightWrench = grfs[1]; 
	
	//filter wrench!
	OpenSimRT::LowPassSmoothFilter::Output grfRightFiltered, grfLeftFiltered;
	if (use_grfm_filter){
		grfRightFiltered =
			grfRightFilter->filter({t, grfRightWrench.toVector()});
		grfRightWrench.fromVector(grfRightFiltered.x);
		grfLeftFiltered =
			grfLeftFilter->filter({t, grfLeftWrench.toVector()});
		grfLeftWrench.fromVector(grfLeftFiltered.x);

		if (!grfRightFiltered.isValid ||
				!grfLeftFiltered.isValid) {
			ROS_WARN_STREAM("GRFS are not valid!");
			return;
		}
	}
	//unpacks iks:
	//TODO
	if(iks.size() == 0)
	{
		ROS_WARN_STREAM("THERE ARE NO IKS!");
		return;
	}
	auto q = iks[0];
	auto qDot = iks[1];
	auto qDDot = iks[2];
	//
	// perform id
	chrono::high_resolution_clock::time_point t1;
	t1 = chrono::high_resolution_clock::now();
	addEvent("id_normal before id", e);
	auto idOutput = id->solve(
			{t, q, qDot, qDDot,
			vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});
	addEvent("id_normal after id",e);

	
	chrono::high_resolution_clock::time_point t2;
	t2 = chrono::high_resolution_clock::now();
	sumDelayMS +=
		chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

	sumDelayMSCounter++;
	//TODO: remove!
	ROS_DEBUG_STREAM("trying to get column labels...");
	try
	{
		std::vector<std::string> how_is_this_set = tauLogger->getColumnLabels();
		if (how_is_this_set.size() != 0)
		{
			std::string out_print;
			for (int kk= 0;kk < how_is_this_set.size(); kk++)
				out_print += "," + how_is_this_set[kk];
			ROS_DEBUG_STREAM("getColumnLabels Response is:" << out_print);
		}
		else
		{
			ROS_WARN_STREAM("it isn't set.");
		}
	}
	catch (...)
	{
		ROS_ERROR_STREAM("tauColumnNames fails...");
	}
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
	try
	{
		//TODO: this is common for everyone that uses common messages, make it a class
		//I need the header!!!
		opensimrt_msgs::CommonTimed msg;
		msg.header = h;
		ROS_DEBUG_STREAM("attempting to print tau!");
		for (double tau_component:idOutput.tau)
		{
			ROS_DEBUG_STREAM("some_tau_component: " << tau_component);
			msg.data.push_back(tau_component);
		}
		msg.events = e;
		pub.publish(msg);	
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR_STREAM("Ros error while trying to publish ID output: " << e.what());
	}
	try{

		// log data (use filter time to align with delay)
		if(recording)
		{
			//ROS_WARN_STREAM("THIS SHOULDNT BE RUNNING");
			tauLogger->appendRow(t, ~idOutput.tau);
			ROS_INFO_STREAM("Tau added data to loggers. "<< counter);
			/*qLogger->appendRow(t, ~q);
			qDotLogger->appendRow(t, ~qDot);
			qDDotLogger->appendRow(t, ~qDDot);
			ROS_INFO_STREAM("q filtered data added data to loggers. "<< counter);
			
			grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
			grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
			ROS_INFO_STREAM("GRFM data added data to loggers. "<< counter);*/

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
	ROS_ERROR_STREAM("deprecated.");
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

void Pipeline::Id::callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik & wrenches",message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback_real_wrenches"); 
	counter++;
	double filtered_t;
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	auto grfs = Osb::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	ROS_WARN_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	
	run(message_ik->header, filtered_t, iks, grfs, newEvents1);	
}

void Pipeline::Id::callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik filtered & wrenches", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	counter++;
	//cant find the right copy constructor syntax. will for loop it
	auto iks = Osb::parse_ik_message(message_ik);
	auto grfs = Osb::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}


