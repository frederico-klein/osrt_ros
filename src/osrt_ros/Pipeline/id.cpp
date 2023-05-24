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
		ROS_INFO_STREAM("left");
		grfLeftIndexes = generateIndexes(grfLeftLabels,input2_labels);
		ROS_INFO_STREAM("right");
		grfRightIndexes = generateIndexes(grfRightLabels, input2_labels);
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


boost::array<int,9> Pipeline::Id::generateIndexes(std::vector<std::string> pick, std::vector<std::string> whole) // point,force, torque
{
	boost::array<int,9> grfIndexes;
	/*ROS_INFO_STREAM("whole.size: " << whole.size());
	for (auto label:whole)
		ROS_INFO_STREAM("whole labels: " << label);
	for (auto label:pick)
		ROS_INFO_STREAM("pick labels: " << label);
	*/

	for (int i= 0; i<9 ; i++)
	{
		//im assuming this is in order. if it isnt this will break
		int j = 0;
		bool found_it = false;
		for (auto label:whole)
		{
			if(label.compare(pick[i])==0)
			{
				// the pick_label and the label are the same, so the index j is what we want
				grfIndexes[i] = j;
				found_it = true;
				break;
			}
			j++;
		}
		if (!found_it)
			ROS_FATAL_STREAM("Did not find label: [" << pick[i] << "]. Cannot proceed. Check if you loaded the correct MOT file or you set the correct Parameters in the launch file for this node.") ;
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

ExternalWrench::Input Pipeline::Id::parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frame)
{
	ExternalWrench::Input wO;
	wO.force[0] = w->wrench.force.x;
	wO.force[1] = w->wrench.force.y;
	wO.force[2] = w->wrench.force.z;
	wO.torque[0] = w->wrench.torque.x;
	wO.torque[1] = w->wrench.torque.y;
	wO.torque[2] = w->wrench.torque.z;
	// now get the translations from the transform
	// for the untranslated version I just want to get the reference in their own coordinate reference frame
	wO.point[0] = 0;
	wO.point[1] = 0;
	wO.point[2] = 0;
	geometry_msgs::TransformStamped nulltransform, actualtransform,inv_t;
	try
	{
		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work

		nulltransform = tfBuffer.lookupTransform("subject_opensim", ref_frame, ros::Time(0));
		wO.point[0] = nulltransform.transform.translation.x;
		wO.point[1] = nulltransform.transform.translation.y;
		wO.point[2] = nulltransform.transform.translation.z;
		//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
		//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
		ROS_DEBUG_STREAM("null transform::\n" << nulltransform);
		//ROS_DEBUG_STREAM("actual transform" << actualtransform);
		//ROS_DEBUG_STREAM("inverse transform" << inv_t);
		//inv_t converts back to opensim
		
		//now convert it:

	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("transform exception: %s",ex.what());
		ros::Duration(1.0).sleep();
		return wO;
	}

	ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
	return wO;

}


void Pipeline::Id::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik) {
	ROS_INFO_STREAM("callback ik called");
}
void Pipeline::Id::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_INFO_STREAM("callback grf called");
}

void Pipeline::Id::print_wrench(ExternalWrench::Input w)
{
	ROS_DEBUG_STREAM("POINT" << w.point[0] << ","<< w.point[1] << "," << w.point[2] );
	ROS_DEBUG_STREAM("FORCE" << w.force[0] << ","<< w.force[1] << "," << w.force[2] );
	ROS_DEBUG_STREAM("TORQUE" << w.torque[0] << ","<< w.torque[1] << "," << w.torque[2] );


}
void Pipeline::Id::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback."); 
	counter++;
	double filtered_t;
	auto iks = parse_ik_message(message_ik, &filtered_t);
	//ROS_DEBUG_STREAM("message_grf\n" << *message_grf);
	auto grfs = get_wrench(message_grf);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	run(message_ik->header, filtered_t, iks, grfs, bothEvents);	
}

std::vector<SimTK::Vector> Pipeline::Id::parse_ik_message(const opensimrt_msgs::CommonTimedConstPtr& message_ik, double* filtered_t)
{
	std::vector<SimTK::Vector> qVec;
	SimTK::Vector qRaw(message_ik->data.size()); //cant find the right copy constructor syntax. will for loop it
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
		return qVec; }
	ROS_DEBUG_STREAM("Filter results are valid");

	*filtered_t = ikFiltered.t;
	//Construct qVec
	qVec.push_back(q);
	qVec.push_back(qDot);
	qVec.push_back(qDDot);

	return qVec;
}

std::vector<SimTK::Vector> Pipeline::Id::parse_ik_message(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik)
{
	std::vector<SimTK::Vector> qVec;
	SimTK::Vector q(message_ik->d0_data.size()),qDot(message_ik->d0_data.size()),qDDot(message_ik->d0_data.size()); 
	for (int j = 0;j < q.size();j++)
	{
		q[j] = message_ik->d0_data[j];
		qDot[j] = message_ik->d1_data[j];
		qDDot[j] = message_ik->d2_data[j];
	}

	//Construct qVec :should be same as above
	qVec.push_back(q);
	qVec.push_back(qDot);
	qVec.push_back(qDDot);
	return qVec;

}


void Pipeline::Id::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) 
{
	auto bothEvents = combineEvents(message_ik, message_grf);
	addEvent("id received ik & grf",bothEvents);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop"); 
	counter++;
	//cant find the right copy constructor syntax. will for loop it
	auto iks = parse_ik_message(message_ik);
	auto grfs = get_wrench(message_grf);
	run(message_ik->header, message_ik->time, iks,grfs, bothEvents);

}	

std::vector<ExternalWrench::Input> Pipeline::Id::get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	// TODO: get wrench message!!!!!!!!!!
	std::vector<ExternalWrench::Input> wrenches;
	ExternalWrench::Input grfRightWrench = parse_message(wr, right_foot_tf_name);
	//cout << "left wrench.";
	ROS_DEBUG_STREAM("rw");
	print_wrench(grfRightWrench);
	ExternalWrench::Input grfLeftWrench = parse_message(wl, left_foot_tf_name);
	ROS_DEBUG_STREAM("lw");
	print_wrench(grfLeftWrench);
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;
}

std::vector<ExternalWrench::Input> Pipeline::Id::get_wrench(const opensimrt_msgs::CommonTimedConstPtr& message_grf)
{
	std::vector<ExternalWrench::Input> wrenches;
	double t = message_grf->time; //TODO: if it isn't the same as in message ik, this will break!
	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
	//cout << "left wrench.";
	ROS_DEBUG_STREAM("rw");
	print_wrench(grfRightWrench);
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
	ROS_DEBUG_STREAM("lw");
	print_wrench(grfLeftWrench);
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;
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
	auto iks = parse_ik_message(message_ik, &filtered_t);
	auto grfs = get_wrench(wl,wr);
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
	auto iks = parse_ik_message(message_ik);
	auto grfs = get_wrench(wl,wr);
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}


