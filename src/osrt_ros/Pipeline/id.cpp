#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/Events.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "message_filters/time_synchronizer.h"
#include "ros/duration.h"
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
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>
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
	sync_real_wrenches(grf_exact_wrench_policy(5),sub,sub_wl,sub_wr), 
	//sync_real_wrenches(grf_approx_wrench_policy(5),sub,sub_wl,sub_wr), 
	//sync_filtered_real_wrenches(sub_filtered,sub_wl,sub_wr,10),
	//sync_filtered_real_wrenches(grf_approx_wrench_filtered_policy(5),sub_filtered,sub_wl,sub_wr),
	sync_filtered_real_wrenches(grf_exact_wrench_filtered_policy(5),sub_filtered,sub_wl,sub_wr),
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
	ExternalWrench::Parameters grfLeftFootPar{
		grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
	grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
			grfLeftPointIdentifier, grfLeftForceIdentifier,
			grfLeftTorqueIdentifier);
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
	ROS_INFO_STREAM("loggers set!");

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
        //CRAZY DEBUG

	pub_grf_left = nh.advertise<opensimrt_msgs::CommonTimed>("debug_grf_left", 1000);
	pub_grf_right = nh.advertise<opensimrt_msgs::CommonTimed>("debug_grf_right", 1000);
	pub_ik = nh.advertise<opensimrt_msgs::CommonTimed>("debug_ik", 1000);
	pub_cop_left = nh.advertise<opensimrt_msgs::CommonTimed>("debug_cop_left", 1000);
	pub_cop_right = nh.advertise<opensimrt_msgs::CommonTimed>("debug_cop_right", 1000);

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
	//cant find the right copy constructor syntax. will for loop it
	auto iks = Osb::parse_ik_message(message_ik);
	auto grfs = Osb::get_wrench(message_grf, grfRightIndexes, grfLeftIndexes);
	run(message_ik->header, message_ik->time, iks,grfs, bothEvents);

}	
//oh, these exist in Osb already...
opensimrt_msgs::CommonTimed Pipeline::Id::conv_ik_to_msg(std_msgs::Header h, SimTK::Vector ik)
{
	opensimrt_msgs::CommonTimed msg;	
	msg.header =h;
	for (auto q:ik)
		msg.data.push_back(q);
	return msg;

}
opensimrt_msgs::CommonTimed Pipeline::Id::conv_grf_to_msg(std_msgs::Header h, ExternalWrench::Input ow)
{
	opensimrt_msgs::CommonTimed msg;	
	msg.header = h;
	msg.data.push_back(ow.point[0]);
	msg.data.push_back(ow.point[1]);
	msg.data.push_back(ow.point[2]);
	msg.data.push_back(ow.force[0]);
	msg.data.push_back(ow.force[1]);
	msg.data.push_back(ow.force[2]);
	msg.data.push_back(ow.torque[0]);
	msg.data.push_back(ow.torque[1]);
	msg.data.push_back(ow.torque[2]);
	


	return msg;
}


void Pipeline::Id::run(const std_msgs::Header h , double t, std::vector<SimTK::Vector> iks, std::vector<ExternalWrench::Input> grfs, opensimrt_msgs::Events e ) 

{
	ROS_DEBUG_STREAM("Received run call. Running Id run loop.");	    
	//ROS_INFO_STREAM("time diff" << h.stamp-last_received_ik_stamp);
	//ROS_INFO_STREAM("this stamp" << h.stamp << "last_received_ik_stamp" << last_received_ik_stamp);
	//unpacks wrenches:

	if(grfs.size() == 0)
	{
		ROS_WARN_STREAM("THERE ARE NO GRFS!");
		return;
	}
	auto grfLeftWrench = grfs[0]; 
	auto grfRightWrench = grfs[1]; 

	//filter wrench!
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

	ROS_DEBUG_STREAM("inverse dynamics ran ok");

	// visualization
	if (true)
	{
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
		//msg.events = e;
		pub.publish(msg);
		pub_grf_left.publish(conv_grf_to_msg(h, grfLeftWrench));
		pub_grf_right.publish(conv_grf_to_msg(h, grfRightWrench));
		pub_ik.publish(conv_ik_to_msg(h, q));
		last_received_ik_stamp = msg.header.stamp;
		sync_output_multi.publish(Osb::get_as_Multi(h,t,q,idOutput.tau));
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR_STREAM("Ros error while trying to publish ID output: " << e.what());
	}
	try{

		// log data (use filter time to align with delay)
		if(recording)
		{
			tauLogger->appendRow(t, ~idOutput.tau);
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
}
bool Pipeline::Id::see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_ERROR_STREAM("deprecated.");
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
	ExternalWrench::Input Pipeline::Id::parse_message(const geometry_msgs::WrenchStampedConstPtr& w, std::string ref_frame, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame)
	{
		auto sometime = w->header.stamp -ros::Duration(0.05); //I hate myself.
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
			//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
			ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);
			ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);
			if (false)
			{
			size_t found = ref_frame.find("left");
			if (found!=std::string::npos)
			{//left
				//i gotta do 2 lookups. one to the frigging food
				// "left_filtered " -"calcn_l"
				//	"calcn_l" - "subject_opensim"
				auto foot_cop = tfBuffer.lookupTransform(ref_frame, "calcn_l", sometime);
				auto foot_pos = tfBuffer.lookupTransform("calcn_l", grf_reference_frame, sometime);
				opensimrt_msgs::CommonTimed msg_cop;
				msg_cop.header.stamp = ros::Time::now();
				msg_cop.data.push_back( foot_cop.transform.translation.x);
				msg_cop.data.push_back( foot_cop.transform.translation.y);
				msg_cop.data.push_back( foot_cop.transform.translation.z);
				msg_cop.data.push_back(foot_pos.transform.translation.x);
				msg_cop.data.push_back(foot_pos.transform.translation.y);
				msg_cop.data.push_back(foot_pos.transform.translation.z);
				pub_cop_left.publish(msg_cop);
			}
			else
			{//right
				//"right_filtered" - "calcn_r"
				//	"calcn_r" - "subject_opensim"
				auto foot_cop = tfBuffer.lookupTransform(ref_frame, "calcn_r", sometime);
				auto foot_pos = tfBuffer.lookupTransform("calcn_r", grf_reference_frame, sometime);
				opensimrt_msgs::CommonTimed msg_cop;
				msg_cop.header.stamp = ros::Time::now();
				msg_cop.data.push_back( foot_cop.transform.translation.x);
				msg_cop.data.push_back( foot_cop.transform.translation.y);
				msg_cop.data.push_back( foot_cop.transform.translation.z);
				msg_cop.data.push_back(foot_pos.transform.translation.x);
				msg_cop.data.push_back(foot_pos.transform.translation.y);
				msg_cop.data.push_back(foot_pos.transform.translation.z);
				pub_cop_right.publish(msg_cop);

			}

			}

			
			nulltransform = tfBuffer.lookupTransform(grf_reference_frame, ref_frame, sometime);
			//nulltransform = tfBuffer.lookupTransform("subject_opensim", ref_frame, ros::Time(0));
			wO.point[0] = nulltransform.transform.translation.x;
			wO.point[1] = nulltransform.transform.translation.y;
			wO.point[2] = nulltransform.transform.translation.z;
			//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
			//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
			//ROS_DEBUG_STREAM("null transform::\n" << nulltransform);
			//ROS_DEBUG_STREAM("actual transform" << actualtransform);
			//ROS_DEBUG_STREAM("inverse transform" << inv_t);
			//inv_t converts back to opensim

			//now convert it:

		}
		catch (tf2::TransformException &ex) {
			ROS_ERROR("tutu-loo: message_convs.cpp parse_message transform exception: %s",ex.what());
			//ros::Duration(1.0).sleep();
			return wO;
		}

		//ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
		return wO;

	}
	std::vector<OpenSimRT::ExternalWrench::Input> Pipeline::Id::get_wrench(const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr, std::string right_foot_tf_name, std::string left_foot_tf_name, tf2_ros::Buffer& tfBuffer, std::string grf_reference_frame)

{

		// TODO: get wrench message!!!!!!!!!!
		std::vector<ExternalWrench::Input> wrenches;
		OpenSimRT::ExternalWrench::Input grfRightWrench = parse_message(wr, right_foot_tf_name, tfBuffer, grf_reference_frame);
		//cout << "left wrench.";
		ROS_DEBUG_STREAM("rw");
		ExternalWrench::Input grfLeftWrench = parse_message(wl, left_foot_tf_name, tfBuffer, grf_reference_frame);
		ROS_DEBUG_STREAM("lw");
		//	return;

		wrenches.push_back(grfLeftWrench);
		wrenches.push_back(grfRightWrench);
		return wrenches;


}


void Pipeline::Id::callback_real_wrenches(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik & wrenches",message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback_real_wrenches"); 
	double filtered_t;
	addEvent("id: getting iks", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	addEvent("id: getting real wrenches", newEvents1);
	auto grfs = Pipeline::Id::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	addEvent("calling run function of id", newEvents1);

	run(message_ik->header, filtered_t, iks, grfs, newEvents1);	
}

void Pipeline::Id::callback_real_wrenches_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const geometry_msgs::WrenchStampedConstPtr& wl, const geometry_msgs::WrenchStampedConstPtr& wr)
{
	auto newEvents1 = addEvent("id received ik filtered & wrenches", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	//cant find the right copy constructor syntax. will for loop it
	addEvent("id: getting iks already filtered.", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik);
	addEvent("id: getting real wrenches", newEvents1);
	auto grfs = Pipeline::Id::get_wrench(wl,wr, right_foot_tf_name, left_foot_tf_name, tfBuffer, grf_reference_frame);
	addEvent("calling run function of id", newEvents1);
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}


