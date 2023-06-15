#include "InverseDynamics.h"
#include "Ros/include/common_node.h"
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

#include "osrt_ros/Pipeline/id_async.h"
#include "opensimrt_bridge/conversions/message_convs.h"
#include "tf2/convert.h"


using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::WrenchSubscriber::WrenchSubscriber(std::string wrench_name_prefix_, std::string calcn_frame_):
	tfListener(tfBuffer), wrench_name_prefix(wrench_name_prefix_), calcn_frame(calcn_frame_)
{


}

void Pipeline::WrenchSubscriber::onInit()

{
	nh.param<bool>("use_grfm_filter", use_grfm_filter, false);
	sub.subscribe(nh,wrench_name_prefix+"/wrench",100);
	pub_grf = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_grf", 1000);
	pub_cop = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_cop", 1000);
	nh.param<std::string>(wrench_name_prefix+"_foot_tf_name", foot_tf_name, wrench_name_prefix+"_foot_forceplate");
	nh.param<std::string>("grf_reference_frame", grf_reference_frame, "map");

	ref_frame = "";//something
}

void Pipeline::WrenchSubscriber::callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
	//places the wrench in the buffer
	wrenchBuffer.push_back(msg);
}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	////HERE I ALREADY HAVE THE WRENCH FROM THE RIGHT TIME
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
const geometry_msgs::WrenchStampedConstPtr Pipeline::WrenchSubscriber::find_wrench_in_buffer(const std_msgs::Header::_stamp_type timestamp)
{
	ROS_FATAL_STREAM("not implemented");
	return geometry_msgs::WrenchStampedConstPtr();
}
bool Pipeline::WrenchSubscriber::get_wrench(const std_msgs::Header::_stamp_type timestamp, ExternalWrench::Input* wO )
{
	//find the actual wrench I need based on timestamp
	auto w = find_wrench_in_buffer(timestamp);	
	ROS_INFO_STREAM("times.. wrench header:"<< w->header << "\nnow: "<<ros::Time::now());
	auto sometime = w->header.stamp; //I hate myself.
	//auto sometime = ros::Time(0); //I hate myself.
	
	//TODO:I actually should read the ref_frame from the wrench like a normal person.
	//NO. we are using this for both the wrenches, so it will use the value for the wrong side if the transform fails!. 
	// now get the translations from the transform
	// for the untranslated version I just want to get the reference in their own coordinate reference frame
	geometry_msgs::TransformStamped nulltransform, actualtransform,inv_t;
	try
	{
		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);
		if (false)
		{
			 //i gotta do 2 lookups. one to the frigging food
			 // "left_filtered " -"calcn_l"
			 //	"calcn_l" - "subject_opensim"
				auto foot_cop = tfBuffer.lookupTransform(ref_frame, calcn_frame, sometime);
				auto foot_pos = tfBuffer.lookupTransform(calcn_frame, grf_reference_frame, sometime);
				opensimrt_msgs::CommonTimed msg_cop;
				msg_cop.header.stamp = ros::Time::now();
				msg_cop.data.push_back( foot_cop.transform.translation.x);
				msg_cop.data.push_back( foot_cop.transform.translation.y);
				msg_cop.data.push_back( foot_cop.transform.translation.z);
				msg_cop.data.push_back(foot_pos.transform.translation.x);
				msg_cop.data.push_back(foot_pos.transform.translation.y);
				msg_cop.data.push_back(foot_pos.transform.translation.z);
				pub_cop.publish(msg_cop);

		}

		//ATTENTION FUTURE FREDERICO:
		//this is actually already correct. what you need to do use this function is to have another fixed transform generating a "subject_opensim" frame of reference and everything should work
		//IT IS OBVIOUSLY COMMING FROM HERE. BUT WHERE HERE?
		//ROS_INFO_STREAM(ref_frame<<" "<<grf_reference_frame);

		nulltransform = tfBuffer.lookupTransform(grf_reference_frame, ref_frame, sometime);
		//nulltransform = tfBuffer.lookupTransform("subject_opensim", ref_frame, ros::Time(0));
		wO->point[0] = nulltransform.transform.translation.x;
		wO->point[1] = nulltransform.transform.translation.y;
		wO->point[2] = nulltransform.transform.translation.z;
		//actualtransform = tfBuffer.lookupTransform("map", ref_frame, ros::Time(0));
		//inv_t = tfBuffer.lookupTransform(ref_frame,"map", ros::Time(0));
		//ROS_DEBUG_STREAM("null transform::\n" << nulltransform);
		//ROS_DEBUG_STREAM("actual transform" << actualtransform);
		//ROS_DEBUG_STREAM("inverse transform" << inv_t);
		//inv_t converts back to opensim
		
		//now convert it:
		tf2::Quaternion q ;
		tf2::fromMsg(nulltransform.transform.rotation,q);
		tf2::Vector3 v_force_orig, v_torque_orig;
		tf2::fromMsg(w->wrench.force, v_force_orig);
		tf2::fromMsg(w->wrench.torque, v_torque_orig);

		tf2::Vector3 v_force_new = quatRotate(q, v_force_orig);
		wO->force[0] =v_force_new.x();
		wO->force[1] =v_force_new.y();
		wO->force[2] =v_force_new.z();
		tf2::Vector3 v_torque_new = quatRotate(q, v_torque_orig);
		wO->torque[0] =v_torque_new.x();
		wO->torque[1] =v_torque_new.y();
		wO->torque[2] =v_torque_new.z();

	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("tutu-loo: message_convs.cpp parse_message transform exception: %s",ex.what());
		//ros::Duration(1.0).sleep();
		return false;
	}

	//ROS_WARN_STREAM("TFs in wrench parsing of geometry_wrench messages not implemented! Rotated frames will fail!");
	return true;

}
std::vector<OpenSimRT::ExternalWrench::Input> Pipeline::IdAsync::get_wrench(const std_msgs::Header::_stamp_type timestamp)

{

	// TODO: get wrench message!!!!!!!!!!
	std::vector<ExternalWrench::Input> wrenches;
	OpenSimRT::ExternalWrench::Input nullWrench;
	nullWrench.force = Vec3{0,0,0};
	nullWrench.torque = Vec3{0,0,0};
	nullWrench.point = Vec3{0,0,0};
	
	static OpenSimRT::ExternalWrench::Input grfRightWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gRw; 
	if(wsR.get_wrench(timestamp, gRw))
		grfRightWrench = *gRw;
	//cout << "left wrench.";
	ROS_DEBUG_STREAM("rw");
	static OpenSimRT::ExternalWrench::Input grfLeftWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gLw; 
	if(wsL.get_wrench(timestamp, gLw))
		grfLeftWrench = *gLw;
	ROS_DEBUG_STREAM("lw");
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;


}

Pipeline::IdAsync::IdAsync(): wsL("left", "calcn_l"), wsR("right", "calcn_r")
{

}

Pipeline::IdAsync::~IdAsync()
{
	ROS_INFO_STREAM("Shutting down Id");
}


void Pipeline::IdAsync::onInit() {
	//TODO: this is technically wrong. if I am subscribing to the version with CommonTimed version, then I definetely want the second label as well, but it will fail if I am not subscribing to this, so this flag needs to be set only in that case
	//get_second_label = false;
	Ros::CommonNode::onInit();

	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//TODO::::: HEre I want to subscribe with a timeSequencer!!!!!!!!!!!!
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	sub0.registerCallback(&Pipeline::IdAsync::callback0,this);
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	sub1.registerCallback(&Pipeline::IdAsync::callback1,this);

	//I will start all the wrenches now:
	wsL.onInit();
	wsR.onInit();


	pub_ik = nh.advertise<opensimrt_msgs::CommonTimed>("debug_ik", 1000);

}

void Pipeline::IdAsync::callback1(const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	ROS_FATAL_STREAM("callback grf called. Not implemented for normal CommonTimedConstPtr messages yet.");
///THis has to do the normal thing????

}
void Pipeline::IdAsync::callback0(const opensimrt_msgs::CommonTimedConstPtr& message_ik)
{
	ROS_INFO_STREAM("callback ik called");
	auto newEvents1 = addEvent("id received ik & wrenches",message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id loop callback_real_wrenches"); 
	double filtered_t;
	addEvent("id: getting iks", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik, &filtered_t, ikfilter);
	addEvent("id: getting real wrenches", newEvents1);
	auto timestamp =message_ik->header.stamp;
	auto grfs = Pipeline::IdAsync::get_wrench(timestamp);
	ROS_DEBUG_STREAM("filtered_t" << filtered_t);
	//	run(ikFiltered.t, iks, grfs);	
	//TODO: maybe this will break?
	addEvent("calling run function of id", newEvents1);

	run(message_ik->header, filtered_t, iks, grfs, newEvents1);	
}

void Pipeline::IdAsync::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik) 
{
	auto newEvents1 = addEvent("id received ik filtered & wrenches", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	//cant find the right copy constructor syntax. will for loop it
	addEvent("id: getting iks already filtered.", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik);
	addEvent("id: getting real wrenches", newEvents1);
	auto timestamp =message_ik->header.stamp;
	auto grfs = Pipeline::IdAsync::get_wrench(timestamp);
	addEvent("calling run function of id", newEvents1);
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}



