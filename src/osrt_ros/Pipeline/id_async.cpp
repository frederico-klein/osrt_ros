#include "osrt_ros/Pipeline/id_async.h"
#include "geometry_msgs/WrenchStamped.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/duration.h"
#include <memory>

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
	ROS_DEBUG_STREAM("called WrenchSubscriber onInit");
	nh.param<bool>("use_grfm_filter", use_grfm_filter, false);
	nh.param<int>("max_buffer_length", max_buffer_length, 1000);
	sub = nh.subscribe(wrench_name_prefix+"/wrench",100, &WrenchSubscriber::callback, this);

	pub_grf = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_grf", 1000);
	pub_cop = nh.advertise<opensimrt_msgs::CommonTimed>(wrench_name_prefix+"/debug_cop", 1000);
	nh.param<std::string>(wrench_name_prefix+"_foot_tf_name", foot_tf_name, wrench_name_prefix+"_foot_forceplate");
	nh.param<std::string>("grf_reference_frame", grf_reference_frame, "map");

}

void Pipeline::WrenchSubscriber::callback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
	ROS_INFO_STREAM("reached a wrench subscriber!");
	//places the wrench in the buffer
	geometry_msgs::WrenchStamped my_wrench = *msg.get();
	wrenchBuffer.push_back(my_wrench);
	ROS_DEBUG_STREAM("what i think i am saving" << my_wrench);
	ROS_DEBUG_STREAM("buffer size" <<wrenchBuffer.size());
	while (wrenchBuffer.size()>max_buffer_length)
	{
		wrenchBuffer.pop_front();
	}

}
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	////HERE I ALREADY HAVE THE WRENCH FROM THE RIGHT TIME
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
const geometry_msgs::WrenchStamped Pipeline::WrenchSubscriber::find_wrench_in_buffer(const std_msgs::Header::_stamp_type timestamp)
{
	if (!wrenchBuffer.empty())
	{
		for (auto w:wrenchBuffer)
		{
			//ROS_DEBUG_STREAM("wrenenrenrenrne" << w);
			if (w.header.stamp > timestamp)
				{
					ROS_DEBUG_STREAM("I found something" << w) ;
				return w;
				}
		}
		ROS_DEBUG_STREAM("first time in buffer	:" <<wrenchBuffer.front().header);
		ROS_DEBUG_STREAM("last time in buffer	:" <<wrenchBuffer.back().header);
		ROS_DEBUG_STREAM("desired time 		:" <<timestamp);
		ROS_FATAL_STREAM("Could not find a wrench that matched the desired timestamp. IK is too fast! Or too slow? Idk..");
	}
	return geometry_msgs::WrenchStamped();
}
bool Pipeline::WrenchSubscriber::get_wrench(const std_msgs::Header::_stamp_type timestamp, ExternalWrench::Input* wO )
{
	//find the actual wrench I need based on timestamp
	auto w = find_wrench_in_buffer(timestamp);
	if (w.header.frame_id == "")
	{
		ROS_WARN_STREAM("header frame_id is empty!!!");
		return false;
	}
		ROS_INFO_STREAM("times.. wrench header:"<< w.header << "\nnow: "<<ros::Time::now());
	auto sometime = w.header.stamp; //I hate myself.
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
				auto foot_cop = tfBuffer.lookupTransform(foot_tf_name, calcn_frame, sometime);
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

		nulltransform = tfBuffer.lookupTransform(grf_reference_frame, foot_tf_name, sometime);
		
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
		tf2::fromMsg(w.wrench.force, v_force_orig);
		tf2::fromMsg(w.wrench.torque, v_torque_orig);

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
	ROS_INFO_STREAM("I got some wrench. so far so good.");
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
	OpenSimRT::ExternalWrench::Input* gRw(&grfRightWrench); 
	if(wsR.get_wrench(timestamp, gRw))
		grfRightWrench = *gRw;
	//cout << "left wrench.";
	ROS_INFO_STREAM("rw");
	static OpenSimRT::ExternalWrench::Input grfLeftWrench=nullWrench;
	OpenSimRT::ExternalWrench::Input* gLw(&grfLeftWrench); 
	if(wsL.get_wrench(timestamp, gLw))
		grfLeftWrench = *gLw;
	ROS_DEBUG_STREAM("lw");
	//	return;

	wrenches.push_back(grfLeftWrench);
	wrenches.push_back(grfRightWrench);
	return wrenches;


}

Pipeline::IdAsync::IdAsync(): wsL("left", "calcn_l"), wsR("right", "calcn_r"), seq__(sub__, ros::Duration(0.3),ros::Duration(0.01),1000), seq_filtered__(sub_filtered__, ros::Duration(0.3),ros::Duration(0.01),1000)
{
	ROS_WARN_STREAM("Are you sure you dont want to set up a custom delay????????????????????????????????????????????????????????????????????????????????????");

}

Pipeline::IdAsync::IdAsync(double delay__): wsL("left", "calcn_l"), wsR("right", "calcn_r"), seq__(sub__, ros::Duration(delay__),ros::Duration(0.01),1000), seq_filtered__(sub_filtered__, ros::Duration(delay__),ros::Duration(0.01),1000)
{
	ROS_INFO_STREAM("subscribing with a delay of: "<<delay__<<"s.");
}

Pipeline::IdAsync::~IdAsync()
{
	ROS_INFO_STREAM("Shutting down Id");
}


void Pipeline::IdAsync::onInit() {
	//TODO: this is technically wrong. if I am subscribing to the version with CommonTimed version, then I definetely want the second label as well, but it will fail if I am not subscribing to this, so this flag needs to be set only in that case
	//get_second_label = false;
	Pipeline::IdCommon::onInit();
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//TODO::::: HEre I want to subscribe with a timeSequencer!!!!!!!!!!!!
	//message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub0;
	//sub0.registerCallback(&Pipeline::IdAsync::callback0,this);
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	
	//this is also wrong because it will get destroyed after this is finished...
	//message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub1;
	//sub1.registerCallback(&Pipeline::IdAsync::callback1,this);

	//I will start all the wrenches now:
	wsL.onInit();
	wsR.onInit();


	pub_ik = nh.advertise<opensimrt_msgs::CommonTimed>("debug_ik", 1000);
	//i need the labels and other stuff maybe
	//Ros::CommonNode::onInit();
	sub.unsubscribe();
	sub_filtered.unsubscribe();
	sub__.subscribe(nh, "input", 1000);
	seq__.registerCallback(&Pipeline::IdAsync::callback0, this);	
	sub_filtered__.subscribe(nh, "input_filtered", 1000);
	seq_filtered__.registerCallback(&Pipeline::IdAsync::callback_filtered, this);
	//I need a slow IK with a ton of delay.

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
	ROS_INFO_STREAM("reached ik filtered subscriber. good news");
	auto newEvents1 = addEvent("id received ik filtered & wrenches", message_ik);
	ROS_DEBUG_STREAM("Received message. Running Id filtered loop callback_real_wrenches_filtered"); 
	//cant find the right copy constructor syntax. will for loop it
	addEvent("id: getting iks already filtered.", newEvents1);
	auto iks = Osb::parse_ik_message(message_ik);
	addEvent("id: getting real wrenches", newEvents1);
	auto timestamp =message_ik->header.stamp;
	auto grfs = Pipeline::IdAsync::get_wrench(timestamp);
	addEvent("calling run function of id", newEvents1);
	ROS_INFO_STREAM("calling run function");
	run(message_ik->header,message_ik->time, iks,grfs, newEvents1);
}



