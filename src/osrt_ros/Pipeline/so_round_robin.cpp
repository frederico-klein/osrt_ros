#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/so.h"
#include "osrt_ros/Pipeline/so_bare.h"
#include "osrt_ros/Pipeline/so_rr.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>
#include <vector>


Pipeline::SoRR::SoRR(const ros::NodeHandle& node_handle, const int num_processes)
	: node_handle_(node_handle), num_processes_(num_processes)
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	//main_subs_ = node_handle_.subscribe("chatter",1000, &SoRR::roundRobin, this);
	for (int i=0; i<num_processes_; i++)
	{
		std::string pub_name = "so_rr"+std::to_string(i);
		Pipeline::SoBare aso;
		sos.push_back(aso);
		pubs_.push_back(node_handle_.advertise<opensimrt_msgs::Dual>(pub_name,1000)); //publishes the input values for each thread
		pubs_filtered.push_back(node_handle_.advertise<opensimrt_msgs::DualPos>(pub_name+"_filtered",1000)); //publishes the input values for each thread
	}
	outcome_pub = node_handle_.advertise<opensimrt_msgs::CommonTimed>("output",1000); //publishes the combined SO values
}

void Pipeline::SoRR::init()
{
	ROS_DEBUG_STREAM("onInitSoRR");
	nh.getParam("get_second_label", get_second_label);
	//get_second_label = false;
	Pipeline::DualSink::onInit();

	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	// TODO: set the column labels, or it will break when you try to use them!
	ROS_INFO_STREAM("Attempting to set loggers.");
	//initializeLoggers("grfRight",grfRightLogger);
	//initializeLoggers("grfLeft", grfLeftLogger);

	message_filters::TimeSynchronizer<opensimrt_msgs::CommonTimed, opensimrt_msgs::CommonTimed> sync(sub, sub2, 500);
	sync.registerCallback(std::bind(&Pipeline::SoRR::callback, this, std::placeholders::_1, std::placeholders::_2));
	sync.registerCallback(&Pipeline::SoRR::callback, this);

	//these need to be shared with the rest:
	for (int i=0; i<num_processes_; i++)
	{
		std::string sub_name= "so_rr"+std::to_string(i);
		subs_.push_back(node_handle_.subscribe<opensimrt_msgs::Dual>(sub_name, 1000, boost::bind(&Pipeline::SoRR::so_rrCallback, this, boost::placeholders::_1, i)));
		subs_filtered.push_back(node_handle_.subscribe<opensimrt_msgs::DualPos>(sub_name+"_filtered", 1000, boost::bind(&Pipeline::SoRR::so_rr_filteredCallback, this, boost::placeholders::_1, i)));
	}
}


void Pipeline::SoRR::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
	//		void Pipeline::SoRR::roundRobin(const std_msgs::String::ConstPtr& msg)
{
	//generates from 2 messages only one message
	opensimrt_msgs::Dual msg;
	msg.q = *message_ik;
	msg.tau = *message_tau;
	pubs_[counter%num_processes_].publish(msg);
	counter++;


}

void Pipeline::SoRR::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
	//		void Pipeline::SoRR::roundRobin(const std_msgs::String::ConstPtr& msg)
{
	//generates from 2 messages only one message
	opensimrt_msgs::DualPos msg;
	msg.qqq = *message_ik;
	msg.tau = *message_tau;
	pubs_filtered[counter%num_processes_].publish(msg);
	counter++;


}

void Pipeline::SoRR::so_rrCallback(const opensimrt_msgs::DualConstPtr& d, int process)
{
	// I need to generate the things to call SoRR here
	//TODO: I can register this event as well \--/
	SimTK::Vector q(d->q.data.size());	
	for(int j=0;j<q.size();j++)
	{
		q[j] = d->q.data[j];
	}
	std::vector<double> tau;
	for(int j=0;j<d->tau.data.size();j++)
	{
		tau.push_back(d->tau.data[j]);
	}
	runRR(d->q.header, d->q.time, q, tau, d->q.events, process);
}
void Pipeline::SoRR::so_rr_filteredCallback(const opensimrt_msgs::DualPosConstPtr& d, int process)
{
	// I need to generate the things to call SoRR here
	//TODO: I can register this event as well \--/
	SimTK::Vector q(d->qqq.d0_data.size());	
	for(int j=0;j<q.size();j++)
	{
		q[j] = d->qqq.d0_data[j];
	}
	std::vector<double> tau;
	for(int j=0;j<d->tau.data.size();j++)
	{
		tau.push_back(d->tau.data[j]);
	}
	runRR(d->qqq.header, d->qqq.time, q, tau, d->qqq.events, process);

}
//this is now runRR and it will call run from SO
void Pipeline::SoRR::runRR(const std_msgs::Header h, double t, SimTK::Vector q, const std::vector<double> tau, opensimrt_msgs::Events e , int process)
{
	ROS_INFO_STREAM("I heard: [something...] in [" << process << "] [thread=" << boost::this_thread::get_id() << "]");
	ROS_INFO("This is a long thread and takes long to execute each thing");
	//I kinda need an output though.
	opensimrt_msgs::CommonTimed out_msg;
	//run actual process
	out_msg = sos[process].run(h,t, q,tau,e);
	
	out_msg.header = h;
	out_msg.events = e;
	SimTK::Vector so_am(2);
	SimTK::Vector so_fm(2);
	ROS_INFO_STREAM("Thread " << process << " finished exectution. [thread=" << boost::this_thread::get_id() <<"]"); 
	//out_msg.data = outcome_of_thread.str();

	outcome_pub.publish(out_msg);
}


