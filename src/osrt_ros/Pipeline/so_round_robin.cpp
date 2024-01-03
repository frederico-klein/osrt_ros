#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "opensimrt_msgs/MultiMessagePosVelAcc.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
#include "osrt_ros/Pipeline/so.h"
#include "osrt_ros/Pipeline/so_bare.h"
#include "osrt_ros/Pipeline/so_rr.h"
#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <SimTKcommon/internal/Vec.h>
#include <vector>


Pipeline::SoRR::SoRR(const ros::NodeHandle& node_handle, const int num_processes)
	: Pipeline::DualSink::DualSink(false), node_handle_(node_handle), num_processes_(num_processes)
{
	if (false)
	{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
		{
			ros::console::notifyLoggerLevelsChanged();
		}
	}
	//main_subs_ = node_handle_.subscribe("chatter",1000, &SoRR::roundRobin, this);
	ROS_WARN_STREAM("Creating process vector and internal publishers");
	sos.reserve(num_processes_);
	for (int i=0; i<num_processes_; i++)
	{
		std::string pub_name = "so_rr/proc_"+std::to_string(i);
		sos.emplace_back(i);
		pubs_.push_back(node_handle_.advertise<opensimrt_msgs::Dual>(pub_name+"/raw",1000)); //publishes the input values for each thread
		pubs_filtered.push_back(node_handle_.advertise<opensimrt_msgs::DualPos>(pub_name+"/filtered",1000)); //publishes the input values for each thread
	}
	ROS_WARN_STREAM("finished creating process vector and publishers.");
	outcome_pub = node_handle_.advertise<opensimrt_msgs::Dual>("output_combined",1000); //publishes the combined SO values
	outcome_multi_pub = node_handle_.advertise<opensimrt_msgs::MultiMessage>("output_multi",1000); //publishes the combined SO values

	//not sure what makes sense here, but I need to set the labels for these guys. actually I know what makes sense and is easy: setting an extra field in the messages with the labels and sending those every time, this would be definitely easier. 
	//Oh well, a decision for future dev to make.
	//
	std::string modelFile = "";
	nh.param<std::string>("model_file",modelFile,"");
	auto model = new OpenSim::Model(modelFile);
	model->initSystem();
	auto muscleNames = model->getMuscles();
	for (size_t i=0; i < muscleNames.getSize(); i++)
	{
		auto thisMuscleName = muscleNames.get(i).getName(); 
		ROS_WARN_STREAM("muscle ["<<i<<"] is:" << thisMuscleName);
		output.labels.push_back(thisMuscleName);	
	}

}

void Pipeline::SoRR::onInit()
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
		std::string sub_name= "so_rr/proc_"+std::to_string(i);
		subs_.push_back(node_handle_.subscribe<opensimrt_msgs::Dual>(sub_name+"/raw", 1000, boost::bind(&Pipeline::SoRR::so_rrCallback, this, boost::placeholders::_1, i)));
		subs_filtered.push_back(node_handle_.subscribe<opensimrt_msgs::DualPos>(sub_name+"/filtered", 1000, boost::bind(&Pipeline::SoRR::so_rr_filteredCallback, this, boost::placeholders::_1, i)));
	}

	//set the logger. 
	//
	soLogger = new OpenSim::TimeSeriesTable;
	soLogger->setColumnLabels(output.labels);
	initializeLoggers("so", soLogger);
}


void Pipeline::SoRR::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
	//		void Pipeline::SoRR::roundRobin(const std_msgs::String::ConstPtr& msg)
{
	//generates from 2 messages only one message
	ROS_DEBUG_STREAM("callback called ik, tau");
	opensimrt_msgs::Dual msg;
	addEvent("so_rr: received msg_id",message_ik);
	msg.q = *message_ik;
	msg.tau = *message_tau;
	auto bothEvents = combineEvents(message_ik,message_tau);
	msg.q.events = bothEvents;
	msg.tau.events = bothEvents;		
	pubs_[counter%num_processes_].publish(msg);
	counter++;


}

void Pipeline::SoRR::callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_tau) 
	//		void Pipeline::SoRR::roundRobin(const std_msgs::String::ConstPtr& msg)
{
	//generates from 2 messages only one message
	ROS_DEBUG_STREAM("callback called ik_filtered, tau");
	opensimrt_msgs::DualPos msg;
	addEvent("so_rr: received msg_id",message_ik);
	msg.qqq = *message_ik;
	msg.tau = *message_tau;
	auto bothEvents = combineEvents(message_ik,message_tau);
	msg.qqq.events = bothEvents;
	msg.tau.events = bothEvents;		
	pubs_filtered[counter%num_processes_].publish(msg);
	counter++;


}
void Pipeline::SoRR::sync_callback(const opensimrt_msgs::MultiMessageConstPtr &message)
{
	ROS_ERROR_STREAM_ONCE("callback called sync_multi");
	opensimrt_msgs::DualPos msg;
	auto newEvents = addEvent("so_rr: received msg_id multi_ik",message);
	msg.qqq.d0_data = message->ik.data;
	msg.qqq.header = message->header;
	msg.qqq.time = message->time;
	msg.tau.data = message->other[0].data;
	msg.tau.header = message->header;
	msg.qqq.events = newEvents;
	msg.tau.events = newEvents;		
	pubs_filtered[counter%num_processes_].publish(msg);
	counter++;
}
void Pipeline::SoRR::sync_callback_filtered(const opensimrt_msgs::MultiMessagePosVelAccConstPtr &message)
{
	ROS_DEBUG_STREAM("callback called sync_multi_filtered");
	opensimrt_msgs::DualPos msg;
	addEvent("so_rr: received msg_id multiPos",message);
	msg.qqq.d0_data = message->d0_data.data; //arg, this is horrible, the same name different content 
	//msg.qqq.d1_data = message->d1_data.data;
	//msg.qqq.d2_data = message->d2_data.data;
	msg.qqq.header = message->header;
	msg.qqq.time = message->time;
	msg.tau.data = message->other[0].data;
	msg.tau.header = message->header;
	msg.qqq.events = message->events;
	msg.tau.events = message->events;
	pubs_filtered[counter%num_processes_].publish(msg);
	counter++;

}

void Pipeline::SoRR::so_rrCallback(const opensimrt_msgs::DualConstPtr& d, int process)
{
	//This is super weird and maybe wrong. the simplest thing was to have each sos have its own callback and bind it to that, but I bound it to a common function and then I distribute it again using runRR...
	ROS_DEBUG_STREAM("so_rrCallback called dual ["<< process << "] [thread=" << boost::this_thread::get_id() << "] time:" <<std::fixed << std::setprecision(6)<< d->q.time);

	
	// I need to generate the things to call SoRR here
	//TODO: I can register this event as well \--/
	SimTK::Vector q(d->q.data.size());	
	for(size_t j=0;j<q.size();j++)
	{
		q[j] = d->q.data[j];
	}
	std::vector<double> tau;
	for(size_t j=0;j<d->tau.data.size();j++)
	{
		tau.push_back(d->tau.data[j]);
	}
	runRR(d->q.header, d->q.time, q, tau, d->q.events, process);
}
void Pipeline::SoRR::so_rr_filteredCallback(const opensimrt_msgs::DualPosConstPtr& d, int process)
{
	ROS_DEBUG_STREAM("so_rrCallback called dual_filtered ["<< process << "] [thread=" << boost::this_thread::get_id() << "]time:"<<std::fixed << std::setprecision(6) << d->qqq.time);
	// I need to generate the things to call SoRR here
	//TODO: I can register this event as well \--/
	SimTK::Vector q(d->qqq.d0_data.size());	
	for(size_t j=0;j<q.size();j++)
	{
		q[j] = d->qqq.d0_data[j];
	}
	std::vector<double> tau;
	for(size_t j=0;j<d->tau.data.size();j++)
	{
		tau.push_back(d->tau.data[j]);
	}
	runRR(d->qqq.header, d->qqq.time, q, tau, d->qqq.events, process);

}
//this is now runRR and it will call run from SO
void Pipeline::SoRR::runRR(const std_msgs::Header h, double t, SimTK::Vector q, const std::vector<double> tau, opensimrt_msgs::Events e , int process)
{
	ROS_DEBUG_STREAM("I heard: [something...] in [" << process << "] [thread=" << boost::this_thread::get_id() << "]");
	//ROS_DEBUG_STREAM("This is a long thread and takes long to execute each thing");
	
	//I kinda need an output though.
	//run actual process
	//auto out_msg = sos[process].run(h,t, q,tau,e);
	opensimrt_msgs::MultiMessage out_msg = sos[process].run2(h,t, q,tau,e);
	
	//out_msg.events = e;
	ROS_DEBUG_STREAM("Thread " << process << " finished execution. [thread=" << boost::this_thread::get_id() <<"]"); 
	//out_msg.data = outcome_of_thread.str();

	//outcome_pub.publish(out_msg);
	//
	//add the output to the logger so it can be saved. 
	//
	ROS_DEBUG_STREAM("t: " << t);
	

	//this does not work because while tableseries can handle unevenly spaced data, it cannot handle out of order data
	//
	//soLogger->appendRow(t, out_msg.other[0].data);
	//publish it
	outcome_multi_pub.publish(out_msg);

}


