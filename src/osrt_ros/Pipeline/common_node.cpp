#include "osrt_ros/Pipeline/common_node.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>

//constructor
Pipeline::CommonNode::CommonNode(bool Debug)
{
	if( Debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
} 
void Pipeline::CommonNode::onInit(int num_sinks)
{
	pub = nh.advertise<opensimrt_msgs::CommonTimed>("output", 1000);
	if (publish_filtered)
		pub_filtered = nh.advertise<opensimrt_msgs::PosVelAccTimed>("output_filtered", 1000);
	outLabelsSrv = nh.advertiseService("out_labels", &CommonNode::outLabels, this);

	if (num_sinks == 1)
	{
		ROS_INFO_STREAM("registering callback");
		//register the callback
		sub.subscribe(nh, "input",10);
		sub.registerCallback(&CommonNode::callback,this);
		//sub = nh.subscribe("input",5, &CommonNode::callback, this);
	}
	else
	{
		ROS_INFO_STREAM("not single_sink, callback isnt registered yet!");
	}
	ros::Rate r(1);
	//output_labels = qTable.getColumnLabels();
	opensimrt_msgs::LabelsSrv l;
	//while(!ros::service::call("in_labels", l))
	if (num_sinks > 0)
	{
		while(ros::ok())
		{
			if(ros::service::call("in_labels", l))
			{
				input_labels = l.response.data;
				break;
			}
			ros::spinOnce();
			ROS_INFO_STREAM("Waiting to read input labels."); 
			r.sleep();
		}
	}
	else if (num_sinks==0)
	{
		ROS_INFO_STREAM("Source node registered (num_sinks = 0)");	    

	}
	else
	{
		ROS_ERROR_STREAM("Unexpected number of sinks:" << num_sinks);

	}
	write_csv = nh.advertiseService("write_csv", &CommonNode::writeCsv, this);
	write_sto = nh.advertiseService("write_sto", &CommonNode::writeSto, this);


}

//destructor
Pipeline::CommonNode::~CommonNode()
{
	if(!at_least_one_logger_initialized)
		ROS_ERROR_STREAM("You forgot to initialize loggers!");
}
bool Pipeline::CommonNode::outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res )
{
	res.data = output_labels;
	published_labels_at_least_once = true;
	ROS_INFO_STREAM("CALLED LABELS SRV");
	return true;
}


void Pipeline::CommonNode::initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger)
{
	//adds loggers to loggers
	//ROS_ERROR_STREAM("initialization not implemented!");
	at_least_one_logger_initialized = true;
	ROS_INFO_STREAM("At least one logger was initialized! Will be able to save this with service");
	NamedTable this_named_table = std::make_pair(logger,logger_name);	
	loggers.push_back(this_named_table);

}
bool Pipeline::CommonNode::writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Csv service called.");
	saveCsvs();
	return true;
}
bool Pipeline::CommonNode::writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Sto service called.");
	saveStos();
	return true;
}
void Pipeline::CommonNode::saveStos()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_dir()+named_table.second+".sto";
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::STOFileAdapter::write(*named_table.first, loggerfilename);
	}
}
void Pipeline::CommonNode::saveCsvs()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_dir()+named_table.second+".csv";
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::CSVFileAdapter::write(*named_table.first, loggerfilename);
	}

}

