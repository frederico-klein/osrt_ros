// Server side implementation of a ROS TF server model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "osrt_ros/UIMU/TfServer.h"

#include <iostream>
#include <sstream>
#include <iterator>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>


// Driver code
TfServer::TfServer (std::vector<std::string> tf_names ) {

	ROS_INFO("Using tf server");
	///read one tf and make it for everyone
	tf::TransformListener listener;
	set_tfs(tf_names);

}

TfServer::~TfServer(void)
{
}

void TfServer::set_tfs(std::vector<std::string> tf_names)
{

	tf_strs = tf_names;
	if (tf_names.size() == 0)
		ROS_FATAL("NO TF NAMES");
	for (auto i:tf_strs)
		ROS_INFO_STREAM("Using reference frame: " << i);
}

void TfServer::set_world_reference(std::string world_name)
{
	world_tf_reference = world_name;
	ROS_INFO("Setting world_tf_reference to %s", world_tf_reference.c_str());	
}

//std::vector<double> TfServer::receive()
bool TfServer::receive()
{
	ROS_DEBUG_STREAM("not simple");
	std::vector<double> myvec;
	// now i need to set this myvec with the values i read for the quaternions somehow
	//std::vector<std::string> tf_strs = {"/a", "b", "c"};	
	double time = ros::Time::now().toSec();// i need to get this from the transform somehow
	myvec.push_back(time);

	for (auto i:tf_strs)
	{
		std::vector<double> anImu = readTransformIntoOpensim(i);
		myvec.insert(myvec.end(),anImu.begin(), anImu.end());
	}
	//for( auto i:myvec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	ROS_DEBUG_STREAM("THIS THING:" << myvec.size());

	output = myvec;
	return true;	

}

std::vector<double> TfServer::readTransformIntoOpensim(std::string tf_name)
{
	ROS_DEBUG_STREAM("Trying to find transform " << tf_name);
	//rosreceive its wrong it should be on a callback basis, 
	tf::StampedTransform transform;
	try{
		/*listener.waitForTransform("/map", tf_name, ros::Time(0), ros::Duration(3.0));
		  listener.lookupTransform("/map", tf_name,
		  ros::Time(0), transform);*/
		listener.waitForTransform(world_tf_reference, tf_name, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform(world_tf_reference, tf_name,
				ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		//ros::Duration(1.0).sleep();
	}
	std::vector<double> myvec;
	// now i need to set this myvec with the values i read for the quaternions somehow

	auto myq = transform.getRotation();
	myvec.push_back(myq.z());
	myvec.push_back(myq.x());
	myvec.push_back(myq.y());
	myvec.push_back(myq.w());
	// now it is a bunch of zeros
	myvec.insert(myvec.end(), 14, -0.010 );
	//for( auto i:myvec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	//ROS_INFO_STREAM("THIS THING:" << myvec.size());

	return myvec;

}

