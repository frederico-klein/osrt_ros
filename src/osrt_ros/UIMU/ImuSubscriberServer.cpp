// Server side implementation of a ROS TF server model

#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "osrt_ros/UIMU/ImuSubscriberServer.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/tf.h"
#include "tf/time_cache.h"
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

OneImuSubscriber::OneImuSubscriber(std::string used_name)
	{
		ros::NodeHandle n;
		ros::Subscriber a_subscriber = n.subscribe(used_name,1,&OneImuSubscriber::callback, this);

	}
void OneImuSubscriber::callback(const sensor_msgs::ImuConstPtr &msg)
	{
		last_message = *msg;
	}

// Driver code
ImuSubscriberServer::ImuSubscriberServer (std::vector<std::string> topic_names, std::string topic_prefix): listener(tfBuffer) {

	ROS_INFO("Using Imu Subscriber server");
	///read one tf and make it for everyone
	set_subscribers(topic_names, topic_prefix);

}

ImuSubscriberServer::~ImuSubscriberServer(void)
{
}

void ImuSubscriberServer::set_subscribers(std::vector<std::string> topic_names, std::string topic_prefix)
{
	ros::NodeHandle n;
	//topic_strs = topic_names;
	if (topic_names.size() == 0)
		ROS_FATAL("NO TOPIC NAMES");
	for (auto a_topic_name:topic_names)
	{
		std::string used_name = topic_prefix+"/" + a_topic_name+topic_suffix; //also ask for suffix?§
		ROS_DEBUG_STREAM("ImuSubscriberServer: Using topic name: " << used_name);
		
		topic_strs.insert(std::make_pair(used_name, OneImuSubscriber(used_name)));
	}
}

void ImuSubscriberServer::set_world_reference(std::string world_name)
{
	world_tf_reference = world_name;
	ROS_INFO("Setting world_tf_reference to %s", world_tf_reference.c_str());	
}

//std::vector<double> ImuSubscriberServer::receive()
bool ImuSubscriberServer::receive()
{
	ROS_DEBUG_STREAM("imu topic subscriber");
	std::vector<double> combined_imu_data_vec;
	// now i need to set this combined_imu_data_vec with the values i read for the quaternions somehow
	//std::vector<std::string> topic_strs = {"/a", "b", "c"};	
	double time = ros::Time::now().toSec();// i need to get this from the transform somehow
	combined_imu_data_vec.push_back(time);

	for (auto i:topic_strs)
	{
		// get from a map, right?
		std::vector<double> anImu = readTopicsIntoOpensim(i.first);
		combined_imu_data_vec.insert(combined_imu_data_vec.end(),anImu.begin(), anImu.end());
	}
	//for( auto i:combined_imu_data_vec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	//ROS_DEBUG_STREAM("THIS THING:" << combined_imu_data_vec.size());

	output = combined_imu_data_vec;
	return true;	

}

tf::Quaternion ImuSubscriberServer::calculate_q_from_sensor(const sensor_msgs::Imu last_imu_message)
{
	tf::Quaternion imu_q;

	tf::StampedTransform transform;
	//geometry_msgs::TransformStamped transform;

	// try setting the transform from the imu if it doesnt exist:
	//
	// 
	//
	// use something like:
	// tfBuffer.setTransform()
	//
	//maybe i need to check with cantransofrm before that... let's hope the imu node publishes the tf too.


	std::string tf_name = last_imu_message.header.frame_id;
	try{
		// the correct way here is to send to buffer and then compute.
		tf1listener.waitForTransform(tf_name, world_tf_reference, ros::Time(0), ros::Duration(3.0));
		tf1listener.lookupTransform(world_tf_reference, tf_name, ros::Time(0), transform); 
	
		imu_q = transform.getRotation();

		//listener.lookupTransform(tf_name, world_tf_reference, ros::Time(0), transform); //flipped, new attempt to try to avoid -w
	}
	catch (tf::TransformException& ex){
		ROS_ERROR("Transform exception! %s",ex.what());
		ROS_FATAL("not implemented calculation from buffer!!!!!!!!");
	}
	// now i need to set this imu_data_vec with the values i read for the quaternions somehow


	return imu_q;
}


tf::Quaternion get_q_from_imu(const sensor_msgs::Imu last_imu_message )
{
	tf::Quaternion imu_q;
		// then the quaternion is already correct
		imu_q.setW(last_imu_message.orientation.w);
		imu_q.setX(last_imu_message.orientation.x);
		imu_q.setY(last_imu_message.orientation.y);
		imu_q.setZ(last_imu_message.orientation.z);

	return imu_q;
	

}


std::vector<double> ImuSubscriberServer::readTopicsIntoOpensim(std::string an_imu_topic_name)
{
	ROS_DEBUG_STREAM("Trying to find the IMU values from  " << an_imu_topic_name);

	ROS_DEBUG_STREAM("getting the transform part! Note, this will be calculated to world_tf_reference frame!!");
	
	sensor_msgs::Imu last_imu_message = topic_strs.find(an_imu_topic_name)->second.last_message;	
	
	tf::Quaternion imu_q;
	if (last_imu_message.header.frame_id == world_tf_reference)
	{
		imu_q = get_q_from_imu(last_imu_message);
	}
	else
	{
		//needs to be calculated. pain!
		imu_q = calculate_q_from_sensor(last_imu_message);
	}

	ROS_DEBUG_STREAM("an_imu_topic_name: " << an_imu_topic_name << " with transform:\n " 	
			<< "w: " << imu_q.getW() 
			//<< " " << imu_q.w() 
			<< "\n" 
			<< "x: " << imu_q.getX() 
			//<< " " << imu_q.x() 
			<< "\n" 
			<< "y: " << imu_q.getY() 
			//<< " " << imu_q.y() 
			<< "\n"
			<< "z: " << imu_q.getZ() 
			//<< " " << imu_q.z() 
			<< "\n");
	//this is converting from ROS quaternions to OPENSIM quaternions. Is this correct?
	
	std::vector<double> imu_data_vec;

	imu_data_vec.push_back(imu_q.w());
	imu_data_vec.push_back(imu_q.x());
	imu_data_vec.push_back(imu_q.y());
	imu_data_vec.push_back(imu_q.z());
	// now it is a bunch of zeros
	
	imu_data_vec.insert(imu_data_vec.end(), 14, -1.010 );

	
	//for( auto i:imu_data_vec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	//ROS_INFO_STREAM("THIS THING:" << imu_data_vec.size());

	//indexes of the fields can be seen in ./UIMUInputDriver.cpp
	//ROS isn't very clear on IMUs, or wasnt at least in ROS1, 
	//see https://answers.ros.org/question/255867/sensor_msgimu-format-specification/
	//and
	//https://github.com/ros-infrastructure/rep/pull/95
	//
	//apparently linear acceleration is just acceleration, i.e. it has gravity (linear acceleration is supposed to have gravity removes) this would maybe be okay if you always had orientation, but that is not the case and some algorithms use this same message with no orientation for imu_raw and then publish another one with orientation filled out from the results of either dead reckoning or a kalman filter type thing. whatever. 
	//in this case here, i am going to use meaning it is just
	//acceleration, i.e. this data will have gravity
	//
	//in the case of ximu3 you can set the ahrs to give you the actual linear acceleration values and if you want to be fancy, you have another node publishing the orientation, this way you would have everything albeit paying the cost of needing to run a bunch of nodes fdoing the kalman filter/dead reckoining computations on the host for each imu you have. 
	//
	//But all docs about this are rather old, this might have changed in ros2 and it's worth noting for migration
	//
	//tl,dr: linear_acceration field here is used to mean accelerometer raw values. This (it seems) is a common way of using sensor_msgs::Imu by the time this was written. 
	//
	imu_data_vec[5] = last_imu_message.linear_acceleration.x;
	imu_data_vec[6] = last_imu_message.linear_acceleration.y;
	imu_data_vec[7] = last_imu_message.linear_acceleration.z;
	imu_data_vec[8] = last_imu_message.angular_velocity.x;
	imu_data_vec[9] = last_imu_message.angular_velocity.y;
	imu_data_vec[10] = last_imu_message.angular_velocity.z;




	return imu_data_vec;

}

