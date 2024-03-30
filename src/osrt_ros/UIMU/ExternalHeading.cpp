#include "osrt_ros/UIMU/ExternalHeading.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Transform.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include "tf2/exceptions.h"
#include "tf2/transform_storage.h"
#include "visualization_msgs/Marker.h"
#include <ostream>
#include <vector>

typedef tf::Vector3 bivector3  ;

bivector3 PLANEXY{0,0,1};

geometry_msgs::Vector3 get_vect_param(ros::NodeHandle handle, std::string name, const std::vector<double>& default_value)
{
	geometry_msgs::Vector3 some_vec;
	std::vector<double> vvv;
	handle.param(name, vvv, default_value);
	some_vec.x =vvv[0];
	some_vec.y =vvv[1];
	some_vec.z =vvv[2];
	ROS_INFO_STREAM(name << " set to:" <<some_vec);
	return some_vec;

}

ExternalHeading::ExternalHeading(): tfListener(tfBuffer)
{
	ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
	nh.param<std::string>("imu_default_frame",imu_default_frame_name, "imu_default_frame");
	nh.param<std::string>("opensim_base_default_frame",opensim_base_default_frame_name, "opensim_default_frame");
	nh.param<std::string>("imu_heading_axis",imu_heading_axis, "-z");
	nh.param<std::string>("imu_base_measured_frame",imu_base_measured_frame_name, "ik/pelvis_imu_raw");
	nh.param<std::string>("added_heading_frame",added_heading_frame_name, "subject_adds_heading2");
	nh.param<std::string>("parent_frame", parent_frame_name, "map");
	std::vector<double> origin_{0,0,0};
	nh.param("origin", origin_, {0,0,0});
	origin.x = origin_[0];
	origin.y = origin_[1];
	origin.z = origin_[2];
	ROS_INFO_STREAM("origin set to:" <<origin);
	//I gotta parse this axis into something I can use. 
	//right now I will just fail if it isnt -z
	if (imu_heading_axis.compare("-z")!=0)
		ROS_ERROR_STREAM("I havent implemented anything other than -z yet. Using heading -z");

	//we should normalize and choose a color here. also publish it here probably.

	heading = get_vect_param(nh,"heading", {0,0,-1});
	opensim_heading = get_vect_param(nh,"opensim_heading", {0,1,0});
	
}


bool ExternalHeading::calibrate_srv(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
{
	calibrate();
	return true;
}

visualization_msgs::Marker ExternalHeading::getArrowForVector(std::string name, geometry_msgs::Vector3 vv, geometry_msgs::Point origin_)
{

		visualization_msgs::Marker m;
		m.header.stamp = ros::Time::now();
		m.header.frame_id = parent_frame_name;
		m.ns = name;
		m.id = 0;
		m.type = visualization_msgs::Marker::ARROW;
		m.action = visualization_msgs::Marker::ADD;
		//geometry_msgs::Point a,b;


		m.points.push_back(origin_);
		geometry_msgs::Point pp;
		pp.x = vv.x;
		pp.y = vv.y;
		pp.z = vv.z;
		m.pose.orientation.w=1;

		m.points.push_back(pp);
		//m.points.push_back(a);
		//m.points.push_back(b);

		m.color.a = 1.0; // Don't forget to set the alpha!
		m.color.r = 0.0;
		m.color.g = 0.0;
		m.color.b = 1.0; // cause I want it to be like z, right.
		m.scale.x = 0.01;
		m.scale.y = 0.02;
		m.scale.z = 0.05;


	return m;
}
visualization_msgs::Marker ExternalHeading::getArrowForVector(std::string name, geometry_msgs::Vector3 vv)
{
	return getArrowForVector(name, vv, origin);
}

geometry_msgs::Point vector_to_point(geometry_msgs::Vector3 vector)
{
	geometry_msgs::Point p;
	p.x= vector.x;
	p.y= vector.y;
	p.z= vector.z;
	
	ROS_INFO_STREAM(yellow << vector << "\n" << green << p << reset);
	return p;
}
geometry_msgs::Vector3 vtov(tf::Vector3 result)
{
	geometry_msgs::Vector3 res;
	res.x = result.getX();
	res.y = result.getY();
	res.z = result.getZ();
	return res;

}
geometry_msgs::Vector3 project_on_plane(geometry_msgs::Vector3 vv, bivector3 a_plane)
{

	tf::Vector3 v{vv.x,vv.y,vv.z};

	auto result = v - tf::tfDot(v, a_plane)*a_plane;

	return vtov(result);
}

double angle_between_vectors(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
	tf::Vector3 A{a.x,a.y,a.z};
	tf::Vector3 B{b.x,b.y,b.z};

	return tf::tfAngle(A,B);
}


geometry_msgs::Vector3 pick_heading_from_tf(geometry_msgs::Transform some_frame, const geometry_msgs::Vector3 some_heading)
{
	//maybe I will tranform something
	tf::Quaternion qq;
	tf::quaternionMsgToTF(some_frame.rotation,qq);
	
	//this will change my original vector, i dont want that
	tf::Vector3 some_copy{ some_heading.x, some_heading.y, some_heading.z};
	
	some_copy = tf::quatRotate(qq,some_copy);
	tf::Vector3 offset{some_frame.translation.x,some_frame.translation.y,some_frame.translation.z};

	ROS_INFO_STREAM(magenta << some_heading << green << some_copy <<reset);
	return vtov(some_copy+offset);

}

double ExternalHeading::calculate_angle(geometry_msgs::Transform default_measure_frame, geometry_msgs::Transform measured_frame, geometry_msgs::Vector3 imu_heading_axis_vector)
{


	auto default_os_ori = vector_to_point( default_measure_frame.translation);
	//maybe I will tranform something
	auto heading_os_default = pick_heading_from_tf(default_measure_frame,opensim_heading);

	auto m = getArrowForVector("heading_os_default", heading_os_default, default_os_ori );
	m.color.b = 0, m.color.g =1;
	debug_markers.push_back(m);
	debug_point = vector_to_point(imu_heading_axis_vector);
	
	auto heading_imu_measured = pick_heading_from_tf(measured_frame,imu_heading_axis_vector);

	auto measure_ori = vector_to_point(measured_frame.translation);
	debug_markers.push_back(getArrowForVector("heading_imu_measured", heading_imu_measured, measure_ori));

	geometry_msgs::Vector3 projected_measured_heading = project_on_plane(heading_imu_measured, PLANEXY);
	
	debug_markers.push_back(getArrowForVector("heading_imu_measured_projected", projected_measured_heading, measure_ori));
	

	//is it just the angle with y from map?
	// then it's just getyaw from tf?
	//
	//and if there is a minus sign, then i just add 180, whatever, this is the crazy debugging, so let's go all out
	//
	auto WTH = vtov(tf::Vector3{opensim_heading.x,opensim_heading.y,opensim_heading.z});
	tf::Quaternion qqq;
	tf::quaternionMsgToTF(measured_frame.rotation,qqq)	;
	double aa = (tf::getYaw(qqq));
	ROS_INFO_STREAM("yaw,,," << aa*180/3.1415);
	
	//im too stupid for this.
	return angle_between_vectors(WTH,projected_measured_heading);
	


	//debug_markers.push_back(getArrowForVector("heading_imu_measured_normal", heading_imu_measured - projected_measured_heading,measure_ori ));

	return angle_between_vectors(projected_measured_heading,heading_os_default);
}

geometry_msgs::PoseStamped ExternalHeading::calibrate()
{
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::PoseStamped heading_pose;

	//get the imu default frame, the opensim default frame and the current imu base measured frame:
	try{
		/*geometry_msgs::TransformStamped imu_default_frame = tfBuffer.lookupTransform(imu_default_frame_name, parent_frame_name, ros::Time(0));
		geometry_msgs::TransformStamped opensim_default_frame = tfBuffer.lookupTransform(opensim_base_default_frame_name, parent_frame_name, ros::Time(0));
		geometry_msgs::TransformStamped imu_base_measured_frame = tfBuffer.lookupTransform(imu_base_measured_frame_name, parent_frame_name, ros::Time(0));
		*/
		geometry_msgs::TransformStamped imu_default_frame = tfBuffer.lookupTransform(parent_frame_name, imu_default_frame_name, ros::Time(0));
		geometry_msgs::TransformStamped opensim_default_frame = tfBuffer.lookupTransform(parent_frame_name, opensim_base_default_frame_name, ros::Time(0));
		geometry_msgs::TransformStamped imu_base_measured_frame = tfBuffer.lookupTransform(parent_frame_name, imu_base_measured_frame_name, ros::Time(0));

		

		ROS_INFO_STREAM(green << "imu_default_frame"<<imu_default_frame.transform.rotation <<reset);
		ROS_INFO_STREAM("imu_base_measured_frame"<< imu_base_measured_frame.transform.rotation);

		heading_pose.header.stamp = ros::Time::now();
		heading_pose.pose.position = origin;

		//now the important part, calculate this damn angle

		//double heading_angle = calculate_angle(imu_default_frame.transform, imu_base_measured_frame.transform, heading);
		double heading_angle = calculate_angle(opensim_default_frame.transform, imu_base_measured_frame.transform, heading);

		ROS_INFO_STREAM(cyan << heading_angle*180/3.1415 <<red <<" degrees"<< reset);
		auto q_heading = tf::createQuaternionFromYaw(heading_angle);

		heading_pose.pose.orientation.w = q_heading.w();
		heading_pose.pose.orientation.x = q_heading.x();
		heading_pose.pose.orientation.y = q_heading.y();
		heading_pose.pose.orientation.z = q_heading.z();

		//send it as a tf for others to use
		geometry_msgs::TransformStamped heading_tf;
		heading_tf.header.stamp = ros::Time::now();
		heading_tf.header.frame_id = parent_frame_name;
		heading_tf.child_frame_id = added_heading_frame_name;

		heading_tf.transform.translation.x = origin.x;
		heading_tf.transform.translation.y = origin.y;
		heading_tf.transform.translation.z = origin.z;

		heading_tf.transform.rotation = heading_pose.pose.orientation;
		br.sendTransform(heading_tf);
	}
	catch(tf2::TransformException &ex) 
	{
		ROS_INFO_STREAM("oops");
	}
	return heading_pose;

}
