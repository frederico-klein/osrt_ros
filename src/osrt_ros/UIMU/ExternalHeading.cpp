#include "osrt_ros/UIMU/ExternalHeading.h"
#include "geometry_msgs/Transform.h"
#include "ros/time.h"
#include "tf/transform_datatypes.h"
#include "tf2/transform_storage.h"
#include <ostream>
#include <vector>

ExternalHeading::ExternalHeading(): tfListener(tfBuffer)
{
	ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
	nh.param<std::string>("imu_default_frame",imu_default_frame_name, "imu_default_frame");
	nh.param<std::string>("opensim_base_default_frame",opensim_base_default_frame_name, "opensim_default_frame");
	nh.param<std::string>("imu_heading_axis",imu_heading_axis, "-z");
	nh.param<std::string>("imu_base_measured_frame",imu_base_measured_frame_name, "imu/pelvis");
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
}
bool ExternalHeading::calibrate_srv(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res)
{
	calibrate();
	return true;
}

double ExternalHeading::calculate_damn_angle(geometry_msgs::Quaternion default_measure_frame, geometry_msgs::Quaternion measured_frame, geometry_msgs::Quaternion new_default_frame)
{
	ROS_FATAL_STREAM("not implemented");
	return 0.0;
}

geometry_msgs::PoseStamped ExternalHeading::calibrate()
{
	geometry_msgs::TransformStamped transformStamped;

	//get the imu default frame, the opensim default frame and the current imu base measured frame:

	geometry_msgs::TransformStamped imu_default_frame = tfBuffer.lookupTransform(imu_default_frame_name, parent_frame_name, ros::Time(0));
	geometry_msgs::TransformStamped opensim_default_frame = tfBuffer.lookupTransform(opensim_base_default_frame_name, parent_frame_name, ros::Time(0));
	geometry_msgs::TransformStamped imu_base_measured_frame = tfBuffer.lookupTransform(imu_base_measured_frame_name, parent_frame_name, ros::Time(0));

	ROS_INFO_STREAM(green << "imu_default_frame"<<imu_default_frame.transform.rotation <<reset);
	ROS_INFO_STREAM("imu_base_measured_frame"<< imu_base_measured_frame.transform.rotation);

	geometry_msgs::PoseStamped heading_pose;
	heading_pose.header.stamp = ros::Time::now();
	heading_pose.pose.position = origin;

	//now the important part, calculate this damn angle

	double heading_angle = calculate_damn_angle(imu_default_frame.transform.rotation, imu_base_measured_frame.transform.rotation, opensim_default_frame.transform.rotation);

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
	return heading_pose;

}
