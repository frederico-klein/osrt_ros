#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include "osrt_ros/UIMU/QuaternionAverage.h"
#include "tf/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <ostream>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <sstream>

using namespace std;

class ExternalAveragePosePublisher
{
	public:
		ros::NodeHandle n;
		ros::Subscriber imu_poses;
		ros::Publisher avg_pose;
		std::string body_frame, imu_cal_frame, own_name, heading_reference_frame;
		tf2_ros::StaticTransformBroadcaster br;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;
		geometry_msgs::Quaternion q_ ; //TODO: this should be a simtk style quaternion message
		std::vector<double> origin{0,0,0};
		ExternalAveragePosePublisher(): tfListener(tfBuffer)
		{
			//n = ros::NodeHandle();
			own_name = n.resolveName("imu_cal");
			ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
			nh.param<std::string>("imu_cal_frame",imu_cal_frame, "xxx_imu");
			nh.param<std::string>("heading_ref_frame",heading_reference_frame, "subject_adds_heading"); //TODO: THIS IS THE DEFAULT FOR TESTING. THE DEFAULT FOR MOST CASES SHOULD BE MAP, I THINK.
			nh.param<std::string>("imu_ref_frame",body_frame, "map");
			nh.param("origin", origin, {0,0,0});
			std::stringstream origin_str;
			for (auto v:origin)
				origin_str << v << ",";
			ROS_WARN_STREAM("Using origin" << origin_str.str() <<" for ExternalAveragePosePublisher" << own_name);
			ROS_INFO_STREAM("started avg_pose publisher reading from node: " << own_name);
			imu_poses = n.subscribe("imu_cal", 1, &ExternalAveragePosePublisher::callback, this);
			avg_pose = n.advertise<geometry_msgs::Quaternion>("avg_pose",1,true);
		}
		//TODO:there should be a service here, which gets the latest tf from body_imu, then I can change those things around with a dynamic reconfigure and get the latest tf when I start the measurement
		// i.e. the setting and the publishing need to be decoupled. 
		void publish_tf_body()
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = ros::Time::now();
			transformStamped.header.frame_id = "map"; //otherwise it will break because it is not yet a part of the same tree. 
			transformStamped.child_frame_id = imu_cal_frame+"_raw"; //this is going to the be the one with added heading angle
			transformStamped.transform.translation.x = origin[0];
			transformStamped.transform.translation.y = origin[1];
			transformStamped.transform.translation.z = origin[2];
			ROS_ERROR_STREAM("q_" << q_); // ???????
			transformStamped.transform.rotation.x = q_.x;
			transformStamped.transform.rotation.y = q_.y;
			transformStamped.transform.rotation.z = q_.z;
			transformStamped.transform.rotation.w = -q_.w; //TODO:this is from the inverse quaternions thing from opensimrt. 
			//we still need to fix the heading though.
/*			tf2::Quaternion q_fixed_opensim_transform{0.0,0.707,0.707,0.0}, q_input{q_.x,q_.y,q_.z,q_.w}, q_result;
			q_result = q_fixed_opensim_transform*q_input;
			transformStamped.transform.rotation.x = q_result.x();
			transformStamped.transform.rotation.y = q_result.y();
			transformStamped.transform.rotation.z = q_result.z();
			transformStamped.transform.rotation.w = q_result.w();
 */
			br.sendTransform(transformStamped);
			// we write it also to the buffer
			tfBuffer.setTransform(transformStamped, "ExternalAveragePosePublisher",true); // it is static
			auto heading_transform = tfBuffer.lookupTransform(heading_reference_frame,imu_cal_frame+"_raw",ros::Time(0)); //it's either like this or the other way around i guess.
			heading_transform.child_frame_id = imu_cal_frame;
			heading_transform.header.frame_id = body_frame;
			heading_transform.transform.translation = transformStamped.transform.translation;
			br.sendTransform(heading_transform);
			


		}

		void callback(const geometry_msgs::PoseArrayPtr& msg)
		{
			ROS_DEBUG_STREAM("Received pose array msg");
			//std_msgs::Header h;
			//h.stamp = ros::Time::now();
			geometry_msgs::Quaternion msg_q;
			//msg.header = h;

			//create from msg something that quaternionAverage can use and get results
			std::vector<Eigen::Vector4f> quaternions;	
			for (auto pose:msg->poses)
			{
				Eigen::Vector4f q;
				q[0] = pose.orientation.x;
				q[1] = pose.orientation.y;
				q[2] = pose.orientation.z;
				q[3] = pose.orientation.w;
				quaternions.push_back(q);
			}
			Eigen::Vector4f quaternionAvg = quaternionAverage(quaternions);
			ROS_DEBUG_STREAM("average:\n " << quaternionAvg);
			//now publish it as a ros quaternion to keep things simple....
			//
			q_.x = quaternionAvg[0];
			q_.y = quaternionAvg[1];
			q_.z = quaternionAvg[2];
			q_.w = quaternionAvg[3];
			publish_tf_body();
			avg_pose.publish(q_);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "external_average_pose_publisher", ros::init_options::AnonymousName);
	ExternalAveragePosePublisher eApp;
	ros::spin();
	return 0;
}

