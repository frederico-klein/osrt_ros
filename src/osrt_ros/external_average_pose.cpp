#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include "osrt_ros/UIMU/QuaternionAverage.h"

using namespace std;

class ExternalAveragePosePublisher
{
	public:
		ros::NodeHandle n;
		ros::Subscriber imu_poses;
		ros::Publisher avg_pose;
		ExternalAveragePosePublisher()
		{
			//n = ros::NodeHandle();
			std::string own_name = n.resolveName("imu_cal");
			ROS_INFO_STREAM("started avg_pose publisher reading from node" << own_name);
			imu_poses = n.subscribe("imu_cal", 1, &ExternalAveragePosePublisher::callback, this);
			avg_pose = n.advertise<geometry_msgs::Quaternion>("avg_pose",1,true);
		}
		void callback(const geometry_msgs::PoseArrayPtr& msg)
		{
			ROS_INFO_STREAM("Received pose array msg");
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
			ROS_INFO_STREAM("average:" << quaternionAvg);
			//now publish it as a ros quaternion to keep things simple....
			//
			geometry_msgs::Quaternion q_ ; //TODO: this should be a simtk style quaternion message
			q_.x = quaternionAvg[0];
			q_.y = quaternionAvg[1];
			q_.z = quaternionAvg[2];
			q_.w = quaternionAvg[3];
			
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

