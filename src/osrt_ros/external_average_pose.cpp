#include "geometry_msgs/Point.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include "osrt_ros/UIMU/QuaternionAverage.h"
#include "ros/service_client.h"
#include "ros/spinner.h"
#include "std_msgs/Float64.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include "tf2_ros/transform_listener.h"
#include <ostream>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>
#include <sstream>
#include "tf/transform_listener.h"

using namespace std;

double avg (std::vector<double> v)
{
	double sum = 0;
	for (auto vi:v)
		sum+=vi;
	return sum/v.size();

}


class ExternalAveragePosePublisher
{
	public:
		//there is maybe like a service that is called calibrate and that service gets 10 tf frames, okay


		bool is_calibrated = false;
		bool is_heading_corrected = false;
		bool use_marker_origin = false;
		ros::NodeHandle n;
		ros::Subscriber heading_subscriber;
		//ros::Subscriber imu_poses;
		std::string body_frame, imu_cal_frame, imu_raw,  own_name, heading_reference_frame;
		size_t num_samples = 10;
		ros::Rate r;
		tf2_ros::StaticTransformBroadcaster br;
		tf2_ros::Buffer tfBuffer;
		tf::TransformListener tf1Listener;
		tf2_ros::TransformListener tfListener;
		geometry_msgs::Quaternion q_ ; //TODO: this should be a simtk style quaternion message
		geometry_msgs::Point avg_point;
		std::vector<double> origin{0,0,0};
		ExternalAveragePosePublisher(): tfListener(tfBuffer), r(100)
	{
		//n = ros::NodeHandle();
		own_name = n.resolveName("imu_cal");
		ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
		nh.param<std::string>("imu_raw",imu_raw, "imu/pelvis");
		nh.param<std::string>("imu_cal_frame",imu_cal_frame, "_imu");
		nh.param<std::string>("heading_ref_frame",heading_reference_frame, "subject_adds_heading"); //TODO: THIS IS THE DEFAULT FOR TESTING. THE DEFAULT FOR MOST CASES SHOULD BE MAP, I THINK.
		nh.param<std::string>("imu_ref_frame",body_frame, "map");
		nh.param<bool>("use_marker_origin",use_marker_origin, false);
		nh.param("origin", origin, {0,0,0});
		std::stringstream origin_str;
		for (auto v:origin)
			origin_str << v << ",";
		ROS_WARN_STREAM("Using origin" << origin_str.str() <<" for ExternalAveragePosePublisher" << own_name);
		ROS_INFO_STREAM("started avg_pose publisher reading from node: " << own_name);
		
	}
		//TODO:there should be a service here, which gets the latest tf from body_imu, then I can change those things around with a dynamic reconfigure and get the latest tf when I start the measurement
		// i.e. the setting and the publishing need to be decoupled. 
		geometry_msgs::PoseArray marker_poses;
		void publish_tf_body()
		{
			geometry_msgs::TransformStamped transformStamped;

			transformStamped.header.stamp = ros::Time::now();
			transformStamped.header.frame_id = heading_reference_frame;  
			transformStamped.child_frame_id = imu_raw+"_0"; //this is going to the be the one with added heading angle
		if (false)
		{
			transformStamped.transform.translation.x = origin[0] + avg_point.x;
			transformStamped.transform.translation.y = origin[1] + avg_point.y;
			transformStamped.transform.translation.z = origin[2] + avg_point.z;
		}
		else if(false)
		{
			transformStamped.transform.translation.x = origin[0] ;
			transformStamped.transform.translation.y = origin[1] ;
			transformStamped.transform.translation.z = origin[2] ;


		}
		else
		{
			transformStamped.transform.translation.x = avg_point.x;
			transformStamped.transform.translation.y = avg_point.y;
			transformStamped.transform.translation.z = avg_point.z;

		}
			ROS_ERROR_STREAM("q_" << q_); // ???????
			transformStamped.transform.rotation.x = q_.x;
			transformStamped.transform.rotation.y = q_.y;
			transformStamped.transform.rotation.z = q_.z;
			transformStamped.transform.rotation.w = q_.w;

			br.sendTransform(transformStamped);

		}

		void add_pose_to_stack()
		{
		try
		{
			// I wait for me to have like, the raw imu input
			tf1Listener.waitForTransform(heading_reference_frame,imu_raw,ros::Time(0), ros::Duration(10)); //it's either like this or the other way around i guess.
			auto Tt =tfBuffer.lookupTransform(heading_reference_frame,imu_raw,ros::Time(0));

			geometry_msgs::Pose p;
			p.orientation = Tt.transform.rotation;
			p.position.x = Tt.transform.translation.x;
			p.position.y = Tt.transform.translation.y;
			p.position.z = Tt.transform.translation.z;
			
			marker_poses.poses.push_back(p);
			size_t num_poses_stored = marker_poses.poses.size(); 
			ROS_INFO_STREAM("num_poses_stored"<< num_poses_stored);
			/*
			eApp.q_.x = Tt.transform.rotation.x;
			eApp.q_.y = Tt.transform.rotation.y;
			eApp.q_.z = Tt.transform.rotation.z;
			eApp.q_.w = Tt.transform.rotation.w;
			eApp.publish_tf_body();
			*/
			if (num_poses_stored > num_samples)
				calibrate();
		}
		catch (tf2::TransformException &ex)

		{
			ROS_ERROR_STREAM("Could not get imu_raw to subject heading.\nAre you publishing IMU/AR values?\n\t" << ex.what());
		}
		ros::spinOnce();
		r.sleep();


		}

		//service for calibration
		void calibration_service()
		{
			//clears poses
			//
			is_calibrated = false;

		}



		void resolve_heading()
		{
				try
				{
				
				tf1Listener.waitForTransform("map",imu_raw+"_0",ros::Time(0), ros::Duration(10)); //it's either like this or the other way around i guess.
				auto heading_transform = tfBuffer.lookupTransform("map",imu_raw+"_0",ros::Time(0)); //it's either like this or the other way around i guess.
				

				//this is for showing the urdf
				heading_transform.child_frame_id = imu_cal_frame;
				heading_transform.header.frame_id = body_frame;
				if (use_marker_origin)
				{

				heading_transform.transform.translation.x = origin[0]+avg_point.x;
				heading_transform.transform.translation.y = origin[1]+avg_point.y;
				heading_transform.transform.translation.z = origin[2] + avg_point.z;
				}
				else
				{
				heading_transform.transform.translation.x = origin[0];
				heading_transform.transform.translation.y = origin[1];
				heading_transform.transform.translation.z = origin[2];


				}
				br.sendTransform(heading_transform);
				is_heading_corrected = true;
				} catch (tf::TransformException &ex)
				{
				}


		}

		void calibrate()
		{
			ROS_DEBUG_STREAM("called calibrate");
			is_calibrated =true;
			//std_msgs::Header h;
			//h.stamp = ros::Time::now();
			geometry_msgs::Quaternion msg_q;
			//msg.header = h;

			//create from msg something that quaternionAverage can use and get results
			
			std::vector<double> X,Y,Z;
			std::vector<Eigen::Vector4f> quaternions;	
			for (auto pose:marker_poses.poses)
			{
				Eigen::Vector4f q;
				q[0] = pose.orientation.x;
				q[1] = pose.orientation.y;
				q[2] = pose.orientation.z;
				q[3] = pose.orientation.w;
				quaternions.push_back(q);
				X.push_back(pose.position.x);
				Y.push_back(pose.position.y);
				Z.push_back(pose.position.z);
			}
			Eigen::Vector4f quaternionAvg = quaternionAverage(quaternions);
			ROS_DEBUG_STREAM("average:\n " << quaternionAvg);
			//now publish it as a ros quaternion to keep things simple....
			//
			q_.x = quaternionAvg[0];
			q_.y = quaternionAvg[1];
			q_.z = quaternionAvg[2];
			q_.w = quaternionAvg[3];
			
			avg_point.x = avg(X);
			avg_point.y = avg(Y);
			avg_point.z = avg(Z);

			publish_tf_body();
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "external_average_pose_publisher", ros::init_options::AnonymousName);
	ExternalAveragePosePublisher eApp;
	ros::Rate r(100);
	while(ros::ok())
	{
		if(!eApp.is_calibrated )
		{
			eApp.add_pose_to_stack();
		}
		else
		{
			eApp.resolve_heading();
		}
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}

