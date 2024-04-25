#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "osrt_ros/UIMU/ExternalHeading.h"
#include "ros/time.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "external_compute_heading", ros::init_options::AnonymousName);
	ExternalHeading eApp;
	ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
	ros::ServiceServer serv_calib = nh.advertiseService("calibrate_heading",&ExternalHeading::calibrate_srv, &eApp);
	bool run_as_service;
	nh.param<bool>("run_as_service", run_as_service, true); // I was running this in a loop, but I am not sure it makes sense. I just want to calibrate it once and maybe when I have service calls, right?

	//ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pppppppp",10,false);
	ros::Publisher pub2 = nh.advertise<visualization_msgs::Marker>("visualization_marker",10,true);
	ros::Rate r(1);
	while(ros::ok())
	{
		/*
		   geometry_msgs::Quaternion q1,q2;
		   q1.w =1;q1.x =0;q1.y =0;q1.z =0;
		   q2.w =1;q2.x =0;q2.y =0;q2.z =0;
		   geometry_msgs::Vector3 v;
		   v.x = 0;
		   v.y = 0;
		   v.z = -1;
		   double heading_angle = eApp.calculate_angle(q1,q2,v);
		   geometry_msgs::PoseStamped pp;
		   pp.header.stamp = ros::Time::now();
		   pp.header.frame_id = "map";
		   tf::Quaternion g = tf::createQuaternionFromYaw(heading_angle);
		   pp.pose.orientation.w =g.w();
		   pp.pose.orientation.x =g.x();
		   pp.pose.orientation.y =g.y();
		   pp.pose.orientation.z =g.z();
		   pp.pose.position = eApp.origin;
		   pub.publish(pp);
		*/
		if (!run_as_service)
		{

			eApp.calibrate();

		}
		// creates marker arrow. color is axis heading, right
		//auto m = eApp.getArrowForVector("imu_heading_axis",eApp.heading);
		// but we should place the imu heading on the imu
		//m.header.frame_id = eApp.imu_default_frame_name;
		for (auto m:eApp.debug_markers)
			pub2.publish(m);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

