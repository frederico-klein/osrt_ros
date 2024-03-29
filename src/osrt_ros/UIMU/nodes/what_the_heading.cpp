#include "ros/ros.h"
#include "osrt_ros/UIMU/ExternalHeading.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "external_compute_heading", ros::init_options::AnonymousName);
	ExternalHeading eApp;
	ros::NodeHandle nh = ros::NodeHandle("~"); //local nodehandle for params, I dont want to ruin the rest of the remaps.
	ros::ServiceServer serv_calib = nh.advertiseService("calibrate_heading",&ExternalHeading::calibrate_srv, &eApp);
	ros::spin();
	return 0;
}

