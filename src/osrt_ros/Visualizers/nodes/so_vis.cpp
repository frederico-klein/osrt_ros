#include "osrt_ros/Visualizers/so_vis.h"

#include "ros/ros.h"
#include "signal.h"

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    ros::shutdown();
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "so_vis");
	ROS_INFO_STREAM("called node so_vis");
	//		ros::NodeHandle n;
    try {
	Visualizers::SoVis vis;	
	//	signal(SIGINT, mySigintHandler);
	ROS_INFO_STREAM("calling onInit");
	vis.onInit();	
	ROS_INFO_STREAM("onInit finished ok.");
	ros::spin();
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    ROS_INFO_STREAM("Goodbye!");
    return 0;
}


