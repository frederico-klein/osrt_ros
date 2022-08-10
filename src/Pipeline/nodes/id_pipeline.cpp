#include "grf_pipe.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "signal.h"
#include "Pipeline/include/id.h"

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    //perenial.write();
    ros::shutdown();
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "id_show");
	ROS_INFO_STREAM("called node InverseDynamics.");
    try {
	Pipeline::Id perenial;

	//	signal(SIGINT, mySigintHandler);
	perenial.onInit();		
			//ros::Subscriber sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1, perenial);	
	ROS_WARN_STREAM("entering spin");
	ros::spin();
    	ROS_INFO_STREAM("Goodbye!");
    	return 0;
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}

