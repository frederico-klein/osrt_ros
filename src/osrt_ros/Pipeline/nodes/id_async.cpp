
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/service_server.h"
#include "signal.h"
#include <memory>
#include "osrt_ros/Pipeline/id_async.h"
#include <dynamic_reconfigure/server.h>

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    //perenial.write();
    ros::shutdown();
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "id_show");
	ROS_INFO_STREAM("called node IdAsync: InverseDynamics.");
    try {
	ros::NodeHandle nh("~");
	double delay;
	nh.param("ik_delay", delay, 0.5);
	auto Delay = std::make_shared<ros::Duration>(delay);
	

	Pipeline::IdAsync perenial(Delay);

		ros::NodeHandle nh1(nh, "delay");
		dynamic_reconfigure::Server<osrt_ros::delayConfig> delay_server_(nh1);
		dynamic_reconfigure::Server<osrt_ros::delayConfig>::CallbackType f;
		f = boost::bind(&Pipeline::IdAsync::reconfigure_delay_callback, &perenial, _1, _2);
	delay_server_.setCallback(f);

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

