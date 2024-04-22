#include "ros/node_handle.h"
#include <Actuators/Thelen2003Muscle.h>

#include <ros/ros.h>

#include <exception>
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>
#include "osrt_ros/UIMU/UIMUnode.h"
#include <osrt_ros/UIMUConfig.h>
#include <osrt_ros/headingConfig.h>

int main(int argc, char** argv) {
	try {
		ros::init(argc, argv, "online_lower_limb_uimu_ik");
		ros::NodeHandle n;
		UIMUnode o;
		dynamic_reconfigure::Server<osrt_ros::UIMUConfig> server;
		dynamic_reconfigure::Server<osrt_ros::UIMUConfig>::CallbackType f;
		f = boost::bind(&UIMUnode::reconfigure_callback, &o, _1, _2);
		server.setCallback(f);

		ros::NodeHandle nh("~");
		ros::NodeHandle nh1(nh, "heading");
		dynamic_reconfigure::Server<osrt_ros::headingConfig> heading_server_(nh1);
		dynamic_reconfigure::Server<osrt_ros::headingConfig>::CallbackType f2;
		f2 = boost::bind(&UIMUnode::reconfigure_heading_callback, &o, _1, _2);
		heading_server_.setCallback(f2);
		// either like this:
		OpenSim::Object* muscleModel = new OpenSim::Thelen2003Muscle();
		o.registerType(muscleModel);
		// or alternatively, more simply:
		// Object::registerType(Thelen2003Muscle());
		bool wait_to_start = false;
		nh.getParam("wait_to_start", wait_to_start);


		if (wait_to_start)
		{
			ros::Rate wait_rate(0.1);
			bool start_now = false;
			boost::function<bool(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)> wait_callback = [&start_now](auto req, auto res) -> bool
			{
				start_now = true;
				return true;
			};
			//create a service, wait for that service call to change state so we can start
			//
			ros::ServiceServer ss = nh.advertiseService("start_now", wait_callback);

			ROS_WARN("not_tested");
			while (!start_now)
			{
				ROS_INFO_STREAM_THROTTLE(1,"Waiting to start");
				wait_rate.sleep();
				ros::spinOnce();
			}
		}

		o.onInit();
		o.run();
	} catch (std::exception& e) {
		cout << "Program crashed while running. Reason: " << e.what() << endl;
		return -1;
	}
	return 0;
}



