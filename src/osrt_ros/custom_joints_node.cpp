#include "ros/message_traits.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include <sstream>

class qJointPublisher
{
	public:
		ros::NodeHandle n;
		ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

		void run()
		{
			ros::Rate loop_rate(10);

			int count = 0;
			while (ros::ok())
			{
				std_msgs::Header h;
				h.stamp = ros::Time::now();
				sensor_msgs::JointState msg;
				msg.header = h;
				ROS_INFO("DD");
				std::vector<std::string> names = {"hip_r_RX","hip_r_RY","hip_r_RZ","knee_r_RX","knee_r_RZ","knee_r_RY","ankle_r","hip_l_RX","hip_l_RY","hip_l_RZ","knee_l_RX","knee_l_RZ","knee_l_RY","ankle_l","back_RX","back_RY","back_RZ"};
				msg.name = names;
				std::vector<double> values;
				for (auto a:names)
					values.push_back(0.0+count*0.01);
				msg.position = values;
				chatter_pub.publish(msg);

				ros::spinOnce();

				loop_rate.sleep();
				++count;
			}

		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	qJointPublisher qJ;
	qJ.run();
	return 0;
}

