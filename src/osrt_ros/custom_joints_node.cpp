#include "opensimrt_msgs/CommonTimed.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include <sstream>
#include "osrt_ros/Pipeline/common_node.h"
#include <map>
using namespace std;

//this is a hack. I would ideally get this from opensim somehow, since it must be calculated before it can be printed on the screen

map<string, int> rjoint_to_ojoint 
{
		{"hip_r_RX", 6},
		{"hip_r_RY", 7},
		{"hip_r_RZ", 8},
		{"knee_r_RX",15},
		{"knee_r_RZ",-1},
		{"knee_r_RY",-1},
		{"ankle_r",17},
		{"hip_l_RX",9},
		{"hip_l_RY",10},
		{"hip_l_RZ",11},
		{"knee_l_RX",16},
		{"knee_l_RZ",-1},
		{"knee_l_RY",-1},
		{"ankle_l",18},
		{"back_RX",12},
		{"back_RY",13},
		{"back_RZ",14},
};

std::vector<std::string> labels = {
	"pelvis_tilt", 		//0
	"pelvis_list", 		//1
	"pelvis_rotation", 	//2
	"pelvis_tx", 		//3
	"pelvis_ty", 		//4
	"pelvis_tz", 		//5
	"hip_flexion_r", 	//6
	"hip_adduction_r",	//7
	"hip_rotation_r",	//8
	"hip_flexion_l",	//9
	"hip_adduction_l",	//10
	"hip_rotation_l",	//11
	"lumbar_extension",	//12
	"lumbar_bending",	//13
	"lumbar_rotation",	//14
	"knee_angle_r",		//15
	"knee_angle_l",		//16
	"ankle_angle_r",	//17
	"ankle_angle_l"};	//18

class qJointPublisher: public Pipeline::CommonNode
{
	public:
		ros::NodeHandle n;
		ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
		int count = 0;
		std::vector<std::string> names = {"hip_r_RX","hip_r_RY","hip_r_RZ","knee_r_RX","knee_r_RZ","knee_r_RY","ankle_r","hip_l_RX","hip_l_RY","hip_l_RZ","knee_l_RX","knee_l_RZ","knee_l_RY","ankle_l","back_RX","back_RY","back_RZ"};

		void callback(const opensimrt_msgs::CommonTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik");
			std_msgs::Header h;
			h.stamp = ros::Time::now();
			sensor_msgs::JointState msg;
			msg.header = h;
			msg.name = names;
			std::vector<double> values;
			for (auto a:names)
			{
				double joint_value = 0;
				int index = rjoint_to_ojoint[a];
				if (index>=0)
					{
						joint_value = msg_ik->data[index];
						//joint_value = msg_ik->data[index]/180*3.14159265;
					}
				values.push_back(joint_value);
			}
			msg.position = values;
			chatter_pub.publish(msg);
			++count;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	qJointPublisher qJ;
	qJ.onInit();
	ros::spin();
	return 0;
}

