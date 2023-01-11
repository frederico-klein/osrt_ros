#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include <sstream>
#include "Ros/include/common_node.h"
#include <map>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

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

std::vector<geometry_msgs::TransformStamped> rotate_then_translate(geometry_msgs::Quaternion q, geometry_msgs::Vector3 t)
{
	std::vector<geometry_msgs::TransformStamped>	v;		
	auto time = ros::Time::now();
	geometry_msgs::TransformStamped pelvisTF_r;
	pelvisTF_r.header.stamp = time;
	pelvisTF_r.header.frame_id = "pelvis_t";
	pelvisTF_r.child_frame_id = "pelvis";
	pelvisTF_r.transform.rotation = q;
	v.push_back(pelvisTF_r);
	//pelvisTF_r.transform.translation = ; default is already zero no need to think about this.
	geometry_msgs::TransformStamped pelvisTF_t;
	pelvisTF_t.header.stamp = time;
	pelvisTF_t.header.frame_id = "subject_opensim";
	pelvisTF_t.child_frame_id = "pelvis_t";
	geometry_msgs::Quaternion eye_q;
	eye_q.w = 1;
	/*geometry_msgs::Quaternion ros_q;
			ros_q.x = 0;
			ros_q.y = 0.7071;
			ros_q.z = 0.7071;
			ros_q.w = 0;
	pelvisTF_t.transform.rotation = ros_q;*/
	pelvisTF_t.transform.rotation = eye_q;
	pelvisTF_t.transform.translation = t;
	v.push_back(pelvisTF_t);
	return v;
}

class qJointPublisher: public Ros::CommonNode
{
	public:
		ros::NodeHandle n;
		qJointPublisher() : Ros::CommonNode(false)
	{}
		ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 2);
		int count = 0;
		std::vector<std::string> names = {"hip_r_RX","hip_r_RY","hip_r_RZ","knee_r_RX","knee_r_RZ","knee_r_RY","ankle_r","hip_l_RX","hip_l_RY","hip_l_RZ","knee_l_RX","knee_l_RZ","knee_l_RY","ankle_l","back_RX","back_RY","back_RZ"};
		tf2_ros::StaticTransformBroadcaster static_broadcaster;
		void callback(const opensimrt_msgs::CommonTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik");
			//std_msgs::Header h;
			//h.stamp = ros::Time::now();
			sensor_msgs::JointState msg;
			//msg.header = h;
			msg.header = msg_ik->header;
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
			/// let's publish a tf for the pelvis now!
			geometry_msgs::TransformStamped pelvisTF;
			pelvisTF.header.stamp = ros::Time::now();
			pelvisTF.header.frame_id = "ground";
			pelvisTF.child_frame_id = "pelvis";
			// ATTENTION: x,y,z are different between OSIM and ROS, so this is likely incorrect. Needs visual inspection
			geometry_msgs::Quaternion r;// = pelvisTF.transform.rotation;
			geometry_msgs::Vector3 t;// = pelvisTF.transform.translation;
			t.x = msg_ik->data[5];
			t.y = msg_ik->data[3];
			t.z = msg_ik->data[4];
			tf2::Quaternion quat;
			//quat.setEuler(msg_ik->data[2], msg_ik->data[1], msg_ik->data[0]); //models is wobbly, needs manual checking!
			//quat.setEuler(0, 0, msg_ik->data[2]); //models is wobbly, needs manual checking!
			quat.setRPY(msg_ik->data[0], msg_ik->data[1], msg_ik->data[2]); //models is wobbly, needs manual checking!
			r.x = quat.z();
			r.y = quat.x();
			r.z = quat.y();
			r.w = quat.w();
			pelvisTF.transform.translation = t;
			pelvisTF.transform.rotation = r;
			/*for (auto tf_:rotate_then_translate(r,t))
				static_broadcaster.sendTransform(tf_);*/
			static_broadcaster.sendTransform(pelvisTF);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	qJointPublisher qJ;
	qJ.onInit();
	ros::spin();
	return 0;
}

