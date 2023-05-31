#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Header.h"
#include <SimTKcommon/Orientation.h>
#include <SimTKcommon/internal/Rotation.h>
#include <sstream>
#include "Ros/include/common_node.h"
#include <map>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std;

tf2::Quaternion convert_zyx_to_quaternion(double z, double y, double x) {
	tf2::Quaternion q;
	double cy = cos(y * 0.5);
	double sy = sin(y * 0.5);
	double cz = cos(z * 0.5);
	double sz = sin(z * 0.5);
	double cx = cos(x * 0.5);
	double sx = sin(x * 0.5);

	q.setW( cy * cz * cx + sy * sz * sx);
	q.setX( cy * sz * cx - sy * cz * sx);
	q.setY( sy * cz * cx + cy * sz * sx);
	q.setZ( sy * sz * cx - cy * cz * sx);

	return q;
}

std::vector<geometry_msgs::TransformStamped> rotate_then_translate(geometry_msgs::Quaternion q, geometry_msgs::Vector3 t)
{
	std::vector<geometry_msgs::TransformStamped>	v;		
	auto time = ros::Time::now();
	geometry_msgs::TransformStamped baseTF_r;
	baseTF_r.header.stamp = time;
	baseTF_r.header.frame_id = "base_t";
	baseTF_r.child_frame_id = "base";
	baseTF_r.transform.rotation = q;
	v.push_back(baseTF_r);
	//baseTF_r.transform.translation = ; default is already zero no need to think about this.
	geometry_msgs::TransformStamped baseTF_t;
	baseTF_t.header.stamp = time;
	baseTF_t.header.frame_id = "subject_opensim";
	baseTF_t.child_frame_id = "base_t";
	geometry_msgs::Quaternion eye_q;
	eye_q.w = 1;
	/*geometry_msgs::Quaternion ros_q;
	  ros_q.x = 0;
	  ros_q.y = 0.7071;
	  ros_q.z = 0.7071;
	  ros_q.w = 0;
	  baseTF_t.transform.rotation = ros_q;*/
	baseTF_t.transform.rotation = eye_q;
	baseTF_t.transform.translation = t;
	v.push_back(baseTF_t);
	return v;
}

class qJointPublisher: public Ros::CommonNode
{
	public:
		ros::NodeHandle n;
		std::string model_base_frame, parent_base_frame;
		int case_translation, case_rotation;
		qJointPublisher() : Ros::CommonNode(false)
	{
		ros::NodeHandle nh("~");
		if (nh.param<int>("case_rotation", case_rotation,2))
			ROS_DEBUG_STREAM("has param!");
		ROS_INFO_STREAM("case_rotation" << case_rotation);
		if (nh.param<int>("case_translation",case_translation, 4))
			ROS_DEBUG_STREAM("has param!");
		ROS_INFO_STREAM("case_translation" << case_translation);
		nh.param<std::string>("model_base_frame", model_base_frame, "model_base");
		nh.param<std::string>("parent_base_frame", parent_base_frame, "map");
		nh.getParam("joint_mapping", RJointToOJoint);
		for (auto el:RJointToOJoint)
			names.push_back(el.first);
		for (auto name:names)
			ROS_DEBUG_STREAM(name);
	}

		ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 2);
		std::map<std::string, std::string> RJointToOJoint;
		std::vector<std::string> names;
		std::map<std::string, int> label_map;
		tf2_ros::StaticTransformBroadcaster static_broadcaster;
		void onInit()
		{
			CommonNode::onInit(1);
			for (int i=0;i<input_labels.size();i++)
			{
				ROS_DEBUG_STREAM(input_labels[i]);
				label_map.insert(std::pair<std::string, int>(input_labels[i],i));
			}
		}
		void pub_pose(std_msgs::Header h, std::vector<double> joint_values, geometry_msgs::Vector3 base_translation, geometry_msgs::Quaternion base_rotation)
		{
			sensor_msgs::JointState msg;
			msg.header = h;
			msg.name = names;
			msg.position = joint_values;
			chatter_pub.publish(msg);
			/// let's publish a tf for the base now!
			geometry_msgs::TransformStamped baseTF;
			baseTF.header.stamp = ros::Time::now();
			baseTF.header.frame_id = parent_base_frame;
			baseTF.child_frame_id = model_base_frame;
			// ATTENTION: x,y,z are different between OSIM and ROS, so this is possibly incorrect. 
			// In any case, the right way should be to set this transformation globally and always use the same instead of defining it everywhere.
			baseTF.transform.translation = base_translation;
			baseTF.transform.rotation = base_rotation;
			/*for (auto tf_:rotate_then_translate(r,t))
			  static_broadcaster.sendTransform(tf_);*/
			ROS_DEBUG_STREAM("initial baseTF" << baseTF);
			static_broadcaster.sendTransform(baseTF);


		}
		void pub_zero()
		{
			std::vector<double> zero_joints(names.size(),0.1);
			//ROS_INFO_STREAM("zero_joints length: " << zero_joints.size() << "vals" );
			//for (auto a:zero_joints)
			//	ROS_INFO_STREAM(a);
			geometry_msgs::Quaternion zero_rot;
			zero_rot.w = 1;
			std_msgs::Header h;
			h.frame_id = "subject";
			ros::Rate poll_rate(100);
			while(chatter_pub.getNumSubscribers() == 0)
				poll_rate.sleep();
			for (int i = 0; i<=10; i++)
			{
				h.stamp = ros::Time::now();
				pub_pose(h, zero_joints, geometry_msgs::Vector3(), zero_rot);
			}

		}
		void callback(const opensimrt_msgs::CommonTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik");
			parse_msg(msg_ik->header, msg_ik->data);
		}
		void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik filtered");
			parse_msg(msg_ik->header, msg_ik->d0_data);
		}
		void parse_msg(std_msgs::Header h, std::vector<double> q)
		{
			std::vector<double> values;
			for (auto a:names)
			{
				double joint_value = 0;
				int index = label_map[RJointToOJoint[a]];
				if (index>=0)
				{
					joint_value = q[index];
					//joint_value = msg_ik->data[index]/180*3.14159265;
				}
				values.push_back(joint_value);
			}
			/// let's publish a tf for the base now!
			// ATTENTION: x,y,z are different between OSIM and ROS, so this is possibly incorrect. 
			// In any case, the right way should be to set this transformation globally and always use the same instead of defining it everywhere.
			geometry_msgs::Quaternion r;// = baseTF.transform.rotation;
			geometry_msgs::Vector3 t;// = baseTF.transform.translation;

			switch (case_translation){
				case 0:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 0");
					t.x = q[3]; t.y = q[4]; t.z = q[5];
					break;
				case 1:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 1");
					t.x = q[3]; t.y = q[5]; t.z = q[4];
					break;
				case 2:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 2");
					t.x = q[4]; t.y = q[3]; t.z = q[5];
					break;
				case 3:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 3");
					t.x = q[4]; t.y = q[5]; t.z = q[3];
					break;
				case 4:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 4");
					t.x = q[5]; t.y = q[3]; t.z = q[4];
					break;
				case 5:
					ROS_INFO_STREAM_ONCE("CASE TRANSLATION 5");
					t.x = q[5]; t.y = q[4]; t.z = q[3];
					break;
				default:
					ROS_ERROR_STREAM("unknown case");
					return;
			}
			auto R_Base = SimTK::Rotation();
			switch (case_rotation){
				case 0:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 0");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[0], SimTK::XAxis, q[1], SimTK::YAxis, q[2], SimTK::ZAxis);
					break;
				case 1:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 1");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[0], SimTK::XAxis, q[2], SimTK::ZAxis, q[1], SimTK::YAxis);
					break;
				case 2:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 2");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[1], SimTK::YAxis, q[0], SimTK::XAxis, q[2], SimTK::ZAxis);
					break;
				case 3:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 3");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[1], SimTK::YAxis, q[2], SimTK::ZAxis, q[0], SimTK::XAxis);
					break;
				case 4:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 4");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[2], SimTK::ZAxis, q[0], SimTK::XAxis, q[1], SimTK::YAxis);
					break;
				case 5:
					ROS_INFO_STREAM_ONCE("CASE ROTATION 5");
					R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[2], SimTK::ZAxis, q[1], SimTK::YAxis, q[0], SimTK::XAxis);
					break;
				default:
					ROS_ERROR_STREAM("unknown case");
					return;
			}
			ROS_INFO_STREAM("radians" << q[0] << " " <<q[1] << " " << q[2]);
			ROS_INFO_STREAM("degrees" << SimTK::convertRadiansToDegrees(q[0]) << " " <<SimTK::convertRadiansToDegrees(q[1]) << " " << SimTK::convertRadiansToDegrees(q[2]));
			ROS_WARN_STREAM("base orientation matrix in OpenSim coordinates :\n" << R_Base);
			//to quaternion
			SimTK::Quaternion qz= R_Base.convertRotationToQuaternion();
			ROS_INFO_STREAM("qz as Quaternion" << qz);
			//assign to r
			//
			r.w = qz[0];
			r.x = qz[1];
			r.y = qz[2];
			r.z = qz[3];
			////t.x = q[5]; t.y = q[3]; t.z = q[4];

			pub_pose(h, values, t, r);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	qJointPublisher qJ;
	qJ.onInit();
	// publish initial zero pose:
	qJ.pub_zero();
	ros::spin();
	return 0;
}

