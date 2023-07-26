#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/MultiMessage.h"
#include "opensimrt_msgs/MultiMessagePosVelAcc.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/Pipeline/dualsink_pipe.h"
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
#include "tf2_ros/transform_broadcaster.h"
#include <map>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

using namespace std;

class OpenSimBaseTfPublisher
{
	public:
		OpenSimBaseTfPublisher()
		{
			ros::NodeHandle nh("~");
			nh.param<std::string>("parent_base_frame", parent_base_frame, "map");
			nh.param<std::string>("opensim_frame", opensim_frame, "subject_opensim");
			opensim_rotation.w = 0.5;
			opensim_rotation.x = 0.5;
			opensim_rotation.y = 0.5;
			opensim_rotation.z = 0.5;
			t.header.frame_id = parent_base_frame;
			t.child_frame_id = opensim_frame;
			t.transform.rotation = opensim_rotation;
			t.transform.translation = geometry_msgs::Vector3();

		}
		std::string parent_base_frame, opensim_frame;
		tf2_ros::StaticTransformBroadcaster tb;
		geometry_msgs::Quaternion opensim_rotation;
		geometry_msgs::TransformStamped t;
		void publish_opensim_base_tf()
		{
			t.header.stamp = ros::Time::now();
			tb.sendTransform(t);
			ros::spinOnce();
		}
};

class qJointPublisher: public Ros::CommonNode
{
	public:
		ros::NodeHandle n;
		std::string model_base_frame, parent_base_frame;
		int case_translation, case_rotation;
		bool re_stamp_base = false;
		ros::Publisher chatter_pub;
		qJointPublisher() : Ros::CommonNode(false)
	{
		ros::NodeHandle nh("~");
		chatter_pub = n.advertise<sensor_msgs::JointState>("joint_states", 2);
		nh.param<std::string>("model_base_frame", model_base_frame, "model_base");
		nh.param<std::string>("parent_base_frame", parent_base_frame, "map");
		nh.getParam("joint_mapping", RJointToOJoint);
		for (auto el:RJointToOJoint)
			names.push_back(el.first);
		for (auto name:names)
			ROS_DEBUG_STREAM(name);
	}

		std::map<std::string, std::string> RJointToOJoint;
		std::vector<std::string> names;
		std::map<std::string, int> label_map;
		tf2_ros::TransformBroadcaster tf_broadcaster;
		void onInit()
		{
			CommonNode::onInit(1);
			for (int i=0;i<input_labels.size();i++)
			{
				ROS_DEBUG_STREAM(input_labels[i]);
				label_map.insert(std::pair<std::string, int>(input_labels[i],i));
			}
		}
		void pub_pose(std_msgs::Header h, std::vector<double> joint_values, std::vector<double> joint_efforts, geometry_msgs::Vector3 base_translation, geometry_msgs::Quaternion base_rotation)
		{
			sensor_msgs::JointState msg;
			msg.header = h;
			msg.name = names;
			msg.position = joint_values;
			msg.effort = joint_efforts;
			/// let's publish a tf for the base now!
			geometry_msgs::TransformStamped baseTF;
			baseTF.header = h;
			if (re_stamp_base)
				baseTF.header.stamp = ros::Time::now();
			baseTF.header.frame_id = parent_base_frame;
			baseTF.child_frame_id = model_base_frame;
			// ATTENTION: x,y,z are different between OSIM and ROS, so this is possibly incorrect. 
			// In any case, the right way should be to set this transformation globally and always use the same instead of defining it everywhere.
			baseTF.transform.translation = base_translation;
			baseTF.transform.rotation = base_rotation;
			/*for (auto tf_:rotate_then_translate(r,t))
			  tf_broadcaster.sendTransform(tf_);*/
			ROS_DEBUG_STREAM("initial baseTF" << baseTF);
			tf_broadcaster.sendTransform(baseTF);
			chatter_pub.publish(msg);


		}
		void pub_zero()
		{
			std::vector<double> zero_joints(names.size(),0.0);
			//ROS_INFO_STREAM("zero_joints length: " << zero_joints.size() << "vals" );
			//for (auto a:zero_joints)
			//	ROS_INFO_STREAM(a);
			geometry_msgs::Quaternion zero_rot;
			zero_rot.w = 1;
			std_msgs::Header h;
			h.frame_id = "map";
			ros::Rate poll_rate(100);
			//while(chatter_pub.getNumSubscribers() == 0)
			//	poll_rate.sleep();
			for (int i = 0; i<=10; i++)
			{
				h.stamp = ros::Time::now();
				pub_pose(h, zero_joints, zero_joints, geometry_msgs::Vector3(), zero_rot);
				poll_rate.sleep();
				ros::spinOnce();
			}

		}
		void sync_callback(const opensimrt_msgs::MultiMessageConstPtr &message)
		{
			ROS_DEBUG_STREAM("Received msg_multi");
			parse_msg(message->header,message->ik.data, message->other[0].data);	
		}
		void sync_callback_filtered(const opensimrt_msgs::MultiMessagePosVelAccConstPtr &message)
		{
			ROS_DEBUG_STREAM("Received msg_multifiltered");
			parse_msg(message->header,message->d0_data.data, message->other[0].data);	
		}
		void callback(const opensimrt_msgs::CommonTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik");
			//auto a = ros::Time::now();
			parse_msg(msg_ik->header, msg_ik->data, std::vector<double>(msg_ik->data.size() ,0.0));
			//auto b = ros::Time::now();
			//ROS_INFO_STREAM("loop time" <<a-b);
		}
		void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& msg_ik)
		{
			ROS_DEBUG_STREAM("Received msg_ik filtered");
			//auto a = ros::Time::now();
			parse_msg(msg_ik->header, msg_ik->d0_data, std::vector<double>(msg_ik->d0_data.size() ,0.0));
			//auto b = ros::Time::now();
			//ROS_INFO_STREAM("loop time filtered" <<a-b);
		}
		void parse_msg(std_msgs::Header h, std::vector<double> q, std::vector<double> id)
		{
			std::vector<double> values, id_values_reshuffled;
			for (auto a:names)
			{
				ROS_DEBUG_STREAM("setting joint " << a);
				double joint_value = 0;
				double joint_effort = 0;
				std::string some_joint = RJointToOJoint[a];
				if (some_joint == "")
				{
					joint_value = 0;
				}
				else
				{
					int index = label_map[RJointToOJoint[a]];
					ROS_DEBUG_STREAM("index: "<< index);
					joint_value = q[index];
					joint_effort = id[index] + 100; 
					//joint_value = msg_ik->data[index]/180*3.14159265;
				}
				values.push_back(joint_value);
				id_values_reshuffled.push_back(joint_effort);
			}
			/// let's publish a tf for the base now!
			// ATTENTION: x,y,z are different between OSIM and ROS, so this is possibly incorrect. 
			// In any case, the right way should be to set this transformation globally and always use the same instead of defining it everywhere.
			geometry_msgs::Quaternion r;// = baseTF.transform.rotation;
			geometry_msgs::Vector3 t;// = baseTF.transform.translation;

			t.x = q[5]; t.y = q[3]; t.z = q[4];
			auto R_Base = SimTK::Rotation();
			R_Base = SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, q[2], SimTK::ZAxis, q[1], SimTK::YAxis, q[0], SimTK::XAxis);
			ROS_DEBUG_STREAM("radians" << q[0] << " " <<q[1] << " " << q[2]);
			ROS_DEBUG_STREAM("degrees" << SimTK::convertRadiansToDegrees(q[0]) << " " <<SimTK::convertRadiansToDegrees(q[1]) << " " << SimTK::convertRadiansToDegrees(q[2]));
			ROS_DEBUG_STREAM("base orientation matrix in OpenSim coordinates :\n" << R_Base);
			//to quaternion
			SimTK::Quaternion qz= R_Base.convertRotationToQuaternion();
			ROS_DEBUG_STREAM("qz as Quaternion" << qz);
			//assign to r
			r.w = qz[0];
			r.x = qz[1];
			r.y = qz[2];
			r.z = qz[3];
			////t.x = q[5]; t.y = q[3]; t.z = q[4];

			pub_pose(h, values, id_values_reshuffled, t, r);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "human_joint_state_publisher");
	// publishes the subject_opensim frame of reference
	OpenSimBaseTfPublisher otf;
	// starts custom_joints publisher
	qJointPublisher qJ;
	qJ.onInit();
	//everything is custom and different from each other. 
	ros::NodeHandle nh("~");
	ros::Subscriber sync_input_sub = nh.subscribe("sync_input", 10, &qJointPublisher::sync_callback, &qJ);
	ros::Subscriber sync_input_filtered_sub = nh.subscribe("sync_filtered_input", 10, &qJointPublisher::sync_callback_filtered, &qJ);

	// publish initial zero pose:
	//qJ.pub_zero();
	otf.publish_opensim_base_tf();	
	ros::spin();
	return 0;
}

