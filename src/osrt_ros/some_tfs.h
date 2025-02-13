/**
 * @author      : $USER (osruser [at]dbefbc0c19ed)
 * @file        : some_tfs.h
 * @date     : Tuesday Nov 14, 2023 16:01:58 UTC
 */
#ifndef SOMETHFS
#define SOMETHFS
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_broadcaster.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon/internal/State.h>
#include <vector>
#include <Actuators/Schutte1993Muscle_Deprecated.h>
//this is a better version of human_state_publisher, aka, forward kinematics! it doesnt work though.

//since i dont know this, lets at least make it a function, so once it is known i can reuse it maybe.
//

class Osim_tf_publisher
{
	public:
		//	we sort of want just common stuff here too, right? this is maybe a common node.
		Osim_tf_publisher(OpenSim::Model& model_, SimTK::State& state_): model(model_), state(state_)
		{
			ros::NodeHandle nh("~");

			OpenSim::Object::RegisterType(OpenSim::Schutte1993Muscle_Deprecated());
			nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
			sync_input_sub = nh.subscribe("joint_states", 10, &Osim_tf_publisher::other_callback, this);
			nh.getParam("bodies_tf", imuBodiesObservationOrder);
			if (imuBodiesObservationOrder.size() == 0)
			{
				ROS_FATAL("IMU observation order not defined!");
				throw(std::invalid_argument("imuObservationOrder not defined."));
			}
			else
			{
				ROS_INFO_STREAM("Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
			}

		}
		geometry_msgs::Transform fromSimTkTransform(SimTK::Transform some_simtk_tf)
		{
			geometry_msgs::Transform some_tf;	
			auto qq = some_simtk_tf.R().convertRotationToQuaternion();
			auto qt = some_simtk_tf.T();
			some_tf.rotation.w = qq[0];
			some_tf.rotation.x = qq[1];
			some_tf.rotation.y = qq[2];
			some_tf.rotation.z = qq[3];
			some_tf.translation.x = qt[0];
			some_tf.translation.y = qt[1];
			some_tf.translation.z = qt[2];
			return some_tf;
		}
		void init()
		{
			ROS_INFO_STREAM("what is q? "<< state.getQ());
			model.realizePosition(state);
			if (model.isValidSystem())
			{
				ROS_INFO_STREAM("Initialized model okay");
			}
			else
			{
				ROS_FATAL_STREAM("model not valid");
				throw(std::runtime_error("cannot read model."));
			}
			for (const auto& label : imuBodiesObservationOrder) {
				const OpenSim::PhysicalFrame* frame = nullptr;
				if ((frame = model.findComponent<OpenSim::PhysicalFrame>(label))) {
					imuBodiesInGround[label] =
						frame->getTransformInGround(state); // R_GB

				}
				else
					ROS_WARN_STREAM("cannot find body:" << label);
			}

		}
		ros::Subscriber sync_input_sub; 
		std::map<std::string, SimTK::Transform> imuBodiesInGround;
		void update(const SimTK::Vector& q)
		{
			std::vector<double> qq{0, 0, 0, 0, 1, 0, 0.785398, 0, 0, -2e-08, -2e-8};
			SimTK::Vector fixed_q(11);
			for (size_t i=0;i<qq.size();i++)
				fixed_q[i] = qq[i];
			auto itself = state.getQ();
			state.setQ(itself);	//this breaks everything

		}
		void other_callback (const sensor_msgs::JointStateConstPtr msg)
		{
			if (model.isValidSystem())
			{
				SimTK::Vector v(msg->position.size());
				for (size_t i=0;i<msg->position.size();i++)
					v[i] =msg->position[i];
				auto numQModel = state.getQ().size();
				if (v.size() == numQModel)
					update(v); 
				else
				{
					ROS_FATAL_STREAM("size of input bodies: "<< v.size() << " is different from model bodies size:" << numQModel);
				}
			}

		}
		void callback(const sensor_msgs::JointStateConstPtr msg)
		{
			// creates some q to send to a model
			if (model.isValidSystem())
			{
				SimTK::Vector v(msg->position.size());
				for (int i=0;i<msg->position.size();i++)
					v[i] =msg->position[i];
				state.updQ() = v;	
				model.realizePosition(state);
				//now publish the tfs, but they will be in opensim frame of reference
				for(const auto& label : imuBodiesObservationOrder) {
					auto some_simtk_tf = imuBodiesInGround[label];
					geometry_msgs::TransformStamped some_tf;	
					some_tf.header = msg->header;
					some_tf.child_frame_id = label;
					some_tf.transform = fromSimTkTransform(some_simtk_tf);
					tf_broadcaster.sendTransform(some_tf);
				}

			}
		}
	private:
		tf2_ros::TransformBroadcaster tf_broadcaster;
		//lets load a model here

		OpenSim::Model& model;
		SimTK::State& state;
		std::vector<std::string> imuBodiesObservationOrder;
		std::string tf_frame_prefix;
};
#endif
