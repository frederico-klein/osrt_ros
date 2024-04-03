#ifndef VISUALIZER_COMMON_HEADER_FBK_27052023
#define VISUALIZER_COMMON_HEADER_FBK_27052023

#include "Ros/include/common_node.h"
#include "Ros/include/saver_node.h"
#include "Settings.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include "OpenSimUtils.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/Object.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <exception>

//TODO:this is rather bad and I should be able to load models using some string input
//TODO: remove this switch statement, find something better.
//
namespace Visualizers
{
	class VisualizerCommon:public Ros::SaverNode
	{
		public:
			VisualizerCommon()
			{
			}
			ros::NodeHandle nh{"~"};
			std::string modelFile, geometryPath;
			OpenSimRT::BasicModelVisualizer *visualizer;
			OpenSim::Model model;	
			int m; // 1 is upper 2 is under...
			ros::Subscriber sub, sub_filtered;
			Ros::Reshuffler input;
			void get_params()
			{
				// subject data
				nh.param<std::string>("model_file", modelFile, "");
				ROS_INFO_STREAM("Using modelFile:" << modelFile);
				nh.param<int>("which_model_1_2", m, 2);
				ROS_DEBUG_STREAM("Finished getting params.");	

			}
			void registerType(OpenSim::Object* muscleModel) //do I even need this?
			{
				OpenSim::Object::registerType(*muscleModel);
			}
			void onInit() 
			{
				get_params();
				
				Ros::SaverNode::onInit();
				// setup model
				input.get_labels(nh);
				ROS_DEBUG_STREAM("Setting up model.");

				OpenSim::Object* muscleModel;

				switch(m)
				{
					case 1:
						muscleModel = new OpenSim::Schutte1993Muscle_Deprecated();
						registerType(muscleModel);
						break;
					case 2:
						muscleModel = new OpenSim::Thelen2003Muscle();
						registerType(muscleModel);
						break;
					default:
						throw std::invalid_argument( "I can use 1, upper or 2, lower. this is hardcoded." );
				}
				model = OpenSim::Model(modelFile);

				nh.param<std::string>("geometry_path", geometryPath, "/srv/data/geometry_mobl");	
				// visualizer
				ROS_DEBUG_STREAM("Setting up visualizer");
				//TODO: remove!
				OpenSim::ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
				before_vis();
				visualizer = new OpenSimRT::BasicModelVisualizer(model);
				after_vis();	

				ROS_DEBUG_STREAM("onInit finished just fine.");
			}
			virtual void before_vis()
			{
				ROS_WARN("Not implemented for VisualizerCommon. initial setup of the thing");
			}
			virtual void after_vis()
			{
				ROS_WARN("Not implemented for VisualizerCommon. post-setup of the thing");
			}

			virtual void after_callback()
			{
				ROS_WARN("Not implemented. does something after the callbacks.");
			}
			void callback(const opensimrt_msgs::CommonTimedConstPtr &msg_ik ) {
				try { // main loop
				      // unroll
					SimTK::Vector q(msg_ik->data.size());
					for (size_t i=0;i<msg_ik->data.size();i++)
					{
						q[i] = msg_ik->data[i];
					}
					visualizer->update(q);
					visualizer->fps->actual_delay = (ros::Time::now().toSec() - msg_ik->header.stamp.toSec())*1000; // this is in ms
					after_callback();
				} catch (std::exception& e) {
					std::cout << e.what() << std::endl;
				}
			}
			void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr &msg_ik ) {
				try { // main loop filtered
				      // unroll
					SimTK::Vector q(msg_ik->d0_data.size());
					for (size_t i=0;i<msg_ik->d0_data.size();i++)
					{
						q[i] = msg_ik->d0_data[i];
					}
					visualizer->update(q);
					visualizer->fps->actual_delay = (ros::Time::now().toSec() - msg_ik->header.stamp.toSec())*1000; // this is in ms
					after_callback();

				} catch (std::exception& e) {
					std::cout << e.what() << std::endl;
				}
			}
	};
}
#endif
