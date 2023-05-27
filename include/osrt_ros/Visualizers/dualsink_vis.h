#ifndef VISUALIZER_DUALSINK_HEADER_FBK_26052023
#define VISUALIZER_DUALSINK_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "osrt_ros/Visualizers/visualizer_common.h"

namespace Visualizers
{

	class DualSinkVis:public Visualizers::VisualizerCommon
	{
		public:
			DualSinkVis() {}
			~DualSinkVis() {}
			void callback(const opensimrt_msgs::DualConstPtr& message);
			void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 

			virtual void modify_vis()
			{
				sub = nh.subscribe("input",1, &Visualizers::DualSinkVis::callback, this);
				sub_filtered = nh.subscribe("input_filtered",1, &Visualizers::DualSinkVis::callback_filtered, this);
				
			}
			virtual void after_callback() {};
	};

}
#endif

