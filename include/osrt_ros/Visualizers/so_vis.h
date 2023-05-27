#ifndef VISUALIZER_SO_HEADER_FBK_26052023
#define VISUALIZER_SO_HEADER_FBK_26052023

#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "osrt_ros/Visualizers/dualsink_vis.h"

namespace Visualizers
{

	class SoVis:public Visualizers::DualSinkVis
	{
		public:
			SoVis() {}
			~SoVis() {}
			void callback(const opensimrt_msgs::DualConstPtr& message);
			void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 

			void before_vis();
			//void after_callback();
	};

}
#endif

