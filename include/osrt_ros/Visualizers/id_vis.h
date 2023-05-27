#ifndef VISUALIZER_ID_HEADER_FBK_27052023
#define VISUALIZER_ID_HEADER_FBK_27052023
#include "OpenSimUtils.h"
#include "opensimrt_msgs/Dual.h"
#include "osrt_ros/Visualizers/grf_vis.h"

namespace Visualizers
{
	class IdVis:public Visualizers::GrfVis
	{
		void callback(const opensimrt_msgs::DualConstPtr& message);
		void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 
		void modify_vis() {
			OpenSimRT::OpenSimUtils::removeActuators(model);
		};
		void after_callback() {};
	};

}


#endif
