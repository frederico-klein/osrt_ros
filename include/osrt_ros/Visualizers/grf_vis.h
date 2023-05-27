#ifndef VISUALIZER_GRFM_HEADER_FBK_27052023
#define VISUALIZER_GRFM_HEADER_FBK_27052023

#include "experimental/GRFMPrediction.h"
#include "opensimrt_msgs/Dual.h"
#include "opensimrt_msgs/DualPos.h"
#include "osrt_ros/Visualizers/dualsink_vis.h"

namespace Visualizers
{
	class GrfVis:public Visualizers::DualSinkVis
	{
		public:
			OpenSimRT::ForceDecorator* rightGRFDecorator;
			OpenSimRT::ForceDecorator* leftGRFDecorator;
			void callback(const opensimrt_msgs::DualConstPtr& message);
			void callback_filtered(const opensimrt_msgs::DualPosConstPtr& message); 
			void modify_vis();
	};
}
#endif

