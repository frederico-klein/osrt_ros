#include "osrt_ros/Visualizers/id_vis.h"
#include "experimental/GRFMPrediction.h"
#include "opensimrt_msgs/Dual.h"
#include <SimTKcommon/internal/BigMatrix.h>

using opensimrt_msgs::DualConstPtr;

void Visualizers::IdVis::callback(const DualConstPtr &msg) {
	//get q from message
	SimTK::Vector q;
	try {
		visualizer->update(q);
		OpenSimRT::GRFMPrediction::Output grfmOutput;
		// after update q
		ROS_DEBUG_STREAM("updated visuals ok");
		rightGRFDecorator->update(grfmOutput.right.point, grfmOutput.right.force);
		leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
	} catch (std::exception &e) {
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" << std::endl
				<< e.what());
	}
}

