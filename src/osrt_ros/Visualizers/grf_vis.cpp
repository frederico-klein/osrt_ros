#include "osrt_ros/Visualizers/grf_vis.h"
#include "experimental/GRFMPrediction.h"
#include "opensimrt_msgs/Dual.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include "opensimrt_bridge/conversions/message_convs.h"

using opensimrt_msgs::DualConstPtr;

void Visualizers::GrfVis::after_vis()
{
	rightGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(rightGRFDecorator);
	leftGRFDecorator = new OpenSimRT::ForceDecorator(SimTK::Green, 0.001, 3);
	visualizer->addDecorationGenerator(leftGRFDecorator);


}

void Visualizers::GrfVis::callback(const DualConstPtr &msg) {
	ROS_ERROR_STREAM("not implemented");
	//get q from message
	SimTK::Vector q(msg->q.data.size());
	for (size_t i=0;i<msg->q.data.size(); i++)
	{
		q[i] = msg->q.data[i];
	}
	//get grfs from message
	OpenSimRT::GRFMPrediction::Output grfmOutput;
	
	try {
		visualizer->update(q);
		// after update q
		ROS_DEBUG_STREAM("updated visuals ok");
		rightGRFDecorator->update(grfmOutput.right.point, grfmOutput.right.force);
		leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
	} catch (std::exception &e) {
		ROS_ERROR_STREAM_ONCE("Error in visualizer. cannot show data!!!!!" << std::endl
				<< e.what());
	}
}

void Visualizers::GrfVis::callback_filtered(const opensimrt_msgs::DualPosConstPtr & msg)
{
	ROS_ERROR_STREAM("not implemented");

}
