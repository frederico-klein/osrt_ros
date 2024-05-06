#include "osrt_ros/Visualizers/id_vis.h"
#include "experimental/GRFMPrediction.h"
#include <SimTKcommon/internal/BigMatrix.h>


void Visualizers::IdVis::callback_multi(const opensimrt_msgs::MultiMessageConstPtr& message)
{
	

	set_delay_from_header(message->header.stamp);
	//simtk vector doesnt have insert...
	SimTK::Vector q(message->ik.data.size()), idOutput_tau(message->other[0].data.size()), grfmLeft(9), grfmRight(9); 
	for (size_t i=0;i<message->ik.data.size(); i++)
	{
		q[i] = message->ik.data[i];
	}
	if(false) //we dont use this
		for (size_t i=0;i<message->other[0].data.size(); i++)
		{
			idOutput_tau[i] = message->other[0].data[i];
		}
	
	ROS_DEBUG_STREAM("creating grfms for visuals");

	for (size_t i=0;i<message->other[1].data.size(); i++)
		{
			grfmLeft[i] = message->other[1].data[i];
		}

	for (size_t i=0;i<message->other[2].data.size(); i++)
		{
			grfmRight[i] = message->other[2].data[i];
		}

	try {
		visualizer->update(q);
		OpenSimRT::GRFMPrediction::Output grfmOutput;
		grfmOutput.left.fromVector(grfmLeft);
		grfmOutput.right.fromVector(grfmRight);
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
