/**
 * @author      : $USER ($USER@f69d3dc1887d)
 * @file        : ground_orientation
 * @created     : Wednesday Jan 03, 2024 09:02:39 UTC
 */

#ifndef GROUND_ORIENTATION_H

#define GROUND_ORIENTATION_H
#include <ros/ros.h>
#include "opensimrt_msgs/GroundProjectionOrientationAtTimeSrv.h"
#include "XmlRpcException.h"

class GroundNormal
{
	public:
		// This guy needs to keep track of ground related stuff. so maybe it knows where the person is, and maybe it has a subscriber to pointcloud where it keeps track of ground related things?
		// I have the impression that if I keep the ground for a bunch of time, this will get super expensive. maybe the best way is to calculate the normals only when needed
		// OR calculate this with like a gpu process type thing and I will have the normals for every frame, 
		// still expensive, but now it is somewhat doable. and i can perk this up, like, using optical flow I can only calculate if things change and so on. 
		// Now this is just a placeholder class though, it does none of it. it will just return the fixed ramp orientation.
		GroundNormal ()
		{
			// Assuming the node is already initialized externally
			nh_ = ros::NodeHandle("~");  // Private node handle for the class
			service_ = nh_.advertiseService("get_ground_projection_orientation", &GroundNormal::orientationCallback, this);
		// Load quaternion parameter from ROS parameter server, default to no rotation
XmlRpc::XmlRpcValue xmlRpcQuaternion;
if (nh_.getParam("rotation_quaternion", xmlRpcQuaternion)) {
    try {
        rotation_quaternion_.x = static_cast<double>(xmlRpcQuaternion[0]);
        rotation_quaternion_.y = static_cast<double>(xmlRpcQuaternion[1]);
        rotation_quaternion_.z = static_cast<double>(xmlRpcQuaternion[2]);
        rotation_quaternion_.w = static_cast<double>(xmlRpcQuaternion[3]);
        ROS_INFO("Loaded quaternion: x=%f, y=%f, z=%f, w=%f", rotation_quaternion_.x, rotation_quaternion_.y, rotation_quaternion_.z, rotation_quaternion_.w);
    } catch (const XmlRpc::XmlRpcException& e) {
        ROS_ERROR("Failed to convert quaternion parameter: %s", e.getMessage().c_str());
        rotation_quaternion_ = geometry_msgs::Quaternion();  // Set a default value
    }
} else {
    ROS_WARN("Failed to load quaternion parameter. Using default.");
    rotation_quaternion_ = geometry_msgs::Quaternion();  // Set a default value
}
		}	
		~GroundNormal() {}; //does nothing now
		bool orientationCallback(opensimrt_msgs::GroundProjectionOrientationAtTimeSrv::Request &req,
				opensimrt_msgs::GroundProjectionOrientationAtTimeSrv::Response &res) {
			// Your logic to calculate orientation based on received point stamped geometry
			// For simplicity, let's say it returns a default orientation
			//res.orientation.x = 0.0;
			//res.orientation.y = 0.0;
			//res.orientation.z = 0.0;
			//res.orientation.w = 1.0;
			res.orientation = rotation_quaternion_;
			return true;
		}


	private:
		ros::NodeHandle nh_;
		ros::ServiceServer service_;
    		geometry_msgs::Quaternion rotation_quaternion_;
};

#endif /* end of include guard GROUND_ORIENTATION_H */

