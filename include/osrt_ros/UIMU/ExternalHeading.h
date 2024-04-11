#ifndef EXTERNAL_HEADING_H_29032024
#define EXTERNAL_HEADING_H_29032024

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "geometry_msgs/Transform.h"
#include "ros/subscriber.h"
#include "std_msgs/Float64.h"
#include "tf/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_srvs/Empty.h" //the warning is wrong
#include "visualization_msgs/Marker.h"
//#include "std_srvs/EmptyRequest.h"
//#include "std_srvs/EmptyResponse.h"

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

typedef tf::Vector3 bivector3  ;

class VectorWithLikeAPointOfApplicationThingy
{
	public:
		geometry_msgs::Vector3 v;
		geometry_msgs::Point p;
	
	void setVector(geometry_msgs::Vector3 vv)
	{
		v.x = vv.x;
		v.y = vv.y;
		v.z = vv.z;
	}
	void setOrigin(geometry_msgs::Point pp)
	{
		p.x = pp.x;
		p.y = pp.y;
		p.z = pp.z;
	}

	VectorWithLikeAPointOfApplicationThingy& operator+(const VectorWithLikeAPointOfApplicationThingy& vv)

{
		//there is no way around this, i think, what do i do with the application point?
		this->v.x+= vv.v.x;
		this->v.y+= vv.v.y;
		this->v.z+= vv.v.z;
		return *this;
}
	VectorWithLikeAPointOfApplicationThingy& operator-(const VectorWithLikeAPointOfApplicationThingy& vv)

{
		//there is no way around this, i think, what do i do with the application point?
		this->v.x-= vv.v.x;
		this->v.y-= vv.v.y;
		this->v.z-= vv.v.z;
		return *this;
}
	geometry_msgs::Vector3 getAsVector() //this feels wrong
		{
			geometry_msgs::Vector3 fullV=v;
			fullV.x+=p.x;
			fullV.y+=p.y;
			fullV.z+=p.z;
			return fullV;
		}
	/*geometry_msgs::Vector3 getAsDisplacedVector()
		{
			geometry_msgs::Vector3 fullV=v;
			fullV.x+=p.x;
			fullV.y+=p.y;
			fullV.z+=p.z;
			return fullV;
		}*/
};

class ExternalHeading
{
	public:
		ros::NodeHandle n;
	std::vector<visualization_msgs::Marker> debug_markers;
		bool flip_sign = false;
		bool is_base_body = false;
		bool subscribe_to_external_heading_topic = false;
		double angle_offset = 0;
		bool bypass_everything = false;	
		double angle_manual = 0;
		ros::Publisher heading_angle_publisher;
		ros::Subscriber heading_angle_subscriber;

		std::string imu_default_frame_name, imu_heading_axis, imu_base_measured_frame_name, opensim_base_default_frame_name, heading_reference_frame, negative_heading_reference_frame, parent_frame_name;
		tf2_ros::StaticTransformBroadcaster br;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;
		geometry_msgs::Point origin,debug_point ;
		geometry_msgs::Vector3 heading, opensim_heading;
		ExternalHeading();
		bool calibrate_srv(std_srvs::EmptyRequest & req, std_srvs::EmptyResponse & res);

		double calculate_angle(geometry_msgs::Transform default_measure_frame, geometry_msgs::Transform measured_frame, geometry_msgs::Vector3 imu_heading_axis_vector);

		void callback(std_msgs::Float64 msg);
		void send_a_heading_to_tf(double heading_angle, std::string heading_frame_name );
		geometry_msgs::PoseStamped calibrate();

		visualization_msgs::Marker getArrowForVector(std::string name, geometry_msgs::Vector3, geometry_msgs::Point origin_);
		visualization_msgs::Marker getArrowForVector(std::string name, geometry_msgs::Vector3);
		visualization_msgs::Marker getArrowForVector(std::string name, VectorWithLikeAPointOfApplicationThingy);

};
#endif // EXTERNAL_HEADING_H_29032024
