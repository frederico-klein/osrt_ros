#include "osrt_ros/Pipeline/wrench_publisher.h"

WrenchPub::WrenchPub()
{
}

WrenchPub::WrenchPub(ros::NodeHandle nh, std::string wrench_frame_name): n(nh), wrench_frame(wrench_frame_name)
{
}

void WrenchPub::onInit()
{
	ROS_INFO_STREAM("registering this wrench publisher");
	wren = n.advertise<geometry_msgs::WrenchStamped>(wrench_frame,1);
	ROS_INFO_STREAM("registering publisher ok");

}

void WrenchPub::publish(std_msgs::Header h, OpenSimRT::ExternalWrench::Input w)
{
	geometry_msgs::TransformStamped ts;
	geometry_msgs::WrenchStamped ws;
	ts.header = h;

	//prepare wrench
	h.frame_id = wrench_frame;
	ws.header = h;
	//TODO:needs checking
	ws.wrench.force.x = w.force[0];
	ws.wrench.force.y = w.force[1];
	ws.wrench.force.z = w.force[2];
	ws.wrench.torque.x = w.torque[0];
	ws.wrench.torque.y = w.torque[1];
	ws.wrench.torque.z = w.torque[2];
	wren.publish(ws);

	//prepare transform now
	ts.child_frame_id = wrench_frame;
	//ts.header.frame_id = "map"; //TODO: this is incorrect, i should have the correct frame here
	//TODO: needs checking
	//ts.transform.rotation = ?
	ts.transform.rotation.x = 0;
	ts.transform.rotation.y = 0;
	ts.transform.rotation.z = 0;
	ts.transform.rotation.w = 1;
	ts.transform.translation.x = w.point[0];
	ts.transform.translation.y = w.point[1];
	ts.transform.translation.z = w.point[2];

	tb.sendTransform(ts);

}
