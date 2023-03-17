/**
 * @author      : frekle (frekle@bml01.mech.kth.se)
 * @file        : events
 * @created     : Friday Mar 17, 2023 14:18:52 CET
 */
#include "osrt_ros/events.h"

void addEvent(std::string event_name, opensimrt_msgs::Event & e)
{
	ROS_INFO_STREAM("Added event" << event_name);
	e.header.stamp = ros::Time::now(); 
	e.name = event_name;
}

void addEvent(std::string event_name, opensimrt_msgs::CommonTimed & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	msg.events.list.push_back(e);

}

void addEvent(std::string event_name, opensimrt_msgs::Events & ee)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	ee.list.push_back(e);

}

opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::CommonTimedConstPtr & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	const std::vector<opensimrt_msgs::Event> ee = msg->events.list;
	std::vector<opensimrt_msgs::Event> eee = ee;
	eee.push_back(e);
	opensimrt_msgs::Events allEvents;
	allEvents.list = eee;
	return allEvents;
}
opensimrt_msgs::Events addEvent(std::string event_name, const opensimrt_msgs::PosVelAccTimedConstPtr & msg)
{
	opensimrt_msgs::Event e;
	addEvent(event_name, e);
	const std::vector<opensimrt_msgs::Event> ee = msg->events.list;
	std::vector<opensimrt_msgs::Event> eee = ee;
	eee.push_back(e);
	opensimrt_msgs::Events allEvents;
	allEvents.list = eee;
	return allEvents;
}

