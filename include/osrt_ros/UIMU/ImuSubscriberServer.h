#ifndef IMUSUBSCRIBERSERVER_H_GBK20240511
#define IMUSUBSCRIBERSERVER_H_GBK20240511

#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "OrientationProvider.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Imu.h"
#include "tf2_ros/buffer.h"


class OneImuSubscriber
{
 public:
	std::string topic_name;
	ros::Subscriber imu_topic;
	void callback(const sensor_msgs::ImuConstPtr &msg);
	sensor_msgs::Imu last_message;
	OneImuSubscriber(std::string used_name);
};

class ImuSubscriberServer: public OrientationProvider {
 public:
	ImuSubscriberServer(std::vector<std::string> topic_names = {"a", "b", "c"}, std::string topic_prefix = "not_set");
	~ImuSubscriberServer();
	bool receive();

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener listener;
	tf::TransformListener tf1listener;
	//std::vector<double> output;
	std::vector<double> readTopicsIntoOpensim(std::string);
	std::map<std::string,OneImuSubscriber> topic_strs;
	//std::vector<ros::Subscriber> imu_topics;
	std::string world_tf_reference = "/map";
	std::string topic_suffix ="/imu";
	void set_world_reference(std::string);
	void set_subscribers(std::vector<std::string> topic_names, std::string topic_prefix);
	tf::Quaternion calculate_q_from_sensor(const sensor_msgs::Imu last_imu_message);
};
#endif
