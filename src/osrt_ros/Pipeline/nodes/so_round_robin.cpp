#include "osrt_ros/Pipeline/so_rr.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "round_robin_hot_mess");
	ros::NodeHandle n("~");
	int num_processes;
	n.param<int>("n_proc", num_processes, 4);
	ROS_WARN_STREAM("num_processes::::" << num_processes);
	Pipeline::SoRR l(n,num_processes);
	l.onInit();
	/**
	 * The MultiThreadedSpinner object allows you to specify a number of threads to use
	 * to call callbacks.  If no explicit # is specified, it will use the # of hardware
	 * threads available on your system.  Here we explicitly specify 4 threads.
	 */
	ros::MultiThreadedSpinner s(num_processes);
	ros::spin(s);

	return 0;
}

