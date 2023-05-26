#include "osrt_ros/Pipeline/so_rr.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "round_robin_hot_mess");
	ros::NodeHandle n;

	Pipeline::SoRR l(n,4);
	l.init();
	/**
	 * The MultiThreadedSpinner object allows you to specify a number of threads to use
	 * to call callbacks.  If no explicit # is specified, it will use the # of hardware
	 * threads available on your system.  Here we explicitly specify 4 threads.
	 */
	ros::MultiThreadedSpinner s(4);
	ros::spin(s);

	return 0;
}

