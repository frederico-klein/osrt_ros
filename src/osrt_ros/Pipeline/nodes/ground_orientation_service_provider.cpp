/**
 * @author      : $USER ($USER@f69d3dc1887d)
 * @file        : ground_orientation_service_provider
 * @created     : Wednesday Jan 03, 2024 09:25:14 UTC
 */

#include "osrt_ros/Pipeline/ground_orientation.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_projection_orientation_node");
    GroundNormal node;

    ros::spin();  // Keep the node running

    return 0;
}

