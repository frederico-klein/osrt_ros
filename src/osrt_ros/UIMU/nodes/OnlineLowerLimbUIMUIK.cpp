/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file TestLowerLimbIMUIKFromFile.cpp
 *
 * @brief Test IK with prerecorded NGIMU data on the lower body.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "osrt_ros/UIMU/TfServer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "std_msgs/Header.h"

#include "osrt_ros/UIMU/IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
//#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
    INIReader ini(INI_FILE);
    auto section = "LOWER_LIMB_NGIMU_OFFLINE";

    // imu calibration settings
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);
    auto imuObservationOrder =
            ini.getVector(section, "IMU_BODIES", vector<string>());

    // driver send rate
    auto rate = ini.getInteger(section, "DRIVER_SEND_RATE", 0);

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    //auto ngimuDataFile =
    //        subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // ngimu input data driver from file
    //UIMUInputDriver driver(ngimuDataFile, rate);
    //this is now a ROS thing, so we need to set it up as a ros node 
    ros::NodeHandle n;
    UIMUInputDriver driver(imuObservationOrder, rate); //tf server
    //TfServer* srv = dynamic_cast<TfServer*>(driver.server);
    //srv->set_tfs(imuObservationOrder);
	//srv->set_tfs({"ximu3","ximu3", "ximu3"});
    driver.startListening();
    auto imuLogger = driver.initializeLogger();
    
    ros::Publisher re_pub = n.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
    ros::Publisher labels_pub = n.advertise<opensimrt_msgs::Labels>("r_labels", 1000, true); //latching topic
    //TODO: publish labels
    ROS_WARN_STREAM("not publishing labels!");

    // calibrator
    IMUCalibrator clb(model, &driver, imuObservationOrder);
    clb.recordNumOfSamples(10);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, {}, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);
    
    // mean delay
    int sumDelayMS = 0;
    int numFrames = 0;

    double previousTime = 0;
    double previousDt = 0;
    
    static tf::TransformBroadcaster br;
    static tf::TransformListener listener;

    try { // main loop
        while (!(driver.shouldTerminate())) {
            // get input from sensors
	    ROS_DEBUG_STREAM("Getting frame:");
	    auto imuData = driver.getFrame();
	    ROS_DEBUG_STREAM("Solving inverse kinematics:" );
            numFrames++;

            if (driver.shouldTerminate())
                    break;

            // solve ik
            chrono::high_resolution_clock::time_point t1;
            t1 = chrono::high_resolution_clock::now();

            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});

            chrono::high_resolution_clock::time_point t2;
            t2 = chrono::high_resolution_clock::now();
            sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
                                  .count();

          
	    ROS_DEBUG_STREAM( "pose is:" << pose.q);
	    
	    opensimrt_msgs::CommonTimed msg;
	    std_msgs::Header h;
	    h.stamp = ros::Time::now();
	    h.frame_id = "subject";
	    msg.header = h;
	    //msg.data.push_back(pose.t);
	    msg.time = pose.t;
	    double Dt = msg.time-previousTime;
    	    double jitter = Dt-previousDt;
	    
	    ROS_DEBUG_STREAM("jitter(us):" << jitter*1000000);
	    ROS_DEBUG_STREAM("delta_t   :" << Dt);
	    ROS_DEBUG_STREAM("T (pose.t):" << msg.time);
		
	    for (double joint_angle:pose.q)
	    {
		ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
		msg.data.push_back(joint_angle);
	    }

	    re_pub.publish(msg);

	    //Visualization
	    visualizer.update(pose.q);
            // record
            imuLogger.appendRow(pose.t, driver.frame);//
            qLogger.appendRow(pose.t, ~pose.q);
	    previousTime = pose.t;
	    previousDt = Dt;
	    if(!ros::ok())
		    break;
        }
    } catch (std::exception& e) {
        cout << "Crashed while executing main loop. " << e.what() << endl;

        driver.shouldTerminate(true);
    }

    cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;
    
    CSVFileAdapter::write( qLogger, "test_lower.csv");
    CSVFileAdapter::write( imuLogger, "test_lower_imus.csv");
    // // store results
    // STOFileAdapter::write(
    //         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
}

int main(int argc, char** argv) {
    try {
	ros::init(argc, argv, "online_lower_limb_uimu_ik");
        run();
    } catch (exception& e) {
        cout << "Program crashed while running. Reason: " << e.what() << endl;
        return -1;
    }
    return 0;
}
