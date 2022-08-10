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
 * @file TestUpperLimbIMUIKFromFile.cpp
 *
 * @brief Test IK with prerecorded NGIMU data on the upper body.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <OpenSim/Common/CSVFileAdapter.h>
//#include <OpenSim/Common/STOFileAdapter.h>

#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "TfServer.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

ros::Publisher chatter_pub;
ros::Publisher poser_pub;

void run() {
    INIReader ini(INI_FILE);
    auto section = "UPPER_LIMB_NGIMU";
    // imu calibration settings
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0.0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0.0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0.0);
    auto imuObservationOrder =
            ini.getVector(section, "IMU_BODIES", vector<string>());

    // driver send rate
    auto rate = ini.getInteger(section, "DRIVER_SEND_RATE", 0);

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    // setup model
    Object::RegisterType(Schutte1993Muscle_Deprecated());
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> markerObservationOrder;
    InverseKinematics::createMarkerTasksFromMarkerNames(model, {}, markerTasks,
                                                        markerObservationOrder);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // ngimu input data driver from file
    //UIMUInputDriver driver(ngimuDataFile, rate);
    ROS_INFO_STREAM("so far so good");
    UIMUInputDriver driver; // tf server
    TfServer* srv = dynamic_cast<TfServer*>(driver.server);
	srv->set_tfs({"ximu3","ximu3", "ximu3"});
    driver.startListening();
    ROS_INFO_STREAM("so far so good 1");

    auto imuLogger = driver.initializeLogger();

    ROS_INFO_STREAM("so far so good 2");
    // calibrator
    IMUCalibrator clb(model, &driver, imuObservationOrder);
    clb.recordNumOfSamples(10);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
    BasicModelVisualizer visualizer(model);

    // mean delay
    int sumDelayMS = 0;
    int numFrames = 0;

    //static tf::TransformBroadcaster br;
    //static tf::TransformListener listener;

    try { // main loop
        while (!driver.shouldTerminate()) {
		/*tf::StampedTransform transform_from_ar;
		try{
      			listener.lookupTransform("/map", "/ar_marker_10",  
                               ros::Time(0), transform_from_ar);
			auto myq = transform_from_ar.getRotation(); //quarternion, but which kind?
	    		ROS_INFO_STREAM("Quat. from ar_marker_10: " 
					<< myq.x() << " " 
					<< myq.y() << " " 
					<< myq.z() << " " 
					<< myq.w() );
   	 	}
    		catch (tf::TransformException ex){
      			ROS_ERROR("%s",ex.what());
      			//ros::Duration(1.0).sleep();
    		}
		*/
            // get input from imus
            auto imuData = driver.getFrame();
            numFrames++;

            // solve ik
            chrono::high_resolution_clock::time_point t1;
            t1 = chrono::high_resolution_clock::now();

            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});
	    
	    //std_msgs::String msg;
	    //std::stringstream ss;
	    //pose is something i can send to cout i think, so this should give me something
	    //
	    //ss << "hello world " << endl;
	    //ss << pose.q[0] << " " << pose.q[1] << " " << pose.q[2];

	    //auto qqqq = imuData.second[0].getQuaternion();
	    //ROS_INFO_STREAM("QQQQ" << qqqq);
	    //	    geometry_msgs::PoseStamped pp;

	    
	    //	    tf2::Quaternion myQuaternion;
	    //            myQuaternion.setRPY(pose.q[0], pose.q[1], pose.q[2]);  // Create this quaternion from roll/pitch/yaw (in radians)
	    //	    geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);
	   
	    //	    pp.header.frame_id = "torax";
	    //	    pp.header.stamp = ros::Time::now();
	    //	    pp.pose.orientation = quat_msg;
	    //	    poser_pub.publish(pp);

	    //tf::Transform transform;
	    //transform.setOrigin( tf::Vector3(0.5, 0.0, 0.0) );
	    //tf::Quaternion q(qqqq[1],qqqq[2],qqqq[3],qqqq[0]); //x,y,z,w
	    //tf::Quaternion q;
	    //ROS_INFO("%f", pose.q[3]);
	    //q.setRPY(pose.q[0], pose.q[1], pose.q[2]);
	    //transform.setRotation(q);
	    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "torax_imu"));


	    //msg.data = ss.str();
	    //ROS_INFO("%s", msg.data.c_str());
	    //chatter_pub.publish(msg);

            chrono::high_resolution_clock::time_point t2;
            t2 = chrono::high_resolution_clock::now();
            sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
                                  .count();

            // visualize
            visualizer.update(pose.q);

            // record
            imuLogger.appendRow(pose.t, driver.frame);//
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) {
        cout << e.what() << endl;

        driver.shouldTerminate(true);
    }

    cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

    CSVFileAdapter::write( qLogger, "test_upper.csv");
    CSVFileAdapter::write( imuLogger, "test_upper_imus.csv");

    // // store results
    // STOFileAdapter::write(
    //         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
}
int main(int argc, char** argv) {
    try {
	/*if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}*/
        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
       	//chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
       	//poser_pub = n.advertise<geometry_msgs::Pose>("poser", 1000);
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}


