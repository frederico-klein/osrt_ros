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
 */
#include "osrt_ros/UIMU/IMUCalibrator.h"
#include "Exception.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "ros/message_traits.h"
#include "ros/time.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

void IMUCalibrator::setup(const std::vector<std::string>& observationOrder) {
//	ros::NodeHandle n("~");
  	nhandle = ros::NodeHandle("~");
	R_heading = SimTK::Rotation();
    //why?
	R_GoGi1 = SimTK::Rotation();
    R_GoGi2 = SimTK::Rotation();

    // copy observation order list
    imuBodiesObservationOrder = std::vector<std::string>(
            observationOrder.begin(), observationOrder.end());

    // initialize system
    state = model.initSystem();
    model.realizePosition(state);

    // get default model pose body orientation in ground
    for (const auto& label : imuBodiesObservationOrder) {
        const OpenSim::PhysicalFrame* frame = nullptr;
	pub.push_back(nhandle.advertise<geometry_msgs::PoseArray>(label +"/imu_cal",1,true)); //latching topic
        if ((frame = model.findComponent<OpenSim::PhysicalFrame>(label))) {
            imuBodiesInGround[label] =
                    frame->getTransformInGround(state).R(); // R_GB
        }
    }
}

geometry_msgs::TransformStamped  publish_tf(SimTK::Quaternion simq, double y_offset, double x_offset,string name)
{
	geometry_msgs::Quaternion rosq;
	geometry_msgs::Vector3 translationRos;
	geometry_msgs::TransformStamped rosTF;
	rosq.w = simq[0];
	rosq.x = simq[1];
	rosq.y = simq[2];
	rosq.z = simq[3];
	translationRos.x = 1+x_offset;
	translationRos.y = -1+y_offset;
	translationRos.z = 1;
	std_msgs::Header h;
	h.frame_id = "map";
	h.stamp = ros::Time::now();
	rosTF.header = h;
	rosTF.transform.rotation = rosq;
	rosTF.transform.translation = translationRos;
	rosTF.child_frame_id = name;
	return rosTF;
}

geometry_msgs::TransformStamped  publish_tf(SimTK::Rotation R, double y_offset, double x_offset,string name)
{
	SimTK::Quaternion simq = R.convertRotationToQuaternion();

return publish_tf(simq, y_offset, x_offset, name);
}

SimTK::Rotation IMUCalibrator::setGroundOrientationSeq(const double& xDegrees,
                                                       const double& yDegrees,
                                                       const double& zDegrees) {
    auto xRad = SimTK::convertDegreesToRadians(xDegrees);
    auto yRad = SimTK::convertDegreesToRadians(yDegrees);
    auto zRad = SimTK::convertDegreesToRadians(zDegrees);


    //my trusty calculator (https://www.andre-gaschler.com/rotationconverter/) tells me that this is in fact a zyx euler rotation.
    auto R_GoGiX = Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, xRad,
                      SimTK::XAxis, yRad, SimTK::YAxis, zRad, SimTK::ZAxis);
    ROS_WARN_STREAM("ground orientation matrix:\n" << R_GoGiX);
    return R_GoGiX;
}

SimTK::Rotation
IMUCalibrator::computeHeadingRotation(const std::string& baseImuName,
                                      const std::string& imuDirectionAxis) {
    if (!imuDirectionAxis.empty() && !baseImuName.empty()) {
        // set coordinate direction based on given imu direction axis given as
        // string
        std::string imuAxis = IO::Lowercase(imuDirectionAxis);
        SimTK::CoordinateDirection baseHeadingDirection(SimTK::ZAxis);
        int direction = 1;
        if (imuAxis.front() == '-') direction = -1;
        const char& back = imuAxis.back();
        if (back == 'x')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::XAxis, direction);
        else if (back == 'y')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::YAxis, direction);
        else if (back == 'z')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::ZAxis, direction);
        else { // Throw, invalid specification
            THROW_EXCEPTION("Invalid specification of heading axis '" +
                            imuAxis + "' found.");
        }

        // find base imu body index in observation order
        baseBodyIndex = std::distance(
                imuBodiesObservationOrder.begin(),
                std::find(imuBodiesObservationOrder.begin(),
                          imuBodiesObservationOrder.end(), baseImuName));

        // get initial measurement of base imu
	//cout << baseBodyIndex << endl;
	ROS_DEBUG_STREAM("baseBodyIndex:" << baseBodyIndex );
	for (int g= 0; g < staticPoseQuaternions.size();g++)
	{
		//cout << staticPoseQuaternions[g] << endl;
		ROS_DEBUG_STREAM("Quaternion for imu[" << g << "]" << staticPoseQuaternions[g]);
	}
        const auto q0 = staticPoseQuaternions[baseBodyIndex];
	ROS_INFO_STREAM("Basebody q0: "<< q0);

	auto inverseq0_rotation_matrix = ~Rotation(q0);

	ROS_INFO_STREAM("inverseq0_rotation_matrix: "<< inverseq0_rotation_matrix);
        const auto base_R = R_GoGi2 * ~Rotation(q0);

	tb.sendTransform(publish_tf(R_GoGi2,0.3,1,"R_GoGi2"));
	tb.sendTransform(publish_tf(R_GoGi1,0.35,1,"R_GoGi1"));
	tb.sendTransform(publish_tf(q0,0.2,1,"just_q0"));

        //const SimTK::Rotation base_R = ~Rotation(q0);

        // get initial direction from the imu measurement (the axis looking
        // front)
        UnitVec3 baseSegmentXheading = base_R(baseHeadingDirection.getAxis());
        if (baseHeadingDirection.getDirection() < 0)
	{
		//this thing here implements the minus sign of the baseHeadingDirection variable which is a parameter we set and it is only used here, it seems.
            baseSegmentXheading = baseSegmentXheading.negate();
	}
	ROS_INFO_STREAM("baseSegmentXheading" << baseSegmentXheading);
        // get frame of imu body
        const PhysicalFrame* baseFrame = nullptr;
        if (!(baseFrame = model.findComponent<PhysicalFrame>(baseImuName))) {
            THROW_EXCEPTION(
                    "Frame of given body name does not exist in the model.");
        }

        // express unit x axis of local body frame to ground frame
        Vec3 baseFrameX = UnitVec3(1, 0, 0);
        const SimTK::Transform& baseXForm =
                baseFrame->getTransformInGround(state);
        Vec3 baseFrameXInGround = baseXForm.xformFrameVecToBase(baseFrameX);

        // compute the angular difference between the model heading and imu
        // heading
	//
	//
	//this is super fishy. let's show this:
	//
	//
	auto baseTF =  publish_tf(baseFrame->getRotationInGround(state),0,1,"wtf_base");
	tb.sendTransform(baseTF);

	auto baseTF_R = publish_tf(base_R,0.1,1,"wtf_base_measured");
	tb.sendTransform(baseTF_R);
        auto angularDifference =
                acos(~baseSegmentXheading * baseFrameXInGround);

        // compute sign
        auto xproduct = baseFrameXInGround % baseSegmentXheading;
        if (xproduct.get(1) < 0) { angularDifference *= -1; }

	ROS_WARN_STREAM("angularDifference: " << angularDifference);
        // set heading rotation (rotation about Y axis)
        R_heading = Rotation(angularDifference , SimTK::YAxis);
        //R_heading = Rotation(-3.1415/2 , SimTK::YAxis);
        
	/*if (abs(angularDifference) > 3.141592/2)
		R_heading = Rotation(3.1415 , SimTK::YAxis);
	else
       */ 	
	//R_heading = Rotation();
	
	//my heading
	//
	//I think it is yaw
	//
	
	//auto baseq = base_R.convertRotationToQuaternion();
	auto baseq = Rotation(inverseq0_rotation_matrix).convertRotationToQuaternion();
	tf2::Quaternion qqqq(baseq[1],baseq[2],baseq[3],baseq[0]);
	tf2::Matrix3x3 m(qqqq);
	double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
	R_heading = Rotation(yaw, SimTK::YAxis);
	ROS_WARN_STREAM("yaw: " << yaw);
	
    } else {
        ROS_WARN("No heading correction is applied. Heading rotation is set to "
                "default");
    }
    ROS_INFO_STREAM("heading orientation matrix:\n" << R_heading);
    tb.sendTransform(publish_tf(R_heading,-0.1,1,"R_heading"));

    return R_heading;
}


void IMUCalibrator::calibrateIMUTasks(
        vector<InverseKinematics::IMUTask>& imuTasks) {
    for (int i = 0; i < staticPoseQuaternions.size(); ++i) {
	ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        const auto& bodyName = imuTasks[i].body;
        
	const auto& q0 = staticPoseQuaternions[i];

	const auto Corrected_Q0 = ~Rotation(q0);
    	ROS_DEBUG_STREAM("Corrected_Q0 orientation matrix:" << bodyName << "\n" << Corrected_Q0);

	const auto R0 = R_GoGi1 * ~Rotation(q0);
	
	tb.sendTransform(publish_tf(R0,0.1*i+0.5,1.1,"ro_"+bodyName));
    	
	ROS_DEBUG_STREAM("R0 orientation matrix:" << bodyName << "\n" << R0);

	Rotation RR ;

	//if (i==baseBodyIndex)
		RR = R_heading;
        const auto R_BS = RR * ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO
    
	tb.sendTransform(publish_tf(R_BS,0.1*i+0.5,1.2,bodyName));
    	
	ROS_DEBUG_STREAM("Fully corrected orientation matrix:" << bodyName << "\n" << R_BS);

        imuTasks[i].orientation = std::move(R_BS);
	ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    }
}

void IMUCalibrator::recordNumOfSamples(const size_t& numSamples) {
    impl->recordNumOfSamples(numSamples);
	computeAvgStaticPoseCommon();
}

void IMUCalibrator::recordTime(const double& timeout) {
    impl->recordTime(timeout);
	computeAvgStaticPoseCommon();
}

void IMUCalibrator::computeAvgStaticPoseCommon()
{
	//TODO: this is sort of slow, check which are the slow bits to parallelize, it is like a mutex and thread join thing for each of the bits below. 
	//i think maybe the waitformessage is the slow part.
    publishCalibrationData(); //TODO: this can be parallelised
    std::vector<SimTK::Quaternion> old_avg_response_list = impl->computeAvgStaticPose();;

    ROS_INFO_STREAM("Now calculating average static pose");
    if (externalAveragingMethod) //this should be a service call, but service don't get saved in rosbags
    {
		ROS_INFO_STREAM("using external averagingMethod!");
    		//ROS_WARN("not yet implemented, using normal method");
		std::vector<SimTK::Quaternion> avg_response_list;
		for (auto imu_name:imuBodiesObservationOrder) //TODO: this can also be parallelised
		{
			geometry_msgs::QuaternionConstPtr res_q;
			geometry_msgs::Quaternion q;
			auto topic_name = nhandle.resolveName(imu_name+"/avg_pose");
			ROS_DEBUG_STREAM("trying to read: " <<topic_name);
			res_q = ros::topic::waitForMessage<geometry_msgs::Quaternion>(imu_name+"/avg_pose", nhandle);
			if (res_q)
			{
			ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
			ROS_DEBUG_STREAM(res_q);
			q = *res_q;
			ROS_DEBUG_STREAM(q);
			ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
			//the first term of the quaternion is w in simtk:
			//Quaternion 	( 		 )  	[inline]

			//Default constructor produces the ZeroRotation quaternion [1 0 0 0] (not NaN - even in debug mode). 
			
			SimTK::Quaternion si_q;
			si_q[0] = q.w;
			si_q[1] = q.x;
			si_q[2] = q.y;
			si_q[3] = q.z;
			//ROS_WARN("quaternionAverage disabled using one, which should result in an identity matrix rotation");
			//SimTK::Quaternion one;
			//avg_response_list.push_back(one);
			avg_response_list.push_back(si_q);
			}
			else
				ROS_FATAL_STREAM("failed to read avg_pose response for imu" << imu_name);
		}
		//staticPoseQuaternions = impl->computeAvgStaticPose();
		staticPoseQuaternions = avg_response_list;
    }
    else {
    	staticPoseQuaternions = impl->computeAvgStaticPose();
    }
    //let's compare the results:
    ROS_INFO_STREAM("\n===== Calibration results ============");
    for (int i=0;i<old_avg_response_list.size(); i++)
    {
	ROS_INFO_STREAM("\nOLD:" <<old_avg_response_list[i] <<
			"\nNEW;" <<staticPoseQuaternions[i]);
    }
    ROS_INFO_STREAM("\n===== End of calibration results =====");
}
void IMUCalibrator::setMethod(bool method) {
	ROS_INFO_STREAM("setting calibration method");
	externalAveragingMethod = method;
}

void IMUCalibrator::publishCalibrationData()
{
    auto ans =  impl->getTableData();
    int i = 0;
    for (auto qT:ans) // so this is not necessarily aligned. it should be a smart thing, like a map
    { 	
	    ROS_DEBUG_STREAM("iterating over imus[" << i << "]: " << imuBodiesObservationOrder[i] );
	    geometry_msgs::PoseArray p_msg;
	    
	    for (auto q:qT)
	    {
		    //ROS_DEBUG_STREAM_ONCE("Is the Quaternion order correct? The SVD method shouldn't care, but maybe that could be wrong?");
		    geometry_msgs::Pose pp;
		    pp.orientation.w = q[0];
		    pp.orientation.x = q[1];
		    pp.orientation.y = q[2];
		    pp.orientation.z = q[3];
		    ROS_DEBUG_STREAM(q);
		    p_msg.poses.push_back(pp);
	    }
	    pub[i].publish(p_msg);
	    i++;
    }


}
