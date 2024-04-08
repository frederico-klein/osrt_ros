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
#include "ros/node_handle.h"
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

const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

void IMUCalibrator::setup(const std::vector<std::string>& observationOrder) {
//	ros::NodeHandle n("~");
  	nhandle = ros::NodeHandle("~");
    	nhandle.param<string>("debug_reference_frame",debug_reference_frame,"map");
	

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

geometry_msgs::TransformStamped  publish_tf(SimTK::Quaternion simq, double z_offset, double x_offset,string name, string debug_reference_frame)
{
	geometry_msgs::Quaternion rosq;
	geometry_msgs::Vector3 translationRos;
	geometry_msgs::TransformStamped rosTF;
	rosq.w = simq[0];
	rosq.x = simq[1];
	rosq.y = simq[2];
	rosq.z = simq[3];
	translationRos.x = -0.6+x_offset;
	translationRos.y = 1.0;
	translationRos.z = -1+z_offset;
	std_msgs::Header h;
	h.frame_id = debug_reference_frame;
	h.stamp = ros::Time::now();
	rosTF.header = h;
	rosTF.transform.rotation = rosq;
	rosTF.transform.translation = translationRos;
	rosTF.child_frame_id = name;
	return rosTF;
}

geometry_msgs::TransformStamped  publish_tf(SimTK::Rotation R, double y_offset, double x_offset,string name, string debug_reference_frame)
{
	ROS_DEBUG_STREAM("im going to publish this tf:\n"<< name <<"\n"<<R);
	SimTK::Quaternion simq = R.convertRotationToQuaternion();

return publish_tf(simq, y_offset, x_offset, name,debug_reference_frame);
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
    ROS_DEBUG_STREAM("ground orientation matrix:\n" << R_GoGiX);
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
	//ROS_DEBUG_STREAM("baseBodyIndex:" << baseBodyIndex );
	for (int g= 0; g < staticPoseQuaternions.size();g++)
	{
		//cout << staticPoseQuaternions[g] << endl;
		ROS_DEBUG_STREAM("Quaternion for imu[" << g << "]" << staticPoseQuaternions[g]);
	}
        const auto q0 = staticPoseQuaternions[baseBodyIndex];
	//ROS_INFO_STREAM("Basebody q0: "<< q0);

	auto inverseq0_rotation_matrix = ~Rotation(q0);

	ROS_INFO_STREAM(cyan << "inverseq0_rotation_matrix: "<< inverseq0_rotation_matrix<<reset);
        const auto base_R = R_GoGi2 * Rotation(q0);
        //const auto base_R = R_GoGi2 * ~Rotation(q0);

	if (true)
	{
	tb.sendTransform(publish_tf(R_GoGi2,0.3,.1,"R_GoGi2",debug_reference_frame));
	tb.sendTransform(publish_tf(~R_GoGi2,0.3,0.15,"R_GoGi2_inverse",debug_reference_frame));
	tb.sendTransform(publish_tf(R_GoGi1,0.35,.1,"R_GoGi1",debug_reference_frame));
	tb.sendTransform(publish_tf(~R_GoGi1,0.35,0.15,"R_GoGi1_inverse",debug_reference_frame));
	tb.sendTransform(publish_tf(q0,0.2,.1,"just_q0_base",debug_reference_frame));
	tb.sendTransform(publish_tf(inverseq0_rotation_matrix,0.2,.15,"q0_inverse_base",debug_reference_frame));
	}
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
	
	ROS_WARN_STREAM("baseFrameXInGround" << baseFrameXInGround);

        // compute the angular difference between the model heading and imu
        // heading
	//
	//
	//this is super fishy. let's show this:
	//
	//
	auto baseTF =  publish_tf(baseFrame->getRotationInGround(state),0,.1,"wtf_base",debug_reference_frame);
	tb.sendTransform(baseTF);
	if (false)
	{
	double offset=0.2;
        const auto base_R__ = R_GoGi2 * Rotation(q0);
	tb.sendTransform(publish_tf(base_R__,0.1,offset,"wtf_base_measured0",debug_reference_frame));
	offset+=0.05;
        const auto base_R__1 = ~R_GoGi2 * Rotation(q0);
	tb.sendTransform(publish_tf(base_R__1,0.1,offset,"wtf_base_measured1",debug_reference_frame));
	offset+=0.05;
        const auto base_R__2 = ~R_GoGi2 * ~Rotation(q0);
	tb.sendTransform(publish_tf(base_R__2,0.1,offset,"wtf_base_measured2",debug_reference_frame));
	offset+=0.05;
        const auto base_R__3 = Rotation(q0)*R_GoGi2;
	tb.sendTransform(publish_tf(base_R__3,0.1,offset,"wtf_base_measured3",debug_reference_frame));
	offset+=0.05;
        const auto base_R__4 = ~Rotation(q0)*R_GoGi2;
	tb.sendTransform(publish_tf(base_R__4,0.1,offset,"wtf_base_measured4",debug_reference_frame));
	offset+=0.05;
        const auto base_R__5 = ~Rotation(q0)*~R_GoGi2;
	tb.sendTransform(publish_tf(base_R__5,0.1,offset,"wtf_base_measured5",debug_reference_frame));
	offset+=0.05;
        const auto base_R__6 = Rotation(q0)*~R_GoGi2;
	tb.sendTransform(publish_tf(base_R__6,0.1,offset,"wtf_base_measured6",debug_reference_frame));
	offset+=0.05;
	offset+=0.05;
	}

	auto baseTF_R = publish_tf(base_R,0.1,.1,"wtf_base_measured",debug_reference_frame);
	tb.sendTransform(baseTF_R);
        auto angularDifference =
                acos(~baseSegmentXheading * baseFrameXInGround);

        // compute sign
        auto xproduct = baseFrameXInGround % baseSegmentXheading;
        if (xproduct.get(1) > 0) { angularDifference *= -1; }

	ROS_WARN_STREAM("angularDifference: " << angularDifference << " ( " << angularDifference/3.14159205*180.0 <<")");
        // set heading rotation (rotation about Y axis)
        R_heading = Rotation(angularDifference , SimTK::YAxis);
    
	for (int jj= 0;jj<10;jj++)
		tb.sendTransform(publish_tf(Rotation(angularDifference*(double(jj)/10.0) , SimTK::YAxis),-0.1,.2,"R_heading"+to_string(jj),debug_reference_frame));
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
	//R_heading = Rotation(yaw, SimTK::YAxis);
	//ROS_WARN_STREAM("yaw: " << yaw << "(" << yaw*180.0/3.14159205);
	
    } else {
        ROS_WARN("No heading correction is applied. Heading rotation is set to "
                "default");
    }
    ros::NodeHandle nh("~");
    bool bypass_everything;
    nh.param<bool>("bypass_everything",bypass_everything,false);

    if (bypass_everything)
    {
    double ext_head;
    nh.param<double>("heading_debug",ext_head,0.0);
    R_heading = Rotation(3.141592/180.0*ext_head , SimTK::YAxis);
    
    ROS_INFO_STREAM("heading orientation matrix:\n" << R_heading);
    }
    tb.sendTransform(publish_tf(R_heading,-0.1,.1,"R_heading",debug_reference_frame));
    return R_heading;
}


void IMUCalibrator::calibrateIMUTasks(
        vector<InverseKinematics::IMUTask>& imuTasks) {
    for (int i = 0; i < staticPoseQuaternions.size(); ++i) {
	ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        const auto& bodyName = imuTasks[i].body;
        
	const auto& q0 = staticPoseQuaternions[i]; //TODO:: maybe make this come from TF directly?

	//const auto Corrected_Q0 = ~Rotation(q0);
    	//ROS_DEBUG_STREAM("Corrected_Q0 orientation matrix:" << bodyName << "\n" << Corrected_Q0);

	const auto R0 = R_GoGi1 * Rotation(q0);
	
	tb.sendTransform(publish_tf(R0,0.1*i+0.5,0.3,"ro_"+bodyName,debug_reference_frame));
    	
	ROS_DEBUG_STREAM("R0 orientation matrix:" << bodyName << "\n" << R0);

	Rotation RR ;

	//if (i==baseBodyIndex)
		RR = R_heading;
	const auto R_BS = RR * ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO
        //const auto R_BS = ~imuBodiesInGround[bodyName]* RR * R0; // ~R_GB * R_GO
        //const auto R_BS = ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO
    
	tb.sendTransform(publish_tf(R_BS,0.1*i+0.5,0.4,bodyName,debug_reference_frame));
    	
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
			
				
			//janky af, this waits for the avg to be available, even though in the new method, we do nothing with this info
			auto topic_name = nhandle.resolveName(imu_name+"/avg_pose");
				ROS_DEBUG_STREAM("trying to read: " <<topic_name);
				res_q = ros::topic::waitForMessage<geometry_msgs::Quaternion>(imu_name+"/avg_pose", nhandle);
			if (old_method_of_getting_averaged)
			{
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
			else
			{
				// this is turning into spaghetti...
				// not sure if it makes sense, but the source of the skeleton (either being republished from ik or id or so right now)
				// and the source of input are different things, or maybe they arent idk. this is like this for now
				string tf_prefix;
				nhandle.param<string>("tf_prefix",tf_prefix,"");
			string imu_calib_name = tf_prefix+imu_name+"_imu"; // this is probably wrong, i make like a complicated name...
			auto imu_calib_from_tf = tfBuffer.lookupTransform(imu_calib_name,tf_prefix+imu_name,ros::Time(0));

				if(res_q)	
				{
					const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

				auto res_q_tf = imu_calib_from_tf.transform.rotation;
				ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
				
				//just in case you are wondering, if no heading is given, these two give the same quaternion. 
				//or almost the same, the tf one is -q, which should be the same, the inverse would be the conjugate (inverted w), 
				//but I repeat, THIS IS NOT THE CASE, if the heading is zero. 
				ROS_INFO_STREAM(magenta <<"[" << res_q->w <<","<< res_q->x <<","<< res_q->y <<","<< res_q->z << "] (w,x,y,z) this if from avg_pose_nonsense!!"<< reset);
				ROS_INFO_STREAM(cyan <<"[" << res_q_tf.w <<","<< res_q_tf.x <<","<< res_q_tf.y <<","<< res_q_tf.z << "] (w,x,y,z) this if from tf!!"<< reset);
				q = imu_calib_from_tf.transform.rotation;
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
			}
		}
		//staticPoseQuaternions = impl->computeAvgStaticPose();
		staticPoseQuaternions = avg_response_list;
    }
    else {
    	staticPoseQuaternions = impl->computeAvgStaticPose();
    }
    //let's compare the results:
    ROS_DEBUG_STREAM("\n===== Calibration results ============");
    for (int i=0;i<old_avg_response_list.size(); i++)
    {
	ROS_DEBUG_STREAM("\nOLD:" <<old_avg_response_list[i] <<
			"\nNEW;" <<staticPoseQuaternions[i]);
    }
    ROS_DEBUG_STREAM("\n===== End of calibration results =====");
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
