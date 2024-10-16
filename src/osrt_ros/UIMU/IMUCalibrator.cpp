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
#include "osrt_ros/FloatRequest.h"
#include "osrt_ros/FloatResponse.h"
#include "ros/datatypes.h"
#include "ros/init.h"
#include "ros/message_traits.h"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "osrt_ros/Float.h"
#include "tf/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <SimTKcommon/internal/CoordinateAxis.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <SimTKcommon/internal/Rotation.h>
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <thread>
#include <vector>

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

inline constexpr auto hash_djb2a(const std::string_view sv) {
	unsigned long hash{ 5381 };
	for (unsigned char c : sv) {
		hash = ((hash << 5) + hash) ^ c;
	}
	return hash;
}

inline constexpr auto operator"" _sh(const char *str, size_t len) {
	return hash_djb2a(std::string_view{ str, len });
}



void IMUCalibrator::setup(const std::vector<std::string>& observationOrder) {
	//	ros::NodeHandle n("~");
	nhandle = ros::NodeHandle("~");
	auto ghandle = ros::NodeHandle();
	nhandle.param<string>("debug_reference_frame",debug_reference_frame,"map");
	std::string tf_prefix;
	nhandle.param<string>("tf_prefix",tf_prefix,"");

	ext_heading_srv = ghandle.serviceClient<osrt_ros::Float>("calibrate_heading", true);

	R_heading = SimTK::Rotation();
	//why?
	R_GoGi1 = SimTK::Rotation();

	// copy observation order list
	imuBodiesObservationOrder = std::vector<std::string>(
			observationOrder.begin(), observationOrder.end());

	for (auto imu_name:imuBodiesObservationOrder) 
	{
		std::string calib_serv_name =tf_prefix+imu_name+"/pose_average/calibrate_pose"; 
		ROS_WARN_STREAM("calibration service name:"<< calib_serv_name);
		autosrv this_srv;
		this_srv.imu = imu_name;
		this_srv.calib_client = ghandle.serviceClient<std_srvs::Empty>(calib_serv_name, true);
		calib_srv.push_back(this_srv);
	}
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

	auto nh = ros::NodeHandle("~");

	nh.param<bool>("send_start_signal_to_external_heading_calibrator",send_start_signal_to_external_heading_calibrator,false); //. this is incorrect. we should use the string from the_method to match this and start the actual services. I can also bypass a ton of stuff here, which would also make sense for this to be different classes, but well..
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
	bool negate = false;
	double angularDifference = 0.0;
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
		for (size_t g= 0; g < staticPoseQuaternions.size();g++)
		{
			//cout << staticPoseQuaternions[g] << endl;
			ROS_DEBUG_STREAM("Quaternion for imu[" << g << "]" << staticPoseQuaternions[g]);
		}
		const auto q0 = staticPoseQuaternions[baseBodyIndex];
		//ROS_INFO_STREAM("Basebody q0: "<< q0);

		auto inverseq0_rotation_matrix = ~Rotation(q0);

		ROS_INFO_STREAM(cyan << "inverseq0_rotation_matrix: "<< inverseq0_rotation_matrix<<reset);
		const auto base_R = R_GoGi1 * Rotation(q0);

		if (true)
		{
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

		//is y the vertical?
		//
		ROS_INFO_STREAM("baseSegmentXheading with every component" << baseSegmentXheading);
		//I don't know the right way of doing this
		baseSegmentXheading.set(1, 0); //IS this it?
		auto new_vec = baseSegmentXheading.normalize();
		baseSegmentXheading.set(0,new_vec.get(0));
		baseSegmentXheading.set(1,new_vec.get(1));
		baseSegmentXheading.set(2,new_vec.get(2));

		ROS_INFO_STREAM("baseSegmentXheading with what i think is the vertical component set to zero" << baseSegmentXheading);
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

		auto baseTF_R = publish_tf(base_R,0.1,.1,"wtf_base_measured",debug_reference_frame);
		tb.sendTransform(baseTF_R);
		angularDifference = acos(~baseSegmentXheading * baseFrameXInGround);

		// compute sign
		auto xproduct = baseFrameXInGround % baseSegmentXheading;
		if (xproduct.get(1) > 0) { angularDifference *= -1; negate=true; }

		ROS_WARN_STREAM("angularDifference: " << angularDifference << " rad ( " << angularDifference/3.14159205*180.0 << " degrees)");
		// set heading rotation (rotation about Y axis)
		R_heading = Rotation(angularDifference , SimTK::YAxis);

	} else {
		ROS_WARN("No heading correction is applied. Heading rotation is set to "
				"default");
	}
	ros::NodeHandle nh("~");
	bool bypass_everything;
	nh.param<bool>("bypass_everything",bypass_everything,false);

	double ext_head_offset;
	nh.param<double>("heading_offset",ext_head_offset,0.0);

	if (bypass_everything)
	{
		double ext_head;
		ROS_WARN_STREAM("attention!! bypass_everything is set to on!");
		nh.param<double>("heading_debug",ext_head,0.0);
		ROS_WARN_STREAM("The meaning of external heading has changed!!!\nThe value of "<< ext_head << " will not be used. Use rqt_reconfigure calls instead.");


		//R_heading = Rotation(3.141592/180.0*ext_head , SimTK::YAxis);

		//ROS_INFO_STREAM("heading orientation matrix:\n" << R_heading);
	}

	if (abs(ext_head_offset) > 0.1)
	{
		ROS_WARN_STREAM("adding external heading offset of (this should be in degrees btw) " << ext_head_offset);
		R_heading = Rotation(3.141592/180.0*(ext_head_offset) + angularDifference , SimTK::YAxis);
	
		
	}
	///fff.. my angle sign calculation is wrong, so i will use this from opensimrt...
	///this  is awful, i hate it
	if (negate)
	{
		baseHeadingAngle = -abs(baseHeadingAngle);
	}
	else
	{
		baseHeadingAngle = abs(baseHeadingAngle);
	}
	auto full_heading = baseHeadingAngle+ext_head_offset;
	//ROS_INFO_STREAM(cyan << "full heading: "<< full_heading<<reset);
	//R_heading = Rotation(3.141592/180.0*(full_heading) , SimTK::YAxis);


	ROS_INFO_STREAM("heading orientation matrix:\n" << R_heading);
	tb.sendTransform(publish_tf(R_heading,-0.1,.1,"R_heading",debug_reference_frame));
	ros::spinOnce();
	return R_heading;
}


void IMUCalibrator::calibrateIMUTasks(
		vector<InverseKinematics::IMUTask>& imuTasks) {
	for (size_t i = 0; i < staticPoseQuaternions.size(); ++i) {
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
		RR = R_heading; //maybe this should be calculated per imu?
		const auto R_BS = RR * ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO
		//const auto R_BS = ~imuBodiesInGround[bodyName]* RR * R0; // ~R_GB * R_GO
		//const auto R_BS = ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO

		tb.sendTransform(publish_tf(R_BS,0.1*i+0.5,0.4,bodyName,debug_reference_frame));

		ROS_DEBUG_STREAM("Fully corrected orientation matrix:" << bodyName << "\n" << R_BS);

		imuTasks[i].orientation = std::move(R_BS);
		ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
	}
}

void IMUCalibrator::calibrate_ext_signals_sender()
{

	osrt_ros::Float b;
	// I also want to tell every service that they need to start acquiring poses and this needs to be non-blocking!QQ
	ROS_INFO_STREAM("trying to send calibration signals to my fellas imu Quaternion pose average buddies");


	//is this faster? this is 13ms without the spin.
	chrono::high_resolution_clock::time_point t1=chrono::high_resolution_clock::now() ;
	std::vector<std::thread> tts;
	for(autosrv& a_calib_src:calib_srv)
	{
		auto ff = [&](){a_calib_src.calib();};
		tts.push_back(std::thread(ff));
	}
	for (std::thread& t : tts)
		if (t.joinable())
			t.join();
	chrono::high_resolution_clock::time_point t2=chrono::high_resolution_clock::now() ;

	ROS_WARN_STREAM("multiple srv call duration in ms:"<<magenta<<chrono::duration_cast<chrono::milliseconds>(t2-t1).count());

	if(!ext_heading_srv.exists())
		ROS_WARN_STREAM("srv:" <<ext_heading_srv.getService() << " does not exist!!!!!!");
	else
	{
		ROS_INFO_STREAM("trying to call" << ext_heading_srv.getService());
		chrono::high_resolution_clock::time_point tsrv1=chrono::high_resolution_clock::now() ;
		ext_heading_srv.call(b);
		chrono::high_resolution_clock::time_point tsrv2=chrono::high_resolution_clock::now() ;
		ros::spinOnce();
		chrono::high_resolution_clock::time_point tsrv3=chrono::high_resolution_clock::now() ;


		ROS_WARN_STREAM("blames:: service call:"<<green<<chrono::duration_cast<chrono::milliseconds>(tsrv2-tsrv1).count()<<" spinning once"<<chrono::duration_cast<chrono::milliseconds>(tsrv3-tsrv2).count());

		ROS_INFO_STREAM("got heading angle " << b.response.data);
		baseHeadingAngle = -b.response.data*180.0/3.141592;
		ROS_INFO_STREAM("setting heading angle to minus that much, " << baseHeadingAngle);

	}
	chrono::high_resolution_clock::time_point t3=chrono::high_resolution_clock::now() ;

	ROS_WARN_STREAM("heading srv call duration in ms:"<<cyan<<chrono::duration_cast<chrono::milliseconds>(t3-t2).count());

}

void IMUCalibrator::recordNumOfSamples(const size_t& numSamples) {
	if(send_start_signal_to_external_heading_calibrator)
		calibrate_ext_signals_sender();
	else
		impl->recordNumOfSamples(numSamples);
	computeAvgStaticPoseCommon();
}

void IMUCalibrator::recordTime(const double& timeout) {
	if(send_start_signal_to_external_heading_calibrator)
		calibrate_ext_signals_sender();
	else
		impl->recordTime(timeout);
	computeAvgStaticPoseCommon();
}

SimTK::Quaternion IMUCalibrator::getAvgQuaternionFromTopics(std::string imu_name)
{
	SimTK::Quaternion si_q;
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

		si_q[0] = q.w;
		si_q[1] = q.x;
		si_q[2] = q.y;
		si_q[3] = q.z;
		//ROS_WARN("quaternionAverage disabled using one, which should result in an identity matrix rotation");
	}
	else
		ROS_FATAL_STREAM("failed to read avg_pose response for imu" << imu_name);
	return si_q;
}

SimTK::Quaternion IMUCalibrator::getAvgQuaternionFromTF(std::string imu_resolved_name)
{
	SimTK::Quaternion si_q;

	auto imu_calib_from_tf = tfBuffer.lookupTransform(imu_resolved_name+"_imu",imu_resolved_name,ros::Time(0));

	try	
	{
		geometry_msgs::Quaternion q;

		ROS_DEBUG_STREAM(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

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

	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("Error getting transform from %s:%s",imu_resolved_name.c_str(), ex.what());
	}

	return si_q;
}

void IMUCalibrator::computeAvgStaticPoseCommon()
{
	std::string the_method;
	string tf_prefix;
	nhandle.param<string>("the_method",the_method,"");
	ROS_INFO_STREAM("Now calculating average static pose");
	switch(hash_djb2a(the_method)) {
		case "old"_sh:
			std::cout << "You entered \'old\'\n";
			ROS_INFO("This is the default method, using the oldest version of averaging quaternions and internal heading.");
			staticPoseQuaternions = impl->computeAvgStaticPose();
			break;
		case "topics"_sh:
			std::cout << "You entered \'topics\'\n";
			ROS_INFO_STREAM("using external averagingMethod!");
			ROS_WARN_STREAM("This is supposed to be the most accurate version with SVD and internal heading*, but it is slower. (It was not tested with external heading.)");
			publishCalibrationData(); //TODO: this can be parallelised
			for (auto imu_name:imuBodiesObservationOrder) //TODO: this can also be parallelised
			{
				SimTK::Quaternion si_q;
				si_q = getAvgQuaternionFromTopics(imu_name);
				staticPoseQuaternions.push_back(si_q);
			}
			break;
		case "services"_sh:
			std::cout << "You entered \'services\'\n";
			ROS_WARN_STREAM("This is an attempt at making all the calculations external and it sort of works. But use at own risk. Also it requires external heading, or it won't be able to find the TF.");
			nhandle.param<string>("tf_prefix",tf_prefix,"");
			for (auto imu_name:imuBodiesObservationOrder) //TODO: this can also be parallelised
			{
				SimTK::Quaternion si_q;
				string imu_resolved_name = tf_prefix+imu_name; 
				si_q = getAvgQuaternionFromTF(imu_resolved_name);
				staticPoseQuaternions.push_back(si_q);
			}
			break;
		default:
			ROS_ERROR_STREAM("Method "<<the_method <<" not recognized!\n");
			break;
	}
	//let's compare the results:
	if (false)
	{
		ROS_DEBUG_STREAM("\n===== Calibration results ============");
		auto old_avg_response_list = impl->computeAvgStaticPose();
		for (size_t i=0;i<old_avg_response_list.size(); i++)
		{
			ROS_DEBUG_STREAM("\nOLD:" <<old_avg_response_list[i] <<
					"\nNEW;" <<staticPoseQuaternions[i]);
		}
		ROS_DEBUG_STREAM("\n===== End of calibration results =====");
	}
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
