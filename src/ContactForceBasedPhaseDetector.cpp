
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/ContactForceBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>

#include "grfm_ros.h"
using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;


Fc::Fc(){
	section = "TEST_CONTACT_FORCE_GRFM_PREDICTION_FROM_FILE";
	Gfrm::init();
	// subject data
	INIReader ini(INI_FILE);
	subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
	
	// virtual contact surface as ground
	auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

	// acceleration-based detector parameters
	auto threshold = ini.getReal(section, "THRESHOLD", 0);

	auto rHeelSphereLocation =
		    ini.getSimtkVec(section, "RIGHT_HEEL_SPHERE_LOCATION", Vec3(0));
	auto lHeelSphereLocation =
		    ini.getSimtkVec(section, "LEFT_HEEL_SPHERE_LOCATION", Vec3(0));
	auto rToeSphereLocation =
		    ini.getSimtkVec(section, "RIGHT_TOE_SPHERE_LOCATION", Vec3(0));
	auto lToeSphereLocation =
		    ini.getSimtkVec(section, "LEFT_TOE_SPHERE_LOCATION", Vec3(0));
	auto contactSphereRadius = ini.getReal(section, "SPHERE_RADIUS", 0);


	// contact force based event detection
	ContactForceBasedPhaseDetector::Parameters detectorParameters;
	detectorParameters.threshold = threshold;
	detectorParameters.windowSize = windowSize;
	detectorParameters.plane_origin = Vec3(0.0, platform_offset, 0.0);
	detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
	detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
	detectorParameters.rToeSphereLocation = rToeSphereLocation;
	detectorParameters.lToeSphereLocation = lToeSphereLocation;
	detectorParameters.sphereRadius = contactSphereRadius;
	detectorParameters.rFootBodyName = rFootBodyName;
	detectorParameters.lFootBodyName = lFootBodyName;
	detector = new ContactForceBasedPhaseDetector(model, detectorParameters);

}

void Fc::operator() (const opensimrt_msgs::CommonTimedConstPtr& message) {
// repeat the simulation `simulationLoops` times
	ROS_INFO_STREAM("Received message"); 
	//for (int k = 0; k < qTable.getNumRows() * simulationLoops; k++) {
	
	// well yes, but actually no.
	// get raw pose from table
	//auto qRaw_old = qTable.getRowAtIndex(i).getAsVector();
	//qRaw_old( qRaw_old +"dddd" +1);
	//std::vector<double> sqRaw = std::vector<double>(message->data.begin() + 1, message->data.end());
	SimTK::Vector qRaw(19); //cant find the right copy constructor syntax. will for loop it
	for (int j = 0;j < qRaw.size();j++)
	{
		//qRaw[j] = sqRaw[j];
		qRaw[j] = message->data[j];
	}
	


	//is it the same
	//if (qRaw.size() != qRaw_old.size())
	//	ROS_FATAL("size is different!");
	//OpenSim::TimeSeriesTable i
	//get_from_subscriber(qRaw,t); //this will set qRaw and t from the subscribert

	//double t_old = qTable.getIndependentColumn()[i];
	
	/*
	 * This is kinda important. The time that matters is the time of the acquisition, I think
	 * The variable time it takes to calculate it doesn't matter too much, UNLESS, it is too big,
	 * then we should probably forget about it and not try to calculate anything!
	 * */
	//NO! I AM NOT SENDING TIME LIKE THIS ANYMORE.
	//double t = message->header.stamp.toSec();
	double t = message->time;
	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-5 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_INFO_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_INFO_STREAM("DeltaT :"<< std::setprecision (15) << t);

	/*if (t_old - t > 0.1)
		ROS_ERROR("Reading from different timestamp! Did I lose a frame");
	else // same timestamp, so we check the indexes are okay.
	{
		for (int it=0; it < qRaw.size(); it++)
		{
			if (qRaw[it] - qRaw_old[it] > 0.1)
				ROS_ERROR("Difference too big");
		}
			
	}
*/
	//	ROS_INFO_STREAM("TAN" << qRaw);

	// increment the time by the total simulation time plus the sampling
	// period, to keep increasing after each simulation loop
//			t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);

	// filter
	auto ikFiltered = filter->filter({t, qRaw});
	auto q = ikFiltered.x;
	auto qDot = ikFiltered.xDot;
	auto qDDot = ikFiltered.xDDot;
	ROS_INFO_STREAM("Filter ran ok");
	// increment loop
/*			if (++i == qTable.getNumRows()) {
	    i = 0;
	    loopCounter++;
	}*/
	if (!ikFiltered.isValid) { return; }
	ROS_INFO_STREAM("Filter results are valid");

	chrono::high_resolution_clock::time_point t1;
	t1 = chrono::high_resolution_clock::now();

		// perform grfm prediction
	detector->updDetector({ikFiltered.t, q, qDot, qDDot});
	ROS_INFO_STREAM("Update detector ok");
	auto grfmOutput = grfm->solve({ikFiltered.t, q, qDot, qDDot});
	ROS_INFO_STREAM("GRFM estimation ran ok");

	chrono::high_resolution_clock::time_point t2;
	t2 = chrono::high_resolution_clock::now();
	sumDelayMS +=
		chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
	sumDelayMSCounter++;

	// project on plane
	grfmOutput.right.point =
		projectionOnPlane(grfmOutput.right.point, grfOrigin);
	grfmOutput.left.point =
		projectionOnPlane(grfmOutput.left.point, grfOrigin);

	// setup ID inputn
	ExternalWrench::Input grfRightWrench = {grfmOutput.right.point,
						grfmOutput.right.force,
						grfmOutput.right.torque};

	ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
					       grfmOutput.left.force,
					       grfmOutput.left.torque};

	ROS_INFO_STREAM("updated visuals ok");

	// solve ID
	auto idOutput = id->solve(
		{ikFiltered.t, q, qDot, qDDot,
		 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});
	
	ROS_INFO_STREAM("inverse dynamics ran ok");

	// visualization
	visualizer->update(q);
	rightGRFDecorator->update(grfmOutput.right.point,
				  grfmOutput.right.force);
	leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);

	// log data (use filter time to align with delay)
	grfRightLogger.appendRow(grfmOutput.t, ~grfmOutput.right.toVector());
	grfLeftLogger.appendRow(grfmOutput.t, ~grfmOutput.left.toVector());
	tauLogger.appendRow(ikFiltered.t, ~idOutput.tau);
	//}
}	
void Fc::finish(){
    cout << "Mean delay: " << double(sumDelayMS) / sumDelayMSCounter << " ms"
	 << endl;

    // Relax tolerance because of floating point errors between target machines
    // (this fails on Windows).
    OpenSimUtils::compareTables(
	    grfRightLogger,
	    TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
					 "force_based/wrench_right.sto"),
	    1e-1);
    OpenSimUtils::compareTables(
	    grfLeftLogger,
	    TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
					 "force_based/wrench_left.sto"),
	    1e-1);
    OpenSimUtils::compareTables(
	    tauLogger,
	    TimeSeriesTable(
		    subjectDir +
		    "real_time/grfm_prediction/force_based/tau.sto"),
	    1e-1);

}

int main(int argc, char **argv) {
    try {
	ros::init(argc, argv, "acceleration_based_swallow_show");
			ros::NodeHandle n;
			ros::Rate loop_rate(10); //TODO: param!

			ros::Subscriber sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1, Fc());	
			ros::spin();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}

