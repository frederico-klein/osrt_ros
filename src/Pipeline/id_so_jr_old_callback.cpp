#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "Pipeline/include/id_so_jr.h"
#include "MuscleOptimization.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void Pipeline::IdSoJr::old_callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
	
//void Pipeline::IdSoJr::operator() (const opensimrt_msgs::CommonTimedConstPtr& message) {
// repeat the simulation `simulationLoops` times
	ROS_ERROR_STREAM("Received message. Running Id loop"); 
	counter++;
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
		qRaw[j] = message_ik->data[j];
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
	//double t = message_ik->header.stamp.toSec();
	double t = message_ik->time;
	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-5 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_DEBUG_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_DEBUG_STREAM("DeltaT :"<< std::setprecision (15) << t);

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
	//	ROS_DEBUG_STREAM("TAN" << qRaw);

	// increment the time by the total simulation time plus the sampling
	// period, to keep increasing after each simulation loop
//			t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);

	

	// here I need to get the WRENCH, which is the GRF
	//
	// TODO: get wrench message!!!!!!!!!!

	//auto grfRightWrench = magic(message_grf);
	//auto grfLeftWrench = magic(message_grf);

	// setup ID inputn

	/*ExternalWrench::Input grfRightWrench = {grfmOutput.right.point,
						grfmOutput.right.force,
						grfmOutput.right.torque};
	ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
					       grfmOutput.left.force,
					       grfmOutput.left.torque};
	*/
	//cout << "right wrench.";
	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
	//cout << "left wrench.";
	//ROS_INFO_STREAM("rw");
	//print_wrench(grfRightWrench);
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
	//ROS_INFO_STREAM("lw");
	//print_wrench(grfLeftWrench);
//	return;



	// filter
	auto ikFiltered = ikfilter->filter({t, qRaw});
	auto q = ikFiltered.x;
	auto qDot = ikFiltered.xDot;
	auto qDDot = ikFiltered.xDDot;
	ROS_DEBUG_STREAM("Filter ran ok");
	// increment loop
/*			if (++i == qTable.getNumRows()) {
	    i = 0;
	    loopCounter++;
	}*/
	if (!ikFiltered.isValid) {
		ROS_DEBUG_STREAM("filter results are NOT valid");
		return; }
	ROS_DEBUG_STREAM("Filter results are valid");

	//filter wrench!
	//
	
        auto grfRightFiltered =
                grfRightFilter->filter({t, grfRightWrench.toVector()});
        grfRightWrench.fromVector(grfRightFiltered.x);
        auto grfLeftFiltered =
                grfLeftFilter->filter({t, grfLeftWrench.toVector()});
        grfLeftWrench.fromVector(grfLeftFiltered.x);

        if (!ikFiltered.isValid || !grfRightFiltered.isValid ||
            !grfLeftFiltered.isValid) {
            return;
        }

        // perform id
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();
        auto idOutput = id->solve(
                {t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

	sumDelayMSCounter++;

	ROS_DEBUG_STREAM("inverse dynamics ran ok");

	//so part
	//
	// I think that tau is ~idOutput.tau
	auto tau = idOutput.tau;
	for (auto somehting:tau)
	{
		cout << somehting << ";;";	
	}
	cout << endl;
        auto soOutput = so->solve({t, q, tau});

        /*chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
*/
        // visualization
	// so loggers
	
	//jr part
	//
	//
	//
	//
	cout << soOutput.am << endl;
	cout << "two equal numbers:" << soOutput.am.size() << " " << model->getMuscles().getSize() << endl;
	// visualization
	try {
        visualizer->update(q, soOutput.am);
	//visualizer->update(q);
	/*rightGRFDecorator->update(grfRightWrench.point,
				  grfRightWrench.force);
	leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);*/
		ROS_DEBUG_STREAM("visualizer ran ok.");
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}

	try{

	// log data (use filter time to align with delay)
	if(false)
	{
		ROS_WARN_STREAM("THIS SHOULDNT BE RUNNING");

		tauLogger->appendRow(ikFiltered.t, ~idOutput.tau);
		grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
		grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
		qLogger->appendRow(ikFiltered.t, ~q);
		qDotLogger->appendRow(ikFiltered.t, ~qDot);
		qDDotLogger->appendRow(ikFiltered.t, ~qDDot);
        	
	// loggers from SO
		// log data (use filter time to align with delay)
        	fmLogger.appendRow(t, ~soOutput.fm);
        	amLogger.appendRow(t, ~soOutput.am);

		ROS_INFO_STREAM("Added data to loggers. "<< counter);
	}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
	//if (counter > 720)
	//	write_();
	//}
}	
