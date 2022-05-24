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
 * @file TestAccelerationGRFMPredictionFromFile.cpp
 *
 * @brief Test the GRF&M prediction method with the AccelerationBased
 * PhaseDetector. Increases the simulation time by repeating the recorded motion
 * X times, in order to provide enough time for the detector to adapt.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
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

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;


class Acc
{
	//TimeSeriesTable qTable;
	//int simulationLoops;
	//int loopCounter;
	LowPassSmoothFilter* filter;
	AccelerationBasedPhaseDetector* detector;
	GRFMPrediction* grfm;
	double sumDelayMS;
	int sumDelayMSCounter;
	//int i;
	InverseDynamics* id;
	BasicModelVisualizer* visualizer;
	ForceDecorator* rightGRFDecorator;
	ForceDecorator* leftGRFDecorator;
	TimeSeriesTable grfRightLogger;
	TimeSeriesTable grfLeftLogger;
	TimeSeriesTable tauLogger;
	Vec3 grfOrigin;
	string subjectDir;

	double previousTime, previousTimeDifference;

	public:
		Acc()
		{
			// subject data
			INIReader ini(INI_FILE);
			auto section = "TEST_ACCELERATION_GRFM_PREDICTION_FROM_FILE";
			subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
			auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
			//auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

			auto grfRightApplyBody =
				ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
			auto grfRightForceExpressed =
				ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
			auto grfRightPointExpressed =
				ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
			auto grfRightPointIdentifier =
				ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
			auto grfRightForceIdentifier =
				ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
			auto grfRightTorqueIdentifier =
				ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

			auto grfLeftApplyBody =
				ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
			auto grfLeftForceExpressed =
				ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
			auto grfLeftPointExpressed =
				ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
			auto grfLeftPointIdentifier =
				ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
			auto grfLeftForceIdentifier =
				ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
			auto grfLeftTorqueIdentifier =
				ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");
			grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", Vec3(0));

			// repeat cyclic motion X times
			//simulationLoops = ini.getInteger(section, "SIMULATION_LOOPS", 0);
			// remove last N samples in motion for smooth transition between loops
			auto removeNLastRows =
				ini.getInteger(section, "REMOVE_N_LAST_TABLE_ROWS", 0);

			// filter
			auto memory = ini.getInteger(section, "MEMORY", 0);
			auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
			auto delay = ini.getInteger(section, "DELAY", 0);
			auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

			// acceleration-based detector parameters
			auto heelAccThreshold = ini.getReal(section, "HEEL_ACC_THRESHOLD", 0);
			auto toeAccThreshold = ini.getReal(section, "TOE_ACC_THRESHOLD", 0);
			auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
			auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
			auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
			auto rHeelLocation =
				ini.getSimtkVec(section, "RIGHT_HEEL_LOCATION_IN_FOOT", Vec3(0));
			auto lHeelLocation =
				ini.getSimtkVec(section, "LEFT_HEEL_LOCATION_IN_FOOT", Vec3(0));
			auto rToeLocation =
				ini.getSimtkVec(section, "RIGHT_TOE_LOCATION_IN_FOOT", Vec3(0));
			auto lToeLocation =
				ini.getSimtkVec(section, "LEFT_TOE_LOCATION_IN_FOOT", Vec3(0));
			auto accLPFilterFreq = ini.getInteger(section, "ACC_LP_FILTER_FREQ", 0);
			auto velLPFilterFreq = ini.getInteger(section, "VEL_LP_FILTER_FREQ", 0);
			auto posLPFilterFreq = ini.getInteger(section, "POS_LP_FILTER_FREQ", 0);
			auto accLPFilterOrder = ini.getInteger(section, "ACC_LP_FILTER_ORDER", 0);
			auto velLPFilterOrder = ini.getInteger(section, "VEL_LP_FILTER_ORDER", 0);
			auto posLPFilterOrder = ini.getInteger(section, "POS_LP_FILTER_ORDER", 0);
			auto posDiffOrder = ini.getInteger(section, "POS_DIFF_ORDER", 0);
			auto velDiffOrder = ini.getInteger(section, "VEL_DIFF_ORDER", 0);

			// grfm parameters
			auto grfmMethod = ini.getString(section, "METHOD", "");
			auto pelvisBodyName = ini.getString(section, "PELVIS_BODY_NAME", "");
			auto rHeelCoPLocation =
				ini.getSimtkVec(section, "RIGHT_HEEL_STATION_LOCATION", Vec3(0));
			auto lHeelCoPLocation =
				ini.getSimtkVec(section, "LEFT_HEEL_STATION_LOCATION", Vec3(0));
			auto rToeCoPLocation =
				ini.getSimtkVec(section, "RIGHT_TOE_STATION_LOCATION", Vec3(0));
			auto lToeCoPLocation =
				ini.getSimtkVec(section, "LEFT_TOE_STATION_LOCATION", Vec3(0));
			auto directionWindowSize =
				ini.getInteger(section, "DIRECTION_WINDOW_SIZE", 0);

			// setup model
			Object::RegisterType(Thelen2003Muscle());
			Model model(modelFile);
			model.initSystem();

			// setup external forces parameters
			ExternalWrench::Parameters grfRightFootPar{
				grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
			auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
					grfRightPointIdentifier, grfRightForceIdentifier,
					grfRightTorqueIdentifier);
			auto grfRightLogger = ExternalWrench::initializeLogger();

			ExternalWrench::Parameters grfLeftFootPar{
				grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
			auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
					grfLeftPointIdentifier, grfLeftForceIdentifier,
					grfLeftTorqueIdentifier);
			auto grfLeftLogger = ExternalWrench::initializeLogger();

			vector<ExternalWrench::Parameters> wrenchParameters;
			wrenchParameters.push_back(grfRightFootPar);
			wrenchParameters.push_back(grfLeftFootPar);

			// get kinematics as a table with ordered coordinates
			//qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
			//		model, ikFile, 0.01);

			// remove last rows in qTable
			//for (int i = 0; i < removeNLastRows; ++i)
			//	qTable.removeRow(qTable.getIndependentColumn().back());

			// setup filters
			LowPassSmoothFilter::Parameters filterParam;
			filterParam.numSignals = model.getNumCoordinates();
			filterParam.memory = memory;
			filterParam.delay = delay;
			filterParam.cutoffFrequency = cutoffFreq;
			filterParam.splineOrder = splineOrder;
			filterParam.calculateDerivatives = true;
			filter = new LowPassSmoothFilter(filterParam);

			// acceleration-based event detector
			AccelerationBasedPhaseDetector::Parameters detectorParameters;
			detectorParameters.heelAccThreshold = heelAccThreshold;
			detectorParameters.toeAccThreshold = toeAccThreshold;
			detectorParameters.windowSize = windowSize;
			detectorParameters.rFootBodyName = rFootBodyName;
			detectorParameters.lFootBodyName = lFootBodyName;
			detectorParameters.rHeelLocationInFoot = rHeelLocation;
			detectorParameters.lHeelLocationInFoot = lHeelLocation;
			detectorParameters.rToeLocationInFoot = rToeLocation;
			detectorParameters.lToeLocationInFoot = lToeLocation;
			detectorParameters.samplingFrequency = 1 / 0.01;
			detectorParameters.accLPFilterFreq = accLPFilterFreq;
			detectorParameters.velLPFilterFreq = velLPFilterFreq;
			detectorParameters.posLPFilterFreq = posLPFilterFreq;
			detectorParameters.accLPFilterOrder = accLPFilterOrder;
			detectorParameters.velLPFilterOrder = velLPFilterOrder;
			detectorParameters.posLPFilterOrder = posLPFilterOrder;
			detectorParameters.posDiffOrder = posDiffOrder;
			detectorParameters.velDiffOrder = velDiffOrder;
			detector = new AccelerationBasedPhaseDetector(model, detectorParameters);

			// grfm prediction
			GRFMPrediction::Parameters grfmParameters;
			grfmParameters.method = GRFMPrediction::selectMethod(grfmMethod);
			grfmParameters.pelvisBodyName = pelvisBodyName;
			grfmParameters.rStationBodyName = rFootBodyName;
			grfmParameters.lStationBodyName = lFootBodyName;
			grfmParameters.rHeelStationLocation = rHeelCoPLocation;
			grfmParameters.lHeelStationLocation = lHeelCoPLocation;
			grfmParameters.rToeStationLocation = rToeCoPLocation;
			grfmParameters.lToeStationLocation = lToeCoPLocation;
			grfmParameters.directionWindowSize = directionWindowSize;
			
			grfm = new GRFMPrediction(model, grfmParameters, detector);

			// id
			id = new InverseDynamics(model, wrenchParameters);
			tauLogger = id->initializeLogger();

			// visualizer
			visualizer = new BasicModelVisualizer(model);
			rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
			visualizer->addDecorationGenerator(rightGRFDecorator);
			leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
			visualizer->addDecorationGenerator(leftGRFDecorator);

			// mean delay
			sumDelayMS = 0;
			sumDelayMSCounter = 0;

			//loopCounter = 0;
			//i = 0;
			previousTime = ros::Time::now().toSec();
			previousTimeDifference = 0;

		}
	
		void operator() (const opensimrt_msgs::CommonTimedConstPtr& message) {
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
		void finish() {
		    cout << "Mean delay: " << double(sumDelayMS) / sumDelayMSCounter << " ms"
			 << endl;

		    // Relax tolerance because of floating point errors between target machines
		    // (this fails on Windows).
		    OpenSimUtils::compareTables(
			    grfRightLogger,
			    TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
							 "acceleration_based/wrench_right.sto"),
			    1e-1);
		    OpenSimUtils::compareTables(
			    grfLeftLogger,
			    TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
							 "acceleration_based/wrench_left.sto"),
			    1e-1);
		    OpenSimUtils::compareTables(
			    tauLogger,
			    TimeSeriesTable(
				    subjectDir +
				    "real_time/grfm_prediction/acceleration_based/tau.sto"),
			    1e-1);
	
		}
};
int main(int argc, char **argv) {
    try {
	ros::init(argc, argv, "swallow_show");
			ros::NodeHandle n;
			ros::Rate loop_rate(10); //TODO: param!

			ros::Subscriber sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1, Acc());	
			ros::spin();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}

