#include "opensimrt_msgs/PosVelAccTimed.h"
#include "osrt_ros/UIMU/IMUCalibrator.h"
//#include "INIReader.h"
#include "InverseKinematics.h"
#include "SignalProcessing.h"
#include "osrt_ros/UIMU/UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Visualization.h"
#include <Actuators/Schutte1993Muscle_Deprecated.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/CSVFileAdapter.h>
//#include <OpenSim/Common/STOFileAdapter.h>

#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "opensimrt_msgs/Labels.h"

#include <exception>
#include "osrt_ros/UIMU/TfServer.h"
#include "Ros/include/common_node.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

class UIMUnode: Ros::CommonNode
{
	public:
		UIMUnode(): Ros::CommonNode(false) //if true debugs
		//UIMUnode(): Ros::CommonNode()
	{}
		std::string DATA_DIR = "/srv/data";
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg, yGroundRotDeg, zGroundRotDeg;
		std::vector<std::string> imuObservationOrder;
		double rate;
		ros::Rate* r;
		std::string subjectDir, modelFile;
		std::string loggerFileNameIK,loggerFileNameIMUs;
		std::string tf_frame_prefix;
		double sumDelayMS = 0, numFrames = 0; 
		double previousTime = 0;
		double previousDt = 0;
		OpenSim::TimeSeriesTable imuLogger, qRawLogger, qLogger, qDotLogger, qDDotLogger;

		UIMUInputDriver *driver;
		InverseKinematics * ik;
		IMUCalibrator * clb;
		BasicModelVisualizer *visualizer;
		bool showMarkers;
		//ros::Publisher re_pub; 
		//filter parameters
		double cutoffFreq;
		int splineOrder, memory, delay;
		OpenSimRT::LowPassSmoothFilter * ikfilter;

		void get_params()
		{
			ros::NodeHandle nh("~");
			nh.param<std::string>("tf_frame_prefix",tf_frame_prefix,"not_set");
			//auto section = "UPPER_LIMB_NGIMU";
			// imu calibration settings

			nh.param<std::string>("imu_direction_axis", imuDirectionAxis, "");

			//    imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
			nh.param<std::string>("imu_base_body", imuBaseBody, "");

			//    imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
			nh.param<double>("imu_ground_rotation_x", xGroundRotDeg, 0.0);
			//   xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0.0);
			nh.param<double>("imu_ground_rotation_y", yGroundRotDeg, 0.0);
			//    yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0.0);
			nh.param<double>("imu_ground_rotation_z", zGroundRotDeg, 0.0);
			//    zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0.0);
			nh.getParam("imu_observation_order", imuObservationOrder);
			if (imuObservationOrder.size() == 0)
			{
				ROS_FATAL("IMU observation order not defined!");
				throw(std::invalid_argument("imuObservationOrder not defined."));
			}
			else
			{
				ROS_INFO_STREAM("Adding tf_frame_prefix [" << tf_frame_prefix<< "] to tfs to be read.");
			}
			//    imuObservationOrder =
			//	    ini.getVector(section, "IMU_BODIES", vector<string>());

			// driver send rate
			nh.param<double>("rate", rate, 0.0);
			r = new ros::Rate(rate);
			//    rate = ini.getInteger(section, "DRIVER_SEND_RATE", 0);

			// subject data
			std::string subjectDir_;
			nh.param<std::string>("subject_dir", subjectDir_, "");
			subjectDir = DATA_DIR + subjectDir_;
			ROS_INFO_STREAM("using subjectDir:" << subjectDir);
			//    subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
			std::string modelFile_;
			nh.param<std::string>("model_file", modelFile_, "");
			modelFile = subjectDir + modelFile_;
			ROS_INFO_STREAM("Using modelFile:" << modelFile);
			//    modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
			//auto ngimuDataFile =
			//        subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");
			nh.param<std::string>("logger_filename_ik", loggerFileNameIK, "test_ik");
			nh.param<std::string>("logger_filename_imus", loggerFileNameIMUs, "test_imus");

			nh.param<bool>("show_markers", showMarkers, false);

			nh.param<bool>("filter_output",publish_filtered, true);
			nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
			nh.param<int>("memory", memory, 0);
			nh.param<int>("spline_order", splineOrder, 0);
			nh.param<int>("delay", delay, 0);

			ROS_DEBUG_STREAM("Finished getting params.");	

		}
		void registerType(Object* muscleModel) //do I even need this?
		{

			Object::registerType(*muscleModel);

		}
		void onInit() 

		{
			get_params();
			Ros::CommonNode::onInit(0); //we are not reading from anything, we are a source
							 // setup model
			ROS_DEBUG_STREAM("Setting up model.");
			Model model(modelFile);
			OpenSimUtils::removeActuators(model);

			// marker tasks
			ROS_DEBUG_STREAM("Setting up markerTasks");
			vector<InverseKinematics::MarkerTask> markerTasks;
			if (showMarkers) //not sure what this does, some interface for VICON .trc files. we are not using it here. 
			{
				vector<string> markerObservationOrder;
				InverseKinematics::createMarkerTasksFromMarkerNames(model, {}, markerTasks,
						markerObservationOrder);
			}

			// imu tasks
			ROS_DEBUG_STREAM("Setting up imuTasks");
			vector<InverseKinematics::IMUTask> imuTasks;
			InverseKinematics::createIMUTasksFromObservationOrder(
					model, imuObservationOrder, imuTasks);

			ROS_DEBUG_STREAM("Starting driver");
			{
				string imuObservationOrderStr;
				for(auto a:imuObservationOrder)
				{
					imuObservationOrderStr +=a+",";
				}
				ROS_INFO_STREAM("Using imu observation " << imuObservationOrderStr);
			}
			ROS_DEBUG_STREAM("Staring UIMUInputDriver with tf_frame_prefix" << tf_frame_prefix << " and rate: " << rate );
			driver = new UIMUInputDriver(imuObservationOrder,tf_frame_prefix,rate); //uses tf server
			driver->startListening();
			imuLogger = driver->initializeLogger();
			initializeLoggers(loggerFileNameIMUs,&imuLogger);

			// calibrator
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");
			clb = new IMUCalibrator(model, driver, imuObservationOrder);
			ROS_DEBUG_STREAM("clb samples");
			clb->recordNumOfSamples(10);
			ROS_DEBUG_STREAM("setGroundOrientationSeq");
			clb->setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
			ROS_DEBUG_STREAM("heading");
			clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis);

			std::cout << boost::stacktrace::stacktrace() << std::endl;
			clb->calibrateIMUTasks(imuTasks);
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");

			// initialize ik (lower constraint weight and accuracy -> faster tracking)
			ROS_DEBUG_STREAM("Setting up IK");
			ik = new InverseKinematics(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
			qRawLogger = ik->initializeLogger();
			initializeLoggers(loggerFileNameIK,&qRawLogger);

			//TODO: publish correct ROS topics
			output_labels = qRawLogger.getColumnLabels();
			string all_labels;
			for (auto l:output_labels)
				all_labels+=l+",";
			ROS_DEBUG_STREAM("Publisher labels: "<<all_labels);

			// visualizer
			ROS_DEBUG_STREAM("Setting up visualizer");
			ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
			visualizer = new BasicModelVisualizer(model);
			if(publish_filtered)
			{
				//filter
				ROS_DEBUG_STREAM("Setting up filter");
				LowPassSmoothFilter::Parameters ikFilterParam;
				ikFilterParam.numSignals = model.getNumCoordinates();
				ikFilterParam.memory = memory;
				ikFilterParam.delay = delay;
				ikFilterParam.cutoffFrequency = cutoffFreq;
				ikFilterParam.splineOrder = splineOrder;
				ikFilterParam.calculateDerivatives = true;
				ROS_DEBUG_STREAM("filter parameters set.");
				ikfilter = new LowPassSmoothFilter(ikFilterParam);
				// initialize filtered loggers
				ROS_DEBUG_STREAM("getting columnNames from model");
				auto columnNames = OpenSimRT::OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
				string columnNamesStr = "";
				for(auto cn:columnNames)
				{
					columnNamesStr+=cn+",";
				}
				ROS_DEBUG_STREAM("got columnNamesStr: " << columnNamesStr);
				qLogger.setColumnLabels(columnNames);
				qDotLogger.setColumnLabels(columnNames);
				qDDotLogger.setColumnLabels(columnNames);
				ROS_DEBUG_STREAM("columnNames for loggers set.");
				initializeLoggers("qLogger",&qLogger);
				initializeLoggers("qDotLogger",&qDotLogger);
				initializeLoggers("qDDotLogger",&qDDotLogger);

			}
			// mean delay
			ROS_DEBUG_STREAM("onInit finished just fine.");
		}


		void run() {
			try { // main loop
				while (!driver->shouldTerminate()) {

					// get input from imus
					ROS_DEBUG_STREAM("Getting frame:");
					auto imuData = driver->getFrame();
					ROS_DEBUG_STREAM("Solving inverse kinematics:" );
					numFrames++;

					// solve ik
					chrono::high_resolution_clock::time_point t1;
					t1 = chrono::high_resolution_clock::now();

					auto pose = ik->solve(
							{imuData.first, {}, clb->transform(imuData.second)});


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

					pub.publish(msg);
					if(publish_filtered)
					{
						auto ikFiltered = ikfilter->filter({pose.t, pose.q});
						auto q = ikFiltered.x;
						auto qDot = ikFiltered.xDot;
						auto qDDot = ikFiltered.xDDot;
						ROS_DEBUG_STREAM("Filter ran ok");
						if (!ikFiltered.isValid) {
							ROS_DEBUG_STREAM("filter results are NOT valid");
							continue; }
						ROS_DEBUG_STREAM("Filter results are valid");
						opensimrt_msgs::PosVelAccTimed msg_filtered;
						msg_filtered.header = h;
						msg_filtered.time = ikFiltered.t;
						//for loop to fill the data appropriately:
						for (int i=0;i<q.size();i++)
						{
							msg_filtered.d0_data.push_back(q[i]);
							msg_filtered.d1_data.push_back(qDot[i]);
							msg_filtered.d2_data.push_back(qDDot[i]);
						}

						pub_filtered.publish(msg_filtered);
						// visualize filtered!
						visualizer->update(q);
						//adding the data to the loggers
						qLogger.appendRow(pose.t,~q);
						qDotLogger.appendRow(pose.t,~qDot);
						qDDotLogger.appendRow(pose.t,~qDDot);

					}
					else
					{
						// visualize
						visualizer->update(pose.q);
					}
					// record
					imuLogger.appendRow(pose.t, driver->frame);//
					qRawLogger.appendRow(pose.t, ~pose.q);
					previousTime = pose.t;
					previousDt = Dt;
					if(!ros::ok())
						break;
					ros::spinOnce();
					r->sleep();
				}
			} catch (std::exception& e) {
				cout << e.what() << endl;

				driver->shouldTerminate(true);
			}

			cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

			//CSVFileAdapter::write( qRawLogger, loggerFileNameIK);
			//CSVFileAdapter::write( imuLogger, loggerFileNameIMUs);

			// // store results
			// STOFileAdapter::write(
			//         qRawLogger, subjectDir + "real_time/inverse_kinematics/qRaw_imu.sto");
		}



};



