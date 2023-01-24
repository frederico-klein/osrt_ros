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
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
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
		UIMUnode(): Ros::CommonNode(true) //if true debugs
						   //UIMUnode(): Ros::CommonNode()
	{}
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg, yGroundRotDeg, zGroundRotDeg;
		std::vector<std::string> imuObservationOrder;
		double rate;
		ros::Rate* r;
		std::string modelFile;
		std::string loggerFileNameIK,loggerFileNameIMUs;
		std::string tf_frame_prefix;
		bool use_external_average_calibration_method = true;
		double sumDelayMS = 0, numFrames = 0; 
		double previousTime = 0;
		double previousDt = 0;
		OpenSim::TimeSeriesTable imuLogger, qRawLogger, qLogger, qDotLogger, qDDotLogger;

		ros::Publisher time_pub, time_ik_pub;
		bool recording = false;
		UIMUInputDriver *driver;
		InverseKinematics * ik;
		IMUCalibrator * clb;
		BasicModelVisualizer *visualizer;
		bool showMarkers;
		bool visualiseIt= false;
		//ros::Publisher re_pub; 
		//filter parameters
		double cutoffFreq;
		int splineOrder, memory, delay;
		OpenSimRT::LowPassSmoothFilter * ikfilter;
		std::vector<ros::Publisher> plottable_outputs;
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
			nh.param<bool>("visualise", visualiseIt, false);
			// subject data
			nh.param<std::string>("model_file", modelFile, "");
			ROS_INFO_STREAM("Using modelFile:" << modelFile);
			nh.param<std::string>("logger_filename_ik", loggerFileNameIK, "test_ik");
			nh.param<std::string>("logger_filename_imus", loggerFileNameIMUs, "test_imus");

			nh.param<bool>("show_markers", showMarkers, false);

			nh.param<bool>("filter_output",publish_filtered, true);
			nh.param<double>("cutoff_freq", cutoffFreq, 0.0);
			nh.param<int>("memory", memory, 0);
			nh.param<int>("spline_order", splineOrder, 0);
			nh.param<int>("delay", delay, 0);

			nh.param<bool>("use_external_average_calibration_method", use_external_average_calibration_method, true);

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
			time_pub = nh.advertise<std_msgs::Int64>("time",1);		
			time_ik_pub = nh.advertise<std_msgs::Int64>("time_ik",1);		
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
;			clb->setMethod(use_external_average_calibration_method);
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
			ros::NodeHandle nh("~");
			ROS_INFO_STREAM("setting plottable_outputs");
			for (auto l:output_labels)
			{

				plottable_outputs.push_back(nh.advertise<std_msgs::Float64>("ik/joints/"+l,1));
				all_labels+=l+",";
			}
				ROS_INFO_STREAM("Publisher labels: "<<all_labels);

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
					opensimrt_msgs::CommonTimed msg;
					std_msgs::Header h;
					h.frame_id = "subject";
					chrono::high_resolution_clock::time_point t1;
					chrono::high_resolution_clock::time_point t3;
				while (ros::ok() && !driver->shouldTerminate()) {
					t1 = chrono::high_resolution_clock::now();

					// get input from imus
					auto imuData = driver->getFrame();

					// solve ik
					auto pose = ik->solve(
							{imuData.first, {}, clb->transform(imuData.second)});

					h.stamp = ros::Time::now();
					msg.header = h;
					msg.data.clear();

					for (int j= 0; j < pose.q.size()-1; j++)
						msg.data.push_back(pose.q[j+1]);

					msg.time = pose.t;
					/*int i = 0;
					for (double joint_angle:pose.q)
					{
						ROS_DEBUG_STREAM("some joint_angle:"<<joint_angle);
						msg.data.push_back(joint_angle);
						std_msgs::Float64 j_msg;
						j_msg.data = joint_angle;
						plottable_outputs[i].publish(j_msg);
						i++;
					}*/
					pub.publish(msg);
					if (recording)
					{
						imuLogger.appendRow(pose.t, driver->frame);//
						qRawLogger.appendRow(pose.t, ~pose.q);
					}
					
					t3 = chrono::high_resolution_clock::now();
                                	std_msgs::Int64 time_msg;
					time_msg.data = std::chrono::duration_cast<std::chrono::microseconds>(t3 -t1).count();
					time_pub.publish(time_msg);
					r->sleep();
				}
			} catch (std::exception& e) {
				cout << e.what() << endl;

				driver->shouldTerminate(true);
			}

			//CSVFileAdapter::write( qRawLogger, loggerFileNameIK);
			//CSVFileAdapter::write( imuLogger, loggerFileNameIMUs);

			// // store results
			// STOFileAdapter::write(
			//         qRawLogger, subjectDir + "real_time/inverse_kinematics/qRaw_imu.sto");
		}



};



