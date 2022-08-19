#include "osrt_ros/UIMU/IMUCalibrator.h"
//#include "INIReader.h"
#include "InverseKinematics.h"
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
#include "osrt_ros/Pipeline/common_node.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

class UIMUnode: Pipeline::CommonNode
{
	public:
		UIMUnode(): Pipeline::CommonNode(false)
	{}
		std::string DATA_DIR = "/srv/data";
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg, yGroundRotDeg, zGroundRotDeg;
		std::vector<std::string> imuObservationOrder;
		double rate;
		ros::Rate* r;
		std::string subjectDir, modelFile;
		std::string csvFileNameIK,csvFileNameIMUs;
		double sumDelayMS = 0, numFrames = 0; 
		double previousTime = 0;
		double previousDt = 0;
		OpenSim::TimeSeriesTable imuLogger, qLogger;

		UIMUInputDriver *driver;
		InverseKinematics * ik;
		IMUCalibrator * clb;
		BasicModelVisualizer *visualizer;
		//ros::Publisher re_pub; 
		void get_params()
		{
			ros::NodeHandle nh("~");

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
			nh.param<std::string>("csv_filename_ik", csvFileNameIK, "test_ik.csv");
			nh.param<std::string>("csv_filename_imus", csvFileNameIMUs, "test_imus.csv");

			ROS_DEBUG_STREAM("Finished getting params.");	

		}
		void registerType(Object* muscleModel) //do I even need this?
		{

			Object::registerType(*muscleModel);

		}
		void onInit() 

		{
			get_params();
			Pipeline::CommonNode::onInit(0); //we are not reading from anything, we are a source
							 // setup model
			ROS_DEBUG_STREAM("Setting up model.");
			Model model(modelFile);
			OpenSimUtils::removeActuators(model);

			// marker tasks
			ROS_DEBUG_STREAM("Setting up markerTasks");
			vector<InverseKinematics::MarkerTask> markerTasks;
			vector<string> markerObservationOrder;
			InverseKinematics::createMarkerTasksFromMarkerNames(model, {}, markerTasks,
					markerObservationOrder);

			// imu tasks
			ROS_DEBUG_STREAM("Setting up imuTasks");
			vector<InverseKinematics::IMUTask> imuTasks;
			InverseKinematics::createIMUTasksFromObservationOrder(
					model, imuObservationOrder, imuTasks);

			// ngimu input data driver from file
			//UIMUInputDriver driver(ngimuDataFile, rate);
			ROS_DEBUG_STREAM("Starting driver");
			for(auto a:imuObservationOrder)
			{
				ROS_INFO_STREAM("Using imu observation " << a);
			}
			driver = new UIMUInputDriver(imuObservationOrder,rate); //tf server
			//ros::NodeHandle n;
			//pub = nh.advertise<opensimrt_msgs::CommonTimed>("r_data", 1000);
			//ros::Publisher labels_pub = n.advertise<opensimrt_msgs::Labels>("r_labels", 1000, true); //latching topic
			//TODO: publish labels
			ROS_WARN_STREAM("not publishing labels!");
			//
			//UIMUInputDriver driver(imuObservationOrder,rate); //tf server
			//TfServer* srv = dynamic_cast<TfServer*>(driver.server);
			//	srv->set_tfs({"ximu3","ximu3", "ximu3"});
			driver->startListening();
			imuLogger = driver->initializeLogger();
			initializeLoggers("imu_logger",&imuLogger);

			// calibrator
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");
			clb = new IMUCalibrator(model, driver, imuObservationOrder);
			ROS_DEBUG_STREAM("clb samples");
			clb->recordNumOfSamples(10);
			ROS_DEBUG_STREAM("r");
			clb->setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
			ROS_DEBUG_STREAM("heading");
			clb->computeHeadingRotation(imuBaseBody, imuDirectionAxis);

			std::cout << boost::stacktrace::stacktrace() << std::endl;
			clb->calibrateIMUTasks(imuTasks);
			ROS_DEBUG_STREAM("Setting up IMUCalibrator");

			// initialize ik (lower constraint weight and accuracy -> faster tracking)
			ROS_DEBUG_STREAM("Setting up IK");
			ik = new InverseKinematics(model, markerTasks, imuTasks, SimTK::Infinity, 1e-5);
			qLogger = ik->initializeLogger();
			initializeLoggers("q_logger",&qLogger);

			// visualizer
			ROS_DEBUG_STREAM("Setting up visualizer");
			ModelVisualizer::addDirToGeometrySearchPaths(DATA_DIR + "/geometry_mobl/");
			visualizer = new BasicModelVisualizer(model);

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

					// visualize
					visualizer->update(pose.q);

					// record
					imuLogger.appendRow(pose.t, driver->frame);//
					qLogger.appendRow(pose.t, ~pose.q);
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

			CSVFileAdapter::write( qLogger, csvFileNameIK);
			CSVFileAdapter::write( imuLogger, csvFileNameIMUs);

			// // store results
			// STOFileAdapter::write(
			//         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
		}



};



