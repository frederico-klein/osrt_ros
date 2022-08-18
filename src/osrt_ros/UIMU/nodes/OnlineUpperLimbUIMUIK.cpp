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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "osrt_ros/UIMU/TfServer.h"
#include "osrt_ros/Pipeline/common_node.h"
#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <boost/stacktrace.hpp>
#include <signal.h>


using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

ros::Publisher chatter_pub;
ros::Publisher poser_pub;

class orientationprovider: Pipeline::CommonNode
{
	public:
		std::string DATA_DIR = "/srv/data";
		std::string imuDirectionAxis;
		std::string imuBaseBody;
		double xGroundRotDeg, yGroundRotDeg, zGroundRotDeg;
		std::vector<std::string> imuObservationOrder;
		double rate;
		
		std::string subjectDir, modelFile;
	    
		static tf::TransformBroadcaster br;
		static tf::TransformListener listener;
		double sumDelayMS = 0, numFrames = 0; 
		OpenSim::TimeSeriesTable imuLogger, qLogger;
		
		UIMUInputDriver *driver;
		InverseKinematics * ik;
		IMUCalibrator * clb;
		BasicModelVisualizer *visualizer;
		
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
			//    imuObservationOrder =
			//	    ini.getVector(section, "IMU_BODIES", vector<string>());

			    // driver send rate
			nh.param<double>("rate", rate, 0.0);
			
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

			ROS_DEBUG_STREAM("Finished getting params.");	

		}
		void onInit() 
		
			{
			    get_params();
			    Pipeline::CommonNode::onInit(0); //we are not reading from anything, we are a source
			    // setup model
			    ROS_DEBUG_STREAM("Setting up model.");
			    Object::RegisterType(Schutte1993Muscle_Deprecated());
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
			    driver = new UIMUInputDriver(imuObservationOrder,rate); //tf server
										    //
			    //UIMUInputDriver driver(imuObservationOrder,rate); //tf server
			    //TfServer* srv = dynamic_cast<TfServer*>(driver.server);
			//	srv->set_tfs({"ximu3","ximu3", "ximu3"});
			    ROS_DEBUG_STREAM("Starting listening");
			    driver->startListening();
			    ROS_DEBUG_STREAM("Starting imuLogger");

			    //imuLogger = driver->initializeLogger();
			    //initializeLoggers("imu_logger",&imuLogger);

			    // calibrator
			    ROS_DEBUG_STREAM("Setting up IMUCalibrator");
			    clb = new IMUCalibrator(model, driver, imuObservationOrder);
			    ROS_DEBUG_STREAM("clb samples");
raise(SIGTRAP); // At the location of the BP.
			    clb->recordNumOfSamples(10);
			    ROS_DEBUG_STREAM("r");
			    clb->setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
			    ROS_DEBUG_STREAM("computing heading: imuBaseBody:" << imuBaseBody << " imuDirectionAxis: " << imuDirectionAxis );
raise(SIGTRAP); // At the location of the BP.
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
				    auto imuData = driver->getFrame();
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

				    // visualize
				    visualizer->update(pose.q);

				    // record
				    imuLogger.appendRow(pose.t, driver->frame);//
				    qLogger.appendRow(pose.t, ~pose.q);
				    if(!ros::ok())
					    break;
				}
			    } catch (std::exception& e) {
				cout << e.what() << endl;

				driver->shouldTerminate(true);
			    }

			    cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;

			    CSVFileAdapter::write( qLogger, "test_upper.csv");
			    CSVFileAdapter::write( imuLogger, "test_upper_imus.csv");

			    // // store results
			    // STOFileAdapter::write(
			    //         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
			}



};



int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
       	chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
       	poser_pub = n.advertise<geometry_msgs::Pose>("poser", 1000);
        orientationprovider o;
	o.onInit();
	o.run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}


