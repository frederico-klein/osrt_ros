#ifndef PIPELINE_HEADER_FBK_31052022
#define PIPELINE_HEADER_FBK_31052022
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include <Common/TimeSeriesTable.h>

#include <message_filters/subscriber.h>


//idk whats going on
#include "Settings.h"

namespace Pipeline
{
	using NamedTable = std::pair<OpenSim::TimeSeriesTable*, std::string >; 
	using NamedTables = std::vector<NamedTable>;

	class CommonNode
	{
		public:
			CommonNode();
			~CommonNode();
			std::vector<std::string> input_labels;
			std::vector<std::string> output_labels;

			ros::NodeHandle nh{"~"};
			//ros::Subscriber sub; //input
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub;

			ros::Publisher pub; //output
			NamedTables loggers;
			std::string data_save_dir{"/data/"};
			bool at_least_one_logger_initialized = false;
			void onInit(int num_sinks = 1);
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback1 not implemented!");
			}
			void saveStos();
			void saveCsvs();
		protected:
			ros::ServiceServer outLabelsSrv;
			bool outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res );
			bool writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool published_labels_at_least_once=false;
			void initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger);

	};
}

#endif
