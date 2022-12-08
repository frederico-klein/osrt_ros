#ifndef PIPELINE_HEADER_FBK_31052022
#define PIPELINE_HEADER_FBK_31052022
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
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
			CommonNode(bool Debug=true);
			~CommonNode();
			std::vector<std::string> input_labels;
			std::vector<std::string> output_labels;

			ros::NodeHandle nh{"~"};
			//ros::Subscriber sub; //input
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub;
			message_filters::Subscriber<opensimrt_msgs::PosVelAccTimed> sub_filtered;

			ros::Publisher pub; //output
			ros::Publisher pub_filtered; //output, but filtered
			bool publish_filtered = false;
			NamedTables loggers;
			std::string data_save_dir()
			{
				std::string data_save_dir_str; 
				nh.param<std::string>("data_save_dir",data_save_dir_str,"/tmp/");
				return data_save_dir_str;

			};
			bool at_least_one_logger_initialized = false;
			void onInit(int num_sinks = 1);
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback1 not implemented!");
			}
			virtual void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback_filtered not implemented!");
			}
			void saveStos();
			void saveCsvs();
			ros::ServiceServer write_csv, write_sto;
		protected:
			ros::ServiceServer outLabelsSrv;
			bool outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res );
			bool writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool published_labels_at_least_once=false;
			void initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger);
			std::vector<int> deshuffle_input;
			std::vector<std::string> desired_label_order;
	};
	template <typename T> std::vector<int> find_matches(std::vector<T> desired, std::vector<T> shuffled)
	{
		std::vector<int> el;
		assert(desired.size() == shuffled.size());
		for (auto d_item: desired)
		{
			for(int i= 0;i<shuffled.size();i++)
			{
				std::string s_item = shuffled[i];
				if (d_item == s_item)
				{
					el.push_back(i);
					break;
				}
			}

		}
		//first lets assert that el has the same length as desired, that is, all shuffled and desired elements are mapped to each other
		assert(el.size() == desired.size());
		// should have in el a vector which maches shuffled to desired.
		// shuffled[el[i]] == desired[i];
		// Let's test it;
		for (int i=0;i<desired.size();i++)
		{
			assert(shuffled[el[i]]==desired[i]);
		}
		return el;
	}	


}

#endif
