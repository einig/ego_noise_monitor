//! ego_noise_monitor/ego_noise_monitor monitors the noise created by the robot itself and the location of the sound source
/*!
*/
#ifndef _EGO_NOISE_MONITOR_H_
#define _EGO_NOISE_MONITOR_H_
#include "ros/ros.h"
#include <ego_noise_monitor/noise_source.h>
#include "ego_noise_monitor/EgoNoiseLoad.h"
#include "ego_noise_monitor/EgoNoiseArray.h"

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"

#include <list>
#include <map>
#include <thread>
#include <fstream>

class EgoNoiseMonitor {
	public:
		//! Constructor
		/*!  */
		EgoNoiseMonitor(ros::NodeHandle nh, ros::Publisher load_publisher);
		~EgoNoiseMonitor();

		void AddNoiseSource(NoiseSourceSystem *noise_source);
		void AddNoiseSource(NoiseSourceTopic *noise_source);
		void AddNoiseSource(NoiseSourceDiag *noise_source);
		std::list<NoiseSourceSystem*> GetNoiseSourcesSystem();
		std::list<NoiseSourceTopic*> GetNoiseSourcesTopic();
		std::list<NoiseSourceDiag*> GetNoiseSourcesDiag();
		std::list<NoiseSource*> GetNoiseSources();
		void TopicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name, RosIntrospection::Parser& parser);
		void DiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg);
		void LoadMonitor();
		void PublishLoad();

	private:
		ros::NodeHandle						nh_;
		std::list<NoiseSource*>				noise_sources_;
		ros::Publisher						load_publisher_;
		ego_noise_monitor::EgoNoiseArray	array_msg_;
		std::thread							load_monitor_thread_;
		std::vector<ros::Subscriber>		load_subscribers_;
};

#endif