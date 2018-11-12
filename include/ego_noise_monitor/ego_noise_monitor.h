//! ego_noise_monitor/ego_noise_monitor monitors the noise created by the robot itself and the location of the sound source
/*!
*/
#ifndef _EGO_NOISE_MONITOR_H_
#define _EGO_NOISE_MONITOR_H_
#include "ros/ros.h"
#include <ego_noise_monitor/noise_source.h>
#include "ego_noise_monitor/EgoNoiseLoad.h"
#include <list>
#include <map>
#include <thread>
#include <fstream>

class EgoNoiseMonitor {
	public:
		//! Constructor
		/*!  */
		EgoNoiseMonitor(ros::Publisher load_publisher);
		~EgoNoiseMonitor();

		void AddNoiseSource(NoiseSource *noise_source);
		std::list<NoiseSource> GetNoiseSources();
		void PublishLoad();
		void LoadMonitor();

	private:
		std::map<NoiseSource,float>		noise_sources_map_;
		ros::Publisher					load_publisher_;
		ego_noise_monitor::EgoNoiseLoad	load_;
		std::thread						load_monitor_thread_;
		std::string						current_load_;
};

#endif