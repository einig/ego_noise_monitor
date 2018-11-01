#include "ros/ros.h"

#include <ego_noise_monitor/ego_noise_monitor.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "monitor_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// Load params
	XmlRpc::XmlRpcValue noise_sources_list;
	pnh.param("ego_noise_sources", noise_sources_list, noise_sources_list);

	for(int i =0; i < noise_sources_list.size(); i++) {
		ROS_WARN_STREAM("Noise_Sources: " << noise_sources_list[i]);
	}

	EgoNoiseMonitor *objEgoNoiseMonitor = new EgoNoiseMonitor();

	ROS_INFO("Initialized class");
	/*while ((pnh.ok()) && (ros::ok)) {
		objEgoNoiseMonitor->Test();
	}*/
    return 0;
}