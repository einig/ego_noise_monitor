#include "ros/ros.h"

#include <ego_noise_monitor/ego_noise_monitor.h>
#include "ego_noise_monitor/EgoNoiseLoad.h"
#include <list>

int main(int argc, char **argv) {
	ros::init(argc, argv, "monitor_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ros::Publisher load_pub = nh.advertise<ego_noise_monitor::EgoNoiseLoad>("/ego_noise_load", 1);
	EgoNoiseMonitor *objEgoNoiseMonitor = new EgoNoiseMonitor(load_pub);

	ros::Rate publish_rate(2);

	// Load params
	XmlRpc::XmlRpcValue ns_param_list;
	pnh.param("ego_noise_sources", ns_param_list, ns_param_list);

	for(int i =0; i < ns_param_list.size(); i++) {
		// Check yaml elements
		XmlRpc::XmlRpcValue &ns = ns_param_list[i];
		bool parse_member = true;
		for (std::string element : {"name", "type", "frame_id", "file", "field"}) {
			if (!ns.hasMember(element)) {
				ROS_ERROR("ego_noise_sources parameter list is missing '%s' element in member %d", element.c_str(), i+1);
				parse_member = false;
			}
		}
		if (parse_member == true) {
			NoiseSourceType ns_type;
			try {
				ns_type = StringToEnum(ns["type"]);
				objEgoNoiseMonitor->AddNoiseSource(new NoiseSource(ns["name"], ns_type, ns["frame_id"], ns["file"], ns["field"]));
			} catch (const std::string& ex) {
				ROS_WARN("Could not create NoiseSource %s from config\n\t%s", static_cast<std::string>(ns["name"]).c_str(), ex.c_str());
			}
		}
	}


	std::list<NoiseSource> ns_list = objEgoNoiseMonitor->GetNoiseSources();
	std::list<NoiseSource>::iterator ns_it;
	for (ns_it = ns_list.begin(); ns_it != ns_list.end(); ++ns_it) {
		ROS_INFO_STREAM(ns_it->name_ << " " << NoiseSourceTypeString[ns_it->type_] << " " << ns_it->frame_id_ << " " << ns_it->file_ << " " << ns_it->field_);
	}
	while ((pnh.ok()) && (ros::ok)) {
		objEgoNoiseMonitor->PublishLoad();
		publish_rate.sleep();
	}
    return 0;
}