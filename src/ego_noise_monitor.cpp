#include <ego_noise_monitor/ego_noise_monitor.h>

EgoNoiseMonitor::EgoNoiseMonitor(ros::Publisher load_publisher) : load_monitor_thread_(&EgoNoiseMonitor::LoadMonitor, this){
	load_publisher_	= load_publisher;
	load_.name		= "";
	load_.type		= 0;
	load_.load		= 0;
	current_load_	= "";
}
EgoNoiseMonitor::~EgoNoiseMonitor() {}

void EgoNoiseMonitor::AddNoiseSource(NoiseSource *noise_source) {
	noise_sources_map_[*noise_source] = 0;
}

std::list<NoiseSource> EgoNoiseMonitor::GetNoiseSources() {
	std::list<NoiseSource> noise_sources_list;
	for (std::pair<NoiseSource,float> map_pair : noise_sources_map_) {
		noise_sources_list.push_back(map_pair.first);
	}
	return noise_sources_list;
}

void EgoNoiseMonitor::PublishLoad() {
	for (std::pair<NoiseSource,float> map_pair : noise_sources_map_) {
		load_.header.stamp = ros::Time::now();
		load_.name = map_pair.first.name_;
		load_.type = map_pair.first.type_;
		load_.header.frame_id = map_pair.first.frame_id_;
		load_.load = map_pair.second;
		load_publisher_.publish(load_);
	}
}

void EgoNoiseMonitor::LoadMonitor() {
	ros::Rate monitor_rate(10);
	while (ros::ok) {
		for (std::pair<NoiseSource,float> map_pair : noise_sources_map_) {
			if (map_pair.first.type_ == NoiseSourceType::FAN) {
				ROS_INFO("It's a FAN");
				std::ifstream file(map_pair.first.file_);
				if (file) {
					std::ifstream::streampos filesize = file.tellg();
					current_load_.reserve(filesize);
					current_load_.assign((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
					ROS_INFO("Fan speed is %s", current_load_.c_str());
					noise_sources_map_[map_pair.first] = std::stoi(current_load_);
				}
			}
			ROS_INFO("Checking: %s", map_pair.first.name_.c_str());
		}
		monitor_rate.sleep();
	}
}