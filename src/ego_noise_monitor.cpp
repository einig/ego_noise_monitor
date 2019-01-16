#include <ego_noise_monitor/ego_noise_monitor.h>

EgoNoiseMonitor::EgoNoiseMonitor(ros::NodeHandle nh, ros::Publisher load_publisher) : load_monitor_thread_(&EgoNoiseMonitor::LoadMonitor, this){
	nh_				= nh;
	load_publisher_	= load_publisher;
	array_msg_.name	= ros::this_node::getName();
	array_msg_.data.clear();
}
EgoNoiseMonitor::~EgoNoiseMonitor() {}

void EgoNoiseMonitor::AddNoiseSource(NoiseSourceSystem *noise_source) {
	noise_sources_.push_back(noise_source);
}

void EgoNoiseMonitor::AddNoiseSource(NoiseSourceTopic *noise_source) {
	noise_sources_.push_back(noise_source);
}

void EgoNoiseMonitor::AddNoiseSource(NoiseSourceDiag *noise_source) {
	noise_sources_.push_back(noise_source);
}

std::list<NoiseSourceSystem*> EgoNoiseMonitor::GetNoiseSourcesSystem() {
	std::list<NoiseSourceSystem*> ns_list;
	for (NoiseSource *noise_source : noise_sources_) {
		if (noise_source->source_type_ == SourceType::SYSTEM) {
			ns_list.push_back(dynamic_cast<NoiseSourceSystem*>(noise_source));
		}
	}
	return ns_list;
}

std::list<NoiseSourceTopic*> EgoNoiseMonitor::GetNoiseSourcesTopic() {
	std::list<NoiseSourceTopic*> ns_list;
	for (NoiseSource *noise_source : noise_sources_) {
		if (noise_source->source_type_ == SourceType::TOPIC) {
			ns_list.push_back(dynamic_cast<NoiseSourceTopic*>(noise_source));
		}
	}
	return ns_list;
}

std::list<NoiseSourceDiag*> EgoNoiseMonitor::GetNoiseSourcesDiag() {
	std::list<NoiseSourceDiag*> ns_list;
	for (NoiseSource *noise_source : noise_sources_) {
		if (noise_source->source_type_ == SourceType::DIAGNOSTICS) {
			ns_list.push_back(dynamic_cast<NoiseSourceDiag*>(noise_source));
		}
	}
	return ns_list;
}

std::list<NoiseSource*> EgoNoiseMonitor::GetNoiseSources() {
	return noise_sources_;
}

void EgoNoiseMonitor::DiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag_msg) {
	NoiseSourceDiag *ns_diag;
	for (diagnostic_msgs::DiagnosticStatus status : diag_msg->status) {
		for (NoiseSourceDiag *ns_diag : EgoNoiseMonitor::GetNoiseSourcesDiag()) {
			if (ns_diag->device_.compare(status.name) == 0) {
				for (diagnostic_msgs::KeyValue key_value : status.values) {
					if (key_value.key.compare(ns_diag->key_) == 0) {
						ns_diag->load_ = std::stof(key_value.value);
						ns_diag->stamp_ = diag_msg->header.stamp;
						ROS_ERROR("%s -- %f", ns_diag->name_.c_str(), ns_diag->load_);
					}
				}
			}
		}
	}
}

// TODO: Remake this for arbitrary topics
void EgoNoiseMonitor::TopicCallback(const topic_tools::ShapeShifter::ConstPtr& msg,
									const std::string &topic_name,
									RosIntrospection::Parser& parser) {
/*	const std::string& datatype   = msg->getDataType();
	const std::string& definition = msg->getMessageDefinition();

	parser.registerMessageDefinition(topic_name, RosIntrospection::ROSType(datatype), definition );

	static std::vector<uint8_t> buffer;
	static std::map<std::string,RosIntrospection::FlatMessage>   flat_containers;

	RosIntrospection::FlatMessage&   flat_container = flat_containers[topic_name];

	buffer.resize(msg->size());
	ros::serialization::OStream stream(buffer.data(), buffer.size());
	msg->write(stream);

	parser.deserializeIntoFlatContainer( topic_name, absl::Span<uint8_t>(buffer), &flat_container, 100);

	// Seach for the key and ensure that there is a key-value pair, assuming they are correctly ordered
	// TODO: verify key_path ends with '/key' and value_path is equal to key_path but ends with '/value'
	for (std::pair<NoiseSourceTopic,float> ns_pair_topic : noise_sources_map_topic_) {
		for (std::pair<RosIntrospection::StringTreeLeaf, std::string>& it : flat_container.name) {
			const std::string& key_path = it.first.toStdString();
			const std::string& key_name = it.second;
			if (key_name.compare(ns_pair_topic.first.key_) == 0) {
				std::pair<RosIntrospection::StringTreeLeaf, std::string>& next = *(&it + 1);
				const std::string& value_path = next.first.toStdString();
				const std::string& value_name = next.second;
				ns_pair_topic.second = std::stof(value_name);
				// publish it
				load_.header.stamp = ros::Time::now();
				load_.name = ns_pair_topic.first.name_;
				load_.type = ns_pair_topic.first.type_;
				load_.header.frame_id = ns_pair_topic.first.frame_id_;
				load_.load = ns_pair_topic.second;
				load_publisher_.publish(load_);
			}
		}
	}*/
}

void EgoNoiseMonitor::LoadMonitor() {
	ros::Rate monitor_rate(10);
	NoiseSourceSystem *ns_file;
	std::string current_load;
	while (ros::ok) {
		for (NoiseSourceSystem *ns_file : EgoNoiseMonitor::GetNoiseSourcesSystem()) {
			std::ifstream file(ns_file->file_);
			if (file) {
				std::ifstream::streampos filesize = file.tellg();
				current_load.reserve(filesize);
				current_load.assign((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
				ns_file->load_ = std::stoi(current_load);
				ns_file->stamp_ = ros::Time::now();
				//ROS_WARN("%s -- %f", ns_file->name_.c_str(), ns_file->load_);
			}
		}
		monitor_rate.sleep();
	}
}

void EgoNoiseMonitor::PublishLoad() {
	array_msg_.data.clear();
	for (NoiseSource *noise_source : noise_sources_) {
		ego_noise_monitor::EgoNoiseLoad load_msg;
		load_msg.header.stamp = noise_source->stamp_;
		load_msg.name = noise_source->name_;
		load_msg.type = noise_source->noise_type_;
		load_msg.header.frame_id = noise_source->frame_id_;
		load_msg.load = noise_source->load_;
		array_msg_.data.push_back(load_msg);
	}
	array_msg_.header.stamp = ros::Time::now();
	load_publisher_.publish(array_msg_);
}