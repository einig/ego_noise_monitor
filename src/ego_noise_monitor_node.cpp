#include "ros/ros.h"

#include <ego_noise_monitor/ego_noise_monitor.h>
#include "ego_noise_monitor/EgoNoiseLoad.h"
#include <list>

int main(int argc, char **argv) {
	ros::init(argc, argv, "monitor_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	ros::Publisher load_pub = nh.advertise<ego_noise_monitor::EgoNoiseArray>("/ego_noise_load", 1);
	EgoNoiseMonitor *egoNoiseMonitor = new EgoNoiseMonitor(nh, load_pub);

	ros::Subscriber diag_sub = nh.subscribe("/diagnostics", 1, &EgoNoiseMonitor::DiagCallback, egoNoiseMonitor);

	ros::Rate publish_rate(2);

	// Load params
	XmlRpc::XmlRpcValue ns_param_list;
	pnh.param("ego_noise_sources", ns_param_list, ns_param_list);

	for(int i =0; i < ns_param_list.size(); i++) {
		// Check yaml elements
		XmlRpc::XmlRpcValue &ns = ns_param_list[i];
		bool parse_file = false;
		bool parse_topic = false;
		bool parse_diag = false;
		std::list<std::string> required_fields = {};
		if (ns.hasMember("file")) {
			parse_file = true;
			required_fields = {"name", "type", "frame_id", "file", "field"};
		} else if (ns.hasMember("topic")) {
			parse_topic = true;
			required_fields = {"name", "type", "frame_id", "topic", "key"};
		} else if (ns.hasMember("device")) {
			parse_diag = true;
			required_fields = {"name", "type", "frame_id", "device", "key"};
		}
		for (std::string element : required_fields) {
			if (!ns.hasMember(element)) {
				ROS_ERROR("ego_noise_sources parameter list is missing '%s' element in member %d", element.c_str(), i+1);
				parse_file = false;
				parse_topic = false;
				parse_diag = false;
			}
		}
		if (parse_file == true) {
			try {
				egoNoiseMonitor->AddNoiseSource(new NoiseSourceSystem(ns["name"], NoiseStringToEnum(ns["type"]), ns["frame_id"], ns["file"], ns["field"]));
			} catch (const std::string& ex) {
				ROS_WARN("Could not create NoiseSource %s from config\n\t%s", static_cast<std::string>(ns["name"]).c_str(), ex.c_str());
			}
		} else if (parse_topic == true) {
			try {
				egoNoiseMonitor->AddNoiseSource(new NoiseSourceTopic(ns["name"], NoiseStringToEnum(ns["type"]), ns["frame_id"], ns["topic"], ns["key"]));
			} catch (const std::string& ex) {
				ROS_WARN("Could not create NoiseSource %s from config\n\t%s", static_cast<std::string>(ns["name"]).c_str(), ex.c_str());
			}
		} else if (parse_diag == true) {
			try {
				egoNoiseMonitor->AddNoiseSource(new NoiseSourceDiag(ns["name"], NoiseStringToEnum(ns["type"]), ns["frame_id"], ns["device"], ns["key"]));
			} catch (const std::string& ex) {
				ROS_WARN("Could not create NoiseSource %s from config\n\t%s", static_cast<std::string>(ns["name"]).c_str(), ex.c_str());
			}
		}
	}

	for (NoiseSource *noise_source : egoNoiseMonitor->GetNoiseSources()) {
		ROS_INFO("%s [%s / %s] : %s", noise_source->name_.c_str(), NoiseTypeString[noise_source->noise_type_], SourceTypeString[noise_source->source_type_], noise_source->frame_id_.c_str());
	}

	std::vector<ros::Subscriber> load_subscribers_;
	// TODO: What happens when a topic is advertised after starting?
	ros::master::V_TopicInfo advertized_topics;
	ros::master::getTopics(advertized_topics);

	RosIntrospection::Parser parser;
	ROS_INFO("Checking advertized topics");
	for (const auto& topic_info: advertized_topics) {
		ROS_INFO("\t %s", topic_info.name.c_str());
	}

	for (NoiseSourceTopic *ns_topic : egoNoiseMonitor->GetNoiseSourcesTopic()) {
		ROS_WARN("Initializing generic subscriber");
		bool found = false;
		std::string topic_name;
		for (const auto& topic_info : advertized_topics) {
			ROS_WARN("Checking %s", topic_info.name.c_str());
			topic_name = ns_topic->topic_;
			if( topic_info.name == topic_name) {
				found = true;
				break;
			}
		}
		if( !found ) {
			printf("This topic has not been published yet: %s\n", ns_topic->topic_.c_str() );
		} else {
			//who is afraid of lambdas and boost::functions ?
			boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
			callback = [egoNoiseMonitor, &parser, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void {
				egoNoiseMonitor->TopicCallback(msg, topic_name, parser);
			};
			load_subscribers_.push_back( nh.subscribe(topic_name, 10, callback) );
			ROS_WARN("LOADED GENERIC SUBSCRIBER %s", topic_name.c_str());
		}
	}

	ROS_WARN("Exited");

	//TODO: spinner count = topic_sources + 2?
	ros::AsyncSpinner spinner(4);
	spinner.start();

	while ((pnh.ok()) && (ros::ok)) {
		ROS_INFO("Node still running");
		egoNoiseMonitor->PublishLoad();
		publish_rate.sleep();
	}
	ros::waitForShutdown();
    return 0;
}