//! ego_noise_monitor/ego_noise_monitor monitors the noise created by the robot itself and the location of the sound source
/*!
*/
#ifndef _NOISE_SOURCE_H_
#define _NOISE_SOURCE_H_
#include "ros/ros.h"

enum NoiseType {
	FAN,
	MOTOR
};

const char* NoiseTypeString[] = {"FAN", "MOTOR"};

NoiseType NoiseStringToEnum(const std::string &s) {
	if (s == "fan" || s == "Fan" || s == "FAN") {
		return FAN;
	} else if (s == "motor" || s == "Motor" || s == "MOTOR") {
		return MOTOR;
	} else {
		throw "Could not parse input string " + s + " to NoiseType";
	}
}

enum SourceType {
	SYSTEM,
	DIAGNOSTICS,
	TOPIC,
	NONE
};

const char* SourceTypeString[] = {"SYSTEM", "DIAGNOSTICS", "TOPIC", "NONE"};

SourceType SourceStringToEnum(const std::string &s) {
	if (s == "system" || s == "System" || s == "SYSTEM") {
		return SYSTEM;
	} else if (s == "diagnostics" || s == "Diagnostics" || s == "DIAGNOSTICS") {
		return DIAGNOSTICS;
	} else if (s == "topic" || s == "Topic" || s == "TOPIC") {
		return TOPIC;
	} else if (s == "none" || s == "None" || s == "NONE") {
		return NONE;
	} else {
		throw "Could not parse input string " + s + " to NoiseType";
	}
}


struct NoiseSource {
	std::string name_;
	NoiseType noise_type_;
	SourceType source_type_;
	std::string frame_id_;
	ros::Time stamp_;
	float load_;

	NoiseSource(std::string name, NoiseType noise_type, SourceType source_type, std::string frame_id) :
				name_(name), noise_type_(noise_type), source_type_(source_type), frame_id_(frame_id), load_(0.0), stamp_(0) {}
	virtual ~NoiseSource() {}
};

struct NoiseSourceSystem : NoiseSource {
	std::string file_;
	std::string field_;

	NoiseSourceSystem(std::string name, NoiseType noise_type, std::string frame_id, std::string file, std::string field) :
					NoiseSource(name, noise_type_, SYSTEM, frame_id), file_(file), field_(field) {}
	virtual ~NoiseSourceSystem() {}
};
struct NoiseSourceTopic : NoiseSource {
	std::string topic_;
	std::string key_;

	NoiseSourceTopic(std::string name, NoiseType noise_type_, std::string frame_id, std::string topic, std::string key) :
					 NoiseSource(name, noise_type_, TOPIC, frame_id), topic_(topic), key_(key) {}
	virtual ~NoiseSourceTopic() {}
};

struct NoiseSourceDiag : NoiseSource {
	std::string device_;
	std::string key_;

	NoiseSourceDiag(std::string name, NoiseType noise_type_, std::string frame_id, std::string device, std::string key) :
					 NoiseSource(name, noise_type_, DIAGNOSTICS, frame_id), device_(device), key_(key) {}
	virtual ~NoiseSourceDiag() {}
};

bool operator <(const NoiseSource& ns_one, const NoiseSource& ns_two) {
	return std::tie(ns_one.name_) < std::tie(ns_two.name_);
}

#endif