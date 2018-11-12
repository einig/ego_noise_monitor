//! ego_noise_monitor/ego_noise_monitor monitors the noise created by the robot itself and the location of the sound source
/*!
*/
#ifndef _NOISE_SOURCE_H_
#define _NOISE_SOURCE_H_
#include "ros/ros.h"

enum NoiseSourceType {
	FAN,
	MOTOR
};

const char* NoiseSourceTypeString[] = {"FAN", "MOTOR"};

NoiseSourceType StringToEnum(const std::string &s) {
	if (s == "fan" || s == "Fan" || s == "FAN") {
		return FAN;
	} else if (s == "motor" || s == "Motor" || s == "MOTOR") {
		return MOTOR;
	} else {
		throw "Could not parse input string " + s + " to NoiseSourceType";

	}
}

struct NoiseSource {
	std::string name_;
	NoiseSourceType type_;
	std::string frame_id_;
	std::string file_;
	std::string field_;

	NoiseSource(std::string name, NoiseSourceType type, std::string frame_id, std::string file, std::string field) :
				name_(name), type_(type), frame_id_(frame_id), file_(file), field_(field) {}

};
bool operator <(const NoiseSource& ns_one, const NoiseSource& ns_two) {
	return std::tie(ns_one.name_) < std::tie(ns_two.name_);
}

#endif