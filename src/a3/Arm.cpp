#include "Arm.hpp"
#include "ArmConstants.hpp"
#include "LcmHandler.hpp"
#include "CoordinateConverter.hpp"
#include "math/angle_functions.hpp"
#include <cmath>
#include <cassert>
#include <iostream>
#include <stdio.h>

Arm* Arm::_instance = new Arm();

Arm::Arm() {}

Arm* Arm::instance() {
	return _instance;
}

std::array<float, 2> Arm::forwardKinematicsCartesian(float theta1, 
	float theta2, float theta3, 
	float d1, float d2, float d3) {

	float x = d1 * std::cos(theta1) + d2 * std::cos(theta1 + theta2) + 
		d3 * std::cos(theta1 + theta2 + theta3);
	float y = d1 * std::sin(theta1) + d2 * std::sin(theta1 + theta2) + 
		d3 * std::sin(theta1 + theta2 + theta3);
	return std::array<float, 2>{{x, y}};
}

std::array<float, 2> Arm::forwardKinematicsPolar(float theta1, 
	float theta2, float theta3,
	float d1, float d2, float d3) {

	float x = d1 * std::cos(theta1) + d2 * std::cos(theta1 + theta2) + 
		d3 * std::cos(theta1 + theta2 + theta3);
	float y = d1 * std::sin(theta1) + d2 * std::sin(theta1 + theta2) + 
		d3 * std::sin(theta1 + theta2 + theta3);
	float r = std::sqrt(x*x + y*y);
	float theta = std::atan2(y, x);
	return std::array<float, 2>{{r, theta}};
}

std::array<float, 2> Arm::forwardKinematicsPolar(const std::array<float, 3>& angles, 
	float d1, float d2, float d3) {

	float x = d1 * std::cos(angles[0]) + d2 * std::cos(angles[0] + angles[1]) + 
		d3 * std::cos(angles[0] + angles[1] + angles[2]);
	float y = d1 * std::sin(angles[0]) + d2 * std::sin(angles[0] + angles[1]) + 
		d3 * std::sin(angles[0] + angles[1] + angles[2]);
	float r = std::sqrt(x*x + y*y);
	float theta = std::atan2(y, x);
	return std::array<float, 2>{{r, theta}};
}

bool Arm::inverseKinematicsCartesian(const std::array<float, 2>& coord, 
		const dynamixel_status_list_t& currStatus,
		std::array<float, 3>& angles) {
	std::array<float, 2> polarCoords = 
		CoordinateConverter::cartesianToPolar(coord);
	return inverseKinematicsPolar(polarCoords, currStatus, angles);
}

bool Arm::inverseKinematicsPolar(const std::array<float, 2>& polarCoords, 
		const dynamixel_status_list_t& currStatus,
		std::array<float, 3>& angles) {
	assert(currStatus.len == 3);

	std::array<float, 3> tempThetas;
	float minError = -1;
	for (tempThetas[1] = 0; tempThetas[1] <= M_PI/2; tempThetas[1] += M_PI/120) {
		// std::cout << "theta: " << tempThetas[1] << '\n';
		float a = -2 * armLength1 * armLength3 * std::sin(tempThetas[1]);
		float b = 2 * armLength2 * armLength3 + 
			2 * armLength1 * armLength3 * std::cos(tempThetas[1]);
		float c = armLength1 * armLength1 + 
			armLength2 * armLength2 + 
			armLength3 * armLength3;
		float r = std::sqrt(a*a + b*b);
		float alpha = std::atan2(b, a);
		float inner = (polarCoords[0] * polarCoords[0] - c - 
			2 * armLength1 * armLength2 * std::cos(tempThetas[1])) / r;
		
		if (inner < -1 || inner > 1) {
			continue;
		}

		float sinInner = std::asin(inner);
		tempThetas[2] = sinInner - alpha;
		// float tempThetas[2]Alt = M_PI - sinInner - alpha;

		float x = armLength1 + 
			armLength2 * std::cos(tempThetas[1]) + 
			armLength3 * std::cos(tempThetas[1] + tempThetas[2]);
		float y = armLength2 * std::sin(tempThetas[1]) +
			armLength3 * std::sin(tempThetas[1] + tempThetas[2]);
		if (x == 0 && y == 0) {
			continue;
		}
		float theta = std::atan2(y, x);
		tempThetas[0] = -theta + polarCoords[1];

		float error = 0;
		for (int i = 0; i < 3; ++i) {
			error += 
				std::pow(currStatus.statuses[i].position_radians - tempThetas[i], 2);
		}

		if (error < minError || minError == -1) {
			minError = error;
			angles = tempThetas;
		}

		// test again with 2nd solution
		/*tempThetas[0] = 2*theta - tempThetas[0];
		tempThetas[1] = -tempThetas[1];
		tempThetas[2] = -tempThetas[2];
		error = 0;
		for (int i = 0; i < 3; ++i) {
			error += 
				std::fabs(currStatus.statuses[i].position_radians - tempThetas[i]);
		}

		if (error < minError || minError == -1) {
			minError = error;
			angles = tempThetas;
		}
		*/
	}

	return minError != -1;
}

void Arm::update(const dynamixel_status_list_t* list) {
	_armMutex.lock();
	_status = *list;

	if (_commands.empty()) {
		_armMutex.unlock();
		return;
	}

	dynamixel_command_list_t& currCommand = _commands.front();

	LcmHandler::instance()->lcm()->publish("ARM_COMMAND", &currCommand);

	float error = 0;
	for (size_t i = 0; i < currCommand.commands.size(); ++i) {
		double diff = 
			std::fabs(eecs467::angle_diff(_status.statuses[i].position_radians,
			currCommand.commands[i].position_radians));
		error += diff;
	}
	if (error < armErrorThresh) {
		_commands.pop_front();
	}

	_armMutex.unlock();
}

void Arm::addCommandLists(const std::vector<dynamixel_command_list_t>& commands) {
	_armMutex.lock();
	_commands.insert(_commands.end(), commands.begin(), commands.end());
	_armMutex.unlock();
}

bool Arm::isMoving() {
	_armMutex.lock();
	bool ret = _commands.empty();
	_armMutex.unlock();
	return !ret;
}
