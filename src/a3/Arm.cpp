#include "Arm.hpp"
#include "ArmConstants.hpp"
#include "LcmHandler.hpp"
#include "math/angle_functions.hpp"
#include <cmath>

Arm* Arm::_instance = new Arm();

Arm::Arm() {}

Arm* Arm::instance() {
	return _instance;
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
