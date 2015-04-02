#ifndef ARM_HPP
#define ARM_HPP

#include <stdint.h>
#include <pthread.h>
#include <array>
#include <vector>
#include <list>
#include <math.h>
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "Mutex.hpp"

class Arm {
private:
	dynamixel_status_list_t _status;
	mutable Mutex _armMutex;

	std::list<dynamixel_command_list_t> _commands;

	Arm();
	static Arm* _instance;

public:
	static Arm* instance();

	void update(const dynamixel_status_list_t* list);

	void addCommandLists(const std::vector<dynamixel_command_list_t>& commands);

	bool isMoving();

};

#endif /* ARM_HPP */
