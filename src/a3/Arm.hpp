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

	/**
	 * @brief gets the location of the center of the hoop
	 * @details will return junk data if update() has never been called
	 * @return (x, y) coordinates in meters of center of hoop
	 */
	static std::array<float, 2> forwardKinematics();

	static std::array<float, 3> inverseKinematics(const std::array<float, 2>& coords);

	void update(const dynamixel_status_list_t* list);

	void addCommandLists(const std::vector<dynamixel_command_list_t>& commands);

	bool isMoving();

};

#endif /* ARM_HPP */
