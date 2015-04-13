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
#include "ArmConstants.hpp"

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
	 * @brief gets the location of the center of the hoop in cartesian coordinates
	 * @param theta(i) angle of motors (with 1 being the base motor)
	 * @param d(i) length of the arm segments (with 1 being the one closest to base)
	 * @return (x, y) coordinates in meters of center of hoop
	 */
	static std::array<float, 2> forwardKinematicsCartesian(float theta1, 
		float theta2, float theta3, 
		float d1 = armLength1, 
		float d2 = armLength2, 
		float d3 = armLength3);

	/**
	 * @brief gets the location of the center of the hoop in polar coordinates
	 * @param theta(i) angle of motors (with 1 being the base motor)
	 * @param d(i) length of the arm segments (with 1 being the one closest to base)
	 * @return (x, y) coordinates in meters of center of hoop
	 */
	static std::array<float, 2> forwardKinematicsPolar(float theta1, 
		float theta2, float theta3,
		float d1 = armLength1, 
		float d2 = armLength2, 
		float d3 = armLength3);

	static std::array<float, 2> forwardKinematicsPolar(const std::array<float, 3>& angles, 
		float d1 = armLength1, 
		float d2 = armLength2, 
		float d3 = armLength3);

	/**
	 * @brief gets the angles of motors required to go to an (x, y) coord
	 * @details will find the closest set, if possible, to the angles in currStatus
	 * 
	 * @param coord (x, y) location to go to
	 * @param currStatus current status of arm
	 * @param angles will be populated by function angle[0] is first motor
	 * @return true if a solution is found
	 */
	static bool inverseKinematicsCartesian(const std::array<float, 2>& coord, 
		const dynamixel_status_list_t& currStatus,
		std::array<float, 3>& angles);

	/**
	 * @brief same thing as above except input is in polar coordinates
	 */
	static bool inverseKinematicsPolar(const std::array<float, 2>& polarCoords, 
		const dynamixel_status_list_t& currStatus,
		std::array<float, 3>& angles);

	void update(const dynamixel_status_list_t* list);

	void addCommandLists(const std::vector<dynamixel_command_list_t>& commands);

	bool isMoving();
};

#endif /* ARM_HPP */
