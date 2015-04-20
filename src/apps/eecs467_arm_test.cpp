#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include "a3/LcmHandler.hpp"
#include <iostream>
#include <stdio.h>
#include <chrono>

#include "eecs467_util.h"

int main() {
	LcmHandler::instance()->launchThreads();

	float r, theta;
	float x, y;
	while (1) {
		// std::cout << "r: ";
		// std::cin >> r;
		// std::cout << "theta: ";
		// std::cin >> theta;
		std::cout << "x: ";
		std::cin >> y;
		y = -y;
		std::cout << "y: ";
		std::cin >> x;
		dynamixel_status_list_t statusList = Arm::instance()->getStatus();

		std::array<float, 3> angles; 
		if (Arm::inverseKinematicsCartesian(
			std::array<float, 2>{{x, y}}, 
			statusList, angles)) {
			printf("Angles: %f, %f, %f\n", angles[0], angles[1], angles[2]);

			dynamixel_command_list_t cmdList = Arm::createCommand(angles);
			Arm::instance()->addCommandList(cmdList);
		} else {
			printf("no solution\n");
		}
	}
}
