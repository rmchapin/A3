#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include "a3/LcmHandler.hpp"
#include <iostream>
#include <stdio.h>
#include <chrono>

#include "eecs467_util.h"

int main() {
	// dynamixel_status_list_t statusList;
	// for (int i = 0; i < 3; i++) {
	// 	dynamixel_status_t status;
	// 	status.position_radians = 0;
	// 	statusList.statuses.push_back(status);
	// }
	// statusList.len = 3;

	// float r = 0.15;
	// float theta = M_PI/5;
	

	// while (1) {
	// 	std::cout << "r: ";
	// 	std::cin >> r;
	// 	std::cout << "theta: ";
	// 	std::cin >> theta;
	// 	std::array<float, 2> polarCoords{{r, theta}};
	// 	std::array<float, 3> angles;

	// 	if (Arm::inverseKinematicsPolar(polarCoords, statusList, angles)) {
	// 		printf("Angles: %f, %f, %f\n", angles[0], angles[1], angles[2]);
	// 	} else {
	// 		printf("no solution\n");
	// 	}

	// 	std::array<float, 2> polarSol = Arm::forwardKinematicsPolar(angles);

	// 	printf("target:\t\t%f, %f\n", r, theta);
	// 	printf("solution:\t%f, %f\n", polarSol[0], polarSol[1]);
	// }

	LcmHandler::instance()->launchThreads();

	float r, theta;
	while (1) {
		std::cout << "r: ";
		std::cin >> r;
		std::cout << "theta: ";
		std::cin >> theta;
		dynamixel_status_list_t statusList = Arm::instance()->getStatus();

		std::array<float, 3> angles; 
		if (Arm::inverseKinematicsPolar(
			std::array<float, 2>{{r, theta}}, 
			statusList, angles)) {
			printf("Angles: %f, %f, %f\n", angles[0], angles[1], angles[2]);

			dynamixel_command_list_t cmdList = Arm::createCommand(angles);
			Arm::instance()->addCommandList(cmdList);
		} else {
			printf("no solution\n");
		}
	}
}
