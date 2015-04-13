#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include <iostream>
#include <stdio.h>

#include "eecs467_util.h"

int main() {
	// std::vector<std::array<double, 3>> pts;
	// pts.push_back(std::array<double, 3>{{1, 2, 1}});
	// pts.push_back(std::array<double, 3>{{2, 3, 2}});
	// pts.push_back(std::array<double, 3>{{3, 4, 1}});
	// pts.push_back(std::array<double, 3>{{4, 6, -1}});

	// auto i = LineFitter::fitCurve(pts);	
	// std::cout << i.first[0]  << '\t' << i.first[1] << '\t' << i.first[2] << '\n';
	// std::cout << i.second[0] << '\t' << i.second[1] << '\n';

	// std::array<double, 2> intersection;
	// LineFitter::getIntersectionZ(1, intersection, i);
	// std::cout << "intersection: " << intersection[0] << '\t' 
	// 	<< intersection[1] << '\n';


	/*
	dynamixel_command_list_t list;
	for (int i = 0; i < 3; ++i) {
		dynamixel_command_t cmd;
		cmd.position_radians = 0;
		cmd.max_torque = 0.5;
		cmd.speed = 0.1;
		list.commands.push_back(cmd);
	}
	list.len = list.commands.size();
	Arm::instance()->addCommandLists(std::vector<dynamixel_command_list_t>{list});
	*/
	// std::array<float, 2> ans = Arm::forwardKinematicsPolar(M_PI/2, M_PI/3, M_PI/4);
	// std::cout << ans[0] << '\t' << ans[1] << '\n';
	dynamixel_status_list_t statusList;
	for (int i = 0; i < 3; i++) {
		dynamixel_status_t status;
		status.position_radians = 0;
		statusList.statuses.push_back(status);
	}
	statusList.len = 3;

	float r = 0.12;
	float theta = -M_PI/5;
	std::array<float, 2> polarCoords{{r, theta}};
	std::array<float, 3> angles;
	if (Arm::inverseKinematicsPolar(polarCoords, statusList, angles)) {
		printf("Angles: %f, %f, %f\n", angles[0], angles[1], angles[2]);
	} else {
		printf("no solution\n");
	}

	std::array<float, 2> polarSol = Arm::forwardKinematicsPolar(angles);

	printf("target:\t\t%f, %f\n", r, theta);
	printf("solution:\t%f, %f\n", polarSol[0], polarSol[1]);
}
