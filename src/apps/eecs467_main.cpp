#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include "a3/LcmHandler.hpp"
#include <iostream>
#include <stdio.h>
#include <chrono>

#include "VxHandler.hpp"
#include "a3/BallFinder.hpp"
#include "a3/GlobalState.hpp"

int main() {
	VxHandler vx(400, 600);
	vx.launchThreads();

	Freenect::Freenect freenect;
	FreenectDevice467& device  = freenect.createDevice<FreenectDevice467>(0);
	device.startDepth();

	image_u32_t* prevDepth = nullptr;

	std::vector<std::array<double, 3>> pts;
	int emptyCount = 0;
	int emptyThresh = 30;
	while (1) {
		image_u32_t* newDepth = device.getDepth();
		if (prevDepth == nullptr) {
			prevDepth = newDepth;
			continue;
		}
		if (prevDepth == nullptr || newDepth == nullptr) {
			continue;
		}
		image_u32_t* diff = BallFinder::imageDiff(prevDepth, newDepth);
		GlobalState::instance()->setIm(diff);

		std::array<double, 3> pt;
		if (!BallFinder::find(prevDepth, newDepth, pt)) {
			emptyCount++;
			if (emptyCount > emptyThresh) {
				emptyCount = 0;
				pts.clear();
			}
			continue;
		}

		pts.push_back(pt);

		image_u32_destroy(prevDepth);
		prevDepth = newDepth;

		auto curve = LineFitter::fitCurve(pts);
		std::array<float, 2> intersection;
		if (!LineFitter::getIntersectionZ(-1, intersection, curve)) {
			continue;
		}

		// try to move arm to place
		std::array<float, 3> angles;
		if (!Arm::inverseKinematicsCartesian(intersection,
			Arm::instance()->getStatus(),
			angles)) {
			continue;
		}

		dynamixel_command_list_t cmd = Arm::createCommand(angles);
		Arm::instance()->addCommandList(cmd);
		
		usleep(1e3);
	}

	return 0;
}
