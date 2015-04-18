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

	Freenect::init();
	Freenect::startDepthCallback();
	Freenect::startVideoCallback();
	Freenect::launchThread();

	image_u32_t* prevDepth = nullptr;

	std::vector<std::array<double, 3>> pts;
	int emptyCount = 0;
	int emptyThresh = 30;
	int count = 0;
	
	
	while (1) {
		// image_u32_t* im = Freenect::getImage();
		// if (im == nullptr) {
		// 	continue;
		// }
		// GlobalState::instance()->setIm(im);

		// image_u32_t* newDepth = Freenect::getDepth();
		// if (prevDepth == nullptr) {
		// 	prevDepth = newDepth;
		// 	continue;
		// }
		// if (prevDepth == nullptr || newDepth == nullptr) {
		// 	continue;
		// }
		// image_u32_t* diff = BallFinder::imageDiff(prevDepth, newDepth);
		// for (int row = 0; row < diff->height; row++) {
		// 	for (int col = 0; col < diff->width; col++) {
		// 		diff->buf[diff->stride * row + col] |= 0xFF000000;
		// 	}
		// }

		// std::array<double, 3> loc;
		// if (BallFinder::find(prevDepth, newDepth, loc)) {
		// 	for (int i = -3; i < 3; i++) {
		// 		for (int j = -3; j < 3; j++) {
		// 			diff->buf[diff->stride * ((int)loc[1] + i) + (int)loc[0] + j] =
		// 				0xFF00FF00;
		// 		}
		// 	}
		// }

		// GlobalState::instance()->setIm(diff);

		// image_u32_destroy(prevDepth);
		// prevDepth = newDepth;
/////////////////

		// char buf[100];
		// sprintf(buf, "image%d.pnm", count++);
		// int res = image_u32_write_pnm(diff, buf);
		// if (res) {
		// 	std::cout << "did not save image successfully\n";
		// }



		// std::array<double, 3> pt;
		// if (!BallFinder::find(prevDepth, newDepth, pt)) {
		// 	emptyCount++;
		// 	if (emptyCount > emptyThresh) {
		// 		emptyCount = 0;
		// 		pts.clear();
		// 	}
		// 	continue;
		// }

		// pts.push_back(pt);


		// auto curve = LineFitter::fitCurve(pts);
		// std::array<float, 2> intersection;
		// if (!LineFitter::getIntersectionZ(-1, intersection, curve)) {
		// 	continue;
		// }

		// // try to move arm to place
		// std::array<float, 3> angles;
		// if (!Arm::inverseKinematicsCartesian(intersection,
		// 	Arm::instance()->getStatus(),
		// 	angles)) {
		// 	continue;
		// }

		// dynamixel_command_list_t cmd = Arm::createCommand(angles);
		// Arm::instance()->addCommandList(cmd);
		
		usleep(1e3);
	}

	return 0;
}
