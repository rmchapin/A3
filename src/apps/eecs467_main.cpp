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
	// do {
	// 	prevDepth = device.getDepth();
	// } while (prevDepth == nullptr);

	// GlobalState::instance()->setIm(prevDepth);
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

		image_u32_destroy(prevDepth);
		prevDepth = newDepth;

		usleep(1e3);
	}

	return 0;
}
