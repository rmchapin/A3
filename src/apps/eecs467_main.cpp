#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include "a3/LcmHandler.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <stdio.h>

#include "VxHandler.hpp"
#include "a3/BallFinder.hpp"
#include "a3/GlobalState.hpp"
#include "a3/BlobDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>


int main() {
	VxHandler vx(400, 600);
	vx.launchThreads();

	
	std::ifstream calibFile("calib.txt");
	if (!calibFile.is_open()) {
		printf("not open!\n");
		exit(1);
	}
	Eigen::Matrix<double, 4, 4> transform;
	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			calibFile >> transform(row, col);
			transform(row, col) *= 0.0254; // inches to meters
		}
	}

	std::ifstream colorFile("yellow.txt");
	std::array<float, 6> hsvThresh;
	for (int i = 0; i < 6; i++) {
		colorFile >> hsvThresh[i];
	}

	Freenect::init();
	Freenect::startDepthCallback();
	Freenect::startVideoCallback();
	Freenect::launchThread();

	std::vector<std::array<double, 3>> pts;
	
	int emptyCount = 0;
	int emptyThresh = 100;
	image_u32_t* rgbIm = nullptr;
	image_u32_t* depthIm = nullptr;
	while (1) {
		rgbIm = Freenect::getImage();
		depthIm = Freenect::getImage();
		if (rgbIm == nullptr || depthIm == nullptr) {
			continue;
		}

		// find ball in rgb
		std::array<float, 6> hsvThresh{{0, 0, 0, 0, 0, 0}};
		std::vector<BlobDetector::Blob> blobs = 
			BlobDetector::findBlobs(rgbIm, hsvThresh, 100);

		if (blobs.size() == 0) {
			image_u32_destroy(rgbIm);
			image_u32_destroy(depthIm);
			emptyCount++;
			if (emptyCount > emptyThresh) {
				emptyCount = 0;
				pts.clear();
			}
			continue;
		} else {
			emptyCount = 0;
		}

		std::sort(blobs.begin(), blobs.end(),
			[](const BlobDetector::Blob& A,
				const BlobDetector::Blob& B) {
				return A.size > B.size;
			});

		BlobDetector::Blob biggest = blobs.front();
		uint16_t depth = depthIm->buf[depthIm->stride * biggest.y + biggest.x];


		image_u32_destroy(rgbIm);
		image_u32_destroy(depthIm);

		// getting real coordinates
		std::array<double, 2> realCoord = Freenect::cameraToWorld(biggest.x, biggest.y, depth);

		// do some manipulation
		Eigen::Matrix<double, 4, 1> pt;
		pt(0) = realCoord[0];
		pt(1) = (double) depth;
		pt(2) = realCoord[1];
		pt(3) = 1;

		Eigen::Matrix<double, 4, 1> newPt = transform * pt;
		printf("ball at: (%lf, %lf, %lf)\n", newPt(0),
			newPt(1), newPt(2));

		pts.push_back(std::array<double, 3>{{
			newPt(0), newPt(1), newPt(2)
			}});
		// pts.push_back(std::array<double, 3>{{realCoord[0], 
			// depth, realCoord[1]}});
		
		if (pts.size() < 3) {
			continue;
		}

		// fitting curve and getting intersection
		auto curve = LineFitter::fitCurve(pts);
		std::array<float, 2> intersection;
		if (!LineFitter::getIntersectionZ(0, intersection, curve)) {
			printf("unable to get intersection\n");
			continue;
		}
		printf("Move arm to: %f, %f\n", intersection[0], intersection[1]);
		

		// try to move arm to place
		std::array<float, 3> angles;
		if (!Arm::inverseKinematicsCartesian(intersection,
			Arm::instance()->getStatus(),
			angles)) {
			printf("unable to move arm to location\n");
			continue;
		}

		dynamixel_command_list_t cmd = Arm::createCommand(angles);
		Arm::instance()->addCommandList(cmd);
		

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

		usleep(1e3);
	}

	return 0;
}
