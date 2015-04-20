#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include "a3/Arm.hpp"
#include "a3/LcmHandler.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <stdio.h>
#include <string>

#include "VxHandler.hpp"
#include "a3/BallFinder.hpp"
#include "a3/GlobalState.hpp"
#include "a3/BlobDetector.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

typedef struct {
	unsigned char a;
	unsigned char b;
	unsigned char g;
	unsigned char r;
} ABGR_p;

typedef struct {
	double h;
	double s;
	double v;
} HSV_p;

HSV_p u32_pix_to_HSV(ABGR_p u32_in)
{
	HSV_p out;
	double min, max, delta;

	//printf("R: %d, G: %d, B: %d\n", u32_in.r, u32_in.g, u32_in.b);

	min = u32_in.r < u32_in.g ? u32_in.r : u32_in.g;
	min = min  < u32_in.b ? min  : u32_in.b;
	
	max = u32_in.r > u32_in.g ? u32_in.r : u32_in.g;
	max = max  > u32_in.b ? max  : u32_in.b;
	
	out.v = max / 255.0;                        // v
	delta = max - min;
	if( max > 0.0 )
	{
		out.s = (delta / max);                  // s
	}
	else
	{
		// r = g = b = 0                        // s = 0, v is undefined
		out.s = 0.0;
		out.h = NAN;                            // its now undefined
		return out;
	}
	
	if( u32_in.r >= max )                        // > is bogus, just keeps compilor happy
	{
		out.h = ( u32_in.g - u32_in.b ) / delta;        // between yellow & magenta
	}
	else
	{
		if( u32_in.g >= max )
			out.h = 2.0 + ( u32_in.b - u32_in.r ) / delta;  // between cyan & yellow
		else
			out.h = 4.0 + ( u32_in.r - u32_in.g ) / delta;  // between magenta & cyan
	}
	
	out.h *= 60.0;                              // degrees
	
	if( out.h < 0.0 )
		out.h += 360.0;

	return out;
}


int main() {
	VxHandler vx(400, 600);
	vx.launchThreads();

	LcmHandler::instance()->launchThreads();
	
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

	std::ifstream colorFile("pink.txt");
	if (!colorFile.is_open()) {
		printf("couldn't open color file\n");
		exit(1);
	}
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
	int emptyThresh = 20;
	image_u32_t* rgbIm = nullptr;
	image_u32_t* depthIm = nullptr;
	while (1) {
		image_u32_t* temp = Freenect::getImage();

		image_u32_t* vxDisp;
		if (temp != nullptr) {
			rgbIm = temp;
			vxDisp = image_u32_copy(rgbIm);

			for (int t = 0; t < vxDisp->width; t++)
			{
				for (int y = 0; y < vxDisp->height; y++)
				{
					//make rgba pixel
					ABGR_p pixel_abgr;
					uint32_t val = vxDisp->buf[vxDisp->stride * y + t];
		 
					pixel_abgr.a = 0xFF & (val >> 24);
					pixel_abgr.b = 0xFF & (val >> 16);
					pixel_abgr.g = 0xFF & (val >> 8);
					pixel_abgr.r = 0xFF & val;

					HSV_p pixel_hsv;
					pixel_hsv = u32_pix_to_HSV(pixel_abgr);

					if ((pixel_hsv.h >= hsvThresh[0]) && 
						(pixel_hsv.h <= hsvThresh[1]) &&
						(pixel_hsv.s >= hsvThresh[2]) &&
						(pixel_hsv.s <= hsvThresh[3]) &&
						(pixel_hsv.v >= hsvThresh[4]) &&
						(pixel_hsv.v <= hsvThresh[5]))
					{
						vxDisp->buf[vxDisp->stride * y + t] = 0xFFE600CB;
					}
				}
			}
			vx.setImage(vxDisp);
		}

		temp = Freenect::getDepth();
		if (temp != nullptr) {
			if (depthIm != nullptr) {
				image_u32_destroy(depthIm);
			}
			depthIm = temp;
		}

		if (rgbIm == nullptr || depthIm == nullptr)
		{
			continue;
		}

		// find ball in rgb
		std::vector<BlobDetector::Blob> blobs = 
			BlobDetector::findBlobs(rgbIm, hsvThresh, 30);

		if (blobs.size() == 0) {
			if (emptyCount > emptyThresh) {
				emptyCount = 0;
				pts.clear();
				dynamixel_command_list_t cmd = Arm::createCommand(std::array<float, 3>{{0, 0, 0}});
				Arm::instance()->addCommandList(cmd);
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

		// getting real coordinates
		std::array<double, 2> realCoord = Freenect::cameraToWorld(biggest.x, biggest.y, depth);

		// do some manipulation
		Eigen::Matrix<double, 4, 1> pt;
		pt(0) = realCoord[0];
		pt(1) = realCoord[1];
		pt(2) = (double) depth;
		pt(3) = 1;

		Eigen::Matrix<double, 4, 1> newPt = transform * pt;
		printf("ball at: (%lf, %lf, %lf)\n", newPt(0),
			newPt(1), newPt(2));

		// pts.push_back(std::array<double, 3>{{
		// 	newPt(0), newPt(1), newPt(2)
		// 	}});
		
		// if (pts.size() < 3) {
		// 	continue;
		// }

		// // fitting curve and getting intersection
		// auto curve = LineFitter::RANSACCurve(pts);
		// // if (curve.first[1] > 0) {
		// // 	printf("curve going up\n");
		// // }
		// printf("linear: %f, %f\n", curve.second[0], curve.second[1]);
		// printf("quad: %f, %f, %f\n", curve.first[0], 
		// 	curve.first[1], curve.first[2]);
		// std::array<float, 2> intersection;
		// if (!LineFitter::getIntersectionZ(0, intersection, curve)) {
		// 	printf("unable to get intersection\n");
		// 	continue;
		// }
		// printf("Move arm to: %f, %f\n", intersection[0], intersection[1]);
		// if (std::isnan(intersection[0]) ||
		// 	std::isnan(intersection[1])) {
		// 	continue;
		// }
		
		// float tempX = -intersection[0];
		// intersection[0] = intersection[1];
		// intersection[1] = tempX;

		// try to move arm to place
		// std::array<float, 3> angles;
		// if (!Arm::inverseKinematicsCartesian(intersection,
		// 	Arm::instance()->getStatus(),
		// 	angles)) {
		// 	printf("unable to move arm to location\n");
		// 	continue;
		// }

		// dynamixel_command_list_t cmd = Arm::createCommand(angles);
		// Arm::instance()->addCommandList(cmd);

		// if (depthIm != nullptr) {
		// 	image_u32_destroy(depthIm);
		// }
		// depthIm = nullptr;
		// if (rgbIm != nullptr) {
		// 	image_u32_destroy(rgbIm);
		// }
		// rgbIm = nullptr;

////////////////////

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
