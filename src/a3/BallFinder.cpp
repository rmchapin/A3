#include "BallFinder.hpp"
#include "BlobDetector.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdio.h>

namespace BallFinder {

bool find(image_u32_t* prev, image_u32_t* curr, std::array<double, 3>& loc) {
	image_u32_t* diff = imageDiff(prev, curr);

	std::vector<BlobDetector::Blob> blobs = BlobDetector::findBlobs(diff,
		[](uint32_t val) {
			int32_t signedVal = (int32_t) val;
			return signedVal > 0;
		}, 100);

	image_u32_destroy(diff);

	if (blobs.size() == 0) {
		return false;
	}

	std::sort(blobs.begin(), blobs.end(), 
		[](const BlobDetector::Blob& A, 
			const BlobDetector::Blob& B) {
			return A.size > B.size;
		});

	BlobDetector::Blob biggest = blobs.front();
	// get depth from curr image of blob
	// get x, y, stuff
	loc[0] = biggest.x;
	loc[1] = biggest.y;
	loc[2] = 0;
	printf("biggest blob: %d\n", biggest.size);

	return true;
}

image_u32_t* imageDiff(image_u32_t* prev, image_u32_t* curr) {
	image_u32_t* diff = image_u32_create(prev->width, prev->height);
	int64_t threshold = 30;
	for (int row = 0; row < prev->height; row++) {
		for (int col = 0; col < prev->width; col++) {
			uint32_t currDepth = curr->buf[curr->stride * row + col];
			uint32_t prevDepth = prev->buf[prev->stride * row + col];

			int64_t sub = std::abs((int64_t)currDepth - (int64_t)prevDepth);
			if (sub > threshold && currDepth != 0 
				&& currDepth < 3000) {
				// std::cout << currDepth << '\n';
				// diff->buf[diff->stride * row + col] = 0xFF00FF00;
				diff->buf[diff->stride * row + col] = sub;
			} else {
				diff->buf[diff->stride * row + col] = 0;
			}
		}
	}

	return diff;
}

image_u32_t* hDeriv(image_u32_t* im) {
	image_u32_t* deriv = image_u32_create(im->width, im->height);
	for (int row = 0; row < im->height; row++) {
		for (int col = 0; col < im->width; col++) {
			int32_t left, right;
			if (col == 0) {
				left = 0;
			} else {
				left = im->buf[im->stride * row + (col - 1)];
			}
			if (col == im->width - 1) {
				right = 0;
			} else {
				right = im->buf[im->stride * row + (col + 1)];
			}
			deriv->buf[deriv->stride * row + col] = right - left;
		}
	}
	return deriv;
}

image_u32_t* vDeriv(image_u32_t* im) {
	image_u32_t* deriv = image_u32_create(im->width, im->height);

	// first row
	for (int col = 0; col < im->width; col++) {
		deriv->buf[deriv->stride + col] = im->buf[im->stride * 2 + col];
	}

	for (int row = 1; row < im->height - 1; row++) {
		for (int col = 0; col < im->width; col++) {
			int32_t left, right;
			left = im->buf[im->stride * row + (col - 1)];
			right = im->buf[im->stride * row + (col + 1)];
			deriv->buf[deriv->stride * row + col] = right - left;
		}
	}

	// last column
	for (int col = 0; col < im->width; col++) {
		deriv->buf[deriv->stride * (im->height - 1) + col]
			= -im->buf[im->stride * (im->height - 2) + col];
	}
	return deriv;	
}

}
