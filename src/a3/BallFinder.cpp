#include "BallFinder.hpp"
#include <cmath>
#include <iostream>
#include <stdio.h>

namespace BallFinder {

image_u32_t* imageDiff(image_u32_t* prev, image_u32_t* curr) {
	image_u32_t* diff = image_u32_create(prev->width, prev->height);
	int64_t threshold = 10;
	for (int row = 0; row < prev->height; row++) {
		for (int col = 0; col < prev->width; col++) {
			uint32_t currDepth = curr->buf[curr->stride * row + col];
			uint32_t prevDepth = prev->buf[prev->stride * row + col];

			int64_t sub = std::abs((int64_t)currDepth - (int64_t)prevDepth);
			if (sub > threshold && currDepth != 0) {
				// std::cout << currDepth << '\n';
				diff->buf[diff->stride * row + col] = 0xFF00FF00;
			}
		}
	}

	return diff;
}

}
