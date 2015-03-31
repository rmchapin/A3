#ifndef BLOB_DETECTOR_HPP
#define BLOB_DETECTOR_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <array>
#include <iostream>
#include <stdint.h>
#include "imagesource/image_u32.h"

namespace BlobDetector {

struct Blob {
	int x, y;
	int size; // number of pixels
};

/**
 * @brief finds all blobs with a certain hsv threshold
 * @details [long description]
 * 
 * @param im im to find blobs from
 * @param hsvThresh threshold values for hsv
 * hue min - hsvThresh[0] (min is 0)
 * hue max - hsvThresh[1] (max is 1)
 * sat min - hsvThresh[2] (min is 0)
 * sat max - hsvThresh[3] (max is 1)
 * value - hsvThresh[4] (min is 0)
 * value - hsvThresh[5] (max is 360)
 * @param minPixels minimum number of pixels for a blob to be considered
 * @return vector of blobs
 */
std::vector<Blob> findBlobs(image_u32_t* im, 
	const std::array<float, 6>& hsvThresh, size_t minPixels);

}

#endif /* BLOB_DETECTOR_HPP */
