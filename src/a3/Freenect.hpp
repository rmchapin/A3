#ifndef FREENECT_HPP
#define FREENECT_HPP


#include "libfreenect.h"
#include "libfreenect-registration.h"
// #include "libfreenect/libfreenect.h"
// #include "libfreenect/libfreenect_registration.h"
#include "Mutex.hpp"

#include "imagesource/image_source.h"
#include "imagesource/image_u32.h"

#include <vector>
#include <stdint.h>
#include <iostream>
#include <utility>
#include <array>

namespace Freenect {

void init();

// gets copy of image (delete it after use)
image_u32_t* getImage();

// get copy of depth (delete if after use)
image_u32_t* getDepth();

void startDepthCallback();

void startVideoCallback();

void launchThread();

std::array<double, 2> cameraToWorld(int cx, int cy, int depth);

}

#endif /* FREENECT_HPP */
