#ifndef BALL_FINDER_HPP
#define BALL_FINDER_HPP

#include <array>
#include "imagesource/image_u32.h"


namespace BallFinder {

std::array<float, 3> find(image_u32_t* prev, image_u32_t* curr);

image_u32_t* imageDiff(image_u32_t* prev, image_u32_t* curr);


}

#endif /* BALL_FINDER_HPP */
