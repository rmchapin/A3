#ifndef BALL_FINDER_HPP
#define BALL_FINDER_HPP

#include <array>
#include "imagesource/image_u32.h"


namespace BallFinder {

bool find(image_u32_t* prev, image_u32_t* curr, std::array<double, 3>& loc);

image_u32_t* imageDiff(image_u32_t* prev, image_u32_t* curr);


}

#endif /* BALL_FINDER_HPP */
