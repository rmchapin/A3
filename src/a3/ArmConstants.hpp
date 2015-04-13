#ifndef ARM_CONSTANTS_HPP
#define ARM_CONSTANTS_HPP

#include <array>

static const float armErrorThresh = 0.1; // radians

static const float basketRadius = 0.1; // meters

static const float armLength1 = 0.1; // meters
static const float armLength2 = 0.1; // meters
static const float armLength3 = 0.07; // meters

static const std::array<float, 3> armWeights{{1, 0.5, 0.25}};

static const std::array<float, 2> armAngleThresh1{{-2.2, 2.2}};
static const std::array<float, 2> armAngleThresh2{{-2.2, 2.2}};
static const std::array<float, 2> armAngleThresh3{{-2, 2}};

static const float armTorque = 0.7;
static const float armSpeed = 0.15;

#endif /* ARM_CONSTANTS_HPP */
