#ifndef ARM_CONSTANTS_HPP
#define ARM_CONSTANTS_HPP

#include <array>

static const float armErrorThresh = 0.1; // radians

static const float basketRadius = 0.1; // meters

static const float armLength1 = 0.1; // meters
static const float armLength2 = 0.1; // meters
static const float armLength3 = 0.13; // meters

static const std::array<float, 3> armWeights{{1, 1, 1}};

static const std::array<float, 2> armAngleThresh1{{-2.1, 2.0}};
static const std::array<float, 2> armAngleThresh2{{-2.0, 2.0}};
static const std::array<float, 2> armAngleThresh3{{-2, 2}};

static const float armTorque = 0.5;
static const float armSpeed = 1;

#endif /* ARM_CONSTANTS_HPP */
