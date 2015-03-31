#ifndef LINE_FITTER_HPP
#define LINE_FITTER_HPP

#include <vector>
#include <array>
#include <utility>

/**
 * @brief fits a curve to a set of 3d points
 * @details This will fit a line on the xy plane (ground plane)
 * and a parabola on the yz plane
 * The curve is limited in only bending in the yz plane
 * 
 * @param pts vector of points in 3d space in (x, y, z) format.
 * size should be at least 3
 * @return returns two arrays
 * first array is the coefficients a + b*z + c*z*z in (a, b, c) form
 * second array is the coefficients a + b*x in (a, b) form
 */
std::pair<std::array<double, 3>, std::array<double, 2>> fitCurve(const std::vector<std::array<double, 3>>& pts);


#endif /* LINE_FITTER_HPP */
