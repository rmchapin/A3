#ifndef LINE_FITTER_HPP
#define LINE_FITTER_HPP

#include <vector>
#include <array>
#include <utility>

namespace LineFitter {

/**
 * @brief fits a curve to a set of 3d points
 * @details This will fit a line on the xy plane (ground plane)
 * and a parabola on the plane that runs along the line with a normal that has no z value
 * The curve is limited in only bending in plane perpendicular 
 * to z axis and contains the line fitted in the xy plane
 * 
 * @param pts vector of points in 3d space in (x, y, z) format.
 * size should be at least 3
 * @return returns two arrays
 * first array is the coefficients (a, b, c) from a + b*r + c*r*r = z
 * where r is the distance from the x intercept point
 * this equation determins the parabola in the plane
 * second array is the coefficients (a, b) from a + b*y = x
 * this will define the line with the plane rests on
 */
std::pair<std::array<double, 3>, std::array<double, 2>> 
fitCurve(const std::vector<std::array<double, 3>>& pts);

std::pair<std::array<double, 3>, std::array<double, 2>> 
RANSACCurve(const std::vector<std::array<double, 3>>& pts);

/**
 * @brief calculates the intersection between a curve and the plane
 * z = zVal
 * obtains xy point at which that happens
 * @details this will only calculate the point
 * with the smallest radius
 * 
 * @param zVal the value that z will intersect
 * @param intersection (x, y) coordinates to be populated by function
 * @param curve the curve used to calculate the intersections
 * 
 * @return true if intersection exists
 */
bool getIntersectionZ(double zVal,
	std::array<float, 2>& intersection,
	const std::pair<std::array<double, 3>, std::array<double, 2>> & curve);

}

#endif /* LINE_FITTER_HPP */
