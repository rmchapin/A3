#include "LineFitter.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

namespace LineFitter {

std::pair<std::array<double, 3>, std::array<double, 2>> 
fitCurve(const std::vector<std::array<double, 3>>& pts) {
	assert(pts.size() >= 3);

	// finding plane with which to constrain parabola
	Eigen::Matrix<double, Eigen::Dynamic, 2> xyMatrix(pts.size(), 2);
	Eigen::Matrix<double, Eigen::Dynamic, 1> xVec(pts.size(), 1);

	for (size_t i = 0; i < pts.size(); ++i) {
		xyMatrix(i, 0) = 1;
		xyMatrix(i, 1) = pts[i][1]; // y

		xVec(i) = pts[i][0];
	}

	// least squares solution for line
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> linear = 
		xyMatrix.colPivHouseholderQr().solve(xVec);


	// finding parabola constrained in plane
	Eigen::Matrix<double, Eigen::Dynamic, 3> rzMatrix(pts.size(), 3);
	Eigen::Matrix<double, Eigen::Dynamic, 1> zVec(pts.size(), 1);

	// getting unit vector in direction of line
	double mag = 1.0 / std::sqrt(linear(1) * linear(1) + 1);
	std::array<double, 2> unitDir{{linear(1) * mag, 1 * mag}};

	for (size_t i = 0; i < pts.size(); ++i) {
		// getting radius of point when projected into plane
		double r = (pts[i][0] - linear(0)) * unitDir[0] +
				pts[i][1] * unitDir[1];

		rzMatrix(i, 0) = 1;
		rzMatrix(i, 1) = r;
		rzMatrix(i, 2) = r * r;

		zVec(i) = pts[i][2];
	}

	// least squares solution for parabola
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> quadratic = 
		rzMatrix.colPivHouseholderQr().solve(zVec);

	return std::pair<std::array<double, 3>, std::array<double, 2>>(
		{{quadratic(0), quadratic(1), quadratic(2)}}, 
		{{linear(0), linear(1)}});
}

bool getIntersectionZ(double zVal,
	std::array<double, 2>& intersection,
	const std::pair<std::array<double, 3>, std::array<double, 2>> & curve) {

	//z = a + b * r + c * r^2
	double descriminant = curve.first[1] * curve.first[1] - 
		4 * curve.first[2] * (curve.first[0] - zVal);

	if (descriminant < 0) {
		return false;
	}

	// quadratic formula, we take only the (-b - sqrt(descr)) / (2a) solution
	double radius = (-curve.first[1] - std::sqrt(descriminant)) /  
		(2 * (curve.first[0] - zVal));

	double theta = std::atan2(curve.second[1], 1);

	intersection[0] = radius * std::cos(theta) + curve.second[0];
	intersection[1] = radius * std::sin(theta);
	return true;
}

}
