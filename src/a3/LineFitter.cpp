#include "LineFitter.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

std::pair<std::array<double, 3>, std::array<double, 2>> 
fitCurve(const std::vector<std::array<double, 3>>& pts) {
	assert(pts.size() >= 3);

	Eigen::Matrix<double, Eigen::Dynamic, 2> xyMatrix(pts.size(), 2);
	Eigen::Matrix<double, Eigen::Dynamic, 1> xVec(pts.size(), 1);

	for (size_t i = 0; i < pts.size(); ++i) {
		xyMatrix(i, 0) = 1;
		xyMatrix(i, 1) = pts[i][1]; // y

		xVec(i) = pts[i][0];
	}

	// finding plane with which to constrain parabola
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> linear = 
		xyMatrix.colPivHouseholderQr().solve(xVec);

	Eigen::Matrix<double, Eigen::Dynamic, 3> yzMatrix(pts.size(), 3);
	Eigen::Matrix<double, Eigen::Dynamic, 1> zVec(pts.size(), 1);

	double mag = 1.0 / sqrt(linear(1) * linear(1) + 1);
	std::array<double, 2> unitDir{{linear(1) * mag, 1 * mag}};

	for (size_t i = 0; i < pts.size(); ++i) {
		// projecting coordinates onto plane
		double r = (pts[i][0] - linear(0)) * unitDir[0] +
				pts[i][1] * unitDir[1];

		yzMatrix(i, 0) = 1;
		yzMatrix(i, 1) = r;
		yzMatrix(i, 2) = r * r;

		zVec(i) = pts[i][2];
	}

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> quadratic = 
		yzMatrix.colPivHouseholderQr().solve(zVec);

	return std::pair<std::array<double, 3>, std::array<double, 2>>(
		{{quadratic(0), quadratic(1), quadratic(2)}}, 
		{{linear(0), linear(1)}});
}

