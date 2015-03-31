#include "LineFitter.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>

std::pair<std::array<double, 3>, std::array<double, 2>> fitCurve(const std::vector<std::array<double, 3>>& pts) {
	assert(pts.size() >= 3);

	Eigen::Matrix<double, Eigen::Dynamic, 3> yzMatrix(pts.size(), 3);
	Eigen::Matrix<double, Eigen::Dynamic, 2> xyMatrix(pts.size(), 2);
	Eigen::Matrix<double, Eigen::Dynamic, 1> xVec(pts.size(), 1);
	Eigen::Matrix<double, Eigen::Dynamic, 1> zVec(pts.size(), 1);

	for (size_t i = 0; i < pts.size(); ++i) {
		yzMatrix(i, 0) = 1;
		yzMatrix(i, 1) = pts[i][1]; // y
		yzMatrix(i, 2) = pts[i][1] * pts[i][1]; // y^2

		xyMatrix(i, 0) = 1;
		xyMatrix(i, 1) = pts[i][1]; // y

		xVec(i) = pts[i][0];
		zVec(i) = pts[i][2];
	}

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> quadratic = yzMatrix.colPivHouseholderQr().solve(zVec);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> linear = xyMatrix.colPivHouseholderQr().solve(xVec);

	std::array<double, 3> quadRet{{quadratic(0), quadratic(1), quadratic(2)}};
	std::array<double, 2> linearRet{{linear(0), linear(1)}};

	return std::pair<std::array<double, 3>, std::array<double, 2>>(quadRet, linearRet);
}
