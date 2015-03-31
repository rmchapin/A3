#include "LineFitter.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cassert>

std::array<float, 3> fitCurve(const std::vector<std::array<float, 3>>& pts) {
	assert(pts.size() >= 3);

	// Eigen::Matrix<double, Eigen::Dynamic, 3> A(pts.size(), 3);
	// for (size_t i = 0; i < pts.size(); ++i) {
	// 	A(i, 0) = 
	// }

}
