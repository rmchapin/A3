#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include <iostream>

#include "eecs467_util.h"

int main() {
	std::vector<std::array<double, 3>> pts;
	pts.push_back(std::array<double, 3>{{1, 2, 1}});
	pts.push_back(std::array<double, 3>{{2, 3, 2}});
	pts.push_back(std::array<double, 3>{{3, 4, 1}});
	pts.push_back(std::array<double, 3>{{4, 6, -1}});

	auto i = LineFitter::fitCurve(pts);	
	std::cout << i.first[0]  << '\t' << i.first[1] << '\t' << i.first[2] << '\n';
	std::cout << i.second[0] << '\t' << i.second[1] << '\n';

	std::array<double, 2> intersection;
	LineFitter::getIntersectionZ(1, intersection, i);
	std::cout << "intersection: " << intersection[0] << '\t' 
		<< intersection[1] << '\n';
}
