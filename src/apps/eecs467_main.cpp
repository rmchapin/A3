#include "a3/LineFitter.hpp"
#include <iostream>


int main() {
	std::vector<std::array<double, 3>> pts;
	pts.push_back(std::array<double, 3>{{1, 1, 1}});
	pts.push_back(std::array<double, 3>{{2, 5, 6}});
	pts.push_back(std::array<double, 3>{{3, 9, 5}});
	auto i = fitCurve(pts);	
	std::cout << i.first[0]  << '\t' << i.first[1] << '\t' << i.first[2] << '\n';
	std::cout << i.second[0] << '\t' << i.second[1] << '\n';
}
