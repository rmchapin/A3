#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include <iostream>


int main() {
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

	std::vector<std::array<double, 3>> pts;
	pts.push_back(std::array<double, 3>{{1, 1, 1}});
	pts.push_back(std::array<double, 3>{{2, 4, 3}});
	pts.push_back(std::array<double, 3>{{3, 6, 5}});
	pts.push_back(std::array<double, 3>{{4, 7, 6}});
	pts.push_back(std::array<double, 3>{{5, 10, 6.2}});

	auto i = fitCurve(pts);	
	std::cout << i.first[0]  << '\t' << i.first[1] << '\t' << i.first[2] << '\n';
	std::cout << i.second[0] << '\t' << i.second[1] << '\n';
}
