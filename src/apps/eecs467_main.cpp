#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include <iostream>

#include "eecs467_util.h"

int main() {
	// eecs467_init(0, NULL);
	// vx_world_t* vxWorld = vx_world_create();
	// vx_event_handler_t* vxeh = (vx_event_handler_t*) calloc(1, sizeof(vx_event_handler_t));
	// // vxeh->impl = 

	// vx_application_t vxApp;
	// vxApp.display_started = eecs467_default_display_started;
	// vxApp.display_finished = eecs467_default_display_finished;
	// vxApp.impl = eecs467_default_implementation_create(vxWorld, vxeh);


	// Freenect::Freenect freenect;
	// FreenectDevice467& device = freenect.createDevice<FreenectDevice467>(0);
	// device.startVideo();
	// device.startDepth();
	
	// while(1) {

	// }

	std::vector<std::array<double, 3>> pts;
	pts.push_back(std::array<double, 3>{{2, 1, 1}});
	pts.push_back(std::array<double, 3>{{3, 4, 3}});
	pts.push_back(std::array<double, 3>{{4, 6, 5}});
	pts.push_back(std::array<double, 3>{{5, 7, 6}});
	pts.push_back(std::array<double, 3>{{6, 10, 6.2}});

	auto i = LineFitter::fitCurve(pts);	
	std::cout << i.first[0]  << '\t' << i.first[1] << '\t' << i.first[2] << '\n';
	std::cout << i.second[0] << '\t' << i.second[1] << '\n';
}
