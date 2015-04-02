#ifndef LCM_HANDLER_HPP
#define LCM_HANDLER_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "Mutex.hpp"

class LcmHandler {
private:
	lcm::LCM _lcm;
	pthread_t _lcmThreadPid;

	LcmHandler();
	static LcmHandler* _instance;
public:
	static LcmHandler* instance();

	void launchThreads();

	void handleArmStatusMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const dynamixel_status_list_t* msg);

	lcm::LCM* lcm();

private:
	static void* lcmHandleThread(void* args);
};

#endif /* LCM_HANDLER_HPP */
