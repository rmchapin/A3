#include "LcmHandler.hpp"
#include "Arm.hpp"

LcmHandler* LcmHandler::_instance = new LcmHandler;

LcmHandler::LcmHandler() {
	_lcm.subscribe("ARM_STATUS", &LcmHandler::handleArmStatusMessage, this);
}

LcmHandler* LcmHandler::instance() {
	return _instance;
}

void LcmHandler::launchThreads() {
	pthread_create(&_lcmThreadPid, NULL, &LcmHandler::lcmHandleThread, this);
}

void LcmHandler::handleArmStatusMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const dynamixel_status_list_t* msg) {
	Arm::instance()->update(msg);
}

lcm::LCM* LcmHandler::lcm() {
	return &_lcm;
}

void* LcmHandler::lcmHandleThread(void* args) {
	LcmHandler* state = (LcmHandler*) args;
	while (1) {
		state->_lcm.handle();
	}
	return NULL;
}
