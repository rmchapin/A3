#include "GlobalState.hpp"

GlobalState* GlobalState::_instance = new GlobalState();

GlobalState::GlobalState() : _im(nullptr) {}

GlobalState* GlobalState::instance() {
	return _instance;
}

image_u32_t* GlobalState::getIm() {
	_globalMutex.lock();
	image_u32_t* ret = nullptr;
	if (_im != nullptr) {
		ret = image_u32_copy(_im);
	}
	_globalMutex.unlock();
	return ret;
}

void GlobalState::setIm(image_u32_t* im) {
	if (_im != nullptr) {
		image_u32_destroy(_im);
	}
	_im = im;

}

