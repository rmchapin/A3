#ifndef GLOBAL_STATE_HPP
#define GLOBAL_STATE_HPP

#include "Mutex.hpp"
#include "imagesource/image_u32.h"

class GlobalState {
private:
	Mutex _globalMutex;
	GlobalState();
	static GlobalState* _instance;
	image_u32_t* _im;

public:
	static GlobalState* instance();

	/**
	 * @brief gets a copy of global image
	 */
	image_u32_t* getIm();

	/**
	 * @brief sets global image with im. Note: globalState will take care of destroying this im
	 */
	void setIm(image_u32_t* im);



};

#endif /* GLOBAL_STATE_HPP */
