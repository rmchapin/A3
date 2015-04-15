#ifndef FREENECT_HPP
#define FREENECT_HPP

#include "libfreenect.hpp"
#include "Mutex.hpp"

#include "imagesource/image_source.h"
#include "imagesource/image_u32.h"

#include <vector>
#include <stdint.h>
#include <iostream>
#include <utility>

class FreenectDevice467 : public Freenect::FreenectDevice {
public:
	FreenectDevice467(freenect_context *_ctx, int _index);
	
	// Do not call directly even in child
	void VideoCallback(void* _rgb_in, uint32_t timestamp);
	
	// Do not call directly even in child
	void DepthCallback(void* _depth_in, uint32_t timestamp);
	
	// gets copy of image (delete it after use)
	image_u32_t* getImage();

	// get copy of depth (delete if after use)
	image_u32_t* getDepth();

private:
	Mutex _rgbMutex;
	Mutex _depthMutex;
	bool _new_rgb;
	bool _new_depth;
	image_u32_t* _rgb_im;
	image_u32_t* _depth_im;
	uint16_t _t_gamma[2048];
};

#endif /* FREENECT_HPP */
