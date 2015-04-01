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
	void VideoCallback(void* _rgb, uint32_t timestamp);
	
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp);
	
	// bool getVideo(Mat& output);
	image_u32_t* getImage();
	
	// bool getDepth(Mat& output);
private:
	// std::vector<uint8_t> m_buffer_depth;
	// std::vector<uint8_t> m_buffer_rgb;
	// std::vector<uint16_t> m_gamma;
	Mutex _rgbMutex;
	Mutex _depthMutex;
	image_u32_t* _im;
	std::vector<uint16_t> _depthBuffer;
};

#endif /* FREENECT_HPP */
