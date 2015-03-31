#ifndef FREENECT_HPP
#define FREENECT_HPP

#include "libfreenect.hpp"
#include "Mutex.hpp"
#include <vector>
#include <stdint.h>
#include <iostream>
#include <utility>

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
	MyFreenectDevice(freenect_context *_ctx, int _index);
	
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp);
	
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp);
	
	// bool getVideo(Mat& output);
	
	// bool getDepth(Mat& output);
private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_rgb;
	std::vector<uint16_t> m_gamma;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	// bool m_new_rgb_frame;
	// bool m_new_depth_frame;
};

#endif /* FREENECT_HPP */
