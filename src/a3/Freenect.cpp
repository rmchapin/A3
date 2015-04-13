#include "Freenect.hpp"
#include <cmath>

FreenectDevice467::FreenectDevice467(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index) {
	_im = image_u32_create(640, 480);
	_depthBuffer.reserve(640 * 480);
	// for( unsigned int i = 0 ; i < 2048 ; i++) {
	// 	float v = i/2048.0;
	// 	v = std::pow(v, 3)* 6;
	// 	m_gamma[i] = v*6*256;
	// }
}

// Do not call directly even in child
void FreenectDevice467::VideoCallback(void* _rgb, uint32_t timestamp) {
	//std::cout << "RGB @" << timestamp << std::endl;
	_rgbMutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);

	for (int in = 0; in < 640*480; in++)
	{
		_im->buf[(in/640) * _im->stride + (in%640)] = ((0xFF << 24) |
										  			  ((rgb[3*in + 2] & 0xFF) << 16) |
										  			  ((rgb[3*in + 1] & 0xFF) << 8) |
										  			  (rgb[3*in] & 0xFF));
	}

	_rgbMutex.unlock();
};

// Do not call directly even in child
void FreenectDevice467::DepthCallback(void* _depth, uint32_t timestamp) {
	//std::cout << "DEPTH @" << timestamp << std::endl;
	_depthMutex.lock();
	// m_depth_mutex.lock();
	// uint16_t* depth = static_cast<uint16_t*>(_depth);
	// depthMat.data = (uchar*) depth;
	// m_new_depth_frame = true;
	// m_depth_mutex.unlock();
	_depthMutex.unlock();
}

image_u32_t* FreenectDevice467::getImage() {
	_rgbMutex.lock();
	image_u32_t* ret = image_u32_copy(_im);
	_rgbMutex.unlock();
	return ret;
}

// bool FreenectDevice467::getVideo(Mat& output) {
// 	m_rgb_mutex.lock();
// 	if(m_new_rgb_frame) {
// 		cv::cvtColor(rgbMat, output, CV_RGB2BGR);
// 		m_new_rgb_frame = false;
// 		m_rgb_mutex.unlock();
// 		return true;
// 	} else {
// 		m_rgb_mutex.unlock();
// 		return false;
// 	}
// }

// bool FreenectDevice467::getDepth(Mat& output) {
// 	m_depth_mutex.lock();
// 	if(m_new_depth_frame) {
// 		depthMat.copyTo(output);
// 		m_new_depth_frame = false;
// 		m_depth_mutex.unlock();
// 		return true;
// 	} else {
// 		m_depth_mutex.unlock();
// 		return false;
// 	}
// }
