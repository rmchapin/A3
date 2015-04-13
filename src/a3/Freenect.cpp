#include "Freenect.hpp"
#include <cmath>

FreenectDevice467::FreenectDevice467(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index) {

	_rgb_im = image_u32_create(640, 480);
	_depth_im = image_u32_create(640, 480);

	_new_rgb = false;
	_new_depth = false;

	for (int i = 0; i < 2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		_t_gamma[i] = v*6*256;
	}
}

// Do not call directly even in child
void FreenectDevice467::VideoCallback(void* _rgb, uint32_t timestamp) {
	//std::cout << "RGB @" << timestamp << std::endl;
	
	_rgbMutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);

	for (int in = 0; in < 640*480; in++)
	{
		_rgb_im->buf[(in/640) * _rgb_im->stride + (in%640)] = ((0xFF << 24) |
												  			  ((rgb[3*in + 2] & 0xFF) << 16) |
												  			  ((rgb[3*in + 1] & 0xFF) << 8) |
												  			  (rgb[3*in] & 0xFF));
	}

	_new_rgb = true;
	_rgbMutex.unlock();
}

// Do not call directly even in child
void FreenectDevice467::DepthCallback(void* _depth, uint32_t timestamp) {
	//std::cout << "DEPTH @" << timestamp << std::endl;
	
	_depthMutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);

	for (int in = 0; in < 640*480; in++)
	{
		int pval = _t_gamma[depth[in]];
		int lb = pval & 0xff;
		uint8_t r, g, b;
		switch (pval>>8)
		{
			case 0:
				r = 255;
				g = 255-lb;
				b = 255-lb;
				break;
			case 1:
				r = 255;
				g = lb;
				b = 0;
				break;
			case 2:
				r = 255-lb;
				g = 255;
				b = 0;
				break;
			case 3:
				r = 0;
				g = 255;
				b = lb;
				break;
			case 4:
				r = 0;
				g = 255-lb;
				b = 255;
				break;
			case 5:
				r = 0;
				g = 0;
				b = 255-lb;
				break;
			default:
				r = 0;
				g = 0;
				b = 0;
				break;
		}

		_depth_im->buf[(in/640) * _rgb_im->stride + (in%640)] =
									((0xFF << 24) | (b << 16) | (g << 8) | r);
	}

	_new_depth = true;
	_depthMutex.unlock();
}

image_u32_t* FreenectDevice467::getImage() {
	_rgbMutex.lock();
	
	image_u32_t* ret = NULL;
	if (_new_rgb)
	{
		ret = image_u32_copy(_rgb_im);
		_new_rgb = false;
	}

	_rgbMutex.unlock();
	return ret;
}

image_u32_t* FreenectDevice467::getDepth() {
	_depthMutex.lock();
	
	image_u32_t* ret = NULL;
	if (_new_depth)
	{
		ret = image_u32_copy(_depth_im);
		_new_depth = false;
	}

	_depthMutex.unlock();
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
