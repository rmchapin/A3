#include "Freenect.hpp"
#include <cmath>
#include <pthread.h>

#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480

namespace Freenect {

static freenect_context* _ctx;
static freenect_device* _dev;
static image_u32_t* _rgb_im;
static image_u32_t* _depth_im;
static bool _new_rgb;
static bool _new_depth;
static Mutex _depthMutex;
static Mutex _rgbMutex;
static pthread_t freenectThreadPid;

void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp);

void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp);

void* freenectThread(void* args);

void init() {
	if (freenect_init(&_ctx, NULL) < 0) {
		printf("Couldn't initialize device\n");
	}

	freenect_set_log_level(_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	if (freenect_open_device(_ctx, &_dev, 0) < 0) {
		printf("Couldn't open device\n");
	}

	freenect_set_depth_callback(_dev, depthCallback);
	freenect_set_video_callback(_dev, rgbCallback);

	freenect_set_video_mode(_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));


	_rgb_im = image_u32_create(640, 480);
	_depth_im = image_u32_create(640, 480);

	_new_rgb = false;
	_new_depth = false;
}

void startDepthCallback() {
	freenect_start_depth(_dev);
}

void startVideoCallback() {
	freenect_start_video(_dev);
}

void launchThread() {
	if (pthread_create(&freenectThreadPid, NULL, freenectThread, NULL)) {
		printf("Couldn't launch freenect thread\n");
	}
}

void* freenectThread(void* args) {
	while (freenect_process_events(_ctx) >= 0);
	return NULL;
}

void depthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
	_depthMutex.lock();
	uint16_t* depthArr = static_cast<uint16_t*>(depth);

	for (int i = 0; i < 640*480; i++) {
		_depth_im->buf[(i/640) * _depth_im->stride + (i%640)] = 
			depthArr[i];
	}

	_new_depth = true;
	_depthMutex.unlock();
}

void rgbCallback(freenect_device *dev, void *rgb, uint32_t timestamp) {
	_rgbMutex.lock();
	uint8_t* rgbArr = static_cast<uint8_t*>(rgb);

	for (int in = 0; in < 640*480; in++)
	{
		_rgb_im->buf[(in/640) * _rgb_im->stride + (in%640)] = ((0xFF << 24) |
			((rgbArr[3*in + 2] & 0xFF) << 16) |
			((rgbArr[3*in + 1] & 0xFF) << 8) |
			(rgbArr[3*in] & 0xFF));
	}

	_new_rgb = true;
	_rgbMutex.unlock();
}

image_u32_t* getImage() {
	_rgbMutex.lock();
	
	image_u32_t* ret = nullptr;
	if (_new_rgb)
	{
		ret = image_u32_copy(_rgb_im);
		_new_rgb = false;
	}

	_rgbMutex.unlock();
	return ret;
}

image_u32_t* getDepth() {
	_depthMutex.lock();
	
	image_u32_t* ret = nullptr;
	if (_new_depth)
	{
		ret = image_u32_copy(_depth_im);
		_new_depth = false;
	}

	_depthMutex.unlock();
	return ret;
}

std::array<double, 2> cameraToWorld(int cx, int cy, int depth) {
	double wx, wy;
	freenect_camera_to_world(_dev, cx, cy, depth, &wx, &wy);
	return std::array<double, 2>{{wx, wy}};
}

}
