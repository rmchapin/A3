#include "VxHandler.hpp"
#include "a3/CoordinateConverter.hpp"

#include <string>
#include <iostream>

VxHandler::VxHandler(int width, int height) :
	windowWidth(width), windowHeight(height) {
	eecs467_init(0, NULL);
	vxWorld = vx_world_create();
	vxeh = (vx_event_handler_t*) calloc(1, sizeof(vx_event_handler_t));
	vxeh->key_event = key_event;
	vxeh->mouse_event = mouse_event;
	vxeh->touch_event = touch_event;
	vxeh->dispatch_order = 0;
	vxeh->impl = this;

	vxApp.display_started = eecs467_default_display_started;
	vxApp.display_finished = eecs467_default_display_finished;
	vxApp.impl = eecs467_default_implementation_create(vxWorld, vxeh);

	im_disp = nullptr;

	pg = pg_create();
	pg_add_buttons(pg,
		"but1", "Button1",
		NULL);
	pgListener = (parameter_listener_t*) calloc(1, sizeof(parameter_listener_t));
	pgListener->impl = this;
	pgListener->param_changed = parameterEventHandler;
	pg_add_listener(pg, pgListener);
}

VxHandler::~VxHandler() {
	pg_destroy(pg);
	free(vxeh);
}

void VxHandler::launchThreads() {
	pthread_create(&renderPid, NULL, &VxHandler::renderThread, this);
	pthread_create(&mainPid, NULL, &VxHandler::mainThread, this);
}

void VxHandler::setImage(image_u32_t *set)
{
	if (set != nullptr)
	{
		renderMutex.lock();
		if (im_disp != nullptr)
		{
			image_u32_destroy(im_disp);
		}
		im_disp = image_u32_copy(set);
		renderMutex.unlock();
	}
}

void* VxHandler::renderThread(void* args) {
	VxHandler* state = (VxHandler*) args;

	while (1) {
		state->renderMutex.lock();
		if (state->im_disp != nullptr)
		{
			vx_object_t* vim = vxo_chain(
				vxo_mat_translate3(-0.5, 
					-0.5 * ((float)state->im_disp->height / state->im_disp->width), 0),
				vxo_mat_scale(1.0 / state->im_disp->width),
				vxo_image_from_u32(state->im_disp, VXO_IMAGE_FLIPY, 0));
			vx_buffer_add_back(vx_world_get_buffer(state->vxWorld, "state"), vim);

			vx_buffer_swap(vx_world_get_buffer(state->vxWorld, "state"));
		}
		state->renderMutex.unlock();
		usleep(1000000/40);
	}

	return NULL;
}

void* VxHandler::mainThread(void* args) {
	VxHandler* state = (VxHandler*) args;

	eecs467_gui_run(&state->vxApp, state->pg, 1024, 768);
	return NULL;
}

int VxHandler::mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, 
	vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
	VxHandler* state = (VxHandler*) vxeh->impl;
	// vx_camera_pos_t contains camera location, field of view, etc
	// vx_mouse_event_t contains scroll, x/y, and button click events

	if ((mouse->button_mask & VX_BUTTON1_MASK) &&
		!(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {
		
		if (state->im_disp != nullptr)
		{
			vx_ray3_t ray;
			vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

			double ground[3];
			vx_ray3_intersect_xy (&ray, 0, ground);
			state->renderMutex.lock();

			std::array<float, 2> screenCoord{{ground[0], ground[1]}};
			//image_u32_t* im = GlobalState::instance()->getIm();
			std::array<int, 2> imageCoord =
				CoordinateConverter::screenToImage(screenCoord, state->im_disp->width, state->im_disp->height);

			uint16_t depth = state->im_disp->buf[state->im_disp->stride * imageCoord[1] + imageCoord[0]];

			state->renderMutex.unlock();
			printf("clicked (%d, %d) with depth %d\n", imageCoord[0], imageCoord[1], depth & 0x00FFFFFF);
		}
	}

	// store previous mouse event to see if the user *just* clicked or released
	state->last_mouse_event = *mouse;

	return 1;
}

int VxHandler::key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
	//state_t *state = vxeh->impl;
	return 0;
}

int VxHandler::touch_event(vx_event_handler_t *vh, vx_layer_t *vl, 
	vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
	return 0; // Does nothing
}

int count = 0;
void VxHandler::parameterEventHandler (parameter_listener_t *pl, 
	parameter_gui_t *pg, const char *name) {

	std::string strName(name);
	if (strName == "but1") {
		image_u32_t* cp = GlobalState::instance()->getIm();
		char buf[100];
		sprintf(buf, "image%d.pnm", count++);
		int res = image_u32_write_pnm(cp, buf);
		if (res) {
			std::cout << "did not save image successfully\n";
		}
	}
	// if (strName == "but1") {
	// 	// calibrate mask
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		CalibrationHandler::instance()->calibrateMask();
	// 	}
	// } else if (strName == "but2") {
	// 	// calibrate hsv
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		CalibrationHandler::instance()->calibrateHsv();
	// 	}
	// } else if (strName == "but3") {
	// 	// calibrate board transform
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		CalibrationHandler::instance()->calibrateBoardTransform();
	// 	}
	// } else if (strName == "but4") {
	// 	RenderInfo renderInfo = GlobalState::instance()->getData();
	// 	//save image
	// 	int res = image_u32_write_pnm(renderInfo.im, imageFileName.c_str());
	// 	if (res) {
	// 		std::cout << "did not save image successfully\n";
	// 	}
	// } else if (strName == "but5") {
	// 	// move arm
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		buttonStates.moveArm = true;
	// 	}
	// } else if (strName == "but6") {
	// 	// coordinate convert
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		CalibrationHandler::instance()->coordTransform();
	// 	}
	// } else if (strName == "but7") {
	// 	// color mask
	// 	buttonStates.colorMask = !buttonStates.colorMask;
	// } else if (strName == "but8") {
	// 	// blob detect
	// 	buttonStates.blobDetect = !buttonStates.blobDetect;
	// } else if (strName == "but9") {
	// 	// drop ball
	// 	if (CalibrationHandler::instance()->isIdle()) {
	// 		buttonStates.dropBall = true;
	// 	}
	// } else if (strName == "but10") {
	//		// show valid board pixels
	//		buttonStates.boardMask = !buttonStates.boardMask;
	// } else if (strName == "but11") {
	// 	GlobalState::instance()->setStart(true);
	// }
}

