#include "a3/LineFitter.hpp"
#include "a3/Freenect.hpp"
#include <iostream>
#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include "eecs467_util.h"

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

typedef struct {
    int x;
    int y;
} pix_coord;

typedef struct {
	unsigned char a;
	unsigned char b;
	unsigned char g;
	unsigned char r;
} ABGR_p;

typedef struct {
	double h;
	double s;
	double v;
} HSV_p;

HSV_p u32_pix_to_HSV(ABGR_p u32_in)
{
	HSV_p out;
	double min, max, delta;

    //printf("R: %d, G: %d, B: %d\n", u32_in.r, u32_in.g, u32_in.b);

    min = u32_in.r < u32_in.g ? u32_in.r : u32_in.g;
    min = min  < u32_in.b ? min  : u32_in.b;
    
    max = u32_in.r > u32_in.g ? u32_in.r : u32_in.g;
    max = max  > u32_in.b ? max  : u32_in.b;
    
    out.v = max / 255.0;                        // v
    delta = max - min;
    if( max > 0.0 )
    {
        out.s = (delta / max);                  // s
    }
    else
    {
        // r = g = b = 0                        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    
    if( u32_in.r >= max )                        // > is bogus, just keeps compilor happy
    {
        out.h = ( u32_in.g - u32_in.b ) / delta;        // between yellow & magenta
    }
    else
    {
        if( u32_in.g >= max )
            out.h = 2.0 + ( u32_in.b - u32_in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( u32_in.r - u32_in.g ) / delta;  // between magenta & cyan
    }
    
    out.h *= 60.0;                              // degrees
    
    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

Freenect::Freenect freenect;
FreenectDevice467& device  = freenect.createDevice<FreenectDevice467>(0);

class state_t
{
    public:
        // vx stuff	
        vx_application_t vxapp;
        vx_world_t * vxworld;
        vx_event_handler_t* vxeh;
        zhash_t * layers;
        vx_gtk_display_source_t* appwrap;
        pthread_mutex_t mutex; // for accessing the arrays
        pthread_t animate_thread;
        image_u32_t *u32_im;
        image_u32_t *revert;

   	    double Hmin, Hmax, Smin, Smax, Vmin, Vmax;
    	pix_coord last_click;
    	pix_coord cp_coords[100];
  	    int cp_index;

  	    bool running;
  	    bool capture;

    public:
        state_t()
        {
            //GUI init stuff
            layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            vxapp.impl= this;
            vxapp.display_started = display_started;
            vxapp.display_finished = display_finished;
            vxworld = vx_world_create();
            vxeh = (vx_event_handler_t*) calloc(1, sizeof(vx_event_handler_t));
			vxeh->key_event = key_event;
			vxeh->mouse_event = mouse_event;
			vxeh->impl = this;

            pthread_mutex_init (&mutex, NULL);

			device.startVideo();
			device.startDepth();

            u32_im = NULL;

			running = true;
			capture = false;
        }

        ~state_t()
        {
            vx_world_destroy(vxworld);
            assert(zhash_size(layers) == 0);
            zhash_destroy(layers);
            pthread_mutex_destroy(&mutex);
            image_u32_destroy(u32_im);
            image_u32_destroy(revert);
        }

        static int mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
        {
        	//state_t *state = (state_t*) vxeh->impl;
        	return (0);
        }

        static int key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
        {
        	//state_t *state = (state_t*) vxeh->impl;
        	return (0);
        }

        static void* render_loop(void* data)
        {
            state_t *state = (state_t*) data;
            int hz = 30;

            state->u32_im = device.getImage();

            while (state->running)
            {
            	pthread_mutex_lock(&state->mutex);

            	//while(state->capture)
            	{
                	if (state->u32_im != NULL)
		            {
		            	image_u32_destroy(state->u32_im);
		            	state->u32_im = device.getImage();
		            }

		            if (state->u32_im != NULL)
		            {
		                vx_object_t *vim = vxo_image_from_u32 (state->u32_im,
		                                                       VXO_IMAGE_FLIPY,
		                                                       VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

		                // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
		                const double scale = 2./state->u32_im->width;
		                vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
		                                    vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
		                                               vxo_mat_translate3 (-state->u32_im->width/2., -state->u32_im->height/2., 0.),
		                                               vim));

		                vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
		            }
            	}

            	//ELSE get from kinect

                pthread_mutex_unlock(&state->mutex);

                usleep(1000000/hz);
            }
            
            return NULL;
        }

        static void display_finished(vx_application_t * app, vx_display_t * disp)
        {
            state_t * state = (state_t *) app->impl;
            pthread_mutex_lock(&state->mutex);
            vx_layer_t * layer = NULL;
            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_remove(state->layers, &disp, NULL, &layer);
            vx_layer_destroy(layer);
            pthread_mutex_unlock(&state->mutex);
        }

        static void display_started(vx_application_t * app, vx_display_t * disp)
        {
            state_t * state = (state_t *) app->impl;
            vx_layer_t * layer = vx_layer_create(state->vxworld);
            vx_layer_set_display(layer, disp);
            pthread_mutex_lock(&state->mutex);
            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_put(state->layers, &disp, &layer, NULL, NULL);
            pthread_mutex_unlock(&state->mutex);
        }


};

int main(int argc, char ** argv)
{
	state_t state;
   	pthread_create(&state.animate_thread, NULL, &state_t::render_loop, &state);

    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    state.appwrap = vx_gtk_display_source_create(&state.vxapp);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(state.appwrap);

    // Quit when GTK closes
    state.capture = 0;
    state.running = 0;
    pthread_join(state.animate_thread,NULL);

    vx_global_destroy();
}
