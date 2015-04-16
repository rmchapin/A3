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
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "imagesource/image_u32.h"


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
        image_u32_t *depth;

        vx_mouse_event_t    last_mouse_event;
   	    double Hmin, Hmax, Smin, Smax, Vmin, Vmax;
    	pix_coord last_click;
    	pix_coord cp_coords[100];
  	    int cp_index;

  	    bool running;
  	    bool capture;

    public:
        state_t()
        {
            eecs467_init(0, NULL);
            vxworld = vx_world_create();
            vxeh = (vx_event_handler_t*) calloc(1, sizeof(vx_event_handler_t));
            vxeh->key_event = key_event;
            vxeh->mouse_event = mouse_event;
            vxeh->dispatch_order = 0;
            vxeh->impl = this;

            vxapp.display_started = eecs467_default_display_started;
            vxapp.display_finished = eecs467_default_display_finished;
            vxapp.impl = eecs467_default_implementation_create(vxworld, vxeh);

            cp_index = 0;
            Hmin = -1.0;
            Hmax = -1.0;
            Smin = -1.0;
            Smax = -1.0;
            Vmin = -1.0;
            Vmax = -1.0;

            pthread_mutex_init (&mutex, NULL);

			device.startVideo();
			device.startDepth();

            u32_im = NULL;
            revert = NULL;
            depth = NULL;

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

        static void clear_all(state_t *state)
        {
            //zero color picker index
            state->cp_index = 0;
            state->Hmin = -1.0;
            state->Hmax = -1.0;
            state->Smin = -1.0;
            state->Smax = -1.0;
            state->Vmin = -1.0;
            state->Vmax = -1.0;
        }

        static void process_cp(state_t *state)
        {  
            ABGR_p pixel_abgr;
            uint32_t val = state->revert->buf[(state->cp_coords[state->cp_index-1].y * state->revert->stride) + state->cp_coords[state->cp_index-1].x];
            pixel_abgr.a = 0xFF & (val >> 24);
            pixel_abgr.b = 0xFF & (val >> 16);
            pixel_abgr.g = 0xFF & (val >> 8);
            pixel_abgr.r = 0xFF & val;
              
            //convert to hsv and update bounds
            HSV_p pixel_hsv;
            pixel_hsv = u32_pix_to_HSV(pixel_abgr);

            if (state->Hmin == -1.0)
            {
                state->Hmin = state->Hmax = pixel_hsv.h;
                state->Smin = state->Smax = pixel_hsv.s;
                state->Vmin = state->Vmax = pixel_hsv.v;
            }
            else
            {
                if (pixel_hsv.h < state->Hmin)
                    state->Hmin = pixel_hsv.h;
                if (pixel_hsv.h > state->Hmax)
                    state->Hmax = pixel_hsv.h;
                if (pixel_hsv.s < state->Smin)
                    state->Smin = pixel_hsv.s;
                if (pixel_hsv.s > state->Smax)
                    state->Smax = pixel_hsv.s;
                if (pixel_hsv.v < state->Vmin)
                    state->Vmin = pixel_hsv.v;
                if (pixel_hsv.v > state->Vmax)
                    state->Vmax = pixel_hsv.v;
            }
            
            printf("Hmin %f, Hmax %f, Smin %f, Smax %f, Vmin %f, Vmax %f\n", state->Hmin, state->Hmax, state->Smin, state->Smax, state->Vmin, state->Vmax);
            
            //update image with hsv bounds
            int p, q;
            for (p = 0; p < state->revert->height; p++)
            {
                for (q = 0; q < state->revert->width; q++)
                {
                    //make rgba pixel
                    ABGR_p pixel_abgr;
                    uint32_t val = state->revert->buf[state->revert->stride * p + q];
         
                    pixel_abgr.a = 0xFF & (val >> 24);
                    pixel_abgr.b = 0xFF & (val >> 16);
                    pixel_abgr.g = 0xFF & (val >> 8);
                    pixel_abgr.r = 0xFF & val;

                    HSV_p pixel_hsv;
                    pixel_hsv = u32_pix_to_HSV(pixel_abgr);

                    if ((pixel_hsv.h >= state->Hmin) && 
                        (pixel_hsv.h <= state->Hmax) &&
                        (pixel_hsv.s >= state->Smin) &&
                        (pixel_hsv.s <= state->Smax) &&
                        (pixel_hsv.v >= state->Vmin) &&
                        (pixel_hsv.v <= state->Vmax))
                    {
                        state->u32_im->buf[state->u32_im->stride * p + q] = 0xFFE600CB;
                    }
                }
            }
        }

        static int mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
        {
            state_t *state = (state_t*) vxeh->impl;

            // vx_camera_pos_t contains camera location, field of view, etc
            // vx_mouse_event_t contains scroll, x/y, and button click events

            if ((mouse->button_mask & VX_BUTTON1_MASK) &&
                !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK))
            {       
                if (state->capture)
                {
                    vx_ray3_t ray;
                    vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

                    double ground[3];
                    vx_ray3_intersect_xy (&ray, 0, ground);

                    state->last_click.x = (int) ((ground[0] + 1.0f) * 0.5f * state->revert->width);
                    state->last_click.y = (int) ((((float)(state->revert->height)/(float)(state->revert->width)) - ground[1])* 0.5f * (float)(state->revert->width));
                    printf("click registered at pix_coord: %d, %d\n", state->last_click.x, state->last_click.y);


                    std::cout << "depth: " << state->depth->buf[state->last_click.y * state->depth->stride + state->last_click.x] << std::endl;
                    // if (state->cp_index < 100)
                    // {
                    //     if ((state->last_click.x >= 0) && (state->last_click.x < state->revert->width) &&
                    //         (state->last_click.y >=0) && (state->last_click.y < state->revert->height))
                    //     {
                    //             printf("point added to cp_coords\n");
                    //             state->cp_coords[state->cp_index] = state->last_click;
                    //             state->cp_index++;
                    //             process_cp(state);
                    //     }
                    // }

                }
                else
                {
                    std::cout << "you must CAPTURE an image" << std::endl;
                }
            }
  
            // store previous mouse event to see if the user *just* clicked or released
            state->last_mouse_event = *mouse;

            return 0;
        }

        static int key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
        {
        	state_t *state = (state_t*) vxeh->impl;

            if (key->key_code == VX_KEY_s && key->released)
            {
                state->capture = true;
            }

            if (key->key_code == VX_KEY_DEL && key->released)
            {
                state->clear_all(state);
                state->capture = false;
            }

            if (key->key_code == VX_KEY_f && key->released)
            {
                if (state->cp_index > 1)
                {
                    //write to file
                    printf("enter name for color range:\n");
                    char path[100];
                    char *ret = fgets(path, 100, stdin);
                    if (ret == path)
                    {
                        int len = strlen(path);
                        path[len - 1] = '\0'; // replace \n with null character because fgets is terrible
                        strcat(path, ".txt");
                    }

                    FILE *fp;
                    fp = fopen(path, "w");
                    fprintf(fp, "%f %f %f %f %f %f\n", state->Hmin, state->Hmax, state->Smin, state->Smax, state->Vmin, state->Vmax);
                    fclose(fp);
                    printf("color range written to file\n");
                }
            }

            if (key->key_code == VX_KEY_p && key->released)
            {
                if (state->capture)
                {
                    (void) image_u32_write_pnm(state->u32_im, "rgb.pnm");
                    (void) image_u32_write_pnm(state->depth, "depth.pnm");
                    std::cout << "images written to file" << std::endl;
                }
                else
                {
                    std::cout << "you must CAPTURE an image" << std::endl;
                }
            }

            if (key->key_code == VX_KEY_l && key->released)
            {
                //read mask
                FILE * fptr = fopen("green.txt", "r");
                if (fptr == NULL)
                {
                    printf("green.txt does not exist, or could not be opened\n");
                    exit(-2);
                }
                fscanf(fptr, "%lf %lf %lf %lf %lf %lf", &state->Hmin, &state->Hmax, &state->Smin, &state->Smax, &state->Vmin, &state->Vmax);
                fclose(fptr);
                std::cout << "color range read from file" << std::cout;
            }

        	return (0);
        }

        static void* render_loop(void* data)
        {
            state_t *state = (state_t*) data;
            int hz = 30;

            while (state->running)
            {
                while(state->capture)
            	{
                    pthread_mutex_lock(&state->mutex);

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

                    pthread_mutex_unlock(&state->mutex);
            	}//while capture

                pthread_mutex_lock(&state->mutex);

                if (state->u32_im != NULL)
                {
                    image_u32_destroy(state->u32_im);
                }

                if (state->revert != NULL)
                {
                    image_u32_destroy(state->revert);
                }

                if (state->depth != NULL)
                {
                    image_u32_destroy(state->depth);
                }

                state->u32_im = device.getImage();
                state->revert = image_u32_copy(state->u32_im);
                state->depth = device.getDepth();

                if (state->u32_im != NULL)
                {
                    // for (int p = 0; p < state->revert->height; p++)
                    // {
                    //     for (int q = 0; q < state->revert->width; q++)
                    //     {
                    //         //make rgba pixel
                    //         ABGR_p pixel_abgr;
                    //         uint32_t val = state->revert->buf[state->revert->stride * p + q];
                 
                    //         pixel_abgr.a = 0xFF & (val >> 24);
                    //         pixel_abgr.b = 0xFF & (val >> 16);
                    //         pixel_abgr.g = 0xFF & (val >> 8);
                    //         pixel_abgr.r = 0xFF & val;

                    //         HSV_p pixel_hsv;
                    //         pixel_hsv = u32_pix_to_HSV(pixel_abgr);

                    //         if ((pixel_hsv.h >= state->Hmin) && 
                    //             (pixel_hsv.h <= state->Hmax) &&
                    //             (pixel_hsv.s >= state->Smin) &&
                    //             (pixel_hsv.s <= state->Smax) &&
                    //             (pixel_hsv.v >= state->Vmin) &&
                    //             (pixel_hsv.v <= state->Vmax))
                    //         {
                    //             state->u32_im->buf[state->u32_im->stride * p + q] = 0xFFE600CB;
                    //         }
                    //     }
                    // }

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
