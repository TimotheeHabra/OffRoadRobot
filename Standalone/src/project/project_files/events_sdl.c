/*
 * Handles the events coming from the keyboard 
 * or from other sources via the SDL framework
 *
 * author: Nicolas Van der Noot
 */
#if defined(SDL) & defined(REAL_TIME)

#include "plot_sdl.h"

#include "controller_def.h"

/*
 * Handles the events (SDL)
 */
void events_sdl(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata)
{
	// -- Vraiables declaration -- //

	SDL_Event event;

	UserIOStruct *uvs;
    ControllerStruct *cvs;


    // -- Variables initialization -- //

    uvs = MBSdata->user_IO;
    cvs = uvs->cvs;


    // -- Handles the events -- //

    // -> you can add new cases for joysticks...
 
	// get event (if any)
	if (SDL_PollEvent(&event)) 
	{
		// analyze the type of event
		switch (event.type)
		{
			// top corner button pressed 
			case SDL_QUIT:
				real_time->simu_quit  = 1;
				real_time->simu_break = 0;
				break;

			// keyboard button pressed
			case SDL_KEYDOWN:

				// id of the key
				printf("Key pressed: %d\n",event.key.keysym.sym);

				// handles keyboard events
		        switch (event.key.keysym.sym)
		        {
		        	// ----- MODIFICATIONS HERE ----- //

		        	// increase mu (key: a)
		        	case 97:
		        			
		        		break;

		        	// decrease mu (key: q)
		        	case 113:
		        		
		        		break;

		        	// ----- AVOID MODIFYING ----- //

		        	// break (key : p)
		        	case 112: 
			        	if (!real_time->simu_break)
			        	{
			        		real_time->simu_break = 1; // break flag
			        	}
			        	else
			        	{
			        		real_time->simu_break = 0;
			        	}		        		
		        		break;

		        	// quit the simulation (key: m)
		        	case 109:
		        		real_time->simu_quit  = 1; // flag to quit the simulation
		        		real_time->simu_break = 0;
		        		break;

		        	// decrease the simulation speed (key: l)
		        	case 108:
		        		real_time->simu_speed_flag--;	
		        		screen_sdl->break_plot_flag = 1;        		
		        		break;

		        	// increase the simulation speed (key: o)
		        	case 111: 
		        		real_time->simu_speed_flag++;	
		        		screen_sdl->break_plot_flag = 1;        		
		        		break;

		        	// activate / deactivate the signals auto-scaling (key: k)
		        	case 107:
			        	if (screen_sdl->signal_auto_scaling)
			        	{
			        		screen_sdl->signal_auto_scaling = 0;
			        	}
			        	else
			        	{
			        		screen_sdl->signal_auto_scaling = 1;
			        	}
			        	screen_sdl->break_plot_flag = 1;
		        		break;

		        	// activate / deactivate the y axis auto-scaling (key: i)
		        	case 105:
			        	if (screen_sdl->plot_auto_scaling)
			        	{
			        		screen_sdl->plot_auto_scaling = 0;
			        	}
			        	else
			        	{
			        		screen_sdl->plot_auto_scaling = 1;
			        	}
			        	screen_sdl->break_plot_flag = 1;
		        		break;
		        	
		        	// broaden x or y axis (key: u)
		        	case 117: 
		        		if (screen_sdl->hor_plot_scaling)
		        		{
		        			screen_sdl->increase_plot_x_diff_flag = 1;		        			
		        		}
		        		else
		        		{
		        			screen_sdl->increase_plot_y_diff_flag = 1;
		        			screen_sdl->plot_auto_scaling         = 0;
		        		}	
		        		screen_sdl->break_plot_flag = 1;	        		
		        		break;

		        	// shrink x or y axis (key: j)
		        	case 106:        		
		        		if (screen_sdl->hor_plot_scaling)
		        		{
		        			screen_sdl->decrease_plot_x_diff_flag = 1;		        			
		        		}
		        		else
		        		{
		        			screen_sdl->decrease_plot_y_diff_flag = 1;
		        			screen_sdl->plot_auto_scaling         = 0;
		        		}	
		        		screen_sdl->break_plot_flag = 1;	        		
		        		break;

		        	// shift x or y axis: up or right (key: y)
		        	case 121:      		
		        		if (screen_sdl->hor_plot_scaling)
		        		{
		        			screen_sdl->right_plot_y_flag = 1;		        			
		        		}
		        		else
		        		{
		        			screen_sdl->up_plot_y_flag    = 1;
		        			screen_sdl->plot_auto_scaling = 0;
		        		}	
		        		screen_sdl->break_plot_flag = 1;	        		
		        		break;

		        	// shift x or y axis: bottom or left (key: h)
		        	case 104:
		        		if (screen_sdl->hor_plot_scaling)
		        		{
		        			screen_sdl->left_plot_y_flag = 1;		        			
		        		}
		        		else
		        		{
		        			screen_sdl->bottom_plot_y_flag = 1;
		        			screen_sdl->plot_auto_scaling  = 0;
		        		}	
		        		screen_sdl->break_plot_flag = 1;	        		
		        		break;

		        	// switch from y axis to x axis auto-scaling and vice-versa (key: n)
		        	case 110:		        		
			        	if (real_time->simu_break)
			        	{
			        		if (screen_sdl->hor_plot_scaling)
				        	{
				        		screen_sdl->hor_plot_scaling = 0;
				        	}
				        	else
				        	{
				        		screen_sdl->hor_plot_scaling = 1;
				        	}
			        	}
			        	else
			        	{
			        		screen_sdl->hor_plot_scaling = 0;
			        	}
			        	screen_sdl->break_plot_flag = 1;
			        	break;

			        // change viewpoint for JNI (key: v)
			        case 118:
			        	real_time->change_viewpoint = 1;
			        	break;
			        	
		        	default:
		        		break;
		        }
				break;
		
			default:
				break;
		}
	}
}

#endif
