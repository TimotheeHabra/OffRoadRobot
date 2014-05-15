/*
 * Configure the SDL screen to plot simulations values
 *
 * author: Nicolas Van der Noot
 */
#if defined(SDL) & defined(REAL_TIME)

#include "plot_sdl.h"
#include "main_simulation.h"
#include "simu_def.h"
//#include "controller_def.h"

/*
 * Assign values for the SDL functions
 */
void get_screen_sdl_functions(double *y_vec, MBSdataStruct *MBSdata)
{
	// -- Variables declaration -- //

	//UserIOStruct *uvs;
    //ControllerStruct *cvs;
    //MusclePropertiesStruct *mvs;

    // -- Variables initialization -- //

    //uvs = MBSdata->user_IO;
    //cvs = uvs->cvs;

    // -- Assign values for the curves -- //

    /*
     * Starts from y_vec[0], up to y_vec[nb_curves]
     * -> Assigns the variable to plot
     */

     y_vec[0] = 0;
    
}


/*
 * Configuration of the SDL screen to plot curves
 */
Screen_sdl* configure_screen_sdl(int init_t_sec, int init_t_usec)
{
	// -- Variables declaration -- //

	int nb_curves, nb_legend_curves;

	double y_min_init, y_max_init;

	char **label_curves;


	// ------ CURVES DEFINITION ------ //

	// colors of the vectors -> modify this order according to your will but keep the same number (NB_CURVES_MAX) of colors

	int color_vec[NB_CURVES_MAX] = {BLUE_SDL, RED_SDL, DARK_GREEN_SDL, PURPLE_SDL, ORANGE_SDL,
									LIGHT_BLUE_SDL, TURQUOISE_SDL, PINK_SDL, LIGHT_GREEN_SDL, 
									DARK_YELLOW_SDL, YELLOW_SDL, GREEN_SDL};

	label_curves = get_char_tab(NB_CURVES_MAX, 20); // do not modify

	// number of curves

	nb_curves        = 1; // curves to plot
	nb_legend_curves = 2; // legends (with scaling) to show

	// initial y bounds

	y_min_init = -1.0; // y min value
	y_max_init =  1.0; // y max value

	// labels of the curves
	label_curves[0] = " ";

	// ------------------------------ //

	return init_screen_sdl(init_t_sec, init_t_usec, NB_SIMU_STEPS, FQC_SREEN, color_vec, y_min_init, y_max_init, nb_curves, nb_legend_curves, label_curves);
}

#endif


