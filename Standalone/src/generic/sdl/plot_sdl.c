/*
 * SDL functions (plot curves and handles keyboard, joystick... inputs) functions
 *
 * author: Nicolas Van der Noot
 */

#if defined(SDL) & defined(REAL_TIME)

#include "plot_sdl.h"
#include "cmake_config.h"

/*
 * Initialize the Screen_sdl structure
 */
Screen_sdl* init_screen_sdl(int init_t_sec, int init_t_usec, int nb_steps, double fqc_screen, int color_vec[NB_CURVES_MAX], 
							double y_min_init, double y_max_init, int nb_curves, int nb_legend_curves, char **label_tab)
{
	// -- Variables declaration -- //

	int i;
	int space_up_label;

	// genral structure
	Screen_sdl *screen_sdl;

	// SDL window
	SDL_Window   *win;

	// SDL Renderer
	SDL_Renderer *ren;

	// SDL colors
	SDL_Color text_color;


	// -- Structure initialization -- //

	screen_sdl = (Screen_sdl*) malloc(sizeof(Screen_sdl));

	// initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		log_SDL_error("SDL_Init");
	}

	// window
    win = SDL_CreateWindow(
        "Real-time graph",                   // window title
        SDL_WINDOWPOS_UNDEFINED,             // initial x position
        SDL_WINDOWPOS_UNDEFINED,             // initial y position
        SCREEN_WIDTH,                        // width, in pixels
        SCREEN_HEIGHT,                       // height, in pixels
        SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL // flags
    );
    if (win == NULL) 
    {
        log_SDL_error("SDL_CreateWindow");
    }
    screen_sdl->win = win;

    // renderer
    ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE | SDL_RENDERER_TARGETTEXTURE);

    // the next line is faster, but it causes problems on some computers
    //ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_TARGETTEXTURE); 
    if (ren == NULL)
    {
		log_SDL_error("SDL_CreateRenderer");
	}
	SDL_RenderClear(ren);
	screen_sdl->ren = ren;

	// TTF (SDL text)
	if (TTF_Init() != 0)
	{
		log_SDL_error("TTF_Init");
	}

	// font for thes axes
	screen_sdl->font_axis = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/HunDIN1451.ttf", FONT_AXIS_SIZE);

	// font for the labels
	screen_sdl->font_label = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/MonospaceTypewriter.ttf", FONT_LABEL_SIZE);

	// font for the small labels
	screen_sdl->font_small_label = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/MonospaceTypewriter.ttf", FONT_SMALL_LABEL_SIZE);

	// font for the scaling
	screen_sdl->font_scaling = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/arrow_7.ttf", FONT_SCALING);

	// font for the arrows
	screen_sdl->font_arrows = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/Arrows.ttf", FONT_ARROWS);

	// font for the play icon
	screen_sdl->font_play = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/Arrows_tbf.ttf", FONT_PLAY_break);

	// font for the break icon
	screen_sdl->font_break = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/PIZZADUDEBULLETS.ttf", FONT_PLAY_break);

	// font for the speed of the simulation
	screen_sdl->font_speed = init_font(PROJECT_ABS_PATH"/src/generic/sdl/SDL_files/Arrows.ttf", FONT_SPEED);


	// text color: black
	text_color.r = 0;
	text_color.g = 0;
	text_color.b = 0;
	text_color.a = 0;
	screen_sdl->text_color = text_color;

	// index of the sdl window and vectors
	screen_sdl->index_vec = 0;
	screen_sdl->index_sdl = 0;

	// index of the simulation vectors
	screen_sdl->index_simu = -1;

	// real-time
	screen_sdl->last_t_usec = t_usec(init_t_sec, init_t_usec);

	// time for a pixel
	screen_sdl->tsim_pixel = 1.0 / fqc_screen;

	// x axis bounds
	screen_sdl->x_min = 0.0;
	screen_sdl->x_max = 0.0;

	// y axis bounds
	screen_sdl->y_min = y_min_init;
	screen_sdl->y_max = y_max_init;

	screen_sdl->last_y_min = screen_sdl->y_min;
	screen_sdl->last_y_max = screen_sdl->y_max;

	screen_sdl->last_legend_y_min = screen_sdl->y_min;
	screen_sdl->last_legend_y_max = screen_sdl->y_min;

	// x label
	screen_sdl->last_x_label_flag = 0;

	// scaling
	screen_sdl->scaling_flag      = 0;
	screen_sdl->last_scaling_flag = 0;

	// break
	screen_sdl->break_plot_flag = 0;

	// string exponents
	screen_sdl->expo_y = 0;
	screen_sdl->expo_x = 0;

	// flag: horizontal plot
	screen_sdl->hor_plot_scaling = 0;

	// flag: bottom indications
	screen_sdl->bottom_flag = 1;

	// flags: modify y axis
	screen_sdl->increase_plot_y_diff_flag = 0;
	screen_sdl->decrease_plot_y_diff_flag = 0;
	screen_sdl->up_plot_y_flag = 0;
	screen_sdl->bottom_plot_y_flag = 0;

	// flags: modify x axis
	screen_sdl->increase_plot_x_diff_flag = 0;
	screen_sdl->decrease_plot_x_diff_flag = 0;
	screen_sdl->left_plot_y_flag = 0;
	screen_sdl->right_plot_y_flag = 0;

	// number of curves
	screen_sdl->nb_curves = nb_curves;

	// full y vectors
	if (nb_curves)
	{
		screen_sdl->y_vectors = (double**) malloc(nb_curves * sizeof(double*));
	    for (i=0; i<nb_curves; i++)
	    {
	    	screen_sdl->y_vectors[i] = (double*) malloc(nb_steps * sizeof(double));
	    }


	    screen_sdl->previous_y_vec = (double*) malloc(nb_curves * sizeof(double));

	    // y values tab for SDL
		screen_sdl->y_tab = (double**) malloc(nb_curves * sizeof(double*));
		for (i=0; i<nb_curves; i++)
		{
			screen_sdl->y_tab[i] = (double*) malloc(SCREEN_PLOT_WIDTH * sizeof(double));
		}

		// minimal and maximal values (for auto-scaling)
		screen_sdl->y_tab_min = (double*) malloc(nb_curves * sizeof(double));
		screen_sdl->y_tab_max = (double*) malloc(nb_curves * sizeof(double));

		screen_sdl->scaling_expo_mult = (int*) malloc(nb_curves*sizeof(int));
		screen_sdl->last_scaling_expo_mult = (int*) malloc(nb_curves*sizeof(int));

	    screen_sdl->scaling_offset  = (double*) malloc(nb_curves*sizeof(double));
	    screen_sdl->last_scaling_offset  = (double*) malloc(nb_curves*sizeof(double));
		
		for(i=0; i<nb_curves; i++)
		{
			screen_sdl->y_tab_min[i] = START_VECTOR;
			screen_sdl->y_tab_max[i] = START_VECTOR;

			screen_sdl->scaling_expo_mult[i]      = 0;
			screen_sdl->last_scaling_expo_mult[i] = 0;

			screen_sdl->scaling_offset[i]      = 0.0;
			screen_sdl->last_scaling_offset[i] = 0.0;
		}
	}

    // full time vector
    screen_sdl->tsim_vec = (double*) malloc(nb_steps * sizeof(double));

	// time vector for SDL
	screen_sdl->t_vec = (double*) malloc(SCREEN_PLOT_WIDTH * sizeof(double));
	for (i=0; i<SCREEN_PLOT_WIDTH; i++)
	{
		screen_sdl->t_vec[i] = 0.0;
	}

	// colors
	for (i=0; i<NB_CURVES_MAX; i++)
	{
		screen_sdl->color_vec[i] = color_vec[i];
	}	

	screen_sdl->last_color = -1;

	// auto-scaling
	screen_sdl->plot_auto_scaling        = 0;
	screen_sdl->signal_auto_scaling      = 0;
	screen_sdl->last_plot_auto_scaling   = 0;
	screen_sdl->last_signal_auto_scaling = 0;

	// legend curves safety
	if (nb_legend_curves > 5)
	{
		nb_legend_curves = 5;
	}

	if (nb_legend_curves > nb_curves)
	{
		nb_legend_curves = nb_curves;
	}

	screen_sdl->nb_legend_curves = nb_legend_curves;

	// position of the labels
	if (nb_legend_curves)
	{
		screen_sdl->pixel_pos_label = (int*) malloc(nb_legend_curves * sizeof(int));

		space_up_label = (SCREEN_WIDTH - UP_COLOR_WIDTH_LOSS_BEGIN - UP_COLOR_WIDTH_LOSS_END) / nb_legend_curves;

		for (i=0; i<nb_legend_curves; i++)
		{
			screen_sdl->pixel_pos_label[i] = UP_COLOR_WIDTH_LOSS_BEGIN + i* space_up_label;
		}
	}
		

	// label tab
	screen_sdl->label_tab = label_tab;

	// clean the screen for the first time
	refresh_screen(screen_sdl);

	// return structure
	return screen_sdl;
}

/*
 * Initialize a new Font variable (pointer)
 */
TTF_Font* init_font(char *folder, int font_size)
{
	TTF_Font* font;

	font = TTF_OpenFont(folder, font_size);

	if (font == NULL)
	{
		log_SDL_error("TTF_OpenFont");
	}

	return font;
}

/*
 * Free the main SDL structure
 */
void free_screen_sdl(Screen_sdl *screen_sdl)
{
	// variable declaration
	int i;

	// fonts
	TTF_CloseFont(screen_sdl->font_label);
	TTF_CloseFont(screen_sdl->font_small_label);
	TTF_CloseFont(screen_sdl->font_axis);
	TTF_CloseFont(screen_sdl->font_scaling);
	TTF_CloseFont(screen_sdl->font_arrows);
	TTF_CloseFont(screen_sdl->font_play);
	TTF_CloseFont(screen_sdl->font_break);
	TTF_CloseFont(screen_sdl->font_speed);

	// SDL main variables
	SDL_DestroyRenderer(screen_sdl->ren);
	SDL_DestroyWindow(screen_sdl->win);
    SDL_Quit();

    free(screen_sdl->tsim_vec);	
	free(screen_sdl->t_vec);
	free_char_tab(screen_sdl->label_tab, NB_CURVES_MAX);

    if (screen_sdl->nb_curves)
    {
    	for (i=0; i<screen_sdl->nb_curves; i++)
		{
			free(screen_sdl->y_tab[i]);
			free(screen_sdl->y_vectors[i]);
		}
		free(screen_sdl->y_tab);
		free(screen_sdl->y_vectors);
		free(screen_sdl->previous_y_vec);
		free(screen_sdl->y_tab_min);
		free(screen_sdl->y_tab_max);
		free(screen_sdl->scaling_expo_mult);
		free(screen_sdl->last_scaling_expo_mult);
	    free(screen_sdl->scaling_offset);
	    free(screen_sdl->last_scaling_offset);
    }

    if (screen_sdl->nb_legend_curves)
	{
		free(screen_sdl->pixel_pos_label);
	}

	free(screen_sdl);
}

/*
 * Show an error message and exit the simulation
 */
void log_SDL_error(char *SDL_function)
{
	printf("%s error: %s\n",SDL_function, SDL_GetError());
	exit(1);
}

/*
 * Change the color in SDL
 */
void SDL_set_color(Screen_sdl * screen_sdl, int color_num)
{
	// variable decalration
	SDL_Renderer *ren;
	
	// no change in the color since previous time
	if (screen_sdl->last_color == color_num)
	{
		return;
	}

	screen_sdl->last_color = color_num;

	ren = screen_sdl->ren;

	// select the good color
	switch (color_num)
	{
		case WHITE_SDL:
			SDL_SetRenderDrawColor(ren, 255, 255, 255, 0);
			break;

		case BLACK_SDL:
			SDL_SetRenderDrawColor(ren, 0, 0, 0, 1);
			break;

		case LIGHT_GRAY_SDL:
			SDL_SetRenderDrawColor(ren, 220, 212, 212, 0);
			break;

		case BLUE_SDL:
			SDL_SetRenderDrawColor(ren, 0, 0, 255, 0);
			break;

		case RED_SDL:
			SDL_SetRenderDrawColor(ren, 255, 0, 0, 0);
			
			break;

		case DARK_GREEN_SDL:
			SDL_SetRenderDrawColor(ren, 33, 75, 31, 0);
			break;

		case PURPLE_SDL:
			SDL_SetRenderDrawColor(ren, 131, 15, 246, 0);
			break;

		case TURQUOISE_SDL:
			SDL_SetRenderDrawColor(ren, 15, 246, 238, 0);
			break;

		case ORANGE_SDL:
			SDL_SetRenderDrawColor(ren, 243, 134, 26, 0);
			break;

		case LIGHT_BLUE_SDL:
			SDL_SetRenderDrawColor(ren, 15, 146, 246, 0);
			break;

		case PINK_SDL:
			SDL_SetRenderDrawColor(ren, 246, 15, 215, 0);
			break;	    	    

		case LIGHT_GREEN_SDL:
			SDL_SetRenderDrawColor(ren, 23, 246, 15, 0);
			break;

		case DARK_YELLOW_SDL:
			SDL_SetRenderDrawColor(ren, 175, 165, 27, 0);
			break;

		case YELLOW_SDL:
			SDL_SetRenderDrawColor(ren, 239, 225, 36, 0);
			break;

		case GREEN_SDL:
			SDL_SetRenderDrawColor(ren, 0, 255, 0, 0);
			break;
	
		default:
			break;
	}
}

/*
 * Refresh the screen with the white color
 */
void refresh_screen(Screen_sdl *screen_sdl)
{
	SDL_set_color(screen_sdl, WHITE_SDL);
	SDL_RenderClear(screen_sdl->ren);
}

/*
 * Draw a pixel, corrdinates: i, j
 * For some mysterious reasons, using the 'SDL_RenderFillRect' 
 * function with 1x1 rectangles is 2 times faster than using 
 * the 'SDL_RenderDrawPoint' function
 */
void draw_pixel(SDL_Renderer *ren, int i, int j)
{
	SDL_Rect rectangle;
	
	rectangle.x = i;
	rectangle.y = j;
	rectangle.w = 1;
	rectangle.h = 1;
	SDL_RenderFillRect(ren, &rectangle);
}

/*
 * Draw a rectangle (x axis: [x1 ; x2] , y axis: [y1 ; y2])
 * x1 <= x2
 * y1 <= y2
 */
void draw_rectangle(SDL_Renderer *ren, int x1, int x2, int y1, int y2)
{
	SDL_Rect rect;

	rect.x = x1;
	rect.y = y1;
	rect.w = x2 - x1 + 1;
	rect.h = y2 - y1 + 1;
	SDL_RenderFillRect(ren, &rect);
}

/*
 * Draw a horizontal line (x axis: [x1 ; x2] , y axis: y)
 * x1 <= x2
 */
void draw_horizontal_line(SDL_Renderer *ren, int x1, int x2, int y)
{
	SDL_RenderDrawLine(ren, x1, y, x2, y);
}

/*
 * Draw a vertical line (x axis: x , y axis: [y1 ; y2])
 * y1 <= y2
 */
void draw_vertical_line(SDL_Renderer *ren, int x, int y1, int y2)
{
	SDL_RenderDrawLine(ren, x, y1, x, y2);
}

/*
 * Print a text
 */
void print_text_sdl(char *str, TTF_Font *font, SDL_Color txt_col, SDL_Renderer *ren, int x, int y, int pos_flag)
{
	SDL_Texture *text_texture;
	
	text_texture = renderText(font, str, txt_col, ren);
	renderTexture(text_texture, ren, x, y, pos_flag);
	SDL_DestroyTexture(text_texture);
}

/*
 * Apply a texture at a given corrdinate point
 */
void renderTexture(SDL_Texture *tex, SDL_Renderer *ren, int x, int y, int pos_flag)
{
	//Setup the destination rectangle to be at the position we want
	SDL_Rect dst;
	SDL_Rect clip;

	//Query the texture to get its width and height to use
	SDL_QueryTexture(tex, NULL, NULL, &dst.w, &dst.h);	

	clip.x = 0;
	clip.y = 0;

	clip.w = dst.w;
	clip.h = dst.h;

	switch (pos_flag)
	{
		case 1: // position: right-bottom corner
			dst.x = x - dst.w;
			dst.y = y - dst.h;
			break;

		case 2: // position: left-mid height
			dst.x = x;
			dst.y = y - (dst.h / 2);
			break;
	
		default:
			dst.x = x;
			dst.y = y;
			break;
	}

	SDL_RenderCopy(ren, tex,  &clip, &dst);
}

/*
 * Create a texture for a message
 */
SDL_Texture* renderText(TTF_Font *font, char *message, SDL_Color text_color, SDL_Renderer *ren)
{
	SDL_Surface *text_surf;
	SDL_Texture *text_texture;

	text_surf = TTF_RenderText_Solid(font, message, text_color);
	if (text_surf == NULL)
	{
		log_SDL_error("TTF_RenderText_Solid");
	}

	text_texture = SDL_CreateTextureFromSurface(ren, text_surf);

	if (text_texture == NULL)
	{
		log_SDL_error("SDL_CreateTextureFromSurface");
	}

	SDL_FreeSurface(text_surf);

	return text_texture;
}

/*
 * Update the vectors and tabs used for the plots
 * along with the min-max vectors used for the auto-scaling
 */
void update_plot_vectors(Screen_sdl *screen_sdl, Simu_real_time *real_time, double t, double *y_vec)
{
	screen_sdl->index_vec++;

	if (screen_sdl->index_vec >= SCREEN_PLOT_WIDTH)
	{
		screen_sdl->index_vec -= SCREEN_PLOT_WIDTH;
	}

	screen_sdl->t_vec[screen_sdl->index_vec] = t;
	
	update_y_min_max(screen_sdl, y_vec);

	update_scale_signals(screen_sdl, real_time, 0);
}

void update_y_tab(Screen_sdl *screen_sdl)
{
	// -- Variables declaration -- //

	int i, j;
	int nb_curves;
	int index_simu, guess_index;
	int prev_index, next_index;

	double cur_tsim;
	double weight_prev;
	double x_min, x_max, diff_x, step_x;
	double request_tsim;
	double tsim_prev, tsim_next;

	double *tsim_vec;
    double **y_vectors;
	double **y_tab;
	double *t_vec;


	// -- Variables initialization -- //

	nb_curves  = screen_sdl->nb_curves;

	if (!nb_curves)
	{
		return;
	}

	tsim_vec   = screen_sdl->tsim_vec;
	y_vectors  = screen_sdl->y_vectors;
	index_simu = screen_sdl->index_simu;

	x_min = screen_sdl->x_min;
	x_max = screen_sdl->x_max;

	y_tab = screen_sdl->y_tab;
	t_vec = screen_sdl->t_vec;

	cur_tsim  = tsim_vec[index_simu];

	if (!cur_tsim)
	{
		return;
	}

	// security
	if (x_min < 0.0)
	{
		x_min = 0.0;
	}

	// step in the x axis for each pixel
	diff_x = x_max - x_min;
	step_x = diff_x / SCREEN_PLOT_WIDTH;

	// tsim value expected
	request_tsim = x_min + (step_x / 2.0);

	// loop on all the pixels
	for(i=0; i<SCREEN_PLOT_WIDTH; i++)
	{
		#ifdef WIN32
		guess_index = (int) ( (request_tsim / cur_tsim) * index_simu);
		#else
		guess_index = (int) round( (request_tsim / cur_tsim) * index_simu);
		#endif

		if (request_tsim <= tsim_vec[guess_index-1])
		{
			prev_index  = guess_index-1;
			next_index  = prev_index;
		}
		else if (request_tsim >= tsim_vec[guess_index+1])
		{
			prev_index  = guess_index+1;
			next_index  = prev_index;
		}
		else if (request_tsim <= tsim_vec[guess_index])
		{
			prev_index = guess_index-1;
			next_index = guess_index;
		}
		else
		{
			prev_index = guess_index;
			next_index = guess_index+1;
		}

		if (prev_index > index_simu)
		{
			prev_index = index_simu;
		}

		if (next_index > index_simu)
		{
			next_index = index_simu;
		}
		
		// time
		tsim_prev = tsim_vec[prev_index];
		tsim_next = tsim_vec[next_index];
		
		// weight for the linear interpolation
		weight_prev = (tsim_next - request_tsim) / (tsim_next - tsim_prev);

		// loop on all the curves
		for(j=0; j<nb_curves; j++)
		{
			y_tab[j][i] = weight_prev * y_vectors[j][prev_index] + (1 - weight_prev) * y_vectors[j][next_index];
		}

		// time vector
		t_vec[i] = weight_prev * tsim_prev + (1 - weight_prev) * tsim_next;

		// new time value
		request_tsim += step_x;
	}

	// update the index for SDL
	screen_sdl->index_vec = SCREEN_PLOT_WIDTH - 1;
}

/*
 * Compute the minimal and maximal times at the beginning and at the end of a break
 */
void update_x_min_max(Screen_sdl *screen_sdl, Simu_real_time *real_time)
{
	double cur_tsim;

	cur_tsim = screen_sdl->tsim_vec[screen_sdl->index_simu];

	screen_sdl->x_max = cur_tsim;
	screen_sdl->x_min = cur_tsim - ( screen_sdl->tsim_pixel * SCREEN_PLOT_WIDTH * get_simu_speed_factor(real_time->simu_speed_flag) );

	limit_x_limits(screen_sdl);
}

/*
 * Update the minimal and maximal value of each signal
 */
void update_y_min_max(Screen_sdl *screen_sdl, double *y_vec)
{
	// variables declaration
	int i;

	double *y_tab_min;
    double *y_tab_max;

    if (!screen_sdl->nb_curves)
    {
    	return;
    }

    // tabs
	y_tab_min = screen_sdl->y_tab_min;
    y_tab_max = screen_sdl->y_tab_max;

    // loop on the different curves
	for(i=0; i<screen_sdl->nb_curves; i++)
	{
		if ((y_vec[i] < y_tab_min[i]) || (y_tab_min[i] == START_VECTOR))
		{
			y_tab_min[i] = y_vec[i];
		}

		if ((y_vec[i] > y_tab_max[i]) || (y_tab_max[i] == START_VECTOR))
		{
			y_tab_max[i] = y_vec[i];
		}
	}
}

/*
 * Update the vectors with the full information (for each time step of the simulation)
 */
void update_full_vectors(Screen_sdl *screen_sdl, double tsim, double *y_vec)
{
	int i;
	int index_simu;

    double **y_vectors;

    if (!screen_sdl->nb_curves)
    {
    	return;
    }

    index_simu = screen_sdl->index_simu;

    y_vectors = screen_sdl->y_vectors;

    index_simu++;

    screen_sdl->tsim_vec[index_simu] = tsim;

    // update the vector
    for (i=0; i<screen_sdl->nb_curves; i++)
    {
    	y_vectors[i][index_simu] = y_vec[i];
    }

    // special case for the first iteration
    if (!index_simu)
    {
    	for (i=0; i<screen_sdl->nb_curves; i++)
    	{
    		screen_sdl->previous_y_vec[i] = y_vec[i];
    	}    	
    }
    screen_sdl->index_simu = index_simu;
}

/*
 * Scaling of the signals
 */
void update_scale_signals(Screen_sdl *screen_sdl, Simu_real_time *real_time, int screen_flag)
{
	// -- Variables declaration -- //

	int i;
	int nb_curves;
    int cur_scaling_expo_mult;

	double y_min_A, y_max_A;
	double cur_y_min, cur_y_max;
	double mean_A, semi_diff_A;
	double cur_mean, cur_semi_diff;
	double min_plot_A, max_plot_A;
	double cur_fact_semi_diff, cur_scale_fact_semi_diff;
    double cur_scale_y_min, cur_scale_y_max, cur_scale_semi_diff;
    double y_scale_super_min, y_scale_super_max;
    double y_min_plot, y_max_plot, diff_y_plot;
    double x_min_plot, x_max_plot, diff_x_plot;
    double diff_y_scale_super;    

    int *scaling_expo_mult;

    double *y_tab_min;
    double *y_tab_max;
    double *scaling_offset;


    // -- Variables initialization -- //

    nb_curves = screen_sdl->nb_curves;

    if (!nb_curves)
    {
    	return;
    }

    y_tab_min = screen_sdl->y_tab_min;
    y_tab_max = screen_sdl->y_tab_max;

    scaling_expo_mult = screen_sdl->scaling_expo_mult;
    scaling_offset    = screen_sdl->scaling_offset;

    // A: first signal
    y_min_A = y_tab_min[0];
    y_max_A = y_tab_max[0];

    mean_A      = 0.5 * (y_min_A + y_max_A);
    semi_diff_A = 0.5 * (y_max_A - y_min_A);

    // safety
    if (!semi_diff_A)
    {
    	semi_diff_A = 0.5;
    }

    // limits for the signal auto-scaling
    min_plot_A = mean_A - PLOT_SCALE_MAX * semi_diff_A;
    max_plot_A = mean_A + PLOT_SCALE_MAX * semi_diff_A;

    // reset during the break
    if (real_time->simu_break)
    {
    	for(i=0; i<nb_curves; i++)
    	{
    		scaling_expo_mult[i] = 0;
    		scaling_offset[i]    = 0.0;
    	}
    }


    // -- Scaling curves -- //

    if (screen_sdl->signal_auto_scaling)
    {
    	// loop on the curves
    	for(i=1; i<nb_curves; i++)
	    {
	    	cur_y_min = y_tab_min[i];
	    	cur_y_max = y_tab_max[i];

	    	// current scaled signal
	    	cur_scale_y_min = scale_signal(cur_y_min, scaling_expo_mult[i], scaling_offset[i]);
	    	cur_scale_y_max = scale_signal(cur_y_max, scaling_expo_mult[i], scaling_offset[i]);

	    	cur_semi_diff      = 0.5 * (cur_y_max - cur_y_min);
			cur_fact_semi_diff = fabs(cur_semi_diff / semi_diff_A);

			cur_scale_semi_diff      = 0.5 * (cur_scale_y_max - cur_scale_y_min);
			cur_scale_fact_semi_diff = fabs(cur_scale_semi_diff / semi_diff_A);

			// Scaling factor

			if ( (cur_scale_fact_semi_diff < (1.0/SCALING_MULT_THRESHOLD)) || (cur_scale_fact_semi_diff > SCALING_MULT_THRESHOLD) )
	    	{
	    		cur_scaling_expo_mult = 0;

		    	if (cur_fact_semi_diff)
		    	{
		    		while (cur_fact_semi_diff > SCALING_MULT_THRESHOLD)
			    	{
			    		cur_scaling_expo_mult++;
			    		cur_fact_semi_diff *= 0.1;
			    	}

			    	while (cur_fact_semi_diff < (1.0 / SCALING_MULT_THRESHOLD) )
			    	{
			    		cur_scaling_expo_mult--;
			    		cur_fact_semi_diff *= 10.0;
			    	}
		    	}

		    	scaling_expo_mult[i] = cur_scaling_expo_mult;

		    	scaling_offset[i] = 0.0;

		    	cur_scale_y_min = scale_signal(cur_y_min, cur_scaling_expo_mult, scaling_offset[i]);
				cur_scale_y_max = scale_signal(cur_y_max, cur_scaling_expo_mult, scaling_offset[i]);

				screen_sdl->scaling_flag = 1;
		    }
		    else
		    {
		    	cur_scaling_expo_mult = scaling_expo_mult[i];
		    }

		    // Scaling offset

	    	if ( (cur_scale_y_min < min_plot_A) || (cur_scale_y_max > max_plot_A))
	    	{
	    		cur_mean = 0.5 * (cur_y_min + cur_y_max);

			    if ( (cur_scale_y_min < min_plot_A) || (cur_scale_y_max > max_plot_A))
			    {
			    	scaling_offset[i] = cur_mean - pow_int(10.0, cur_scaling_expo_mult) * mean_A;
			    }
			    screen_sdl->scaling_flag = 1;
			}
	    }
    }
    // no auto-scaling
    else
    {
    	for(i=0; i<nb_curves; i++)
    	{
    		if (scaling_expo_mult[i] != 0)
    		{
    			scaling_expo_mult[i]     = 0;
    			screen_sdl->scaling_flag = 1;
    		}
    		if (scaling_offset[i] != 0.0)
    		{
    			scaling_offset[i]        = 0.0;
    			screen_sdl->scaling_flag = 1;
    		}
    	}
    }

    
    // -- Scaling Plot limits -- //

    // y axis
    y_min_plot  = screen_sdl->y_min;
	y_max_plot  = screen_sdl->y_max;
	diff_y_plot = y_max_plot - y_min_plot;

	// x axis
	x_min_plot  = screen_sdl->x_min;
	x_max_plot  = screen_sdl->x_max;
	diff_x_plot = x_max_plot - x_min_plot;

	// auto scaling: y axis
    if (screen_sdl->plot_auto_scaling)
    {
    	y_scale_super_min = y_min_A;
	    y_scale_super_max = y_max_A;

	    for(i=1; i<nb_curves; i++)
	    {
	    	cur_scale_y_min = scale_signal(y_tab_min[i], scaling_expo_mult[i], scaling_offset[i]);
	    	cur_scale_y_max = scale_signal(y_tab_max[i], scaling_expo_mult[i], scaling_offset[i]);

	    	if (cur_scale_y_min < y_scale_super_min)
	    	{
	    		y_scale_super_min = cur_scale_y_min;
	    	}

	    	if (cur_scale_y_max > y_scale_super_max)
	    	{
	    		y_scale_super_max = cur_scale_y_max;
	    	}
	    }

	    if (screen_flag || (y_scale_super_min < y_min_plot + SCALING_PLOT_TOO_SMALL * diff_y_plot) ||
	    	 (y_scale_super_max > y_max_plot - SCALING_PLOT_TOO_SMALL * diff_y_plot))
	    {
	    	diff_y_scale_super = y_scale_super_max - y_scale_super_min;

	    	screen_sdl->y_min = y_scale_super_min - SCALING_PLOT_OPTIMAL_INCREASE * diff_y_scale_super;
	    	screen_sdl->y_max = y_scale_super_max + SCALING_PLOT_OPTIMAL_INCREASE * diff_y_scale_super;

	    	screen_sdl->scaling_flag = 1;
	    }
	    else if ((y_scale_super_min > y_min_plot + SCALING_PLOT_TOO_HIGH  * diff_y_plot) ||
	    	     (y_scale_super_max < y_max_plot - SCALING_PLOT_TOO_HIGH  * diff_y_plot) )
	    {
	    	diff_y_scale_super = y_scale_super_max - y_scale_super_min;

	    	screen_sdl->y_min = y_scale_super_min - SCALING_PLOT_OPTIMAL_DECREASE * diff_y_scale_super;
	    	screen_sdl->y_max = y_scale_super_max + SCALING_PLOT_OPTIMAL_DECREASE * diff_y_scale_super;

	    	screen_sdl->scaling_flag = 1;    	
	    }
    }

    // no auto-scaling: y axis
    else if (screen_sdl->increase_plot_y_diff_flag)
    {
    	screen_sdl->increase_plot_y_diff_flag = 0;

    	screen_sdl->y_min = y_min_plot - MANUAL_SCALING_PLOT * diff_y_plot;
    	screen_sdl->y_max = y_max_plot + MANUAL_SCALING_PLOT * diff_y_plot;

    	screen_sdl->scaling_flag = 1;
    }
    else if (screen_sdl->decrease_plot_y_diff_flag)
    {
    	screen_sdl->decrease_plot_y_diff_flag = 0;

    	screen_sdl->y_min = y_min_plot + 0.5 * MANUAL_SCALING_PLOT * diff_y_plot;
    	screen_sdl->y_max = y_max_plot - 0.5 * MANUAL_SCALING_PLOT * diff_y_plot;

    	screen_sdl->scaling_flag = 1;
    }
    else if (screen_sdl->up_plot_y_flag)
    {
    	screen_sdl->up_plot_y_flag = 0;

    	screen_sdl->y_min = y_min_plot + MANUAL_OFFSET_PLOT * diff_y_plot;
    	screen_sdl->y_max = y_max_plot + MANUAL_OFFSET_PLOT * diff_y_plot;

    	screen_sdl->scaling_flag = 1;
    }
    else if (screen_sdl->bottom_plot_y_flag)
    {
    	screen_sdl->bottom_plot_y_flag = 0;

    	screen_sdl->y_min = y_min_plot - MANUAL_OFFSET_PLOT * diff_y_plot;
    	screen_sdl->y_max = y_max_plot - MANUAL_OFFSET_PLOT * diff_y_plot;

    	screen_sdl->scaling_flag = 1;
    }

    // x axis
    if (screen_sdl->increase_plot_x_diff_flag)
    {
    	screen_sdl->increase_plot_x_diff_flag = 0;

    	screen_sdl->x_min = x_min_plot - MANUAL_SCALING_PLOT * diff_x_plot;
    	screen_sdl->x_max = x_max_plot + MANUAL_SCALING_PLOT * diff_x_plot;

    	limit_x_limits(screen_sdl);

    }
    else if (screen_sdl->decrease_plot_x_diff_flag)
    {
    	screen_sdl->decrease_plot_x_diff_flag = 0;

    	screen_sdl->x_min = x_min_plot + 0.5 * MANUAL_SCALING_PLOT * diff_x_plot;
    	screen_sdl->x_max = x_max_plot - 0.5 * MANUAL_SCALING_PLOT * diff_x_plot;

    	limit_x_limits(screen_sdl);
    }
    else if (screen_sdl->right_plot_y_flag)
    {
    	screen_sdl->right_plot_y_flag = 0;

    	screen_sdl->x_min = x_min_plot + MANUAL_OFFSET_PLOT * diff_x_plot;
    	screen_sdl->x_max = x_max_plot + MANUAL_OFFSET_PLOT * diff_x_plot;

    	limit_x_limits(screen_sdl);
    }
    else if (screen_sdl->left_plot_y_flag)
    {
    	screen_sdl->left_plot_y_flag = 0;

    	screen_sdl->x_min = x_min_plot - MANUAL_OFFSET_PLOT * diff_x_plot;
    	screen_sdl->x_max = x_max_plot - MANUAL_OFFSET_PLOT * diff_x_plot;

    	limit_x_limits(screen_sdl);
    }
}

/*
 * Limit the x axis minimal and maximal values
 */
void limit_x_limits(Screen_sdl *screen_sdl)
{
	double cur_tsim;

	cur_tsim = screen_sdl->tsim_vec[screen_sdl->index_simu];

	if (screen_sdl->x_max > cur_tsim)
	{
		screen_sdl->x_max = cur_tsim;
	}

	if (screen_sdl->x_min < 0.0)
	{
		screen_sdl->x_min = 0.0;
	}

}

/*
 * Return the value 'no_scale_nb' after scaling
 */
double scale_signal(double no_scale_nb, int scaling_expo_mult, double scaling_offset)
{
	double scale_nb;

	scale_nb = no_scale_nb - scaling_offset;

	if (scaling_expo_mult)
	{
		scale_nb *= ( 1.0 / pow_int(10.0, scaling_expo_mult) );
	}

	return scale_nb;
}

/*
 * Return the value 'scale_nb' before scaling
 */
double unscale_signal(double scale_nb, int scaling_expo_mult, double scaling_offset)
{
	return (scale_nb * pow_int(10.0, scaling_expo_mult)) + scaling_offset;
}


/*
 * Create a string to represent the value of the real signal (no scaled)
 */
void str_scaling(char *str, int scaling_expo_mult, double scaling_offset)
{
	// variables decalration
	int exp_inf;
	char str_offset[10];

	// offset present
	if (scaling_offset)
	{
		exp_inf = expo_ten_inf(scaling_offset);

		if ((exp_inf > 2) || (exp_inf < -1))
		{
			sprintf(str_offset,"%.1fe%d", fabs(scaling_offset / pow_int(10.0, exp_inf)) , exp_inf );
		}
		else
		{
			if (exp_inf < 0)
			{
				sprintf(str_offset,"%.3f", scaling_offset);
			}
			else
			{
				sprintf(str_offset,"%.2f", scaling_offset);
			}
		}
	}
	
	// scaling with multiplication
	if (scaling_expo_mult)
	{
		if (scaling_offset)
		{
			if (scaling_offset > 0.0)
			{
				sprintf(str, "(1e%d x + %s)", scaling_expo_mult, str_offset);
			}
			else
			{
				sprintf(str, "(1e%d x - %s)", scaling_expo_mult, str_offset);
			}
		}
		else
		{
			sprintf(str, "(1e%d x)", scaling_expo_mult);
		}
	}
	// no scaling with multiplication
	else
	{
		if (scaling_offset)
		{
			
			if (scaling_offset > 0.0)
			{
				sprintf(str, "(+ %s)", str_offset);
			}
			else
			{
				sprintf(str, "(- %s)", str_offset);
			}
		}
		else
		{
			sprintf(str, " ");
		}
	}
}

/*
 * Create a string representing bounds
 * Return the exponent (base 10) of these bounds used for the string
 */
int str_label(char *str, double low_nb, double high_nb)
{
	// variables declaration
	int exp_inf;
	double diff_nb;
	double div_exp;

	// string computation
	diff_nb = fabs(high_nb - low_nb);

	exp_inf  = expo_ten_inf(diff_nb);

	div_exp = pow_int(10.0, exp_inf);

	if ((exp_inf > 2) || (exp_inf < -1))
	{
		sprintf(str,"[%.2f ; %.2f]e%d", low_nb / div_exp , high_nb / div_exp, exp_inf);
	}
	else
	{
		sprintf(str,"[%.2f ; %.2f]", low_nb, high_nb );
	}

	return exp_inf;
}

/*
 * Create a string representing a number
 */
void str_axis(char *str, double nb, int expo)
{
	double div_exp;

	div_exp = pow_int(10.0, expo);

	if ((expo > 2) || (expo < -1))
	{
		sprintf(str,"%.2f", nb / div_exp);
	}
	else
	{
		sprintf(str,"%.2f", nb);
	}
}

/*
 * Main function used to plot signals along with the corresponding indications
 */
void plot_screen_sdl(Screen_sdl *screen_sdl, Simu_real_time *real_time, double tsim, int screen_flag)
{
	//  -- Variables declaration -- //

	int i, j, k;
	int found_y_value;
	int nb_curves_sdl;
	int index_simu;
	int this_y_pixel;
	int init_x_index, end_x_index;
	int y_pixel_nul_axis;
	int index_vec, index_sdl, next_index_sdl;
	int second_next_index_sdl, erase_index_sdl;
	int y_pixel_label;
	int x_end_label_plot;
	int x_pixel_label;
	int ind_vec;
	int pixel_pos_label;
	int nb_legend_curves;
	
	double pixel_y_min, pixel_y_max;
	double pixel_height;
	double y_min_sdl, y_max_sdl;
	double prev_diff_y_sdl;
	double prev_y_diff_label;
	double prev_y_value;
	double this_y, prev_y;
	double this_y0, this_y1;
	double diff_y_sdl;
	double alpha_nul_axis;
	double y_value;
	double y_diff_label;
	double tsim_end;
	double tsim_x_label;
	double legend_x_min, legend_x_max;
	double tsim_x_label_minus_1, tsim_x_label_minus_2;
	double speed_factor_plot;

	char str[80];

	int *color_vec;
	int *scaling_expo_mult;
	int *last_scaling_expo_mult;

    double *previous_y_vec;
    double **y_vectors;
    double **y_tab;
	double *t_vec;
	double *scaling_offset;
	double *last_scaling_offset;

	SDL_Color text_color;

	SDL_Renderer *ren;

	TTF_Font *font_label;
	TTF_Font *font_small_label;
	TTF_Font *font_axis;
	TTF_Font *font_scaling;
	TTF_Font *font_arrows;
	TTF_Font *font_play;
	TTF_Font *font_break;
	TTF_Font *font_speed;


	// -- Variables initialization -- //

    if (screen_flag==1)
    {
    	update_y_tab(screen_sdl);
    }

	t_vec             = screen_sdl->t_vec;
	y_tab             = screen_sdl->y_tab;
	ren               = screen_sdl->ren;
	color_vec         = screen_sdl->color_vec;
	font_label        = screen_sdl->font_label;
	font_small_label  = screen_sdl->font_small_label;	
	font_axis         = screen_sdl->font_axis;
	font_scaling      = screen_sdl->font_scaling;
	font_arrows       = screen_sdl->font_arrows;
	font_play         = screen_sdl->font_play;
	font_break        = screen_sdl->font_break;
	font_speed        = screen_sdl->font_speed;
	text_color        = screen_sdl->text_color;
	scaling_expo_mult = screen_sdl->scaling_expo_mult;
    scaling_offset    = screen_sdl->scaling_offset;
    index_simu        = screen_sdl->index_simu;
    previous_y_vec    = screen_sdl->previous_y_vec;
    y_vectors         = screen_sdl->y_vectors;
	index_vec         = screen_sdl->index_vec;
	index_sdl         = screen_sdl->index_sdl;
	nb_curves_sdl     = screen_sdl->nb_curves;
    y_min_sdl         = screen_sdl->y_min;
	y_max_sdl         = screen_sdl->y_max;

	// next index (after the current one)
	next_index_sdl = index_sdl + 1;
	if (next_index_sdl >= SCREEN_WIDTH)
	{
		next_index_sdl -= SCREEN_WIDTH;
	}

	// index after the next one
	second_next_index_sdl = next_index_sdl + 1;
	if (second_next_index_sdl >= SCREEN_WIDTH)
	{
		second_next_index_sdl -= SCREEN_WIDTH;
	}

	// index to erase with white color
	erase_index_sdl = next_index_sdl + X_ERASE_WIDTH;
	if (erase_index_sdl >= SCREEN_WIDTH)
	{
		erase_index_sdl -= SCREEN_WIDTH;
	}

	// difference in y axis
	diff_y_sdl      = y_max_sdl - y_min_sdl;
	prev_diff_y_sdl = screen_sdl->last_y_max - screen_sdl->last_y_min;

	// difference in y axis for the axis labels
	y_diff_label      = diff_y_sdl / Y_LABEL_NB;
	prev_y_diff_label = prev_diff_y_sdl / Y_LABEL_NB;

	// height of a pixel
	pixel_height = diff_y_sdl / SCREEN_PLOT_HEIGHT;
	
	// height corresponding to 0
	alpha_nul_axis = y_max_sdl / diff_y_sdl;
	#ifdef WIN32
	y_pixel_nul_axis = (alpha_nul_axis * PLOT_Y_END + (1-alpha_nul_axis) * PLOT_Y_START);
	#else
	y_pixel_nul_axis = round(alpha_nul_axis * PLOT_Y_END + (1-alpha_nul_axis) * PLOT_Y_START);
	#endif

	// -- Refresh screen -- //

	if (screen_flag)
	{
		refresh_screen(screen_sdl);
		screen_sdl->last_scaling_flag = 0;
	}


	// -- Light gray for up and bottom -- //

	SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);

	if (screen_flag)
	{
		// up
		draw_rectangle(ren, 0, SCREEN_WIDTH-1, 0, PLOT_Y_START-2);

		// bottom
		draw_rectangle(ren, 0, SCREEN_WIDTH-1, PLOT_Y_END+1, SCREEN_HEIGHT-1);
	}

	
	// -- Horizontal black lines -- //

	SDL_set_color(screen_sdl, BLACK_SDL);

	if (screen_flag)
	{
		draw_horizontal_line(ren, 0, SCREEN_WIDTH-1, PLOT_Y_START-1);
		draw_horizontal_line(ren, 0, SCREEN_WIDTH-1, PLOT_Y_END);
	}


	// -- Vertical black line -- //

	SDL_set_color(screen_sdl, BLACK_SDL);

	if (screen_flag == 1)
	{
		draw_vertical_line(ren, PLOT_X_END-1, PLOT_Y_START, PLOT_Y_END-1);
	}
	else if(!screen_flag)
	{
		draw_vertical_line(ren, next_index_sdl, PLOT_Y_START, PLOT_Y_END-1);
	}


	// -- Erase with white rectangle -- //

	if(!screen_flag)
	{
		SDL_set_color(screen_sdl, WHITE_SDL);

		if (erase_index_sdl < second_next_index_sdl)
		{
			draw_rectangle(ren, second_next_index_sdl, SCREEN_WIDTH-1, PLOT_Y_START, PLOT_Y_END-1);
			draw_rectangle(ren, 0, erase_index_sdl, PLOT_Y_START, PLOT_Y_END-1);
		}
		else
		{
			draw_rectangle(ren, second_next_index_sdl, erase_index_sdl, PLOT_Y_START, PLOT_Y_END-1);
		}
	}


	// -- Curves -- //
	
	if (screen_flag != 2) // not for the first initialization
	{
		// indexes to work with
		if (screen_flag == 1)
		{
			init_x_index = 0;
			end_x_index = PLOT_X_END-2;
		}
		else
		{
			init_x_index = index_sdl;
			end_x_index  = index_sdl;
		}
		
		// loop on all the pixels in the width (x axis)
		for (i = init_x_index; i <= end_x_index; i++)
		{
			// black line for auto scaling
			if ((!screen_flag) && (screen_sdl->scaling_flag) && (!screen_sdl->last_scaling_flag))
			{
				SDL_set_color(screen_sdl, BLACK_SDL);
				draw_vertical_line(ren, i, PLOT_Y_START, PLOT_Y_END-1);
			}
			else 
			{
				// erase with a white line
				if (!screen_flag)
				{
					SDL_set_color(screen_sdl, WHITE_SDL);
					draw_vertical_line(ren, i, PLOT_Y_START, PLOT_Y_END-1);
				}

				// loop on the pixels in tghe height (y axis)
				for (j = 0; j < SCREEN_PLOT_HEIGHT; j++)
				{
					found_y_value = 0; // remains white pixel

					// y pixel position
					this_y_pixel = PLOT_Y_START + j;

					pixel_y_max = y_max_sdl   - (j - (NB_Y_PIXELS_WIDTh/2)) * pixel_height;
					pixel_y_min = pixel_y_max - pixel_height * NB_Y_PIXELS_WIDTh;

					// x pixel position
					if (!screen_flag)
					{
						ind_vec = index_vec;
					}
					else
					{
						ind_vec = index_vec + 2 + i;
					}

					// limit
					if (ind_vec >= SCREEN_PLOT_WIDTH)
					{
						ind_vec -= SCREEN_PLOT_WIDTH;
					}

					// loop on all the curves
					for (k=0; k<nb_curves_sdl; k++)
					{
						// current y value and previous one
						if (!screen_flag)
						{
							this_y = y_vectors[k][index_simu];
							prev_y = previous_y_vec[k];					
						}
						else
						{
							this_y = y_tab[k][ind_vec];					

							if (ind_vec <= 0)
							{
								prev_y = y_tab[k][SCREEN_PLOT_WIDTH-1];
							}
							else
							{
								prev_y = y_tab[k][ind_vec-1];
							}
						}

						// scaling of the signals
						if (this_y != START_VECTOR)
						{
							this_y = scale_signal(this_y, scaling_expo_mult[k], scaling_offset[k]);
						}

						if (prev_y != START_VECTOR)
						{
							prev_y = scale_signal(prev_y, scaling_expo_mult[k], scaling_offset[k]);
						}	

						
						// smaller signal
						if (prev_y < this_y)
						{
							this_y0 = prev_y;
							this_y1 = this_y;
						}
						else
						{
							this_y0 = this_y;
							this_y1 = prev_y;
						}


						// safety
						if (this_y0 == START_VECTOR)
						{
							this_y0 = this_y1;
						}
						else if (this_y1 == START_VECTOR)
						{
							this_y1 = this_y0;
						}

						// pixel no more white (correponding curve found)
						if (((pixel_y_min <= prev_y)  && (prev_y <= pixel_y_max)) || 
							((pixel_y_min <= this_y)  && (this_y <= pixel_y_max)) || 
							((this_y0 <= pixel_y_min) && (pixel_y_max <= this_y1)))
						{
							SDL_set_color(screen_sdl, color_vec[k]);
							found_y_value = 1;
							break;
						}
					}
					
					// no pixel to color (corresponding curve not found)
					if (!found_y_value)
					{
						// dashed line for the nul axis
						if (this_y_pixel == y_pixel_nul_axis)
						{
							if (i%(2*NUL_AXIS_DASH_WIDTH) > NUL_AXIS_DASH_WIDTH)
							{
								SDL_set_color(screen_sdl, BLACK_SDL);
								found_y_value = 1;
							}
							
						}
					}
				
					// pixel to color (corresponding curve found)
					if (found_y_value)
					{
						draw_pixel(ren, i, this_y_pixel);
					}
				}

				// store the current value for the next iteration
				for (j=0; j<nb_curves_sdl; j++)
				{
					previous_y_vec[j] = y_vectors[j][index_simu];
				}
			}

			// scaling flag
			if (!screen_flag)
			{
				screen_sdl->last_scaling_flag = screen_sdl->scaling_flag;
			}

			screen_sdl->scaling_flag = 0;
		}
	}


	// -- Bottom legend -- //

	if (screen_flag || (screen_sdl->bottom_flag == 1))
	{
		// flag
		screen_sdl->bottom_flag = 0;

		// no changing labels
		if (screen_flag)
		{
			// auto-scaling
			sprintf(str, "k: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_SIGNAL_SCALING_X_POS, BOTTOM_MID_Y_INDICATIONS, 2);

			sprintf(str, "i: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_PLOT_SCALING_X_POS, BOTTOM_MID_Y_INDICATIONS, 2);

			// graph navigation
			sprintf(str, "u: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_PLOT_MOVE_X_POS_1, SCALING_UP_INDICATIONS, 2);

			sprintf(str, "j: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_PLOT_MOVE_X_POS_1, SCALING_BOTTOM_INDICATIONS, 2);

			sprintf(str, "y: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_PLOT_MOVE_X_POS_2, SCALING_UP_INDICATIONS, 2);

			sprintf(str, "h: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_PLOT_MOVE_X_POS_2, SCALING_BOTTOM_INDICATIONS, 2);

			// plot limits
			sprintf(str, "x axis: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_XY_LABEL_X_POS, X_AXIS_INDICATION, 2);

			sprintf(str, "y axis: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_XY_LABEL_X_POS, Y_AXIS_INDICATION, 2);

			// simulation speed and time
			sprintf(str, "speed: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_TIME_SPEED_X_POS, X_AXIS_INDICATION, 2);

			sprintf(str, "time: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_TIME_SPEED_X_POS, Y_AXIS_INDICATION, 2);

			// break - quit
			sprintf(str, "p: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_QUIT_X_POS, LEGEND_SPEED_INDICATION_1, 2);

			sprintf(str, "m: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_QUIT_X_POS, LEGEND_SPEED_INDICATION_2, 2);

			sprintf(str, "F");
			print_text_sdl(str, font_break, text_color, ren, QUIT_X_POS, Y_AXIS_INDICATION, 2);

			sprintf(str, "o: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_SPEED_X_POS, LEGEND_SPEED_INDICATION_1, 2);

			sprintf(str, "l: ");
			print_text_sdl(str, font_small_label, text_color, ren, LEGEND_SPEED_X_POS, LEGEND_SPEED_INDICATION_2, 2);

			sprintf(str, "a");
			print_text_sdl(str, font_speed, text_color, ren, SPEED_X_POS, SPEED_INDICATION_1, 2);

			sprintf(str, "b");
			print_text_sdl(str, font_speed, text_color, ren, SPEED_X_POS, SPEED_INDICATION_2, 2);

			// x axis scaling
			if (screen_sdl->hor_plot_scaling)
			{
				sprintf(str, "A");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_UP_INDICATIONS, 2);
				sprintf(str, "B");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_UP_INDICATIONS, 2);

				sprintf(str, "Q");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_BOTTOM_ARROW_INDICATIONS_1, 2);
				sprintf(str, "R");
				print_text_sdl(str, font_arrows, text_color, ren, LEGEND_PLOT_MOVE_X_POS_1_BIS, SCALING_BOTTOM_ARROW_INDICATIONS_1, 2);

				sprintf(str, "A");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_2, SCALING_UP_INDICATIONS_2, 2);

				sprintf(str, "B");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_2, SCALING_BOTTOM_ARROW_INDICATIONS_1, 2);
			}
			// y axis scaling
			else
			{
				sprintf(str, "C");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_UP_INDICATIONS, 2);
				sprintf(str, "D");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_UP_INDICATIONS, 2);

				sprintf(str, "T");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_BOTTOM_ARROW_INDICATIONS_2, 2);
				sprintf(str, "S");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_1, SCALING_BOTTOM_ARROW_INDICATIONS_3, 2);

				sprintf(str, "C");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_2, SCALING_UP_INDICATIONS, 2);

				sprintf(str, "D");
				print_text_sdl(str, font_arrows, text_color, ren, PLOT_MOVE_X_POS_2, SCALING_BOTTOM_ARROW_INDICATIONS_1, 2);
			}
			
			// break
			if (real_time->simu_break)
			{
				sprintf(str, "n: ");
				print_text_sdl(str, font_small_label, text_color, ren, LEGEND_SWITCH_HOR_VERT_X_POS, BOTTOM_MID_Y_INDICATIONS, 2);

				sprintf(str, "&");
				print_text_sdl(str, font_scaling, text_color, ren, SWITCH_HOR_VERT_X_POS, SCALING_INDICATIONS, 2);

				sprintf(str, ")");
				print_text_sdl(str, font_break, text_color, ren, QUIT_X_POS, X_AXIS_INDICATION, 2);
			}
			// simulation running
			else
			{
				sprintf(str, "I");
				print_text_sdl(str, font_play, text_color, ren, QUIT_X_POS, X_AXIS_INDICATION, 2);
			}
		}

		// signal au scaling: yes
		if ((screen_sdl->signal_auto_scaling) && ((!screen_sdl->last_signal_auto_scaling) || screen_flag) )
		{
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, SIGNAL_SCALING_X_POS, LEGEND_PLOT_SCALING_X_POS-1, PLOT_Y_END + 1, SCREEN_HEIGHT-1);

			sprintf(str, "o");
			print_text_sdl(str, font_scaling, text_color, ren, SIGNAL_SCALING_X_POS, SCALING_INDICATIONS, 2);
		}

		// signal auto-scaling: no
		else if ((!screen_sdl->signal_auto_scaling) && ((screen_sdl->last_signal_auto_scaling) || screen_flag) )
		{
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, SIGNAL_SCALING_X_POS, LEGEND_PLOT_SCALING_X_POS-1, PLOT_Y_END + 1, SCREEN_HEIGHT-1);

			sprintf(str, "p");
			print_text_sdl(str, font_scaling, text_color, ren, SIGNAL_SCALING_X_POS, SCALING_INDICATIONS, 2);
		}

		// y axis auto-sclaing: yes
		if ((screen_sdl->plot_auto_scaling) && ((!screen_sdl->last_plot_auto_scaling) || screen_flag) )
		{
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, PLOT_SCALING_X_POS, LEGEND_PLOT_MOVE_X_POS_1-1, PLOT_Y_END + 1, SCREEN_HEIGHT-1);

			sprintf(str, "x"); 
			print_text_sdl(str, font_scaling, text_color, ren, PLOT_SCALING_X_POS, SCALING_INDICATIONS, 2);
		}

		// y axis auto-scaling: no
		else if ((!screen_sdl->plot_auto_scaling) && ((screen_sdl->last_plot_auto_scaling) || screen_flag) )
		{
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, PLOT_SCALING_X_POS, LEGEND_PLOT_MOVE_X_POS_1-1, PLOT_Y_END + 1, SCREEN_HEIGHT-1);

			sprintf(str, ",");
			print_text_sdl(str, font_scaling, text_color, ren, PLOT_SCALING_X_POS, SCALING_INDICATIONS, 2);
		}

		// store current values of auto-scaling for the next iteration
		screen_sdl->last_signal_auto_scaling = screen_sdl->signal_auto_scaling;
		screen_sdl->last_plot_auto_scaling = screen_sdl->plot_auto_scaling;



		// x label range

		// corresponding index
		ind_vec = index_vec+1;
		if (ind_vec >= SCREEN_PLOT_WIDTH)
		{
			ind_vec -= SCREEN_PLOT_WIDTH;
		}

		// minimal value
		legend_x_min = t_vec[ind_vec];

		if (!ind_vec && ((2*legend_x_min - t_vec[1]) < 0.0))
		{
			legend_x_min = 0.0;
		}

		// maximal value
		tsim_x_label_minus_1 = t_vec[index_vec];

		ind_vec = index_vec - 1;

		if (ind_vec < 0)
		{				
			ind_vec += PLOT_X_END;
		}

		tsim_x_label_minus_2 = t_vec[ind_vec];

		legend_x_max = tsim_x_label_minus_1 + (tsim_x_label_minus_1 - tsim_x_label_minus_2);

		// change the x axis bounds indication
		if (screen_flag || (screen_sdl->last_legend_x_min != legend_x_min) || (screen_sdl->last_legend_x_max != legend_x_max))
		{
			// erase previous value
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, XY_LABEL_X_POS, LEGEND_TIME_SPEED_X_POS-1, PLOT_Y_END + 1, BOTTOM_MID_Y_INDICATIONS);

			screen_sdl->expo_x = str_label(str, legend_x_min, legend_x_max);

			print_text_sdl(str, font_label, text_color, ren, XY_LABEL_X_POS, X_AXIS_INDICATION, 2);

			// store current values of auto-scaling for the next iteration
			screen_sdl->last_legend_x_min = legend_x_min;
			screen_sdl->last_legend_x_max = legend_x_max;
		}


		// y label range

		// change the y axis bounds indication
		if (screen_flag || (screen_sdl->last_legend_y_min != screen_sdl->y_min) || (screen_sdl->last_legend_y_max != screen_sdl->y_max))
		{
			// erase previous value
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, XY_LABEL_X_POS, LEGEND_TIME_SPEED_X_POS-1, BOTTOM_MID_Y_INDICATIONS + 1, SCREEN_HEIGHT - 1);

			screen_sdl->expo_y = str_label(str, screen_sdl->y_min, screen_sdl->y_max);
	
			print_text_sdl(str, font_label, text_color, ren, XY_LABEL_X_POS, Y_AXIS_INDICATION, 2);

			// store current values of auto-scaling for the next iteration
			screen_sdl->last_y_min = screen_sdl->y_min;
			screen_sdl->last_y_max = screen_sdl->y_max;

			screen_sdl->last_legend_y_min = screen_sdl->y_min;
			screen_sdl->last_legend_y_max = screen_sdl->y_max;
		}



		// simulation speed

		// change the simulation speed indication
		if (screen_flag || (real_time->last_real_simu_speed_factor != real_time->real_simu_speed_factor))
		{
			// erase previous value
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, TIME_SPEED_X_POS, LEGEND_SPEED_X_POS-1, PLOT_Y_END + 1, BOTTOM_MID_Y_INDICATIONS);

			if (screen_flag)
			{
				speed_factor_plot = get_simu_speed_factor(real_time->simu_speed_flag);
			}
			else
			{
				speed_factor_plot = real_time->real_simu_speed_factor;
			}			

			if (speed_factor_plot < 0.1)
			{
				sprintf(str, "%.3fx", speed_factor_plot);
			}
			else
			{
				sprintf(str, "%.2fx", speed_factor_plot);
			}

			print_text_sdl(str, font_label, text_color, ren, TIME_SPEED_X_POS, X_AXIS_INDICATION, 2);

			// store current values of auto-scaling for the next iteration
			real_time->last_real_simu_speed_factor = speed_factor_plot;
		}

		// simulation time

		// change the simulation time indication
		if (screen_flag || (tsim != real_time->last_tsim))
		{
			// erase previous value
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);
			draw_rectangle(ren, TIME_SPEED_X_POS, LEGEND_SPEED_X_POS-1, BOTTOM_MID_Y_INDICATIONS + 1, SCREEN_HEIGHT - 1);

			tsim_end = real_time->tsim_end;

			sprintf(str, "%.0f%% (%.1f/%.0f)", (tsim / tsim_end) * 100.0, tsim, tsim_end);
			print_text_sdl(str, font_label, text_color, ren, TIME_SPEED_X_POS, Y_AXIS_INDICATION, 2);

			// store current values of auto-scaling for the next iteration
			real_time->last_tsim = tsim;
		}
	}
	
	
	// -- X label -- //

	x_pixel_label = X_LABEL_WIDTH; // initial value for x
	SDL_set_color(screen_sdl, BLACK_SDL);

	// break screen
	if (screen_flag == 1)
	{	
		// loop on the x values
		while (x_pixel_label <= PLOT_X_END)
		{
			// vertical black line
			draw_vertical_line(ren, x_pixel_label - 1, PLOT_Y_END - X_LABEL_HEIGHT - 2, PLOT_Y_END - 1);

			// first values
			if (x_pixel_label < PLOT_X_END)
			{
				ind_vec = index_vec + x_pixel_label + 1;
				if (ind_vec >= PLOT_X_END)
				{				
					ind_vec -= PLOT_X_END;
				}

				tsim_x_label = t_vec[ind_vec];
			}
			// last value
			else
			{
				ind_vec = index_vec + x_pixel_label - 1;
				if (ind_vec >= PLOT_X_END)
				{				
					ind_vec -= PLOT_X_END;
				}

				tsim_x_label_minus_2 = t_vec[ind_vec];

				ind_vec = index_vec + x_pixel_label;
				if (ind_vec >= PLOT_X_END)
				{				
					ind_vec -= PLOT_X_END;
				}

				tsim_x_label_minus_1 = t_vec[ind_vec];

				tsim_x_label = tsim_x_label_minus_1 + (tsim_x_label_minus_1 - tsim_x_label_minus_2);
			}

			// string
			str_axis(str, tsim_x_label, screen_sdl->expo_x);

			// print
			print_text_sdl(str, font_axis, text_color, ren, x_pixel_label - X_LABEL_SHIFT_X - 1, PLOT_Y_END - X_LABEL_SHIFT_Y, 1);
			
			// next value
			x_pixel_label += X_LABEL_WIDTH;
		}

		// restore counter and flag
		screen_sdl->x_label_print_count = -1;
		screen_sdl->last_x_label_flag   = 1;
		
	}
	// running simulation (time to plot a new x label)
	else if((!screen_flag) && (screen_sdl->x_label_print_count >= X_LABEL_WIDTH-1))
	{
		screen_sdl->x_label_print_count = 0;

		// vertical black line
		draw_vertical_line(ren, index_sdl, PLOT_Y_END - X_LABEL_HEIGHT - 2, PLOT_Y_END - 1);

		tsim_x_label = tsim;

		sprintf(str, "%.2f", tsim_x_label);

		print_text_sdl(str, font_axis, text_color, ren, index_sdl - X_LABEL_SHIFT_X, PLOT_Y_END - X_LABEL_SHIFT_Y, 1);
	}
	// running simulation (no time to plot a new x label)
	else if(!screen_flag)
	{
		// increase the counter
		screen_sdl->x_label_print_count++;

		if (screen_sdl->last_x_label_flag)
		{
			screen_sdl->last_x_label_flag = 0;

			draw_vertical_line(ren, PLOT_X_END-1, PLOT_Y_END - X_LABEL_HEIGHT - 2, PLOT_Y_END - 1);
		}
	}
	


	// -- Y label -- //

	SDL_set_color(screen_sdl, BLACK_SDL);

	y_pixel_label = PLOT_Y_START;
	y_value       = y_max_sdl;
	prev_y_value  = screen_sdl->last_y_max;

	// store current values of auto-scaling for the next iteration
	screen_sdl->last_y_min = screen_sdl->y_min;
	screen_sdl->last_y_max = screen_sdl->y_max;

	screen_sdl->expo_y = str_label(str, screen_sdl->y_min, screen_sdl->y_max);

	if (screen_flag != 2)
	{
		// break screen
		if (screen_flag == 1)
		{
			x_end_label_plot = PLOT_X_END;
		}
		// running simulation
		else
		{
			x_end_label_plot = next_index_sdl + 1;
		}

		// loop on the y values
		while (y_pixel_label <= PLOT_Y_END)
		{
			str_axis(str, y_value, screen_sdl->expo_y);

			// first value
			if (y_pixel_label == PLOT_Y_START)
			{
				print_text_sdl(str, font_axis, text_color, ren, x_end_label_plot + Y_LABEL_SHIFT_X, y_pixel_label + Y_LABEL_SHIFT_Y_START, 2);		
			}
			// last value
			else if (y_pixel_label == PLOT_Y_END)
			{
				print_text_sdl(str, font_axis, text_color, ren, x_end_label_plot + Y_LABEL_SHIFT_X, y_pixel_label + Y_LABEL_SHIFT_Y_END, 2);
			}
			// other values
			else
			{
				draw_horizontal_line(ren, x_end_label_plot, x_end_label_plot + Y_LABEL_WIDTH - 1, y_pixel_label);

				print_text_sdl(str, font_axis, text_color, ren, x_end_label_plot + Y_LABEL_WIDTH + Y_LABEL_SHIFT_X, y_pixel_label + Y_LABEL_SHIFT_Y, 2);
			}		

			// iterate
			y_value      -= y_diff_label;
			prev_y_value -= prev_y_diff_label;
			
			y_pixel_label += Y_LABEL_HEIGHT;
		}
	}


	// -- Up legend -- /
	
    scaling_expo_mult = screen_sdl->scaling_expo_mult;
    scaling_offset    = screen_sdl->scaling_offset;

	nb_legend_curves = screen_sdl->nb_legend_curves;

	// break of first plot: write curves labels
	if (screen_flag)
	{
		for (i=0; i<nb_legend_curves; i++)
		{
			pixel_pos_label = screen_sdl->pixel_pos_label[i];

			SDL_set_color(screen_sdl, screen_sdl->color_vec[i]);

			draw_rectangle(ren, pixel_pos_label- UP_COLOR_SPACE_LABEL - UP_COLOR_WIDTH_LABEL, pixel_pos_label- UP_COLOR_SPACE_LABEL - 1,
								UP_FIRST_Y_INDICATIONS - 1, UP_FIRST_Y_INDICATIONS + 1);
			
			sprintf(str, "%s", screen_sdl->label_tab[i]);
			print_text_sdl(str, font_label, text_color, ren, pixel_pos_label, UP_FIRST_Y_INDICATIONS, 2);

			str_scaling(str, scaling_expo_mult[i], scaling_offset[i]);
			print_text_sdl(str, font_small_label, text_color, ren, pixel_pos_label, UP_SECOND_Y_INDICATIONS, 2);
		}
	}

	// write scaling indications
    last_scaling_expo_mult = screen_sdl->last_scaling_expo_mult;
    last_scaling_offset    = screen_sdl->last_scaling_offset;

	for (i=0; i<nb_legend_curves; i++)
	{
		pixel_pos_label = screen_sdl->pixel_pos_label[i];

		if ( (last_scaling_offset[i] != scaling_offset[i]) || (last_scaling_expo_mult[i] != scaling_expo_mult[i]))
		{
			// erase previous data
			SDL_set_color(screen_sdl, LIGHT_GRAY_SDL);

			if (i == nb_legend_curves - 1)
			{
				draw_rectangle(ren, pixel_pos_label, SCREEN_WIDTH - 1, UP_SEPARATION_INDICATION, PLOT_Y_START-2);
			}
			else
			{
				draw_rectangle(ren, pixel_pos_label, screen_sdl->pixel_pos_label[i+1] - 1, UP_SEPARATION_INDICATION, PLOT_Y_START-2);
			}

			// new data
			str_scaling(str, scaling_expo_mult[i], scaling_offset[i]);
			print_text_sdl(str, font_small_label, text_color, ren, pixel_pos_label, UP_SECOND_Y_INDICATIONS, 2);
		}

		// store current values of auto-scaling for the next iteration
		last_scaling_expo_mult[i] = scaling_expo_mult[i];
		last_scaling_offset[i]    = scaling_offset[i];
	}

	// iterate SDL index
	if (!screen_flag)
	{
		index_sdl++;

		if (index_sdl >= SCREEN_WIDTH)
		{
			index_sdl -= SCREEN_WIDTH;
		}

		screen_sdl->index_sdl = index_sdl;
	}
	// reset SDL index
	else if (screen_flag == 1)
	{
		screen_sdl->index_sdl = PLOT_X_END-1;
	}

	// -- Apply the plot -- //
	SDL_RenderPresent(ren);
}

#endif
