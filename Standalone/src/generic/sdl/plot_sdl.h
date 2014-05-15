/*
 * SDL functions (plot curves and handles keyboard, joystick... inputs) main header
 *
 * author: Nicolas Van der Noot
 */
#ifndef __PLOT_SDL_H_INCLUDED__  // guard against multiple/recursive includes
#define __PLOT_SDL_H_INCLUDED__

#if defined(SDL) & defined(REAL_TIME)

#include "SDL.h"
#undef main
#include "SDL_ttf.h"
#include "SDL_opengl.h"

#include "useful_functions.h"
#include "real_time.h"
#include "MBSfun.h"

#if defined(JNI) & defined (REAL_TIME)
#include "jni_functions.h"
#endif

// screen plot dimensions
#define SCREEN_WIDTH  660
#define SCREEN_HEIGHT 520

#define FQC_SREEN 50.0

// y axis: main parts
#define SCREEN_UP_HEIGHT_LOSS 40
#define SCREEN_BOTTOM_HEIGHT_LOSS 40
#define PLOT_Y_START SCREEN_UP_HEIGHT_LOSS
#define PLOT_Y_END (SCREEN_HEIGHT - SCREEN_BOTTOM_HEIGHT_LOSS)
#define PLOT_Y_MID ((PLOT_Y_START + PLOT_Y_END) / 2)
#define SCREEN_PLOT_HEIGHT (PLOT_Y_END - PLOT_Y_START)

// x axis: main parts
#define SCREEN_WIDTH_LOSS 60
#define PLOT_X_START 0
#define PLOT_X_END (SCREEN_WIDTH - SCREEN_WIDTH_LOSS)
#define SCREEN_PLOT_WIDTH (PLOT_X_END - PLOT_X_START)
#define X_ERASE_WIDTH SCREEN_WIDTH_LOSS

// up indications
#define UP_FIRST_Y_INDICATIONS (SCREEN_UP_HEIGHT_LOSS / 3)
#define UP_SECOND_Y_INDICATIONS (UP_FIRST_Y_INDICATIONS + ((3*SCREEN_UP_HEIGHT_LOSS) / 8))
#define UP_SEPARATION_INDICATION ((UP_FIRST_Y_INDICATIONS + UP_SECOND_Y_INDICATIONS) / 2)
#define UP_COLOR_WIDTH_LABEL 15
#define UP_COLOR_SPACE_LABEL 5
#define UP_COLOR_WIDTH_LOSS_BEGIN (UP_COLOR_SPACE_LABEL + UP_COLOR_SPACE_LABEL + 40)
#define UP_COLOR_WIDTH_LOSS_END 20

// bottom indications: y axis
#define BOTTOM_MID_Y_INDICATIONS (SCREEN_HEIGHT - (SCREEN_BOTTOM_HEIGHT_LOSS / 2))
#define SCALING_INDICATIONS (BOTTOM_MID_Y_INDICATIONS + 3)
#define X_AXIS_INDICATION (BOTTOM_MID_Y_INDICATIONS - 8)
#define Y_AXIS_INDICATION (BOTTOM_MID_Y_INDICATIONS + 10)
#define SPEED_INDICATION_1 (BOTTOM_MID_Y_INDICATIONS - 4)
#define SPEED_INDICATION_2 (BOTTOM_MID_Y_INDICATIONS + 14)
#define LEGEND_SPEED_INDICATION_1 (SPEED_INDICATION_1 - 5)
#define LEGEND_SPEED_INDICATION_2 (SPEED_INDICATION_2 - 5)
#define SCALING_BOTTOM_INDICATIONS (SCREEN_HEIGHT - (SCREEN_BOTTOM_HEIGHT_LOSS / 3)+2)
#define SCALING_BOTTOM_ARROW_INDICATIONS_1 (SCALING_BOTTOM_INDICATIONS + 2)
#define SCALING_BOTTOM_ARROW_INDICATIONS_2 (SCALING_BOTTOM_INDICATIONS)
#define SCALING_BOTTOM_ARROW_INDICATIONS_3 (SCALING_BOTTOM_INDICATIONS + 5)
#define SCALING_UP_INDICATIONS (SCREEN_HEIGHT - ((2*SCREEN_BOTTOM_HEIGHT_LOSS) / 3))
#define SCALING_UP_INDICATIONS_2 (SCALING_UP_INDICATIONS + 2)

// bottom indications: x axis
#define LEGEND_SIGNAL_SCALING_X_POS 10
#define SIGNAL_SCALING_X_POS 22
#define LEGEND_PLOT_SCALING_X_POS 45
#define PLOT_SCALING_X_POS 57
#define LEGEND_PLOT_MOVE_X_POS_1 93
#define LEGEND_PLOT_MOVE_X_POS_1_BIS 110
#define LEGEND_PLOT_MOVE_X_POS_2 118
#define PLOT_MOVE_X_POS_1 105
#define PLOT_MOVE_X_POS_2 130
#define LEGEND_SWITCH_HOR_VERT_X_POS 148
#define SWITCH_HOR_VERT_X_POS 160
#define LEGEND_XY_LABEL_X_POS 210
#define XY_LABEL_X_POS 250
#define LEGEND_TIME_SPEED_X_POS 405
#define TIME_SPEED_X_POS 445
#define LEGEND_SPEED_X_POS 580
#define SPEED_X_POS 592
#define LEGEND_QUIT_X_POS 620
#define QUIT_X_POS 632

// font sizes
#define FONT_AXIS_SIZE 12
#define FONT_LABEL_SIZE 12
#define FONT_SMALL_LABEL_SIZE 10
#define FONT_SCALING 25
#define FONT_ARROWS 15
#define FONT_PLAY_break 14
#define FONT_SPEED 25

// x labels
#define X_LABEL_NB 10
#define X_LABEL_WIDTH (SCREEN_PLOT_WIDTH / X_LABEL_NB)
#define X_LABEL_HEIGHT 10
#define X_LABEL_SHIFT_X 2
#define X_LABEL_SHIFT_Y 0

// y labels
#define Y_LABEL_NB 10
#define Y_LABEL_HEIGHT (SCREEN_PLOT_HEIGHT / Y_LABEL_NB)
#define Y_LABEL_WIDTH 10
#define Y_LABEL_SHIFT_X 3
#define Y_LABEL_SHIFT_Y 2
#define Y_LABEL_SHIFT_Y_START (Y_LABEL_SHIFT_Y + 9)
#define Y_LABEL_SHIFT_Y_END (Y_LABEL_SHIFT_Y - 10)

// scaling
#define PLOT_SCALE_MAX 4.0
#define SCALING_MULT_THRESHOLD 4.0
#define SCALING_PLOT_TOO_SMALL 0.0
#define SCALING_PLOT_TOO_HIGH 0.4
#define SCALING_PLOT_OPTIMAL_INCREASE 0.25
#define SCALING_PLOT_OPTIMAL_DECREASE 0.15
#define MANUAL_SCALING_PLOT 0.5
#define MANUAL_OFFSET_PLOT 0.25

// other
#define NUL_AXIS_DASH_WIDTH 8
#define NB_Y_PIXELS_WIDTh 3
#define X_AXIS_NB_Y_PIXELS_WIDTH 2
#define NB_CURVES_MAX 12
#define START_VECTOR 12345


// -- Colors enumeration -- //

enum{WHITE_SDL, BLACK_SDL, LIGHT_GRAY_SDL, BLUE_SDL, RED_SDL, DARK_GREEN_SDL, PURPLE_SDL, TURQUOISE_SDL,
	ORANGE_SDL, LIGHT_BLUE_SDL, PINK_SDL, LIGHT_GREEN_SDL, DARK_YELLOW_SDL, YELLOW_SDL, GREEN_SDL};


// -- Structures -- //


// Screen of the plot strcuture
typedef struct Screen_sdl
{
    // SDL pointers
	SDL_Window   *win;
	SDL_Renderer *ren;

    // fonts pointers
	TTF_Font     *font_label;
	TTF_Font     *font_small_label;
	TTF_Font     *font_axis;
	TTF_Font     *font_scaling;
    TTF_Font     *font_arrows;
    TTF_Font     *font_play;
    TTF_Font     *font_break;
    TTF_Font     *font_speed;

    // colors
    SDL_Color text_color;

    // other variables

    int break_plot_flag;
    int plot_auto_scaling;
    int signal_auto_scaling;
    int last_plot_auto_scaling;
    int last_signal_auto_scaling;
    int hor_plot_scaling;
    int last_t_usec;
    int index_vec;
    int index_sdl;
    int last_x_label_flag;
    int expo_y;
    int expo_x;
    int increase_plot_y_diff_flag;
    int decrease_plot_y_diff_flag;
    int up_plot_y_flag;
    int bottom_plot_y_flag;
    int increase_plot_x_diff_flag;
    int decrease_plot_x_diff_flag;
    int left_plot_y_flag;
    int right_plot_y_flag;
    int nb_curves;
    int nb_legend_curves;
    int bottom_flag;
    int index_simu;
    int scaling_flag;
    int last_scaling_flag;
    int last_color;
    int x_label_print_count;

    double tsim_pixel;
    double y_min, y_max;
    double x_min, x_max;
    double last_y_min, last_y_max;
    double last_legend_y_min, last_legend_y_max;
    double last_legend_x_min, last_legend_x_max;

    int color_vec[NB_CURVES_MAX];

    int *scaling_expo_mult;
    int *last_scaling_expo_mult;
    int *pixel_pos_label;

    double *t_vec;
    double *y_tab_min;
    double *y_tab_max;
    double *tsim_vec;
    double *previous_y_vec;
    double *scaling_offset;
    double *last_scaling_offset;

    double **y_tab;
    double **y_vectors;

    char **label_tab;

} Screen_sdl;


// -- Functions prototypes -- //

// configure functions
Screen_sdl* configure_screen_sdl(int init_t_sec, int init_t_usec);
void get_screen_sdl_functions(double *y_vec, MBSdataStruct *MBSdata);

// SDL inputs (keyboard...)
void events_sdl(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata);


// SDL
Screen_sdl* init_screen_sdl(int init_t_sec, int init_t_usec, int nb_steps, double fqc_screen, int color_vec[NB_CURVES_MAX], 
                            double y_min_init, double y_max_init, int nb_curves, int nb_legend_curves, char **label_tab);
TTF_Font* init_font(char *folder, int font_size);
void free_screen_sdl(Screen_sdl *screen_sdl);

void log_SDL_error(char *SDL_function);
void SDL_set_color(Screen_sdl * screen_sdl, int color_num);
void refresh_screen(Screen_sdl *screen_sdl);
void draw_pixel(SDL_Renderer *ren, int i, int j);
void draw_rectangle(SDL_Renderer *ren, int x1, int x2, int y1, int y2);
void draw_horizontal_line(SDL_Renderer *ren, int x1, int x2, int y);
void draw_vertical_line(SDL_Renderer *ren, int x, int y1, int y2);
void print_text_sdl(char *str, TTF_Font *font, SDL_Color txt_col, SDL_Renderer *ren, int x, int y, int pos_flag);
void renderTexture(SDL_Texture *tex, SDL_Renderer *ren, int x, int y, int pos_flag);
SDL_Texture* renderText(TTF_Font *font, char *message, SDL_Color text_color, SDL_Renderer *ren);

void update_plot_vectors(Screen_sdl * screen_sdl, Simu_real_time *real_time, double t, double *y_vec);
void update_scale_signals(Screen_sdl *screen_sdl, Simu_real_time *real_time, int screen_flag);
void update_full_vectors(Screen_sdl *screen_sdl, double tsim, double *y_vec);
void update_x_min_max(Screen_sdl *screen_sdl, Simu_real_time *real_time);
void update_y_min_max(Screen_sdl *screen_sdl, double *y_vec);
void update_y_tab(Screen_sdl *screen_sdl);

void str_scaling(char *str, int scaling_expo_mult, double scaling_offset);
int str_label(char *str, double low_nb, double high_nb);
void str_axis(char *str, double nb, int expo);

void limit_x_limits(Screen_sdl *screen_sdl);
double scale_signal(double no_scale_nb, int scaling_expo_mult, double scaling_offset);
double unscale_signal(double scale_nb, int scaling_expo_mult, double scaling_offset);

void plot_screen_sdl(Screen_sdl *screen_sdl, Simu_real_time *real_time, double tsim, int screen_flag);


#if defined(SDL) & defined (REAL_TIME)
#if defined(JNI) & defined (REAL_TIME)
void break_gestion(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim, JNI_struct* jni_struct);
#else
void break_gestion(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim);
#endif

#if defined(JNI) & defined (REAL_TIME)
void events_simu(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *simu_go, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim, JNI_struct* jni_struct);
#else
void events_simu(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *simu_go, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim);
#endif
#endif

#endif

#endif
