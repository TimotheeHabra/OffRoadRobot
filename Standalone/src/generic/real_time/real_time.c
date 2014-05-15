/*
 * Functions to handle the real-time features
 *
 * author: Nicolas Van der Noot
 */
#ifdef REAL_TIME

#include "real_time.h"
#include "main_simulation.h"
#include "useful_functions.h"
#include "project_info.h"

/*
 * Initialize one real-time constraint
 */
Real_time_constraint* init_real_time_constraint(int delta_t_usec, int cur_t_usec, double tsim)
{
	// variable (pointer) to return
	Real_time_constraint *constraint;

	// memory allocation
	constraint = (Real_time_constraint*) malloc(sizeof(Real_time_constraint));

	// time between 2 refresh moments (in us)
	constraint->delta_t_usec = delta_t_usec;

	// next gate for this constraint
	constraint->next_t_usec  = cur_t_usec + delta_t_usec;          // real time (in us)
	constraint->next_tsim    = tsim       + delta_t_usec * 1.0e-6; // simulation time (in s)

	return constraint;
}


Simu_real_time* init_real_time(int init_t_sec, int init_t_usec)
{
	// Real-time structure
	Simu_real_time *real_time;

	// tab with the time constraints
	double fqc_tab[NB_REAL_TIME_CONSTRAINTS];

	int cur_t_usec;

	cur_t_usec = t_usec(init_t_sec, init_t_usec);

	fqc_tab[0]  = FQC_SCREEN; // constraints
	fqc_tab[1]  = FQC_JNI;

	real_time = init_simu_real_time(NB_REAL_TIME_CONSTRAINTS, fqc_tab, cur_t_usec, TSIM_INIT, TSIM_END);

	return real_time;
}

/*
 * Initialize the structure with all the real-time constraints
 */
Simu_real_time* init_simu_real_time(int nb_constraints, double *fqc_tab, int cur_t_usec, double tsim, double tsim_end)
{
	// -- Variables decalration -- //
	int i;
	int cur_next_t_usec, min_next_t_usec;
	double cur_next_tsim, min_next_tsim;

	Simu_real_time *real_time;
	Real_time_constraint **constraints;


	// -- Initialization -- //

	// check the number of constraints
	if (nb_constraints < 0)
	{
		printf("Error: negative number of real_time constraints!\n");
		exit(1);
	}

	// memory allocation
	real_time = (Simu_real_time*) malloc(sizeof(Simu_real_time));

	// not the default time constraint
	if (nb_constraints > 0)
	{
		// flag
		real_time->no_additional_constraint = 0;

		// memory allocation: general structure
		constraints = (Real_time_constraint**) malloc(nb_constraints*sizeof(Real_time_constraint*));
		
		// memory allocation: each constraint (one by one) 
		for (i=0; i<nb_constraints; i++)
		{
			#ifdef WIN32
			constraints[i] = init_real_time_constraint((int) (1e6 / fqc_tab[i]), cur_t_usec, tsim);
			#else
			constraints[i] = init_real_time_constraint((int) round(1e6 / fqc_tab[i]), cur_t_usec, tsim);
			#endif
		}		
	}
	// default time constraint
	else
	{
		// flag
		real_time->no_additional_constraint = 1;

		// one default constraint
		nb_constraints = 1;

		// memory allocation: general structure
		constraints = (Real_time_constraint**) malloc(1*sizeof(Real_time_constraint*));

		// memory allocation: default constraint
		constraints[0] = init_real_time_constraint(DEFAULT_DELAT_T_USEC, cur_t_usec, tsim);
	}


	min_next_tsim   = 0.0;
	min_next_t_usec = 0;

	// get the first constraint (the most restrictive)
	for (i=0; i<nb_constraints; i++)
	{
		cur_next_tsim   = constraints[i]->next_tsim;
		cur_next_t_usec = constraints[i]->next_t_usec;

		if (!i)
		{
			min_next_tsim   = cur_next_tsim;
			min_next_t_usec = cur_next_t_usec;
		}
		else
		{
			if(cur_next_tsim < min_next_tsim)
			{
				min_next_tsim = cur_next_tsim;
			}
			if (cur_next_t_usec < min_next_t_usec)
			{
				min_next_t_usec = cur_next_t_usec;
			}
		}
	}

	real_time->next_tsim_gate   = min_next_tsim;
	real_time->next_t_usec_gate = min_next_t_usec;

	// number of constraints
	real_time->nb_constraints  = nb_constraints;

	// flags
	real_time->simu_quit       = 0; // quit the simulation
	real_time->simu_break      = INIT_BREAK; // break in the simulation
	real_time->simu_speed_flag = 0; // normal simulation speed

	// end of the simulation time (in s)
	real_time->tsim_end = tsim_end; 

	// simulation speed
	real_time->last_real_simu_speed_factor = 1.0;
	real_time->real_simu_speed_factor      = 1.0;

	// view point of the simulation
	real_time->change_viewpoint = 1;
	real_time->viewpoint_nb     = START_VIEWPOINT-1;

	// constraints structure
	real_time->constraints = constraints;

	// return the real-time structure
	return real_time;
}

/*
 * Free one real-time constraint
 */
void free_real_time_constraint(Real_time_constraint* constraint)
{
	free(constraint);
}

/*
 * Free the structure of all the real-time constraints
 */ 
void free_simu_real_time(Simu_real_time *real_time)
{
	int i;

	for (i=0; i<real_time->nb_constraints; i++)
	{
		free_real_time_constraint(real_time->constraints[i]);
	}

	free(real_time->constraints);
	free(real_time);
}


/*
 * Update one real-time constraint
 */
void update_real_time_constraint(Real_time_constraint *constraint, int simu_speed_flag)
{
	int delta_t_usec;
	double simu_speed_factor;

	// simulation speed
	simu_speed_factor = get_simu_speed_factor(simu_speed_flag);

	// next gates
	delta_t_usec = constraint->delta_t_usec;

	constraint->next_t_usec += delta_t_usec;
	constraint->next_tsim   += simu_speed_factor * delta_t_usec * 1.0e-6;
}


/*
 * Update the strcuture of all the constriants
 */
void update_simu_real_time(Simu_real_time *real_time)
{
	int i;
	int cur_next_t_usec, min_next_t_usec;
	double cur_next_tsim, min_next_tsim;

	min_next_t_usec = 0;
	min_next_tsim   = 0.0;

	// loop on all the constraints
	for (i=0; i<real_time->nb_constraints; i++)
	{
		cur_next_t_usec = real_time->constraints[i]->next_t_usec;
		cur_next_tsim   = real_time->constraints[i]->next_tsim;

		// first element is the minimum
		if (!i)
		{
			min_next_t_usec = cur_next_t_usec;
			min_next_tsim   = cur_next_tsim;
		}
		else
		{
			if(cur_next_t_usec < min_next_t_usec)
			{
				min_next_t_usec = cur_next_t_usec;
			}
			if(cur_next_tsim < min_next_tsim)
			{
				min_next_tsim = cur_next_tsim;
			}
		}
	}

	real_time->next_t_usec_gate = min_next_t_usec;
	real_time->next_tsim_gate   = min_next_tsim;
}

/*
 * Simulation factor defpending on the 'simu_speed_flag' flag
 */
double get_simu_speed_factor(int simu_speed_flag)
{
	if (simu_speed_flag < 0)
    {
        return pow_int(2.0,simu_speed_flag);
    }
    else if (simu_speed_flag > 0)
    {
        return simu_speed_flag + 1.0;
    }
    else
    {
        return 1.0;
    }
}

/*
 * Updates the real-time constraints after a break
 */ 
void update_real_time_constraints_break(Simu_real_time *real_time, int delta_break_u_sec)
{
	int i;

	Real_time_constraint **constraints;

	constraints = real_time->constraints;

	// loop on all the constraints
	for (i=0; i<real_time->nb_constraints; i++)
	{
		constraints[i]->next_t_usec += delta_break_u_sec;
	}

	update_simu_real_time(real_time);
}


#if defined(SDL) & defined (REAL_TIME)

/*
 * Real-time and plot SDL gestion during simulation break
 */
#if defined(JNI) & defined (REAL_TIME)
void break_gestion(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim, JNI_struct* jni_struct)
#else
void break_gestion(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim)
#endif
{
	// variables declaration
	int start_break_t_usec;
	int delta_break_u_sec;

	// start of the break instant
	start_break_t_usec = t_usec(init_t_sec, init_t_usec);

	// first break plot
	update_x_min_max(screen_sdl, real_time);
	update_scale_signals(screen_sdl, real_time, 1);
	plot_screen_sdl(screen_sdl, real_time, tsim, 1);

	while (real_time->simu_break == 1)
	{
		// handle events
		events_sdl(screen_sdl, real_time, MBSdata);

		// break plot
		if (screen_sdl->break_plot_flag)
		{
			screen_sdl->break_plot_flag = 0;

			update_scale_signals(screen_sdl, real_time, 1);
			plot_screen_sdl(screen_sdl, real_time, tsim, 1);
		}

		#if defined(JNI) & defined (REAL_TIME)
		if (real_time->change_viewpoint)
		{
			update_jni(jni_struct, MBSdata, real_time);
		}
		#endif
	}

	// update variables after the break
	delta_break_u_sec = t_usec(init_t_sec, init_t_usec) - start_break_t_usec;
	*speed_last_t_usec += delta_break_u_sec;
	update_real_time_constraints_break(real_time, delta_break_u_sec);

	screen_sdl->hor_plot_scaling = 0;

	// last_break_plot
	update_x_min_max(screen_sdl, real_time);
	plot_screen_sdl(screen_sdl, real_time, tsim, 1);
}

#if defined(JNI) & defined (REAL_TIME)
void events_simu(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *simu_go, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim, JNI_struct* jni_struct)
#else
void events_simu(Screen_sdl *screen_sdl, Simu_real_time *real_time, MBSdataStruct *MBSdata, int *simu_go, int *speed_last_t_usec, int init_t_sec, int init_t_usec, double tsim)
#endif
{
	// -- handle events -- //
	events_sdl(screen_sdl, real_time, MBSdata);

	// quit the simulation if needed
	if (real_time->simu_quit == 1)
	{
		*simu_go = 0;
	}

	// break in the simulation if needed
	if (real_time->simu_break == 1)
	{
		#if defined(JNI) & defined (REAL_TIME)
		break_gestion(screen_sdl, real_time, MBSdata, speed_last_t_usec, init_t_sec, init_t_usec, tsim, jni_struct);
		#else
		break_gestion(screen_sdl, real_time, MBSdata, speed_last_t_usec, init_t_sec, init_t_usec, tsim);
		#endif
	}
}

#endif
	
#endif
