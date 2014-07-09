/*
 * Main loop of the simulation
 *
 * author: Nicolas Van der Noot
 */

#include "main_simulation.h"

#include "integrator.h"

#ifdef WRITE_FILES
#include "write_files.h"
#endif

#ifdef SIMBODYVIZ
#include "simbody_viz.h"
#endif

void loop_simulation(Loop_inputs *loop_inputs)
{
    // -- Variables decalration -- //

    int i,k;
    int nvar, nstep;
    int simu_go;
    
    double x, h;
    double x1, x2; 

    double *ystart;
    double *v,*vout,*dv;
    
    LocalDataStruct *lds; 
    MBSdataStruct *MBSdata; 

    #ifdef WRITE_FILES
    Write_files *write_files;
    #endif

    #if defined(JNI) & defined (REAL_TIME)
    JNI_struct* jni_struct;
    #endif

    #ifdef REAL_TIME
    int cur_t_usec;

    int init_t_sec, init_t_usec;

    // simulation time
    double tsim;

    // real-time (in us) vs simulation (in s) variables
    int speed_last_t_usec, speed_new_t_usec;
    double speed_last_tsim, speed_new_tsim;

    // Real-time constraints
    Real_time_constraint **constraints;

    // Real-time constraints main structure
    Simu_real_time *real_time;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    double y_vec[NB_CURVES_MAX];

    Screen_sdl *screen_sdl;
    #endif

    // -- Variables initialization -- //

    nvar  = loop_inputs->nvar;
    nstep = loop_inputs->nstep;

    x1 = loop_inputs->x1;
    x2 = loop_inputs->x2;

    ystart = loop_inputs->ystart;
    
    lds     = loop_inputs->lds;
    MBSdata = loop_inputs->MBSdata;

    v    = dvector(1,nvar);
    vout = dvector(1,nvar);
    dv   = dvector(1,nvar);

    // Load starting values
    for (i=1; i <= nvar; i++)
    {
        v[i] = ystart[i];
    }

    x = x1;
    h = (x2 - x1) / nstep; // [s]

    #ifdef WRITE_FILES
    write_files = loop_inputs->write_files;
    #endif

    #if defined(JNI) & defined (REAL_TIME)
    jni_struct = loop_inputs->jni_struct;
    #endif

    #ifdef REAL_TIME
    // simulation time
    tsim = TSIM_INIT;

    init_t_sec  = loop_inputs->init_t_sec;
    init_t_usec = loop_inputs->init_t_usec;

    // current eral-time [us]
    cur_t_usec = t_usec(init_t_sec, init_t_usec);

    // real-time variables
    speed_last_t_usec = cur_t_usec;
    speed_new_t_usec  = cur_t_usec;
    speed_last_tsim   = TSIM_INIT;
    speed_new_tsim    = TSIM_INIT;

    // structures
    real_time   = loop_inputs->real_time;
    constraints = real_time->constraints;
    #endif

    #if defined(SDL) & defined(REAL_TIME)
    // strcuture
    screen_sdl = loop_inputs->screen_sdl;

    // first plot
    plot_screen_sdl(screen_sdl, real_time, tsim, 2);
    #endif

    // flag : 0 if the simulation must be stopped (1 otherwise)
    simu_go = 1;

    // Simulation over nstep steps: loop
    for (k = 1; (k <= nstep) && simu_go; k++)
    {
        // user own functions (controller_files and simulation_files)
        user_compute_output(MBSdata,lds);

        // .anim
        #ifdef WRITE_FILES
        update_write_files(write_files, MBSdata);
        #endif

        // SDL window
        #if defined(SDL) & defined(REAL_TIME)
        // assign values for the plot
        get_screen_sdl_functions(y_vec, MBSdata);

        // update simulation vectors for the plot
        update_full_vectors(screen_sdl, tsim, y_vec);
        #endif
        
        #ifdef REAL_TIME
        // check actions related to the Real-time constraints
        for (i=0; i<NB_REAL_TIME_CONSTRAINTS; i++)
        {
            if (tsim >= constraints[i]->next_tsim)
            {
                // plot screen sdl
                #if defined(SDL) & defined (REAL_TIME)                
                if (!i)
                {
                    update_plot_vectors(screen_sdl, real_time, tsim, y_vec);
                    plot_screen_sdl(screen_sdl, real_time, tsim, 0);
                }
                #endif

                // JNI visualization
                #if defined(JNI) & defined (REAL_TIME)
                if (i == 1)
                {
                    update_jni(jni_struct, MBSdata, real_time);
					#ifdef SIMBODYVIZ
					reportSimbody((void *)MBSdata->user_IO->simbodyStruct->p_simbodyVariables);
					#endif

                }                
                #endif
                
                // new real-time constraints
                update_real_time_constraint(constraints[i], real_time->simu_speed_flag);
            }
        }

        // handle events (coming from the keyboard...)
        #if defined (SDL) & defined(JNI) & defined (REAL_TIME)
        events_simu(screen_sdl, real_time, MBSdata, &simu_go, &speed_last_t_usec, init_t_sec, init_t_usec, tsim, jni_struct);
        #elif defined (SDL) & defined (REAL_TIME)
        events_simu(screen_sdl, real_time, MBSdata, &simu_go, &speed_last_t_usec, init_t_sec, init_t_usec, tsim);
        #endif

        // gate locked: waiting time
        if (tsim >= real_time->next_tsim_gate)
        {
            // current time
            cur_t_usec = t_usec(init_t_sec, init_t_usec);

            // loop in order to wait to respect the real-time constraints
            while (real_time->next_t_usec_gate > cur_t_usec)
            {
                // current time
                cur_t_usec = t_usec(init_t_sec, init_t_usec);

                // handle events (coming from the keyboard...)
               #if defined (SDL) & defined(JNI) & defined (REAL_TIME)
                events_simu(screen_sdl, real_time, MBSdata, &simu_go, &speed_last_t_usec, init_t_sec, init_t_usec, tsim, jni_struct);
                #elif defined (SDL) & defined (REAL_TIME)
                events_simu(screen_sdl, real_time, MBSdata, &simu_go, &speed_last_t_usec, init_t_sec, init_t_usec, tsim);
                #endif
            }

            // in case there is no additional constraint
            if (real_time->no_additional_constraint)
            {
                update_real_time_constraint(real_time->constraints[0], real_time->simu_speed_flag);
            }

            // update real-time strcuture and related variables
            update_simu_real_time(real_time);
        }

        // computes the real time speed factor of the simulation (every REAL_TIME_SPEED_PERIOD_USEC s)
        if (cur_t_usec - speed_last_t_usec > REAL_TIME_SPEED_PERIOD_USEC)
        {
            speed_new_t_usec = cur_t_usec;
            speed_new_tsim   = tsim;

            real_time->real_simu_speed_factor = (speed_new_tsim - speed_last_tsim) / (1.0e-6 * (speed_new_t_usec - speed_last_t_usec));

            // change the plot
            #if defined(SDL) & defined (REAL_TIME)
            screen_sdl->bottom_flag = 1;
            #endif
            
            speed_last_t_usec = speed_new_t_usec;
            speed_last_tsim   = speed_new_tsim;
        }
        #endif


        #ifdef YARP
            updateDataFromYarp(loop_inputs->RobotranYarp_interface, MBSdata);
        #endif
       
        /*
         * Main routine of the integrator.
         *
         * Starting from initial values ystart[1..nvar] known at x1 use fourth-order Runge-Kutta
         * to advance nstep equal increments h to x2. The user-supplied routine derivs(x,v,dvdx)
         * evaluates derivatives. Results are stored in the global variables y[1..nvar][1..nstep+1]
         * and xx[1..nstep+1].
         */

        (*derivs)(x,v,dv,lds,MBSdata);

        rk4(v,dv,nvar,x,h,vout,derivs,lds,MBSdata);

        if ((double)(x+h) == x)
        {
            nrerror("Step size too small in routine odeint");
        } 


        #ifdef YARP
            updateDataToYarp(loop_inputs->RobotranYarp_interface, MBSdata);
        #endif

        x += h;

        for (i=1;i<=nvar;i++)
        {
            v[i] = vout[i];
        }

        /*
         * Real-time update
         */

        #ifdef REAL_TIME
        // update simulation time
        tsim = MBSdata->tsim;

        // update real time
        cur_t_usec = t_usec(init_t_sec, init_t_usec);
        #endif

        // stop the simulation if 'stop_out == 1'
        #ifdef FLAG_STOP
        if (MBSdata->user_IO->stop_out)
        {
            simu_go = 0;
        }
        #endif
    }

    // -- Free memory -- //

    free_dvector(dv,1,nvar);
    free_dvector(vout,1,nvar);
    free_dvector(v,1,nvar);

    //return 0;//MBSdata->user_IO->fitness_opti;
}

