/*
 * Functions used to get the time expressed in us (micro seconds)
 *
 * author: Nicolas Van der Noot
 */

#include "useful_functions.h"
#include "real_time.h"

/*
 * Returns the absolute time of the computer
 * t_sec:  time in s
 * t_usec: time in ms
 */
void time_get(int *t_sec, int *t_usec)
{
    struct timeval timeValue;

    gettimeofday( &timeValue, NULL );
    *t_sec  = (int)timeValue.tv_sec;
    *t_usec = (int)timeValue.tv_usec;
}

/*
 * Returns the elpsed time (in us) 
 * since time 'init_t' (expressed in s and us)
 */ 
int t_usec(int init_t_sec, int init_t_usec)
{
    int current_t_sec, current_t_usec;

    time_get(&current_t_sec, &current_t_usec);

    return 1e6 * (current_t_sec - init_t_sec) + (current_t_usec - init_t_usec);
}

#ifdef WIN32
int gettimeofday (struct timeval *tp, void *tz)
{
    struct _timeb timebuffer;
    _ftime (&timebuffer);
    tp->tv_sec = timebuffer.time;
    tp->tv_usec = timebuffer.millitm * 1000;
    return 0;
}

#endif
