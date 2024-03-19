// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */


static inline void tsnorm(struct timespec *ts)
{
   while (ts->tv_nsec >= NSEC_PER_SEC) {
      ts->tv_nsec -= NSEC_PER_SEC;
      ts->tv_sec++;
   }
}


/* Subtract the ‘struct timeval’ values X and Y, (X-Y)
   storing the result in RESULT.
   Return 1 if the difference is negative, otherwise 0. */
int timespec_subtract (struct timespec *result, struct timespec *x, struct timespec *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_nsec < y->tv_nsec) {
    int n_sec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * n_sec;
    y->tv_sec += n_sec;
  }
  if (x->tv_nsec - y->tv_nsec > 1000000000) {
    int n_sec = (x->tv_nsec - y->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * n_sec;
    y->tv_sec -= n_sec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_nsec = x->tv_nsec - y->tv_nsec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}




int main()
{

    static struct timespec t, t_toc, t_lasttoc, t_duration;
    double us_err, us_err_min=0, us_err_max=0, us_err_avg=0, us_err_rms=0;
    long int n_tocs=0; 
    
    clock_gettime(CLOCK_MONOTONIC ,&t);
    /* start after one second */
    //t.tv_sec++;
    long int period = 500000;
    
    while(1) {
            //timekeeping//
        t_lasttoc = t_toc;
        clock_gettime(CLOCK_MONOTONIC, &t_toc); //TOC
        n_tocs++;
        timespec_subtract (&t_duration, &t_toc, &t_lasttoc); //get duration
        us_err = ((t_duration.tv_nsec-period)/1000.); //in us
        if (n_tocs > 2) { //do statistics, omit first 2 values
            if (us_err < us_err_min) us_err_min = us_err;
            if (us_err > us_err_max) us_err_max = us_err;
            us_err_avg = (us_err_avg*(n_tocs-1)+us_err)/n_tocs;
            us_err_rms = sqrt(((us_err_rms)*(us_err_rms)*(n_tocs-1)+us_err*us_err)/n_tocs);
        }


	t.tv_nsec += (period); //add a period to the cycle
	tsnorm(&t);

        if (!(n_tocs%500)){
            printf("%+5.2f [us] Min: %+5.2f [us] Max:%+5.2f [us] Avg:%+5.2f [us] RMS: %5.2f [us]\n",us_err, us_err_min, us_err_max, us_err_avg, us_err_rms);
        }

        //The famous sleep till next tick.
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t);
}



    return 0;
}

