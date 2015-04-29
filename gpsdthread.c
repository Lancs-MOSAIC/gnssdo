/* gpsdthread.c
 *
 * Communicate with gpsd to get GNSS receiver status
 */

#include <stdio.h>
#include <errno.h>
#include <gps.h>
#include <pthread.h>
#include <time.h>
#include "gpsdthread.h"

void *gpsdthread(void *ptarg)
{
  struct gps_data_t data;
  struct gpsdthread_context *ctx;
  int r;
  double last_update = 0;
  struct timespec ts;

  ctx = (struct gpsdthread_context *)ptarg;

  r = gps_open("localhost", DEFAULT_GPSD_PORT, &data);
  if (r != 0) {
    fprintf(stderr, "gps_open: %s\n", gps_errstr(errno));
    return NULL;
  }

  gps_stream(&data, WATCH_ENABLE | WATCH_JSON, NULL);

  for (;;) {

    r = gps_waiting(&data, 500000);

    if (!r && (errno != 0))  {
	fprintf(stderr, "gps_waiting: %s\n", gps_errstr(errno));
	return NULL;
    }

    else if (r == 1) {
      errno = 0;
      r = gps_read(&data);
      if (r == 0)
	fprintf(stderr, "Surprisingly, no data available!\n");
      else if (r == -1) {
        if (errno == 0) {
	  fprintf(stderr, "Socket has closed\n");
	  return NULL;
        }
        else {
          fprintf(stderr, "gps_read: %s\n", gps_errstr(errno));
          return NULL;
        }
      }
      else {
        if (data.set & STATUS_SET) {
          data.set ^= STATUS_SET;
	  /*
          printf("%.3f %.6f %.6f st:%d sats:%d tdop:%.2f satsvis:%d\n",
		 data.fix.time, data.fix.latitude,
                 data.fix.longitude, data.status, 
                 data.satellites_used, data.dop.tdop, data.satellites_visible);
	  */
	  pthread_mutex_lock(&ctx->mutex);
	  ctx->status = data.status;
	  pthread_mutex_unlock(&ctx->mutex);

	  clock_gettime(CLOCK_REALTIME, &ts);
	  last_update = (double)ts.tv_sec + (double)ts.tv_nsec / 1.0E9;

        }
     }
      
      /* check if status has been updated recently enough */

      clock_gettime(CLOCK_REALTIME, &ts);
      double now = (double)ts.tv_sec + (double)ts.tv_nsec / 1.0E9;

      if ((last_update > 0) && ((now - last_update) > 3)) {
	/* update is old, invalidate status */
	pthread_mutex_lock(&ctx->mutex);
	ctx->status = STATUS_NO_FIX;
	pthread_mutex_unlock(&ctx->mutex);
      }

    }
  }

  gps_close(&data);

  return NULL;
}

