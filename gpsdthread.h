#ifndef _GPSDTHREAD_H
#define _GPSDTHREAD_H

#include <pthread.h>

struct gpsdthread_context {
  pthread_mutex_t mutex;
  int status; /* GNSS fix status */
};

void *gpsdthread(void *ptarg);

#endif /* _GPSDTHREAD_H */
