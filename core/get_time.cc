/*
 * get_time.cc
 *
 *  Created on: Oct 7, 2018
 *      Author: amyznikov
 */

#include "get_time.h"

// REALTIME ms
double get_realtime_ms(void)
{
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return (double) (t.tv_sec * 1000 + t.tv_nsec / 1e6);
}

// MONOTONIC ms
double get_monotonic_ms(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double) (t.tv_sec * 1000 + t.tv_nsec / 1e6);
}
