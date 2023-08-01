/*
 * get_time.h
 *
 *  Created on: Oct 7, 2018
 *      Author: amyznikov
 */
#pragma once
#ifndef __c_get_time_h___
#define __c_get_time_h___

#include <time.h>


// REALTIME sec
double get_realtime_sec(void);

// REALTIME ms
double get_realtime_ms(void);

// MONOTONIC ms
double get_monotonic_ms(void);

#endif /* __c_get_time_h___ */
