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
#include <inttypes.h>


// REALTIME sec
double get_realtime_sec(void);

// REALTIME ms
double get_realtime_ms(void);

// MONOTONIC ms
double get_monotonic_ms(void);

uint64_t realtime_sec_to_ser_timestamp(double realtime_sec);
double realtime_sec_from_ser_timestamp(uint64_t ts);

#endif /* __c_get_time_h___ */
