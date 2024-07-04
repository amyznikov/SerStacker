/*
 * get_time.cc
 *
 *  Created on: Oct 7, 2018
 *      Author: amyznikov
 */

#include "get_time.h"

// REALTIME sec
double get_realtime_sec(void)
{
  struct timespec t;
  clock_gettime(CLOCK_REALTIME, &t);
  return (double) (t.tv_sec + t.tv_nsec / 1e9);
}

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


//
// The code is based on https://github.com/cgarry/ser-player.git
//

static constexpr uint64_t C_SEPASECONDS_PER_SECOND = 10000000;
static constexpr uint64_t m_sepaseconds_per_microsecond = 10;
static constexpr uint64_t m_septaseconds_per_part_minute = C_SEPASECONDS_PER_SECOND * 6;
static constexpr uint64_t m_septaseconds_per_minute = C_SEPASECONDS_PER_SECOND * 60;
static constexpr uint64_t m_septaseconds_per_hour = C_SEPASECONDS_PER_SECOND * 60 * 60;
static constexpr uint64_t m_septaseconds_per_day = m_septaseconds_per_hour * 24;
static constexpr uint32_t m_days_in_400_years = 303 * 365 + 97 * 366;
static constexpr uint64_t m_septaseconds_per_400_years = m_days_in_400_years * m_septaseconds_per_day;


bool is_leap_year(uint32_t year)
{
  if( (year % 400) == 0 ) { // If year is divisible by 400 then is_leap_year
    return true;
  }

  if( (year % 100) == 0 ) { // Else if year is divisible by 100 then not_leap_year
    return false;
  }

  if( (year % 4) == 0 ) { // Else if year is divisible by 4 then is_leap_year
    return true;
  }

  // Else not_leap_year
  return false;
}


//
// Convert real time to timestamp
//
uint64_t date_to_timestamp(int32_t year, int32_t month, int32_t day, int32_t hour, int32_t minute, int32_t second,
    int32_t microsec)
{
  uint64_t ts = 0;
  int32_t yr;

  // Add 400 year blocks
  for( yr = 1; yr < (year - 400); yr += 400 ) {
    ts += m_septaseconds_per_400_years;
  }

  // Add 1 years
  for( ; yr < year; yr++ ) {
    uint32_t days_this_year = 365;
    if( is_leap_year(yr) ) {
      days_this_year = 366;
    }

    ts += (days_this_year * m_septaseconds_per_day);
  }

  // Add months
  for( int mon = 1; mon < month; mon++ ) {
    switch (mon) {
      case 4:   // April
      case 6:   // June
      case 9:   // September
      case 11:  // Novenber
        ts += (30 * m_septaseconds_per_day);
        break;
      case 2:  // Feburary
        if( is_leap_year(year) ) {
          ts += (29 * m_septaseconds_per_day);
        }
        else {
          ts += (28 * m_septaseconds_per_day);
        }

        break;
      default:
        ts += (31 * m_septaseconds_per_day);
        break;
    }
  }

  // Add days
  ts += ((day - 1) * m_septaseconds_per_day);

  // Add hours
  ts += (hour * m_septaseconds_per_hour);

  // Add minutes
  ts += (minute * m_septaseconds_per_minute);

  // Add seconds
  ts += (second * C_SEPASECONDS_PER_SECOND);

  // Micro seconds
  ts += (microsec * m_sepaseconds_per_microsecond);

  // Output result
  return ts;
}

uint64_t realtime_sec_to_ser_timestamp(double realtime_sec)
{
  return (uint64_t )(1e7 * realtime_sec);
}

double realtime_sec_from_ser_timestamp(uint64_t ts)
{
  return 1e-7 * ts;
}
