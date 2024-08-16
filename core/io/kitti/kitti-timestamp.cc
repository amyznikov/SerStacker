/*
 * kitti-timestamp.cc
 *
 *  Created on: Sep 16, 2018
 *      Author: amyznikov
 */

#ifdef _MSC_VER
# pragma warning (disable:4996)
# define _CRT_SECURE_NO_WARNINGS
# define _USE_MATH_DEFINES
# define strcasecmp(a, b) 	_stricmp(a, b)
#endif 

#include "kitti-timestamp.h"
#include <time.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////
using namespace std;



/** @brief Read velodine time stamps from specified file
 *
 * The expected time stamp format looks like this:
 *  2011-09-29 12:19:46.230015023
 *
 *  It is then converted to julian date as floating-point seconds
 */
bool read_kitti_velodyne_timestamps(const string & tsfilename, vector<double> & stamps)
{
  FILE * fp;
  char line[1024];
  int year, month, day, hour, minute;
  double seconds;

  struct tm t;
  memset(&t, 0, sizeof(t));

  if ( !(fp = fopen(tsfilename.c_str(), "rt")) ) {
    CF_FATAL("fopen('%s') fails: %s", tsfilename.c_str(), strerror(errno));
    return false;
  }

  while ( fgets(line, sizeof(line), fp) ) {
    if ( sscanf(line, "%4d-%2d-%2d %2d:%2d:%lf", &year, &month, &day, &hour, &minute, &seconds) == 6 ) {

      t.tm_year = year - 1900;  // Year - 1900
      t.tm_mon = month - 1;     // months since January - [0,11]
      t.tm_mday = day;          // day of the month - [1,31]
      t.tm_hour = hour;         // hours since midnight - [0,23]
      t.tm_min = minute;        // minutes after the hour - [0,59]
      t.tm_sec = 0;             // seconds after the minute - [0,59]
      t.tm_isdst = 0;           // DST.   [-1/0/1]

      stamps.emplace_back(mktime(&t) + seconds);
    }
  }

  fclose(fp);

  return true;
}


///////////////////////////////////////////////////////////////////////////////
