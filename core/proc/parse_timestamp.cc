/*
 * parse_timestamp.cc
 *
 *  Created on: Aug 18, 2024
 *      Author: amyznikov
 */

#include "parse_timestamp.h"
#include <time.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if __WIN32__ || __WIN64__ //  __MINGW32__ || __MINGW64__
#define timegm _mkgmtime
#endif

bool parse_timestamp_from_filename(const std::string & pathfilename, double * ts)
{

  const std::string fname =
      get_file_name(pathfilename);

  if( !fname.empty() ) {

    // saturn3.20240817_232556_GMT.32F.tiff

    const std::vector<std::string> tokens =
        strsplit(fname, ".-");

    for( const std::string & token : tokens ) {

      int Y, M, D, h, m, s;
      char suffix[32] = "";

      if( sscanf(token.c_str(), "%4d%2d%2d_%2d%2d%2d_%31s", &Y, &M, &D, &h, &m, &s, suffix) == 7 ) {

        // * ts
        struct tm tm = {
            .tm_sec = s, /* Seconds. [0-60] (1 leap second) */
            .tm_min = m, /* Minutes. [0-59] */
            .tm_hour = h, /* Hours. [0-23] */
            .tm_mday = D, /* Day.   [1-31] */
            .tm_mon = M - 1, /* Month. [0-11] */
            .tm_year = Y - 1900, /* Year - 1900.  */
            .tm_wday = 0, /* Day of week. [0-6] */
            .tm_yday = 0, /* Days in year.[0-365] */
            .tm_isdst = 0, /* DST.   [-1/0/1]*/
        };

        if ( strcasecmp(suffix, "GMT") == 0 ) {
          *ts = timegm(&tm);
          // CF_DEBUG("timestamp token '%s' parsed to %.3f GMT", token.c_str(), *ts);
        }
        else {
          *ts = mktime(&tm);
          // CF_DEBUG("timestamp token '%s' parsed to %.3f LT", token.c_str(), *ts);
        }

        return true;
      }
    }
  }

  return false;
}


