/*
 * ssprintf.cc
 *
 *  Created on: Nov 26, 2016
 *      Author: amyznikov
 */
#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <stdarg.h>
#include <stdio.h>
#include "ssprintf.h"


// C-style string formating
std::string ssprintf(const char * format, ...)
{
  char * s = NULL;
  va_list arglist;
  std::string ss;

  va_start(arglist, format);
  if ( vasprintf(&s, format, arglist) > 0 ) {
    ss = s;
  }
  va_end(arglist);

  free(s);

  return ss;
}

