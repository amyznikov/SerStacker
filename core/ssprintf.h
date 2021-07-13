/*
 * ssprintf.h
 *
 *  Created on: Nov 26, 2016
 *      Author: amyznikov
 */
#pragma once
#ifndef __ssprintf_h__
#define __ssprintf_h__

#include <string>
#include <inttypes.h>

///////////////////////////////////////////////////////////////////////////////
// C-style string formating
std::string ssprintf(const char * format, ...)
  __attribute__ ((__format__ (printf, 1, 2)));


///////////////////////////////////////////////////////////////////////////////
#endif /* __ssprintf_h__ */
