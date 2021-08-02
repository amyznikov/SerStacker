/*
 * strsplit.cc
 *
 *  Created on: Nov 21, 2017
 *      Author: amyznikov
 */

#include <string.h>
#include "strsplit.h"

using namespace std;

size_t strsplit(const string & s,
    vector<string> & tokens,
    const string & _delims)
{
  char buf[s.size() + 1];
  const char * delims = _delims.c_str();
  char * tok = strtok(strcpy(buf, s.c_str()), delims);
  size_t n = 0;

  for ( ; tok; ++n ) {
    tokens.emplace_back(tok);
    tok = strtok(NULL, delims);
  }

  return n;
}


