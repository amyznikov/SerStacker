/*
 * stdstring.cc
 *
 *  Created on: Dec 27, 2021
 *      Author: amyznikov
 */

#include "stdstring.h"


// int8_t
std::string toString(int8_t v)
{
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRId8, v);
  return s;
}

bool fromString(const char * s, int8_t * v)
{
  return s && sscanf(s, "%" SCNd8, v) == 1;
}

// uint8_t
std::string toString(uint8_t v)
{
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRIu8, v);
  return s;
}

bool fromString(const char * s, uint8_t * v)
{
  return s && sscanf(s, "%" SCNu8, v) == 1;
}

// int16_t
std::string toString(int16_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId16, v);
  return s;
}

bool fromString(const char * s, int16_t * v)
{
  return s && sscanf(s, "%" SCNd16, v) == 1;
}

// uint16_t
std::string toString(uint16_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu16, v);
  return s;
}

bool fromString(const char * s, uint16_t * v) {
  return s && sscanf(s, "%" SCNu16, v) == 1;
}

// int32_t
std::string toString(int32_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId32, v);
  return s;
}

bool fromString(const char * s, int32_t * v) {
  return s && sscanf(s, "%" SCNd32, v) == 1;
}

// uint32_t
std::string toString(uint32_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu32, v);
  return s;
}

bool fromString(const char * s, uint32_t * v)
{
  return s && sscanf(s, "%" SCNu32, v) == 1;
}

// int64_t
std::string toString(int64_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId64, v);
  return s;
}

bool fromString(const char * s, int64_t * v)
{
  return s && sscanf(s, "%" SCNd64, v) == 1;
}

// uint64_t
std::string toString(uint64_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu64, v);
  return s;
}

bool fromString(const char * s, uint64_t * v)
{
  return s && sscanf(s, "%" SCNu64, v) == 1;
}

// float
std::string toString(float v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%f", v);
  return s;
}

bool fromString(const char * s, float * v) {
  return s && sscanf(s, "%f", v) == 1;
}

// double
std::string toString(double v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%lf", v);
  return s;
}

bool fromString(const char * s, double * v) {
  return s && sscanf(s, "%lf", v) == 1;
}

// string
std::string toString(const std::string & v)
{
  return v;
}

bool fromString(const char * s, std::string * v)
{
  * v =  s ? s : "" ;
  return true;
}

// bool
std::string toString(bool v)
{
  return v ? "true" : "false";
}

bool fromString(const char * s, bool * v)
{
  int x;

  if ( strcasecmp(s, "true") == 0 || strcasecmp(s, "on") == 0 || strcasecmp(s, "yes") == 0 ) {
    *v = true;
  }
  else if ( strcasecmp(s, "false") == 0 || strcasecmp(s, "off") == 0 || strcasecmp(s, "no") == 0 ) {
    *v = false;
  }
  else if ( sscanf(s, "%d", &x) == 1 ) {
    *v = x != 0;
  }
  else {
    return false;
  }

  return true;
}


