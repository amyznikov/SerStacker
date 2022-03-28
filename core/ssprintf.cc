/*
 * ssprintf.cc
 *
 *  Created on: Dec 27, 2021
 *      Author: amyznikov
 */

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include "ssprintf.h"
#include <core/debug.h>

/**
 * C-style string formating
 */
std::string vssprintf(const char * format, va_list arglist)
{
  char * s = NULL;
  std::string ss;

  if ( vasprintf(&s, format, arglist) > 0 ) {
    ss = s;
  }

  free(s);
  return ss;
}

/**
 * C-style string formating
 */
std::string ssprintf(const char * format, ...)
{
  std::string ss;

#ifdef _MSC_VER
# pragma warning (disable:4996)

  constexpr int buffer_size = 8 * 1024;

  std::vector<char> very_large_buffer(buffer_size);

  char * s = very_large_buffer.data();

  va_list arglist;

  int n;

  va_start(arglist, format);
  n = vsnprintf(s, buffer_size - 1, format, arglist);
  va_end(arglist);

  if ( n >= buffer_size - 1 ) {
    fprintf(stderr, "String buffer overflow in %s: %d\n", __FILE__, __LINE__);
  }

  ss = s;

#else

  char * s = nullptr;
  va_list arglist;

  va_start(arglist, format);
  if ( vasprintf(&s, format, arglist) > 0 ) {
    ss = s;
  }
  va_end(arglist);

  free(s);

#endif

  return ss;
}



/**
 * Split input string into tokens using specified delimiters
 * */
size_t strsplit(const std::string & s, std::vector<std::string> & tokens, const std::string & _delims)
{
  char buf[s.size() + 1];

  const char * delims =
      _delims.c_str();

  char * tok =
      strtok(strcpy(buf, s.c_str()),
          delims);

  size_t n = 0;

  for ( ; tok; ++n ) {
    tokens.emplace_back(tok);
    tok = strtok(NULL, delims);
  }

  return n;
}

/**
 * Split input string into tokens using specified delimiters
 * */
std::vector<std::string> strsplit(const std::string & s, const std::string & _delims)
{
  std::vector<std::string> tokens;
  strsplit(s, tokens, _delims);
  return tokens;
}


// int8_t
std::string toString(int8_t v)
{
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRId8, v);
  return s;
}

bool fromString(const std::string & s, int8_t * v)
{
  return sscanf(s.c_str(), "%" SCNd8, v) == 1;
}

// uint8_t
std::string toString(uint8_t v)
{
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRIu8, v);
  return s;
}

bool fromString(const std::string & s, uint8_t * v)
{
  return sscanf(s.c_str(), "%" SCNu8, v) == 1;
}

// int16_t
std::string toString(int16_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId16, v);
  return s;
}

bool fromString(const std::string & s, int16_t * v)
{
  return sscanf(s.c_str(), "%" SCNd16, v) == 1;
}

// uint16_t
std::string toString(uint16_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu16, v);
  return s;
}

bool fromString(const std::string & s, uint16_t * v)
{
  return sscanf(s.c_str(), "%" SCNu16, v) == 1;
}

// int32_t
std::string toString(int32_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId32, v);
  return s;
}

bool fromString(const std::string & s, int32_t * v)
{
  return sscanf(s.c_str(), "%" SCNd32, v) == 1;
}

// uint32_t
std::string toString(uint32_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu32, v);
  return s;
}

bool fromString(const std::string & s, uint32_t * v)
{
  return sscanf(s.c_str(), "%" SCNu32, v) == 1;
}

// int64_t
std::string toString(int64_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId64, v);
  return s;
}

bool fromString(const std::string & s, int64_t * v)
{
  return sscanf(s.c_str(), "%" SCNd64, v) == 1;
}

// uint64_t
std::string toString(uint64_t v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu64, v);
  return s;
}

bool fromString(const std::string & s, uint64_t * v)
{
  return sscanf(s.c_str(), "%" SCNu64, v) == 1;
}

// float
std::string toString(float v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%g", v);
  return s;
}

bool fromString(const std::string & s, float * v)
{
  return sscanf(s.c_str(), "%f", v) == 1;
}

// double
std::string toString(double v)
{
  char s[256];
  snprintf(s, sizeof(s) - 1, "%g", v);
  return s;
}

bool fromString(const std::string & s, double * v)
{
  return sscanf(s.c_str(), "%lf", v) == 1;
}

// string
std::string toString(const std::string & v)
{
  return v;
}

bool fromString(const std::string & s, std::string * v)
{
  if ( v != &s ) {
    * v = s;
  }
  return true;
}

// bool
std::string toString(bool v)
{
  return v ? "true" : "false";
}

bool fromString(const std::string & ss, bool * v)
{
  int x;

  const char * s = ss.c_str();

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


bool parse_key_value_pairs(const std::string & text, std::vector<std::pair<std::string, std::string>> & dst)
{
  if ( !text.empty() ) {

    static const auto trim_spaces =
        [](char text[]) -> char * {

          while ( isspace(*text) ) {
            ++text;
          }
          char * tmp = text;
          while ( *tmp && !isspace(*tmp) ) {
            ++tmp;
          }
          *tmp = 0;

          return text;
        };

    const std::vector<std::string> tokens =
        strsplit(text, ";");

    dst.reserve(dst.size() +
        tokens.size());

    for ( const std::string & token : tokens ) {

      char keyname[1024] = "";
      char keyvalue[1024] = "";

      if ( sscanf(token.c_str(), "%1023[^=]=%1023s", keyname, keyvalue) != 2 ) {
        CF_ERROR("Syntax error in stereo matcher args: '%s'; keyname='%s' keyvalue='%s'",
            token.c_str(), keyname, keyvalue);
        return false;
      }

      const char * key =
          trim_spaces(keyname);

      const char * value =
          trim_spaces(keyvalue);

      if ( *key && *value ) {

        dst.emplace_back(
            std::make_pair(key,
                value));

      }
    }
  }

  return true;
}

bool parse_object_type_and_args(const std::string & text,
    std::string & output_objtype,
    std::vector<std::pair<std::string, std::string>> & output_objparams)
{
  std::string objparams;

  const std::string::size_type pos =
      text.find(':');

  if ( pos == std::string::npos ) {

    output_objtype = text;

    return true;
  }

  output_objtype =
      text.substr(0, pos);

  return parse_key_value_pairs(text.substr(pos + 1),
      output_objparams);
}




