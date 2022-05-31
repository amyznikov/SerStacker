/*
 * ssprintf.h
 *
 *  Created on: Dec 27, 2021
 *      Author: amyznikov
 *
 *  Utility routines very often required
 *  to work with std::string's
 */

#pragma once
#ifndef __ssprintf_h__
#define __ssprintf_h__

#include <cinttypes>
#include <cstdio>
#include <string>
#include <vector>
#include <string.h>
#include <type_traits>
#include <core/debug.h>


/**
 * ssprintf()
 *  C-style std::string formating
 */
std::string ssprintf(const char * format, ...)
#ifndef _MSC_VER
  __attribute__ ((__format__ (printf, 1, 2)))
#endif
  ;

/**
 * vssprintf()
 *  C-style std::string formating with variable argument list
 */
std::string vssprintf(const char * format,
    va_list arglist);


/**
 * strsplit()
 *
 * Utility routine to split input std::string into tokens using specified delimiters.
 * Similar to Qt QString::split().
 *
 * Example:
 * @code
 *   std::vector<std::string> tokens;

 *   strsplit("This is; a ; some string", tokens, " ;");

 *   for ( const std::string & token: tokens ) {
 *      printf("%s\n", token.c_stc());
 *   }
 * @endcode
 *
 */
size_t strsplit(const std::string & s,
    std::vector<std::string> & tokens,
    const std::string & _delims);

/**
 * strsplit()
 * @overload
 *
 * Utility routine to split input std::string into tokens using specified delimiters.
 * Similar to Qt QString::split().
 *
 */
std::vector<std::string> strsplit(const std::string & s,
    const std::string & _delims);

/**
 * Below are the routines for data conversion from/to std::string
 */

// int8_t
std::string toString(int8_t v);
bool fromString(const std::string & s, int8_t * v);

// uint8_t
std::string toString(uint8_t v);
bool fromString(const std::string & s, uint8_t * v);

// int16_t
std::string toString(int16_t v);
bool fromString(const std::string & s, int16_t * v);

// uint16_t
std::string toString(uint16_t v);
bool fromString(const std::string & s, uint16_t * v);

// int32_t
std::string toString(int32_t v);
bool fromString(const std::string & s, int32_t * v);

// uint32_t
std::string toString(uint32_t v);
bool fromString(const std::string & s, uint32_t * v);

// int64_t
std::string toString(int64_t v);
bool fromString(const std::string & s, int64_t * v);

// uint64_t
std::string toString(uint64_t v);
bool fromString(const std::string & s, uint64_t * v);

// float
std::string toString(float v);
bool fromString(const std::string & s, float * v);

// double
std::string toString(double v);
bool fromString(const std::string & s, double * v);

// bool
std::string toString(bool v);
bool fromString(const std::string & s, bool * v);


// string
std::string toString(const std::string & v);
bool fromString(const std::string & s, std::string * v);

// std::pair<>
template<class T1, class T2>
std::string toString(const std::pair<T1, T2> & p) {
  return ssprintf("%s;%s", toString(p.first).c_str(), toString(p.second).c_str());
}
template<class T1, class T2>
bool fromString(const std::string & s, std::pair<T1, T2> * v)
{
  const std::vector<std::string> tokens = strsplit(s, ":;, \t");
  return tokens.size() == 2 && fromString(tokens[0], &v->first) && fromString(tokens[1], &v->second);
}


// enums

struct c_enum_member {
  int value;
  const char * name;
  const char * tooltip;
};

typedef const c_enum_member * (*get_enum_members_proc)();

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
  const c_enum_member *>::type members_of();


template<class T>
inline constexpr typename std::enable_if<std::is_enum<T>::value,
  get_enum_members_proc>::type get_members_of()
{
  return &members_of<T>;
}

template<class T>
inline constexpr typename std::enable_if<!std::is_enum<T>::value,
  get_enum_members_proc>::type get_members_of()
{
  return nullptr;
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
  const char *>::type toString(const enum_type & v)
{
  const c_enum_member * members =
      members_of<enum_type>();

  if ( members ) {
    for ( int i = 0; members[i].name && *members[i].name; ++i ) {
      if ( members[i].value == v ) {
        return members[i].name;
      }
    }
  }
  return "";
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
  bool>::type fromString(const std::string & s, enum_type * v)
{
  if ( !s.empty()) {

    const char * cs =
        s.c_str();

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;

      if ( sscanf(cs, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( members[i].value == x ) {
            *v = static_cast<enum_type>(members[i].value);
            return true;
          }
        }
      }
      else {  // then try string
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( strcasecmp(members[i].name, cs) == 0 ) {
            *v = static_cast<enum_type>(members[i].value);
            return true;
          }
        }
      }
    }
  }

  return false;
}


template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
  enum_type>::type fromString(const std::string & s, enum_type defval)
{
  if ( !s.empty() ) {

    const char * cs =
        s.c_str();

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;
      if ( sscanf(cs, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( members[i].value == x ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
      else {  // then try string
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( strcasecmp(members[i].name, cs) == 0 ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
    }
  }
  return defval;
}



template<class T>
inline std::string toString(const std::vector<T> & v) {
  std::string s;
  for ( int i = 0, n = v.size(); i < n; ++i ) {
    s += toString(v[i]);
    if ( i < n - 1 ) {
      s += ";";
    }
  }
  return s;
}

template<class T>
inline bool fromString(const std::string & s, std::vector<T> * v)
{
  const std::vector<std::string> tokens =
      strsplit(s, " \t\n;:");

  v->clear(), v->reserve(tokens.size());
  for ( int i = 0, n = tokens.size(); i < n; ++i ) {
    T value;
    if ( !fromString(tokens[i], &value) ) {
      return false;
    }
    v->emplace_back(value);
  }

  return true;
}


// opencv types
#ifdef CV_VERSION

// cv::Vec
template<class T, int n>
inline std::string toString(const cv::Vec<T, n> & v) {
  std::string s;
  for ( int i = 0; i < n; ++i ) {
    s += ssprintf("%g", (double)v[i]);
    if ( i < n - 1 ) {
      s += ";";
    }
  }
  return s;
}

template<class T, int n>
inline bool fromString(const std::string & s, cv::Vec<T, n> * v)
{
  const std::vector<std::string> tokens =
      strsplit(s, " \t\n;:");

  if ( tokens.size() != n ) {
    return false;
  }

  for ( int i = 0; i < n; ++i ) {
    if ( !fromString(tokens[i], &v->val[i]) ) {
      return false;
    }
  }

  return true;
}

// cv::Scalar
template<class T>
inline std::string toString(const cv::Scalar_<T> & v) {
  std::string s;
  for ( int i = 0; i < 4; ++i ) {
    s += ssprintf("%g", (double)v[i]);
    if ( i < 3 ) {
      s += ";";
    }
  }
  return s;
}

template<class T>
inline bool fromString(const std::string & s, cv::Scalar_<T> * v)
{
  const std::vector<std::string> tokens =
      strsplit(s, " \t\n;:");

  if ( tokens.empty() ) {
    return false;
  }

  const int n =
      std::min(4, (int)tokens.size());

  for ( int i = 0; i < n; ++i ) {
    if ( !fromString(tokens[i], &v->val[i]) ) {
      return false;
    }
  }

  return true;
}

// cv::Point
template<class T>
inline bool fromString(const std::string & s, const char fmt[], cv::Point_<T> * v) {
  return sscanf(s.c_str(), fmt, &v->x, &v->y) == 2 ;
}
inline bool fromString(const std::string & s, cv::Point_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const std::string & s, cv::Point_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const std::string & s, cv::Point_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const std::string & s, cv::Point_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const std::string & s, cv::Point_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const std::string & s, cv::Point_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const std::string & s, cv::Point_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const std::string & s, cv::Point_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const std::string & s, cv::Point_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const std::string & s, cv::Point_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Point_<T> & v) {
  return ssprintf("%g;%g", (double)v.x, (double)v.y);
}
template<class T>
inline bool fromString(const std::string & s, cv::Point_<T> * v) {
  return fromString(s.c_str(), v);
}


// cv::Size
template<class T>
inline bool fromString(const std::string & s, const char fmt[], cv::Size_<T> * v)
{
  const int n = sscanf(s.c_str(), fmt, &v->width, &v->height);
  if ( n == 1 ) {
    v->height = v->width;
  }
  return n >= 1;
}
inline bool fromString(const std::string & s, cv::Size_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:x] " "%" SCNd8, v);
}
inline bool fromString(const std::string & s, cv::Size_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:x] " "%" SCNu8, v);
}
inline bool fromString(const std::string & s, cv::Size_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:x] " "%" SCNd16, v);
}
inline bool fromString(const std::string & s, cv::Size_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:x] " "%" SCNu16, v);
}
inline bool fromString(const std::string & s, cv::Size_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:x] " "%" SCNd32, v);
}
inline bool fromString(const std::string & s, cv::Size_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:x] " "%" SCNu32, v);
}
inline bool fromString(const std::string & s, cv::Size_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:x] " "%" SCNd64, v);
}
inline bool fromString(const std::string & s, cv::Size_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:x] " "%" SCNu64, v);
}
inline bool fromString(const std::string & s, cv::Size_<float> * v) {
  return fromString(s, "%f" " %*[;:x] " "%f", v);
}
inline bool fromString(const std::string & s, cv::Size_<double> * v) {
  return fromString(s, "%lf" " %*[;:x] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Size_<T> & v) {
  return ssprintf("%gx%g", (double)v.width, (double)v.height);
}

// cv::Point3
template<class T>
inline bool fromString(const std::string & s, const char fmt[], cv::Point3_<T> * v) {
  return sscanf(s.c_str(),  fmt, &v->x, &v->y, &v->z) ==  3;
}
inline bool fromString(const std::string & s, cv::Point3_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const std::string & s, cv::Point3_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const std::string & s, cv::Point3_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const std::string & s, cv::Point3_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const std::string & s, cv::Point3_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const std::string & s, cv::Point3_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const std::string & s, cv::Point3_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const std::string & s, cv::Point3_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const std::string & s, cv::Point3_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const std::string & s, cv::Point3_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Point3_<T> & v) {
  return ssprintf("%g;%g;%g", (double)v.x, (double)v.y, (double)v.z);
}

// cv::Rect
template<class T>
inline bool fromString(const std::string & s, const char fmt[], cv::Rect_<T> * v) {
  return sscanf(s.c_str(),  fmt, &v->x, &v->y, &v->width, &v->height) == 4;
}
inline bool fromString(const std::string & s, cv::Rect_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8 " %*[;:] ""%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const std::string & s, cv::Rect_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8 " %*[;:] " "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const std::string & s, cv::Rect_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16 " %*[;:] " "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const std::string & s, cv::Rect_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16 " %*[;:] " "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const std::string & s, cv::Rect_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32 " %*[;:] " "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const std::string & s, cv::Rect_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32 " %*[;:] " "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const std::string & s, cv::Rect_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64 " %*[;:] " "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const std::string & s, cv::Rect_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64 " %*[;:] " "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const std::string & s, cv::Rect_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f" " %*[;:] " "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const std::string & s, cv::Rect_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf" " %*[;:] " "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Rect_<T> & v) {
  return ssprintf("%g;%g;%g;%g", (double)v.x, (double)v.y, (double)v.width, (double)v.height);
}
#endif // CV_VERSION



/**
 * Parse the list of 'key=value' pairs separated by semicolon
 */
bool parse_key_value_pairs(const std::string & text,
    std::vector<std::pair<std::string,
    std::string>> & dst);

/**
 * Parse the object name followed by the list of 'key=value' pairs
 * separated by semicolon
 */
bool parse_object_type_and_args(const std::string & text,
    std::string & output_objtype,
    std::vector<std::pair<std::string,
    std::string>> & output_objparams);


#endif /* __ssprintf_h__ */
