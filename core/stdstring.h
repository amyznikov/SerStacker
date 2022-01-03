/*
 * stdstring.h
 *
 *  Created on: Dec 27, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __stdstring_h__
#define __stdstring_h__

#include <cinttypes>
#include <cstdio>
#include <string>
#include <string.h>

// int8_t
std::string toString(int8_t v);
bool fromString(const char * s, int8_t * v);

// uint8_t
std::string toString(uint8_t v);
bool fromString(const char * s, uint8_t * v);

// int16_t
std::string toString(int16_t v);
bool fromString(const char * s, int16_t * v);

// uint16_t
std::string toString(uint16_t v);
bool fromString(const char * s, uint16_t * v);

// int32_t
std::string toString(int32_t v);
bool fromString(const char * s, int32_t * v);

// uint32_t
std::string toString(uint32_t v);
bool fromString(const char * s, uint32_t * v);

// int64_t
std::string toString(int64_t v);
bool fromString(const char * s, int64_t * v);

// uint64_t
std::string toString(uint64_t v);
bool fromString(const char * s, uint64_t * v);

// float
std::string toString(float v);
bool fromString(const char * s, float * v);

// double
std::string toString(double v);
bool fromString(const char * s, double * v);

// bool
std::string toString(bool v);
bool fromString(const char * s, bool * v);


// string
std::string toString(const std::string & v);
bool fromString(const char * s, std::string * v);
bool fromString(const std::string & s, std::string * v);



// enums

struct c_enum_member {
  int value;
  const char * name;
  const char * tooltip;
};

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    const c_enum_member *>::type members_of();

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
    bool>::type fromString(const char * s, enum_type * v)
{
  if ( s && *s ) {

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;

      if ( sscanf(s, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( members[i].value == x ) {
            *v = static_cast<enum_type>(members[i].value);
            return true;
          }
        }
      }
      else {  // then try string
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( strcasecmp(members[i].name, s) == 0 ) {
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
    bool>::type fromString(const std::string & s, enum_type * v)
{
  return fromString(s.c_str(), v);
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    enum_type>::type fromString(const char * s, const enum_type & defval)
{
  if ( s && *s ) {

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;
      if ( sscanf(s, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( members[i].value == x ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
      else {  // then try string
        for ( int i = 0; members[i].name && *members[i].name; ++i ) {
          if ( strcasecmp(members[i].name, s) == 0 ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
    }
  }
  return defval;
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    enum_type>::type fromString(const std::string & s, const enum_type & defval)
{
  return fromString(s.c_str(), defval);
}

// opencv
#ifdef CV_VERSION

// cv::Point
template<class T>
inline bool fromString(const char * s, const char fmt[], cv::Point_<T> * v) {
  return sscanf(s, fmt, &v->x, &v->y) == 2 ;
}
inline bool fromString(const char * s, cv::Point_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const char * s, cv::Point_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const char * s, cv::Point_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const char * s, cv::Point_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const char * s, cv::Point_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const char * s, cv::Point_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const char * s, cv::Point_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const char * s, cv::Point_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const char * s, cv::Point_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const char * s, cv::Point_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Point_<T> & v) {
  return ssprintf("%g;%g", v.x, v.y);
}
template<class T>
inline bool fromString(const std::string & s, cv::Point_<T> * v) {
  return fromString(s.c_str(), v);
}


// cv::Size
template<class T>
inline bool fromString(const char * s, const char fmt[], cv::Size_<T> * v)
{
  const int n = sscanf(s, fmt, &v->width, &v->height);
  if ( n == 1 ) {
    v->height = v->width;
  }
  return n >= 1;
}
inline bool fromString(const char * s, cv::Size_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const char * s, cv::Size_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const char * s, cv::Size_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const char * s, cv::Size_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const char * s, cv::Size_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const char * s, cv::Size_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const char * s, cv::Size_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const char * s, cv::Size_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const char * s, cv::Size_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const char * s, cv::Size_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Size_<T> & v) {
  return ssprintf("%g;%g", v.x, v.y);
}
template<class T>
inline bool fromString(const std::string & s, cv::Size_<T> * v) {
  return fromString(s.c_str(), v);
}


// cv::Point3
template<class T>
inline bool fromString(const char * s, const char fmt[], cv::Point3_<T> * v) {
  return sscanf(s, fmt, &v->x, &v->y, &v->z) ==  3;
}
inline bool fromString(const char * s, cv::Point3_<int8_t> * v) {
  return fromString(s, "%" SCNd8 " %*[;:] " "%" SCNd8 " %*[;:] " "%" SCNd8, v);
}
inline bool fromString(const char * s, cv::Point3_<uint8_t> * v) {
  return fromString(s, "%" SCNu8 " %*[;:] " "%" SCNu8 " %*[;:] " "%" SCNu8, v);
}
inline bool fromString(const char * s, cv::Point3_<int16_t> * v) {
  return fromString(s, "%" SCNd16 " %*[;:] " "%" SCNd16 " %*[;:] " "%" SCNd16, v);
}
inline bool fromString(const char * s, cv::Point3_<uint16_t> * v) {
  return fromString(s, "%" SCNu16 " %*[;:] " "%" SCNu16 " %*[;:] " "%" SCNu16, v);
}
inline bool fromString(const char * s, cv::Point3_<int32_t> * v) {
  return fromString(s, "%" SCNd32 " %*[;:] " "%" SCNd32 " %*[;:] " "%" SCNd32, v);
}
inline bool fromString(const char * s, cv::Point3_<uint32_t> * v) {
  return fromString(s, "%" SCNu32 " %*[;:] " "%" SCNu32 " %*[;:] " "%" SCNu32, v);
}
inline bool fromString(const char * s, cv::Point3_<int64_t> * v) {
  return fromString(s, "%" SCNd64 " %*[;:] " "%" SCNd64 " %*[;:] " "%" SCNd64, v);
}
inline bool fromString(const char * s, cv::Point3_<uint64_t> * v) {
  return fromString(s, "%" SCNu64 " %*[;:] " "%" SCNu64 " %*[;:] " "%" SCNu64, v);
}
inline bool fromString(const char * s, cv::Point3_<float> * v) {
  return fromString(s, "%f" " %*[;:] " "%f" " %*[;:] " "%f", v);
}
inline bool fromString(const char * s, cv::Point3_<double> * v) {
  return fromString(s, "%lf" " %*[;:] " "%lf" " %*[;:] " "%lf", v);
}
template<class T>
inline std::string toString(const cv::Point3_<T> & v) {
  return ssprintf("%g;%g;%g", v.x, v.y);
}
template<class T>
inline bool fromString(const std::string & s, cv::Point3_<T> * v) {
  return fromString(s.c_str(), v);
}
#endif // CV_VERSION


template<class T>
inline bool fromString(const std::string & s, T * v)
{
  return fromString(s.c_str(), v);
}

#endif /* __stdstring_h__ */
