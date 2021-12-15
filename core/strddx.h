/*
 * strddx.h
 *
 *  Created on: Dec 5, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_enum__h__
#define __c_enum__h__

#include "enum.h"
#include <core/ssprintf.h>
#include <opencv2/opencv.hpp>

// int8_t
inline std::string toString(int8_t v) {
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRId8, v);
  return s;
}
inline bool fromString(const char * s, int8_t * v) {
  return s && sscanf(s, "%" SCNd8, v) == 1;
}
inline bool fromString(const std::string & s, int8_t * v) {
  return fromString(s.c_str(), v);
}

// uint8_t
inline std::string toString(uint8_t v) {
  char s[256] = "";
  snprintf(s, sizeof(s) - 1, "%" PRIu8, v);
  return s;
}
inline bool fromString(const char * s, uint8_t * v) {
  return s && sscanf(s, "%" SCNu8, v) == 1;
}
inline bool fromString(const std::string & s, uint8_t * v) {
  return fromString(s.c_str(), v);
}

// int16_t
inline std::string toString(int16_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId16, v);
  return s;
}
inline bool fromString(const char * s, int16_t * v) {
  return s && sscanf(s, "%" SCNd16, v) == 1;
}
inline bool fromString(const std::string & s, int16_t * v) {
  return fromString(s.c_str(), v);
}

// uint16_t
inline std::string toString(uint16_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu16, v);
  return s;
}
inline bool fromString(const char * s, uint16_t * v) {
  return s && sscanf(s, "%" SCNu16, v) == 1;
}
inline bool fromString(const std::string & s, uint16_t * v) {
  return fromString(s.c_str(), v);
}

// int32_t
inline std::string toString(int32_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId32, v);
  return s;
}
inline bool fromString(const char * s, int32_t * v) {
  return s && sscanf(s, "%" SCNd32, v) == 1;
}
inline bool fromString(const std::string & s, int32_t * v) {
  return fromString(s.c_str(), v);
}

// uint32_t
inline std::string toString(uint32_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu32, v);
  return s;
}
inline bool fromString(const char * s, uint32_t * v) {
  return s && sscanf(s, "%" SCNu32, v) == 1;
}
inline bool fromString(const std::string & s, uint32_t * v) {
  return fromString(s.c_str(), v);
}

// int64_t
inline std::string toString(int64_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRId64, v);
  return s;
}
inline bool fromString(const char * s, int64_t * v) {
  return s && sscanf(s, "%" SCNd64, v) == 1;
}
inline bool fromString(const std::string & s, int64_t * v) {
  return fromString(s.c_str(), v);
}

// uint64_t
inline std::string toString(uint64_t v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%" PRIu64, v);
  return s;
}
inline bool fromString(const char * s, uint64_t * v) {
  return s && sscanf(s, "%" SCNu64, v) == 1;
}
inline bool fromString(const std::string & s, uint64_t * v) {
  return fromString(s.c_str(), v);
}

// float
inline std::string toString(float v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%f", v);
  return s;
}
inline bool fromString(const char * s, float * v) {
  return s && sscanf(s, "%f", v) == 1;
}
inline bool fromString(const std::string & s, float * v) {
  return fromString(s.c_str(), v);
}

// double
inline std::string toString(double v) {
  char s[256];
  snprintf(s, sizeof(s) - 1, "%lf", v);
  return s;
}
inline bool fromString(const char * s, double * v) {
  return s && sscanf(s, "%lf", v) == 1;
}
inline bool fromString(const std::string & s, double * v) {
  return fromString(s.c_str(), v);
}

// string
inline std::string toString(const std::string & v) {
  return v;
}
inline bool fromString(const char * s, std::string * v) {
  * v =  s ? s : "" ;
  return true;
}
inline bool fromString(const std::string & s, std::string * v) {
  *v = s;
  return true;
}


#ifdef CV_VERSION

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



#endif




struct c_property {
  const char * name;
  const char * tooltip;
  const c_enum_member * enumeration;
  c_property(const char * _name, const char * _tooltip,
      const c_enum_member * _enumeration = nullptr) :
      name(_name), tooltip(_tooltip), enumeration(_enumeration)
  {
  }
};

#define STRDDX(prop, name, value, getit) \
  if ( name == #prop ) { \
    if ( getit ) { \
       value = toString(prop()); \
       return true; \
    } \
    else { \
      auto v = prop(); \
      if ( !fromString(value, &v) ) { \
        CF_DEBUG("fromString('%s') fails",\
            value.c_str());\
      } \
      else { \
        set_##prop(v); \
        return true; \
      } \
    } \
    return false; \
  }



#endif /* __c_enum__h__ */
