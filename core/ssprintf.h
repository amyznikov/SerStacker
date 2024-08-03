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
#include <cstdarg>
#include <string>
#include <vector>
#include <string.h>
#include <type_traits>
#include <algorithm>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif


#if _MSC_VER
#ifndef strcasecmp
# define strcasecmp stricmp
#endif
#endif

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

// long long
std::string toString(long long v);

// unsigned long long
std::string toString(unsigned long long v);

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


struct c_enum_member {
  int value;
  std::string name;
  std::string comment;
};

typedef const c_enum_member * (*get_enum_members_proc)();

template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  const c_enum_member *> members_of();


template<class T>
inline const typename std::enable_if_t<std::is_enum_v<T>,
  get_enum_members_proc> get_members_of()
{
  return &members_of<T>;
}

template<class T>
inline const typename std::enable_if_t<!std::is_enum_v<T>,
  get_enum_members_proc> get_members_of()
{
  return nullptr;
}

inline const c_enum_member* enum_member(int v, const c_enum_member members[])
{
  if( members ) {
    for( int i = 0; !members[i].name.empty(); ++i ) {
      if( members[i].value == (int) (v) ) {
        return &members[i];
      }
    }
  }
  return nullptr;
}

template<class enum_type>
inline typename std::enable_if_t<std::is_enum_v<enum_type>,
  const c_enum_member *> enum_member(enum_type v)
{
  return enum_member((int)v, members_of<enum_type>());
}


template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  const std::string &> toString(const enum_type & v)
{
  const c_enum_member * members =
      members_of<enum_type>();

  if ( members ) {
    for ( int i = 0; !members[i].name.empty(); ++i ) {
      if ( members[i].value == (int)(v) ) {
        return members[i].name;
      }
    }
  }

  static const std::string empty_string;
  return empty_string;
}

template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  const char *> toCString(const enum_type & v)
{
  const c_enum_member * members =
      members_of<enum_type>();

  if ( members ) {
    for ( int i = 0; !members[i].name.empty(); ++i ) {
      if ( members[i].value == (int)(v) ) {
        return members[i].name.c_str();
      }
    }
  }

  return "";
}


template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  const std::string &> comment_for(const enum_type & v)
{
  const c_enum_member * members =
      members_of<enum_type>();

  if ( members ) {
    for ( int i = 0; !members[i].name.empty(); ++i ) {
      if ( members[i].value == v ) {
        return members[i].comment;
      }
    }
  }

  static const std::string empty_string;
  return empty_string;
}


inline const c_enum_member* fromString(const std::string & s, const c_enum_member members[])
{
  if( members && !s.empty() ) {

    const char *cs =
        s.c_str();

    int x;

    if( sscanf(cs, "%d", &x) == 1 ) {  // try numeric first
      for( int i = 0; !members[i].name.empty(); ++i ) {
        if( members[i].value == x ) {
          return &members[i];
        }
      }
    }
    else {  // then try string
      for( int i = 0; !members[i].name.empty(); ++i ) {
        if( strcasecmp(members[i].name.c_str(), cs) == 0 ) {
          return &members[i];
        }
      }
    }
  }

  return nullptr;
}

template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  bool> fromString(const std::string & s, enum_type * v)
{
  if ( !s.empty()) {

    const char * cs =
        s.c_str();

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;

      if ( sscanf(cs, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; !members[i].name.empty(); ++i ) {
          if ( members[i].value == x ) {
            *v = static_cast<enum_type>(members[i].value);
            return true;
          }
        }
      }
      else {  // then try string
        for ( int i = 0; !members[i].name.empty(); ++i ) {
          if ( strcasecmp(members[i].name.c_str(), cs) == 0 ) {
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
typename std::enable_if_t<std::is_enum_v<enum_type>,
  enum_type> fromString(const std::string & s, enum_type defval)
{
  if ( !s.empty() ) {

    const char * cs =
        s.c_str();

    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {

      int x;
      if ( sscanf(cs, "%d", &x) == 1 ) {  // try numeric first
        for ( int i = 0; !members[i].name.empty(); ++i ) {
          if ( members[i].value == x ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
      else {  // then try string
        for ( int i = 0; !members[i].name.empty(); ++i ) {
          if ( strcasecmp(members[i].name.c_str(), cs) == 0 ) {
            return static_cast<enum_type>(members[i].value);
          }
        }
      }
    }
  }
  return defval;
}


inline std::string flagsToString(int flags, const c_enum_member * membs)
{
  std::string s;

  if( !membs ) {
    s = ssprintf("0x%X", flags);
  }
  else {
    for( ; !membs->name.empty(); ++membs ) {

      if( flags & membs->value ) {

        if( !s.empty() ) {
          s += " | ";
        }

        s += membs->name;
      }
    }
  }

  return s;
}


template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  std::string> flagsToString(int flags)
{
  return flagsToString(flags, members_of<enum_type>());
}


inline int flagsFromString(const std::string & s, const c_enum_member *membs)
{

  int flags = 0;

  if( membs ) {

    std::vector<std::string> tokens =
        strsplit(s, "| \r\n\t");

    for ( const std::string & token : tokens ) {

      const char * s =
          token.c_str();

      for ( int i = 0; !membs[i].name.empty(); ++i ) {
        if ( strcasecmp(membs[i].name.c_str(), s) == 0 ) {
          flags |= membs[i].value;
          break;
        }
      }
    }
  }

  return flags;
}

template<class enum_type>
typename std::enable_if_t<std::is_enum_v<enum_type>,
  int> flagsFromString(const std::string & s)
{

  int flags = 0;

  if( sscanf(s.c_str(), "%d", &flags) == 1 ) {
    return flags;
  }

  const c_enum_member *membs =
      members_of<enum_type>();

  if( membs ) {

    std::vector<std::string> tokens =
        strsplit(s, "| \r\n\t");

    for ( const std::string & token : tokens ) {

      const char * s =
          token.c_str();

      for ( int i = 0; !membs[i].name.empty(); ++i ) {
        if ( strcasecmp(membs[i].name.c_str(), s) == 0 ) {
          flags |= membs[i].value;
          break;
        }
      }
    }
  }

  return flags;
}


class c_enum_members
{
public:
  typedef c_enum_members this_class;
  typedef std::vector<c_enum_member>::iterator iterator;
  typedef std::vector<c_enum_member>::const_iterator const_iterator;


  c_enum_members()
  {
    members_.emplace_back(c_enum_member{-1});
  }

  const c_enum_member * data() const
  {
    return members_.data();
  }

  void clear()
  {
    members_.clear();
    members_.emplace_back(c_enum_member{-1});
  }

  int size() const
  {
    return members_.size();
  }

  const c_enum_member& operator [](int index) const
  {
    return members_[index];
  }

  void add(int value, const std::string & name, const std::string & desc)
  {
    members_.insert(members_.begin() + members_.size() - 1,
        c_enum_member { value, name, desc });
  }


  iterator find(int value)
  {
    return std::find_if(members_.begin(), members_.end(),
        [value](const auto & m) {
          return value == m.value;
        });
  }

  const_iterator find(int value) const
  {
    return std::find_if(members_.begin(), members_.end(),
        [value](const auto & m) {
          return value == m.value;
        });
  }

  iterator find(const std::string & name)
  {
    return std::find_if(members_.begin(), members_.end(),
        [name](const auto & m) {
          return name == m.name;
        });
  }

  const_iterator find(const std::string & name) const
  {
    return std::find_if(members_.begin(), members_.end(),
        [name](const auto & m) {
          return name == m.name;
        });
  }

protected:
  std::vector<c_enum_member> members_;
};

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

  int i = 0;

  for ( ; i < n; ++i ) {
    if ( !fromString(tokens[i], &v->val[i]) ) {
      return false;
    }
  }
  for ( ; i < 4; ++i ) {
    v->val[i] = v->val[n-1];
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
inline bool fromString(const std::string & s, cv::Rect_<T> * v)
{
  double x, y, w, h;

  if ( sscanf(s.c_str(), "%lf %*[,;] %lf %*[,; \t\r\n] %lf %*[xX] %lf", &x, &y, &w, &h) == 4 ) {
    v->x = (T)x;
    v->y = (T)y;
    v->width = (T)w;
    v->height = (T)h;
    return true;
  }

  if ( sscanf(s.c_str(), "%lf %*[,;] %lf %*[,;] %lf %*[,;] %lf", &x, &y, &w, &h) == 4 ) {
    v->x = (T)x;
    v->y = (T)y;
    v->width = (T)(w - x + 1);
    v->height = (T)(h - y + 1);
    return true;
  }

  return false;
}

template<class T>
inline std::string toString(const cv::Rect_<T> & v) {
  return ssprintf("%g;%g; %gx%g", (double)v.x, (double)v.y, (double)v.width, (double)v.height);
}

template<class T, int m, int n>
std::string toString(const cv::Matx<T, m, n> & mat)
{
  std::string s;
  for ( int i = 0; i < m; ++i ) {
    for ( int j = 0; j < n; ++j ) {
      s += toString(mat(i, j));
      if ( j < n - 1 ) {
        s += "\t";
      }
    }
    s += "\n";
  }

  return s;
}

template<class T, int m, int n>
bool fromString(const std::string & s, cv::Matx<T, m, n> * mat)
{
  std::vector<std::string> tokens =
      strsplit(s, " \t\n\r");

  if ( tokens.size() != m * n ) {
    return false;
  }

  T v;
  for ( int i = 0; i < m; ++i ) {
    for ( int j = 0; j < n; ++j ) {
      if ( !fromString(tokens[i * n + j], &v) ) {
        return false;
      }
      (*mat)(i, j) = v;
    }
  }

  return true;
}

template<>
inline const c_enum_member* members_of<cv::NormTypes>()
{
  static const c_enum_member members[] = {
      { cv::NORM_L1, "NORM_L1", "cv::NORM_L1" },
      { cv::NORM_L2, "NORM_L2", "cv::NORM_L2" },
      { cv::NORM_L2SQR, "NORM_L2SQR", "cv::NORM_L2SQR" },
      { cv::NORM_INF, "NORM_INF", "cv::NORM_INF" },
      { cv::NORM_HAMMING, "HAMMING", "cv::NORM_HAMMING" },
      { cv::NORM_HAMMING2, "HAMMING2", "cv::NORM_HAMMING2" },
      { cv::NORM_L1 }
  };

  return members;
}


template<>
inline const c_enum_member* members_of<cv::CmpTypes>()
{
  static const c_enum_member members[] = {
      { cv::CMP_EQ, "EQ", "src is equal to value" },
      { cv::CMP_GT, "GT", "src is greater than value" },
      { cv::CMP_GE, "GE", "src is greater than or equal to  value" },
      { cv::CMP_LT, "LT", "src is less than  value" },
      { cv::CMP_LE, "LE", "src is less than or equal to  value" },
      { cv::CMP_NE, "NE", "src is not equal to  value" },
      { cv::CMP_GT },
  };

  return members;
}

template<>
inline const c_enum_member * members_of<cv::InterpolationFlags>()
{
  static const c_enum_member members[] = {
      {cv::INTER_LINEAR, "LINEAR", "bilinear interpolation"},
      {cv::INTER_NEAREST, "NEAREST", "nearest neighbor interpolation"},
      {cv::INTER_CUBIC, "CUBIC", "bicubic interpolation"},
      {cv::INTER_AREA, "AREA", "resampling using pixel area relation. It may be a preferred method for image decimation, as "
      "it gives moire'-free results. But when the image is zoomed, it is similar to the INTER_NEAREST "
      "method."},
      {cv::INTER_LANCZOS4, "LANCZOS4", "Lanczos interpolation over 8x8 neighborhood"},
      {cv::INTER_LINEAR_EXACT, "LINEAR_EXACT", "Bit exact bilinear interpolation"},
#if CV_VERSION_CURRRENT >= CV_VERSION_INT(4, 5, 0)
      {cv::INTER_NEAREST_EXACT, "NEAREST_EXACT", "Bit exact nearest neighbor interpolation. This will produce same results as "
      "the nearest neighbor method in PIL, scikit-image or Matlab."},
#endif
      {cv::INTER_LINEAR},
  };

  return members;
}


template<>
inline const c_enum_member * members_of<cv::BorderTypes>()
{
  static const c_enum_member members[] = {
    {cv::BORDER_CONSTANT, "BORDER_CONSTANT", "iiiiii|abcdefgh|iiiiiii"},
    {cv::BORDER_REPLICATE, "BORDER_REPLICATE", "aaaaaa|abcdefgh|hhhhhhh"},
    {cv::BORDER_REFLECT, "BORDER_REFLECT", "fedcba|abcdefgh|hgfedcb"},
    {cv::BORDER_WRAP, "BORDER_WRAP", "cdefgh|abcdefgh|abcdefg"},
    {cv::BORDER_REFLECT_101, "BORDER_REFLECT_101", "gfedcb|abcdefgh|gfedcba"},
    {cv::BORDER_TRANSPARENT, "BORDER_TRANSPARENT", "uvwxyz|abcdefgh|ijklmno"},
    {cv::BORDER_ISOLATED, "BORDER_ISOLATED", "do not look outside of ROI"},
    {cv::BORDER_REFLECT_101}
  };

  return members;
}

template<>
inline const c_enum_member * members_of<cv::MorphTypes>()
{
  static const c_enum_member members[] = {
      {cv::MORPH_ERODE, "ERODE", ""},
      {cv::MORPH_DILATE, "DILATE", ""},
      {cv::MORPH_OPEN, "OPEN", "opening operation"},
      {cv::MORPH_CLOSE, "CLOSE", "closing operation"},
      {cv::MORPH_GRADIENT, "GRADIENT", "morphological gradient"},
      {cv::MORPH_TOPHAT, "TOPHAT", "top hat"},
      {cv::MORPH_BLACKHAT, "BLACKHAT", "black hat"},
      {cv::MORPH_HITMISS, "HITMISS", "hit or miss,\n Only supported for CV_8UC1 binary images"},
      {cv::MORPH_ERODE},
  };

  return members;
}

template<>
inline const c_enum_member * members_of<cv::MorphShapes>()
{
  static const c_enum_member members[] = {
      {cv::MORPH_RECT, "RECT", "a rectangular structuring element"},
      {cv::MORPH_CROSS, "CROSS", "a cross-shaped structuring element"},
      {cv::MORPH_ELLIPSE, "ELLIPSE", "an elliptic structuring element"},
      {cv::MORPH_RECT},
  };

  return members;
}

template<>
inline const c_enum_member * members_of<cv::TermCriteria::Type>()
{
  static const c_enum_member members[] = {
      {cv::TermCriteria::COUNT, "COUNT", "the maximum number of iterations or elements to compute"},
      {cv::TermCriteria::EPS, "EPS", "the desired accuracy or change in parameters at which the iterative algorithm stops"},
      {cv::TermCriteria::COUNT},
  };

  return members;
}

template<>
inline const c_enum_member* members_of<cv::_InputArray::KindFlag>()
{
  static const c_enum_member members[] = {
      { cv::_InputArray::NONE, "NONE", "" },
      { cv::_InputArray::MAT, "MAT", "" },
      { cv::_InputArray::MATX, "MATX", "" },
      { cv::_InputArray::STD_VECTOR, "STD_VECTOR", "" },
      { cv::_InputArray::STD_VECTOR_VECTOR, "STD_VECTOR_VECTOR", "" },
      { cv::_InputArray::STD_VECTOR_MAT, "STD_VECTOR_MAT", "" },
#if OPENCV_ABI_COMPATIBILITY < 500
      { cv::_InputArray::EXPR, "EXPR", "" },
#endif
      { cv::_InputArray::OPENGL_BUFFER, "OPENGL_BUFFER", "" },
      { cv::_InputArray::CUDA_HOST_MEM, "CUDA_HOST_MEM", "" },
      { cv::_InputArray::CUDA_GPU_MAT, "CUDA_GPU_MAT", "" },
      { cv::_InputArray::UMAT, "UMAT", "" },
      { cv::_InputArray::STD_VECTOR_UMAT, "STD_VECTOR_UMAT", "" },
      { cv::_InputArray::STD_BOOL_VECTOR, "STD_BOOL_VECTOR", "" },
      { cv::_InputArray::STD_VECTOR_CUDA_GPU_MAT, "STD_VECTOR_CUDA_GPU_MAT", "" },
#if OPENCV_ABI_COMPATIBILITY < 500
      { cv::_InputArray::STD_ARRAY, "STD_ARRAY", "" },
#endif
      { cv::_InputArray::STD_ARRAY_MAT, "STD_ARRAY_MAT", "" },
      { cv::_InputArray::NONE }
  };

  return members;
}

#endif // CV_VERSION



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
typename std::enable_if_t<std::is_scalar_v<T>,
  bool> fromString(const std::string & s, std::vector<T> * v)
{
  const std::vector<std::string> tokens =
      strsplit(s, " \t\n;:,");

  v->clear(), v->reserve(tokens.size());
  for( int i = 0, n = tokens.size(); i < n; ++i ) {
    T value;
    if( !fromString(tokens[i], &value) ) {
      return false;
    }
    v->emplace_back(value);
  }

  return true;
}

template<class T>
typename std::enable_if_t<!std::is_scalar_v<T>,
  bool> fromString(const std::string & s, std::vector<T> * v)
{
  const std::vector<std::string> tokens =
      strsplit(s, "|");

  v->clear(), v->reserve(tokens.size());
  for( int i = 0, n = tokens.size(); i < n; ++i ) {
    T value;
    if( !fromString(tokens[i], &value) ) {
      return false;
    }
    v->emplace_back(value);
  }

  return true;
}


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
