/*
 * enum.h
 *
 *  Created on: Dec 7, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __enum_h__
#define __enum_h__

#include <string.h>
#include <string>
#include <type_traits>

struct c_enum_member {
  const char * name;
  const char * tooltip;
  int value;
};

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    const c_enum_member *>::type members_of()
{
  return nullptr;
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    const char *>::type toCString(const enum_type & v)
{
  const c_enum_member * members =
      members_of<enum_type>();

  if ( members ) {
    for ( int i = 0; members[i].name; ++i ) {
      if ( members[i].value == v ) {
        return members[i].name;
      }
    }
  }
  return "";
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    std::string>::type toString(const enum_type & v)
{
  return std::string(toCString(v));
}

template<class enum_type>
typename std::enable_if<std::is_enum<enum_type>::value,
    bool>::type fromString(const char * s, enum_type * v)
{
  if ( s && *s ) {
    const c_enum_member * members =
        members_of<enum_type>();

    if ( members ) {
      for ( int i = 0; members[i].name; ++i ) {
        if ( strcasecmp(members[i].name, s) == 0 ) {
          *v = static_cast<enum_type>(members[i].value);
          return true;
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
      for ( int i = 0; members[i].name; ++i ) {
        if ( strcasecmp(members[i].name, s) == 0 ) {
          return static_cast<enum_type>(members[i].value);
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

#endif /* __enum_h__ */
