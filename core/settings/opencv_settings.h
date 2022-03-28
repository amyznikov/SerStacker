/*
 * opencv_settings.h
 *
 *  Created on: Feb 12, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __opencv_settings_h__
#define __opencv_settings_h__

#include <opencv2/opencv.hpp>
#include <core/ssprintf.h>
#include <core/settings.h>

template<class T> struct c_config_type_traits<cv::Point_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Point3_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Size_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T> struct c_config_type_traits<cv::Rect_<T>> { static constexpr int type = CONFIG_TYPE_GROUP; };
template<class T, int cn> struct c_config_type_traits<cv::Vec<T, cn>> { static constexpr int type = CONFIG_TYPE_ARRAY; };
template<class T, int m, int n> struct c_config_type_traits<cv::Matx<T, m, n>> { static constexpr int type = CONFIG_TYPE_ARRAY; };

template<class T>
inline bool save_settings(c_config_setting item, const cv::Size_<T> & value)
{
  return item.isGroup() && item.set("width", value.width) && item.set("height", value.height);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Size_<T> * value) {

  switch(item.type()) {
    case CONFIG_TYPE_GROUP:
      return item.get("width", &value->width) && item.get("height", &value->height);

    case CONFIG_TYPE_STRING: {
      std::string ss;
      return item.get(&ss) && fromString(ss, value);
    }

    case CONFIG_TYPE_LIST:
    case CONFIG_TYPE_ARRAY:
      return item.length() == 2 && item[0].get(&value->width) && item[1].get(&value->height) ;
  }

  return false;
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Point_<T> & value)
{
  return item.isGroup() && item.set("x", value.x) &&
      item.set("y", value.y);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Point_<T> * value)
{
  switch(item.type()) {
    case CONFIG_TYPE_GROUP:
      return item.get("x", &value->x) && item.get("y", &value->y);

    case CONFIG_TYPE_STRING: {
      std::string ss;
      return item.get(&ss) && fromString(ss, value);
    }

    case CONFIG_TYPE_LIST:
    case CONFIG_TYPE_ARRAY:
      return item.length() == 2 && item[0].get(&value->x) && item[1].get(&value->y) ;
  }
  return false;
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Point3_<T> & value)
{
  return item.isGroup() && item.set("x", value.x) && item.set("y", value.y) &&
      item.set("z", value.z);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Point3_<T> * value)
{
  switch(item.type()) {
    case CONFIG_TYPE_GROUP:
      return item.get("x", &value->x) && item.get("y", &value->y) && item.get("z", &value->x);
    case CONFIG_TYPE_STRING: {
      std::string ss;
      return item.get(&ss) && fromString(ss, value);
    }
    case CONFIG_TYPE_LIST:
    case CONFIG_TYPE_ARRAY:
      return item.length() == 3 && item[0].get(&value->x) && item[1].get(&value->y) &&
          item[2].get(&value->z) ;
  }
  return false;
}




template<class T>
inline bool save_settings(c_config_setting item, const cv::Rect_<T> & value)
{
  return item.isGroup() && item.set("x", value.x) && item.set("y", value.y) &&
      item.set("width", value.width) && item.set("height", value.height);
}
template<class T>
inline bool load_settings(c_config_setting item, cv::Rect_<T> * value)
{
  switch ( item.type() ) {
  case CONFIG_TYPE_GROUP :
    return item.get("x", &value->x) && item.get("y", &value->y) &&
        item.get("width", &value->width) && item.get("height", &value->height);

  case CONFIG_TYPE_STRING: {
    std::string ss;
    return item.get(&ss) && fromString(ss, value);
  }

  case CONFIG_TYPE_LIST :
    case CONFIG_TYPE_ARRAY :
    return item.length() == 4 && item[0].get(&value->x) && item[1].get(&value->y) &&
        item[2].get(&value->width) && item[2].get(&value->height);
  }

  return false;
}




template<class T, int cn>
inline bool save_settings(c_config_setting array, const cv::Vec<T, cn> & value)
{
  if ( array.isArray() ) {
    while (array.length() ) {
      array.remove_element(0);
    }
    for ( int i = 0; i < cn; ++i ) {
      if ( !array.add(value[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}
template<class T, int cn>
inline bool load_settings(c_config_setting array, cv::Vec<T, cn> * value)
{
  if ( (array.isArray() || array.isList()) && array.length() == cn ) {
    for ( int i = 0; i < cn; ++i ) {
      if ( !array.get(i, &value->val[i]) ) {
        return false;
      }
    }
    return true;
  }
  return false;
}

template<class T>
inline bool save_settings(c_config_setting group, const cv::Scalar_<T> & v)
{
  group.set("v0", v.val[0]);
  group.set("v1", v.val[1]);
  group.set("v2", v.val[2]);
  group.set("v3", v.val[3]);
  return true;
}

template<class T>
inline bool load_settings(c_config_setting group, cv::Scalar_<T> * v)
{
  if ( group.isGroup() ) {
    group.get("v0", &v->val[0]);
    group.get("v1", &v->val[1]);
    group.get("v2", &v->val[2]);
    group.get("v3", &v->val[3]);
    return true;
  }
  return false;
}

template<class T, int m, int n>
inline bool save_settings(c_config_setting array, const cv::Matx<T, m, n> & value)
{
  if ( array.isArray() ) {
    while (array.length() ) {
      array.remove_element(0);
    }
    for ( int i = 0; i < m; ++i ) {
      for ( int j = 0; j < n; ++j ) {
        if ( !array.add(value(i,j)) ) {
          return false;
        }
      }
    }
    return true;
  }
  return false;
}
template<class T, int m, int n>
inline bool load_settings(c_config_setting array, cv::Matx<T, m, n> * value)
{
  if ( (array.isArray() || array.isList()) && array.length() == m * n ) {
    for ( int i = 0; i < m; ++i ) {
      for ( int j = 0; j < n; ++j ) {
        if ( !array.get(i * n + j, &(*value)(i,j)) ) {
          return false;
        }
      }
    }
    return true;
  }
  return false;
}


bool load_settings(c_config_setting settings,
    cv::TermCriteria * t);

#endif /* __opencv_settings_h__ */
