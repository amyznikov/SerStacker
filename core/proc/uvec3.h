/*
 * uvec3.h
 *
 *  Created on: Dec 21, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __uvec3_h__
#define __uvec3_h__

#include <opencv2/opencv.hpp>

/**
 * 3D Vector of unit length specified in spherical coordinates by two angles
 */
template<class _Tp>
class UVec3
{
public:
  typedef UVec3 this_class;


  template<class _Tp1, class _Tp2>
  static inline void to_spherical(_Tp1 x, _Tp1 y, _Tp1 z, _Tp2 * phi, _Tp2 * theha)
  {
    *phi = (_Tp2)std::atan2(y, x);
    *theha = (_Tp2)std::atan2(std::sqrt(x * x + y * y), z);
  }

  template<class _Tp1, class _Tp2>
  static inline void to_spherical(const cv::Vec<_Tp1, 3> & T, _Tp2 * phi, _Tp2 * theha)
  {
    *phi = (_Tp2)std::atan2(T(1), T(0));
    *theha = (_Tp2)std::atan2(std::sqrt(T(0) * T(0) + T(1) * T(1)), T(2));
  }

  template<class _Tp1>
  static inline cv::Vec<_Tp1, 3> from_spherical(_Tp1 phi, _Tp1 theta)
  {
    return cv::Vec<_Tp1, 3>(std::sin(theta) * std::cos(phi),
        std::sin(theta) * std::sin(phi),
        std::cos(theta));
  }


  inline UVec3() :
    _phi(0),
    _theta(0)
  {
  }

  inline UVec3(_Tp phi, _Tp theta) :
    _phi(phi),
    _theta(theta)
  {
  }

  inline UVec3(const this_class & rhs) :
    _phi(rhs._phi),
    _theta(rhs._theta)
  {
  }


  template<class _Tp1>
  inline UVec3(_Tp1 x, _Tp1 y, _Tp1 z)
  {
    to_spherical(x, y, z, &_phi, &_theta);
  }

  template<class _Tp1>
  inline UVec3(const cv::Vec<_Tp1, 3> & rhs)
  {
    to_spherical(rhs, &_phi, &_theta);
  }

  inline this_class& operator =(const this_class & rhs)
  {
    if( this != &rhs ) {
      _phi = rhs._phi;
      _theta = rhs._theta;
    }
    return *this;
  }

  template<class _Tp1>
  inline this_class& operator =(const cv::Vec<_Tp1, 3> & rhs)
  {
    to_spherical(rhs, &_phi, &_theta);
    return *this;
  }

  inline cv::Vec<_Tp, 3> vec() const
  {
    return from_spherical(_phi, _theta);
  }

  inline operator cv::Vec<_Tp, 3> () const
  {
    return vec();
  }

  inline _Tp & phi()
  {
    return _phi;
  }

  inline const _Tp & phi() const
  {
    return _phi;
  }

  inline _Tp & theta()
  {
    return _theta;
  }

  inline const _Tp & theta() const
  {
    return _theta;
  }

  friend inline this_class operator - (const this_class & lhs, const this_class & rhs)
  {
    return this_class(lhs._phi - rhs._phi, lhs._theta - rhs._theta);
  }

  friend inline this_class operator + (const this_class & lhs, const this_class & rhs)
  {
    return this_class(lhs._phi + rhs._phi, lhs._theta + rhs._theta);
  }

protected:
  _Tp _phi, _theta;
};

typedef UVec3<double> UVec3d;
typedef UVec3<float> UVec3f;


#endif /* __uvec3_h__ */
