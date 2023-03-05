/*
 * pose.h
 *
 *  Created on: Mar 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __pose_h__
#define __pose_h__

#include <opencv2/opencv.hpp>


/** @brief build_rotation()
 * Simple utility function to construct rotation matrices from 3 given angles (in radians)
 */
template<class T>
inline void build_rotation(T ax, T ay, T az,
    cv::Matx<T, 3, 3> * Rx,
    cv::Matx<T, 3, 3> * Ry,
    cv::Matx<T, 3, 3> * Rz)
{
  typedef cv::Matx<T, 3, 3> Mat33;

  if( Rx ) {

    const T cx = std::cos(ax);
    const T sx = std::sin(ax);

    *Rx = Mat33(
        1, 0, 0,
        0, cx, -sx,
        0, sx, cx);
  }

  if( Ry ) {

    const T cy = std::cos(ay);
    const T sy = std::sin(ay);

    *Ry = Mat33(
        cy, 0, sy,
        0, 1, 0,
        -sy, 0, cy);
  }

  if( Rz ) {

    const T cz = std::cos(az);
    const T sz = std::sin(az);

    *Rz = Mat33(
        cz, -sz, 0,
        sz, cz, 0,
        0, 0, 1);
  }
}

/** @brief build_rotation()
 * Simple utility function to construct rotation matrices from 3 given angles (in radians)
 */
template<class T>
inline cv::Matx<T, 3, 3> build_rotation(T ax, T ay, T az)
{
  cv::Matx<T, 3, 3> Rx, Ry, Rz;

  build_rotation(ax, ay, az,
      &Rx, &Ry, &Rz);

  return Rz * Ry * Rx;
}

/** @brief build_rotation()
 * Simple utility function to construct rotation matrices from 3 given angles (in radians)
 */
template<class T>
inline cv::Matx<T, 3, 3> build_rotation(const cv::Vec<T, 3> & A)
{
  return build_rotation<T>(A(0), A(1), A(2));
}

/** @brief build_pose()
 * Combine given 3x3 rotation matrix R and 3x1 translation vector T into 3x4 projection matrix [R|T]
 * */
template<class C>
inline cv::Matx<C, 3, 4> build_pose(const cv::Matx<C, 3, 3> & R, const cv::Vec<C, 3> & T)
{
  return cv::Matx<C, 3, 4>(
      R(0, 0), R(0, 1), R(0, 2), T(0),
      R(1, 0), R(1, 1), R(1, 2), T(1),
      R(2, 0), R(2, 1), R(2, 2), T(2));
}

/** @brief euler_angles()
 * Simple utility function to compute euler angles in radians from given rotation matrix R
 */
template<class C>
inline cv::Vec<C, 3> euler_angles(const cv::Matx<C, 3, 3> & R)
{
  typedef cv::Vec<C, 3> cvVec;

  const double sy =
      sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

  const bool is_singular =
      sy < 1e-6;

  return is_singular ?
      cvVec(atan2(-R(1, 2), R(1, 1)), atan2(-R(2, 0), sy), 0) :
      cvVec(atan2(R(2, 1), R(2, 2)), atan2(-R(2, 0), sy), atan2(R(1, 0), R(0, 0)));
}


/** @brief
 * Return the matrix representation of  the cross product with x.
 *
 * <https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication>
 * */
template<class C>
inline cv::Matx<C, 3, 3> cross_product_matrix(const cv::Vec<C, 3> & x)
{
  return cv::Matx<C, 3, 3>(
       0.0,  -x(2), +x(1),
      +x(2),  0.0,  -x(0),
      -x(1), +x(0),  0.0);
}

/** @brief
 * Compute matrix of cofactors for given input matrix E
 *
 * Berthold K.P. Horn, "Recovering Baseline and Orientation from Essential Matrix"
 *
 * 2.3 Recovering the Orientation
 *
 * <http://www-labs.iro.umontreal.ca/~sherknie/articles/HornBKP/essential.pdf>
 * */
template<class C>
inline cv::Matx<C, 3, 3> cofactors_matrix(const cv::Matx<C, 3, 3> & E)
{
  const cv::Vec<C, 3> e1(E(0, 0), E(1, 0), E(2, 0));
  const cv::Vec<C, 3> e2(E(0, 1), E(1, 1), E(2, 1));
  const cv::Vec<C, 3> e3(E(0, 2), E(1, 2), E(2, 2));

  const cv::Vec<C, 3> e23 = e2.cross(e3);
  const cv::Vec<C, 3> e31 = e3.cross(e1);
  const cv::Vec<C, 3> e12 = e1.cross(e2);

  return cv::Matx<C, 3, 3>(
      e23(0), e23(1), e23(2),
      e31(0), e31(1), e31(2),
      e12(0), e12(1), e12(2));
}

#endif /* __pose_h__ */
