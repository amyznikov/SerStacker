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

/** @brief build_rotation()
 * Simple utility function to construct rotation matrices from 3 given angles (in radians)
 */
template<class T>
inline cv::Matx<T, 3, 3> build_rotation2(T ax, T ay, T az)
{
  cv::Matx<T, 3, 3> Rx, Ry, Rz;

  build_rotation(ax, ay, az,
      &Rx, &Ry, &Rz);

  return Rx * Ry * Rz;
}


/** @brief build_rotation()
 * Simple utility function to construct rotation matrices from 3 given angles (in radians)
 */
template<class T>
inline cv::Matx<T, 3, 3> build_rotation2(const cv::Vec<T, 3> & A)
{
  return build_rotation2<T>(A(0), A(1), A(2));
}

/** @brief build_pose()
 * Compose given 3x3 rotation matrix R and 3x1 translation vector T into 3x4 projection matrix [R|T]
 * */
template<class C>
inline cv::Matx<C, 3, 4> build_pose(const cv::Matx<C, 3, 3> & R, const cv::Vec<C, 3> & T)
{
  return cv::Matx<C, 3, 4>(
      R(0, 0), R(0, 1), R(0, 2), T(0),
      R(1, 0), R(1, 1), R(1, 2), T(1),
      R(2, 0), R(2, 1), R(2, 2), T(2));
}

/** @brief split_pose()
 * Decompose given 3x4 R|T pose matrix into 3x3 rotation matrix R and 3x1 translation vector T
 * */

template<class T1, class T2, class T3>
inline void split_pose(const cv::Matx<T3, 3, 4> & RT, /*out*/ cv::Matx<T1, 3, 3> & R, /*out*/ cv::Vec<T2, 3> & T)
{
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      R(i, j) = RT(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    T(i) = RT(i, 3);
  }
}

/** @brief split_pose()
 * Decompose given 4x4 R|T pose matrix into 3x3 rotation matrix R and 3x1 translation vector T
 * */
template<class T1, class T2, class T3>
inline void split_pose(const cv::Matx<T3, 4, 4> & RT, /*out*/ cv::Matx<T1, 3, 3> & R, /*out*/ cv::Vec<T2, 3> & T)
{
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      R(i, j) = RT(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    T(i) = RT(i, 3);
  }
}

/** @brief invert_pose()
 * Invert 3x4 R|T pose matrix
 * */
template<class C>
void invert_pose(const cv::Matx<C, 3, 4> & RT, /*out*/cv::Matx<C, 3, 4> * RTi)
{
  cv::Matx<C, 3, 3> R;
  cv::Vec<C, 3> T;

  split_pose(RT, R, T);

  R = R.inv();
  T = -R * T;

  build_pose(R, T, *RTi);
}

/** @brief invert_pose()
 * Invert given R|T pose
 * */
template<class C>
void invert_pose(const cv::Matx<C, 3, 3> & R, const cv::Vec3d & T,
    /*out*/cv::Matx<C, 3, 3> * Ri, /*out*/cv::Vec3d * Ti )
{
  *Ri = R.t();
  *Ti = -(*Ri) * T;
}

template<class C>
cv::Matx<C, 3, 4> invert_pose(const cv::Matx<C, 3, 4> & RT)
{
  cv::Matx<C, 3, 3> R;
  cv::Vec<C, 3> T;

  split_pose(RT, R, T);

  R = R.inv();
  T = -R * T;

  return build_pose(R, T);
}



//split_rt_matrix

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


//@brief Trey Wilson closed-form solution for shortest distance between two angles (radians).
//       Angles do NOT need to be normalized.
//@see <https://gist.github.com/shaunlebron/8832585>
inline double short_angle_dist(double a1, double a2)
{
  const double da = fmod(a2 - a1, CV_2PI);
  return fmod(2 * da, CV_2PI) - da;
}

//@brief Use of Trey Wilson closed-form solution for shortest distance between two angles (radians) for angle interpolation.
//  Input angles do NOT need to be normalized.
//  Output angles MAY need to be normalized.
//  For more alternatives see for example <https://stackoverflow.com/questions/2708476/rotation-interpolation>
inline double angle_interpolate(double t1, double a1, double t2, double a2, double t)
{
  return a1 + short_angle_dist(a1, a2) * (t - t1) / (t2 - t1);
}

//@brief linear interpolation assuming t1 <= t < t2
inline double linint(double t1, double y1, double t2, double y2, double t)
{
  return y1 + (t - t1) * (y2 - y1) / (t2 - t1);
}

//@brief angle wrapping
inline double wrap_angle_0_2pi(double a)
{
  if ( a < 0 ) {
    a += CV_2PI;
  }
  if ( a >= CV_2PI) {
    a -= CV_2PI;
  }
  return a;
}

//@brief angle wrapping
inline double wrap_angle_minus_pi_plus_pi(double a)
{
  if ( a <= -CV_PI ) {
    a += CV_2PI;
  }
  if ( a > CV_PI ) {
    a -= CV_2PI;
  }
  return a;
}



//@brief Compose 3x4 (R|T) matrix from rotation matrix R and translation vector T.
//      The 3x4 (R|T) matrix is composed from R and T.
//@param   R  3x3 rotation matrix
//@param   T  3x1 translation vector
//@return  RT 3x4 (R|T) matrix
template<class T1, class T2, class T3>
static void compose_rt_matrix(const cv::Matx<T1, 3, 3> & R, const cv::Vec<T2, 3> & T,
    /* out */cv::Matx<T3, 3, 4> & RT)
{
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      RT(i, j) = R(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    RT(i, 3) = T(i);
  }
}

//@brief Compose 4x4 (R|T) matrix from rotation matrix R and translation vector T.
//  The (4x4) RT matrix is composed from R and T by padding with zeros,
//  filling with R|T and setting (R|T)(4,4) to 1.
//@see  readme.txt in
//      <http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip>
//@param   R  3x3 rotation matrix
//@param   T  3x1 translation vector
//@return  RT 4x4 (R|T) matrix
template<class T1, class T2, class T3>
static void compose_rt_matrix(const cv::Matx<T1, 3, 3> & R,
    const cv::Vec<T2, 3> & T,
    /* out */cv::Matx<T3, 4, 4> & RT)
{
  RT = cv::Matx<T3, 4, 4>::zeros();
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      RT(i, j) = R(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    RT(i, 3) = T(i);
  }
  RT(3, 3) = 1;
}



template<class T1, class T2, class T3>
static void split_rt_matrix(const cv::Matx<T3, 3, 4> & RT, /*out*/ cv::Matx<T1, 3, 3> & R, /*out*/ cv::Vec<T2, 3> & T)
{
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      R(i, j) = RT(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    T(i) = RT(i, 3);
  }
}

template<class T1, class T2, class T3>
static void split_rt_matrix(const cv::Matx<T3, 4, 4> & RT, /*out*/ cv::Matx<T1, 3, 3> & R, /*out*/ cv::Vec<T2, 3> & T)
{
  for ( int i = 0; i < 3; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      R(i, j) = RT(i, j);
    }
  }
  for ( int i = 0; i < 3; ++i ) {
    T(i) = RT(i, 3);
  }
}

template<class C>
static void invert_rt_matrix(const cv::Matx<C, 3, 4> & RT,
    /*out*/cv::Matx<C, 3, 4> * RTi)
{
  cv::Matx<C, 3, 3> R;
  cv::Vec<C, 3> T;

  split_rt_matrix(RT, R, T);

  R = R.inv(), T = -R * T;

  compose_rt_matrix(R, T, *RTi);
}

template<class C>
static void invert_rt_matrix(const cv::Matx<C, 3, 3> & R, const cv::Vec3d & T,
    /*out*/cv::Matx<C, 3, 3> * Ri, /*out*/cv::Vec3d * Ti )
{
  *Ri = R.t();
  *Ti = -(*Ri) * T;
}


template<class _Tp>
static inline cv::Vec<_Tp, 3> from_spherical(_Tp phi, _Tp theta)
{
  return cv::Vec<_Tp, 3>(std::sin(theta) * std::cos(phi),
      std::sin(theta) * std::sin(phi),
      std::cos(theta));
}

template<class _Tp>
static inline void to_spherical(const cv::Vec<_Tp, 3> & T, _Tp * phi, _Tp * theha)
{
  *phi = std::atan2(T(1), T(0));
  *theha = std::atan2(std::sqrt(T(0) * T(0) + T(1) * T(1)), T(2));
}

#endif /* __pose_h__ */
