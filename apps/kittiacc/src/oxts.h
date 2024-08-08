/*
 * oxts.h
 *
 *  Created on: Sep 5, 2018
 *      Author: amyznikov
 *
 *  This code is used for some deal with OxTS data.
 *
 *  Ref:
 *    <https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames>
 *
 */
#pragma once
#ifndef __oxts_h__
#define __oxts_h__

#include <opencv2/opencv.hpp>

/**@brief
OXTS/IMU data representation.
Following to KITTI, the ISO 8855 earth-fixed system (ENU: East North Up) is assumed for primary reference frame
*/
struct oxts_data
{
  double ts;        ///<  time stamp [sec]

  double lat;       ///<  latitude of the oxts-unit [-pi/2..pi/2]
  double lon;       ///<  longitude of the oxts-unit [0..2pi]
  double alt;       ///<  altitude of the oxts-unit  [m]

  double roll;      ///<  roll (bank) angle,  0 = level, positive = left side up [-pi..pi]
  double pitch;     ///<  pitch angle, 0 = level, positive = front down [-pi/2..pi/2]
  double yaw;       ///<  yaw (heading) angle, 0 = east,  positive = counter clockwise [-pi..pi]

  double vn;        ///<  velocity towards north [m/s]
  double ve;        ///<  velocity towards east [m/s]
  double vu;        ///<  upward velocity, i.e. perpendicular to earth-surface [m/s]


  double vf;        ///<  forward velocity, i.e. parallel to earth-surface [m/s]
  double vl;        ///<  leftward velocity, i.e. parallel to earth-surface [m/s]

  double ax;        ///<  acceleration in x, i.e. in direction of vehicle front (m/s^2)
  double ay;        ///<  acceleration in y, i.e. in direction of vehicle left (m/s^2)
  double az;        ///<  acceleration in z, i.e. in direction of vehicle top (m/s^2)

  double af;        ///<  forward acceleration (m/s^2)
  double al;        ///<  leftward acceleration (m/s^2)
  double au;        ///<  upward acceleration (m/s^2)

  double wx;        ///<  angular rate around x (rad/s)
  double wy;        ///<  angular rate around y (rad/s)
  double wz;        ///<  angular rate around z (rad/s)

  double wf;        ///<  angular rate around forward axis (rad/s)
  double wl;        ///<  angular rate around leftward axis (rad/s)
  double wu;        ///<  angular rate around upward axis (rad/s)

  double posacc;    ///<  position accuracy (m)
  double velacc;    ///<  velocity accuracy (north/east in m/s)

  int navstat;      ///<  navigation status
  int numsats;      ///<  number of satellites tracked by primary GPS receiver
  int posmode;      ///<  position mode of primary GPS receiver
  int velmode;      ///<  velocity mode of primary GPS receiver
  int orimode;      ///<  orientation mode of primary GPS receiver
};



///@brief just resets all oxts data fields to zeros
inline void oxts_clear(struct oxts_data * c)
{
  memset(c, 0, sizeof(*c));
}



///@brief lower_bound() returns first position 'pos' at which pos->ts >= ts
template<class OxtsArrayIterator>
inline OxtsArrayIterator oxts_lower_bound(const OxtsArrayIterator & begin, const OxtsArrayIterator & end, double ts)
{
  return std::lower_bound(begin, end, ts,
      [](const oxts_data & oxts, double timestamp) -> bool {
        return oxts.ts < timestamp;
      });
}

///@brief upper_bound() returns first position 'pos' at which pos->ts > ts
template<class OxtsArrayIterator>
inline OxtsArrayIterator oxts_upper_bound(const OxtsArrayIterator & begin, const OxtsArrayIterator & end, double ts)
{
  return std::upper_bound(begin, end, ts,
      [](double timestamp, const oxts_data & oxts) -> bool {
        return timestamp < oxts.ts;
      });
}


///@brief Uses oxts_lower_bound() to find first index 'i' in oxts[] array at which oxts[i].ts >= ts
/// DEPRECATED, is not removed yet only for compatibility with obsolete code.
inline size_t oxts_binary_search_timestamp(const std::vector<oxts_data> & oxts, double ts)
{
  return oxts_lower_bound(oxts.begin(), oxts.end(), ts) - oxts.begin();
}


///@brief Uses oxts_lower_bound() to find first oxts record  which oxts_out.ts >= ts
/// Not sure this routine is really useful, but it is still used somewhere in code.
bool oxts_binary_search(const std::vector<oxts_data> & oxts, double ts,
    /* out*/struct oxts_data * oxts_out);

///@brief Interpolate between oxts1 and oxts2 for requested time stamp ts,
///  and return interpolated values into oxtsi.
/// Note that actually the extrapolation will made if ts is out of [oxts1..oxts2] timestamps range
void oxts_interpolate(const oxts_data & oxts1, const oxts_data & oxts2, double ts,
    /*out*/ oxts_data * oxtsi);

///@brief Search oxts by time stamp assuming oxts[] is sorted by increasing oxts.ts,
///   interpolate for specific ts, and return interpolated values into oxtsi.
/// Note that actually the extrapolation will made if ts is out of [oxts1..oxts2] timestamps range
///
/// This function returns false if requested ts is not in the range covered by oxts[] timestamps,
/// therefore data stored in oxtsi are extrapolated (not interpolated)
bool oxts_interpolate_for_timestamp(const std::vector<oxts_data> & oxts, double ts,
    /*out*/ oxts_data * oxtsi);


///@brief  OXTS PATH INTEGRATOR.
/// NOT DOCUMENTED BECAUSE IS STILL HIGHLY EXPERIMENTAL,
/// DO NOT REVIEW OR USE IT!!!!
void oxts_integrate(const oxts_data & oxts_start, const oxts_data & oxts_end,
    const std::vector<oxts_data> & oxts_path,
    /* out */ cv::Vec3d * CumulativeTranslation,
    /* out */ cv::Matx33d * CumulativeRotation);

//! @} oxts


///////////////////////////////////////////////////////////////////////////////
#endif /* __oxts_h__ */
