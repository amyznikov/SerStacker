/*
 * c_gpx_interpolation.h
 *
 *  Created on: Sep 15, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_gpx_interpolation_h__
#define __c_gpx_interpolation_h__

#include "gpx.h"

class c_gpx_interpolation
{
public:
  typedef c_gpx_interpolation
      this_class;

  struct Landmark
  {
    int videoFrameIndex;
    int gpxPointIndex;
  };

  typedef std::vector<Landmark>::iterator
      landmark_iterator;

  typedef std::vector<Landmark>::const_iterator
      landmark_const_iterator;

  void set_gpx_track(const c_gpx_track * track);
  const c_gpx_track * gpx_track() const;

  void set_landmark(int videoFrameIndex, int gpxPointIndex);
  void remove_landmark(int videoFrameIndex);

  landmark_iterator landmark_lower_bound(int videoFrameIndex) ;
  landmark_const_iterator landmark_lower_bound(int videoFrameIndex) const;

  bool interpolate_for_frame(int videoFrameIndex,
      c_gps_position * gps) const;

//protected:
//  bool interpolate_for_frame(int targetVideoFrameIndex,
//      int landmark0, int landmark1,
//      c_gps_position * gps) const;

protected:
  const c_gpx_track * _gpx_track = nullptr;
  std::vector<Landmark> _landmarks;

};

#endif /* __c_gpx_interpolation_h__ */
