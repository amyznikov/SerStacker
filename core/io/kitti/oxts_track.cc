/*
 * oxts_tarck.cc
 *
 *  Created on: Sep 5, 2019
 *      Author: amyznikov
 */

#include "oxts_track.h"
#include <core/proc/pose.h>


c_oxts_track::c_oxts_track(const std::string & _name )
  : name(_name)
{
}

size_t c_oxts_track::size() const
{
  return oxts_.size();
}

bool c_oxts_track::empty() const
{
  return oxts_.empty();
}

void c_oxts_track::clear()
{
  oxts_.clear();
}

void c_oxts_track::release()
{
  oxts_ = std::vector<oxts_data>();
}

void c_oxts_track::shrink_to_fit()
{
  oxts_.shrink_to_fit();
}

std::vector<oxts_data> & c_oxts_track::oxts()
{
  return oxts_;
}

const std::vector<oxts_data> & c_oxts_track::oxts() const
{
  return oxts_;
}

oxts_data & c_oxts_track::oxts(size_t index)
{
  return oxts_[index];
}

const oxts_data & c_oxts_track::oxts(size_t index) const
{
  return oxts_[index];
}

oxts_data & c_oxts_track::front()
{
  return oxts_.front();
}

const oxts_data & c_oxts_track::front() const
{
  return oxts_.front();
}


oxts_data & c_oxts_track::back()
{
  return oxts_.back();
}

const oxts_data & c_oxts_track::back() const
{
  return oxts_.back();
}


oxts_data & c_oxts_track::middle()
{
  return oxts_[oxts_.size()/2];
}

const oxts_data & c_oxts_track::middle() const
{
  return oxts_[oxts_.size()/2];
}


std::vector<oxts_data>::iterator c_oxts_track::begin()
{
  return oxts_.begin();
}

std::vector<oxts_data>::const_iterator c_oxts_track::begin() const
{
  return oxts_.begin();
}

std::vector<oxts_data>::iterator c_oxts_track::end()
{
  return oxts_.end();
}

std::vector<oxts_data>::const_iterator c_oxts_track::end() const
{
  return oxts_.end();
}


const cv::Matx34d & c_oxts_track::imu2ref() const
{
  return imu2ref_;
}

void c_oxts_track::set_imu2ref(const cv::Matx34d & imu2ref, bool update_inverse)
{
  imu2ref_ = imu2ref;
  if ( update_inverse ) {
    invert_rt_matrix(imu2ref_, &ref2imu_);
  }
}

void c_oxts_track::set_imu2ref(const cv::Matx33d & R, const cv::Vec3f & T, bool update_inverse)
{
  compose_rt_matrix(R, T, imu2ref_);
  if ( update_inverse ) {
    invert_rt_matrix(imu2ref_, &ref2imu_);
  }
}

const cv::Matx34d & c_oxts_track::ref2imu() const
{
  return ref2imu_;
}

void c_oxts_track::set_ref2imu(const cv::Matx34d & ref2imu, bool update_inverse)
{
  ref2imu_ = ref2imu;
  if ( update_inverse ) {
    invert_rt_matrix(ref2imu_, &imu2ref_);
  }
}

void c_oxts_track::set_ref2imu(const cv::Matx33d & R, const cv::Vec3f & T, bool update_inverse)
{
  compose_rt_matrix(R, T, ref2imu_);
  if ( update_inverse ) {
    invert_rt_matrix(ref2imu_, &imu2ref_);
  }
}

void c_oxts_track::sort_by_timestamp()
{
  std::sort(oxts_.begin(), oxts_.end(),
      [](const oxts_data & prev, const oxts_data & next ) {
        return prev.ts < next.ts;
      });
}

bool c_oxts_track::interpolate_for_timestamp(double ts, oxts_data * out) const
{
  return oxts_interpolate_for_timestamp(oxts_, ts, out);
}

void c_oxts_track::apply_oxts_corrections(double roll, double pitch, double yaw, double ts)
{
  for ( oxts_data & oxts : oxts_ ) {
    oxts.roll += roll;
    oxts.pitch += pitch;
    oxts.yaw += yaw;
    oxts.ts += ts;
  }
}


