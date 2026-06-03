/*
 * c_jovian_derotation_remap.h
 *
 *  Created on: May 25, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_derotation_remap_h__
#define __c_jovian_derotation_remap_h__

#include <opencv2/opencv.hpp>
#include <core/proc/feature2d/ellipsoid.h>
#include <core/ctrlbind/ctrlbind.h>

struct c_jovian_derotation_remap_options
{
  // Jupiter daily rotation period
  double jovian_rotation_period_sec = 9 * 3600. + 55 * 60. + 40.632;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_jovian_derotation_remap_options> & ctx)
{
  using S = c_jovian_derotation_remap_options;
  ctlbind(ctls, "Rotation Period [sec]", ctx(&S::jovian_rotation_period_sec),
      "Set Jovian daily rotation period in sec");
  ctlbind_menu_button(ctls, "Standard roration periods >>", ctx);
  ctlbind_item(ctls, "System I  : 9h 50m 30.003s", ctx, [](auto * obj) {
      obj->jovian_rotation_period_sec = 9 * 3600 + 50 * 60 + 30.003;
      return true;
    });
  ctlbind_item(ctls, "System II : 9h 55m 40.632s", ctx, [](auto * obj) {
      obj->jovian_rotation_period_sec = 9 * 3600 + 55 * 60 + 40.632;
      return true;
    });
  ctlbind_item(ctls, "System III: 9h 55m 30s", ctx, [](auto * obj) {
      obj->jovian_rotation_period_sec = 9 * 3600 + 55 * 60 + 30;
      return true;
    });
}


class c_jovian_derotation_remap
{
public:
  void set_opts(const c_jovian_derotation_remap_options & opts);
  const c_jovian_derotation_remap_options & opts() const;

  void set_reference_pose(const cv::Size & image_size, const cv::Point2d & center, const cv::Vec3d & axes, const cv::Vec3d & pose);
  void compute_derotation_for_angle(double longitude_rotation_radians, double wscale = 1);
  void compute_derotation_for_time(double deltat_sec, double wscale = 1);

  const cv::Mat2f & rmap() const
  {
    return _rmap;
  }

  const cv::Mat1f & wmap() const
  {
    return _wmap;
  }

  const cv::Mat1b & rmask() const
  {
    return _rmask;
  }

  const cv::Mat1f & rcounter() const
  {
    return _rcounter;
  }

  const cv::Size & image_size() const
  {
    return _image_size;
  }

  const cv::Point2d & center() const
  {
    return _center;
  }

  const cv::Vec3d & axes() const
  {
    return _axes;
  }

  const cv::Vec3d & current_pose() const
  {
    return _current_pose;
  }

  const cv::Vec3d & target_pose() const
  {
    return _target_pose;
  }

  const cv::Matx33d & Rcurrent() const
  {
    return _Rcurrent;
  }

  const cv::Matx33d & Rtarget() const
  {
    return _Rtarget;
  }

protected:
  c_jovian_derotation_remap_options _opts;
  cv::Size _image_size;
  cv::Point2d _center;
  cv::Vec3d _axes;
  cv::Vec3d _current_pose;
  cv::Vec3d _target_pose;

  cv::Matx33d _Rcurrent;
  cv::Matx33d _Rtarget;

  cv::Mat2f _rmap;
  cv::Mat1f _wmap;
  cv::Mat1b _rmask;
  cv::Mat1f _rcounter;
};

#endif /* __c_jovian_derotation_remap_h__ */
