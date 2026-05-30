/*
 * c_jovian_ellipse_detector.h
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_jovian_ellipse_detector_h__
#define __c_jovian_ellipse_detector_h__

#include <opencv2/opencv.hpp>
#include <core/proc/extract_channel.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/feature2d/ellipsoid.h>

enum JOVIAN_ELLIPSE_DETECTION_METHOD {
  JOVIAN_ELLIPSE_DETECTION_PCA,
  JOVIAN_ELLIPSE_DETECTION_STENSOR,
};

struct c_jovian_ellipse_detector_options
{
  JOVIAN_ELLIPSE_DETECTION_METHOD method = JOVIAN_ELLIPSE_DETECTION_STENSOR;
  color_channel_type gradient_channel = color_channel_min_inensity;
  c_simple_planetary_disk_detector_options planetary_disk_detector_options;
  double planetary_disk_tilt = 0; // [deg]
  double sigma_noise = 3;
  int nscale = 2;
  double neps = 1e-3;
  cv::Point2f offset;
  bool gweighted = true;
};

bool serialize_base_jovian_ellipse_detector_options(c_config_setting section, bool save,
    c_jovian_ellipse_detector_options & opts);

inline bool save_settings(c_config_setting section, const c_jovian_ellipse_detector_options & opts)
{
  return serialize_base_jovian_ellipse_detector_options(section, true,
      const_cast<c_jovian_ellipse_detector_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_jovian_ellipse_detector_options * opts)
{
  return serialize_base_jovian_ellipse_detector_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_jovian_ellipse_detector_options> & ctx)
{
  using S = c_jovian_ellipse_detector_options;

  ctlbind(ctls, "method", ctx(&S::method), "");
  ctlbind(ctls, "gradient_channel", ctx(&S::gradient_channel), "");
  ctlbind(ctls, "sigma_noise", ctx(&S::sigma_noise), "");
  ctlbind(ctls, "nscale", ctx(&S::nscale), "");
  ctlbind(ctls, "neps", ctx(&S::neps), "");
  ctlbind(ctls, "gweighted", ctx(&S::gweighted), "");
  ctlbind(ctls, "tilt [deg]", ctx(&S::planetary_disk_tilt), "");
  ctlbind(ctls, "offset [px]", ctx(&S::offset), "");

  ctlbind_expandable_group(ctls, "planetary disk detection",
      [&, ctx = CTL_CONTEXT(ctx, planetary_disk_detector_options)]() {
        ctlbind(ctls, ctx);
      });

  ctlbind_menu_button(ctls, "Options >>", ctx);
  ctlbind_item(ctls, "Copy parameters to clipboard", ctx, [](const auto * obj) {
    return ctlbind_copy_config_to_clipboard("c_jovian_ellipse_detector_options",*obj), false;
  });
  ctlbind_item(ctls, "Paste parameters from clipboard", ctx, [](auto * obj) {
      return ctlbind_paste_config_from_clipboard("c_jovian_ellipse_detector_options", obj);
    });
}

class c_jovian_ellipse_detector
{
public:
  void set_options(const c_jovian_ellipse_detector_options & v);
  const c_jovian_ellipse_detector_options& options() const;
  c_jovian_ellipse_detector_options& options();

  // main method
  bool detect_jovian_ellipse(cv::InputArray _image, cv::InputArray mask = cv::noArray());
  const cv::RotatedRect& final_planetary_disk_ellipse() const;

  // for debug / visualization
  const cv::Mat& grayscale_image() const;
  const cv::Mat& normalized_image() const;
  const cv::Mat1b & gradient_mask() const;
  const cv::Mat1f & g_image() const;
  const cv::Mat1f & gx_image() const;
  const cv::Mat1f & gy_image() const;
  const cv::Mat1b& detected_planetary_disk_mask() const;
  const cv::Mat1b& detected_planetary_disk_edge() const;
  const cv::Mat1b& final_planetary_disk_mask() const;

  const cv::Point2d & center() const
  {
    return _center;
  }
  const cv::Vec3d & axes() const
  {
    return _axes;
  }
  const cv::Vec3d & pose() const
  {
    return _pose;
  }

protected:
  double compute_jovian_orientation_stensor();
  double compute_jovian_orientation_pca();

protected:
  c_jovian_ellipse_detector_options _opts;

  cv::Mat _grayscale_image;
  cv::Mat _normalized_image;
  cv::Mat1b _detected_planetary_disk_mask;
  cv::Mat1b _gradient_mask;
  cv::Mat1f _kderiv, _ksmooth;
  cv::Mat1f _gx, _gy, _g;

  cv::Mat1b _detected_planetary_disk_edge;
  cv::Mat1b _final_planetary_disk_mask;

  //cv::RotatedRect _ellipseAMS;
  cv::Rect _detected_component_rect;
  cv::RotatedRect _final_planetary_disk_ellipse;

  cv::Point2d _center;
  cv::Vec3d _axes;
  cv::Vec3d _pose;
};




#endif /* __c_jovian_ellipse_detector_h__ */

