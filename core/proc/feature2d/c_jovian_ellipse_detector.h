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

enum JOVIAN_ELLIPSE_DETECTION_METHOD {
  JOVIAN_ELLIPSE_DETECTION_PCA,
  JOVIAN_ELLIPSE_DETECTION_STENSOR,
};

struct c_jovian_ellipse_detector_options
{
  JOVIAN_ELLIPSE_DETECTION_METHOD method = JOVIAN_ELLIPSE_DETECTION_STENSOR;
  color_channel_type maxcolor_channel = color_channel_min_inensity;
  c_simple_planetary_disk_detector_options planetary_disk_detector_options;
  double sigma_noise = 5;
  cv::Point2f offset;
  bool g2 = true;
  bool gweighted = true;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_jovian_ellipse_detector_options> & ctx)
{
  using S = c_jovian_ellipse_detector_options;

  ctlbind(ctls, "method", ctx(&S::method), "");
  ctlbind(ctls, "gradient_channel", ctx(&S::maxcolor_channel), "");
  ctlbind(ctls, "g2", ctx(&S::g2), "");
  ctlbind(ctls, "gweighted", ctx(&S::gweighted), "");
  ctlbind(ctls, "sigma_noise", ctx(&S::sigma_noise), "");
  ctlbind(ctls, "offset", ctx(&S::offset), "");
  ctlbind_expandable_group(ctls, "planetary disk detection",
      [&, ctx = CTL_CONTEXT(ctx, planetary_disk_detector_options)]() {
        ctlbind(ctls, ctx);
      });
}

bool serialize_base_jovian_ellipse_detector_options(c_config_setting section, bool save,
    c_jovian_ellipse_detector_options & opts);

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
  const cv::Mat& maxcolor_image() const;
  const cv::Mat1b & maxcolor_mask() const;
  const cv::Mat1f & g_image() const;
  const cv::Mat1f & gx_image() const;
  const cv::Mat1f & gy_image() const;
  const cv::Mat1b& detected_planetary_disk_mask() const;
  const cv::Mat1b& detected_planetary_disk_edge() const;
  const cv::Mat1b& final_planetary_disk_mask() const;

protected:
  double compute_jovian_orientation_stensor();
  double compute_jovian_orientation_pca();

protected:
  c_jovian_ellipse_detector_options _opts;

  cv::Mat _grayscale_image;
  cv::Mat _maxcolor_image;
  cv::Mat1b _detected_planetary_disk_mask;
  cv::Mat1b _maxcolor_mask;
  cv::Mat1f _kderiv, _ksmooth;
  cv::Mat1f _gx, _gy, _g;

  cv::Mat1b _detected_planetary_disk_edge;
  cv::Mat1b _final_planetary_disk_mask;

  //cv::RotatedRect _ellipseAMS;
  cv::Rect _detected_component_rect;
  cv::RotatedRect _final_planetary_disk_ellipse;
};

cv::Rect compute_ellipse_bounding_box(const cv::RotatedRect & rc);
cv::Rect compute_ellipse_crop_box(const cv::RotatedRect & rc, const cv::Size & total_image_size, int margin = -1);


#endif /* __c_jovian_ellipse_detector_h__ */

