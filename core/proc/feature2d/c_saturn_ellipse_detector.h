/*
 * c_saturn_ellipse_detector.h
 *
 *  Created on: Jul 14, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_saturn_ellipse_detector_h__
#define __c_saturn_ellipse_detector_h__

#include <opencv2/opencv.hpp>
#include <core/proc/extract_channel.h>
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/feature2d/ellipsoid.h>

struct c_saturn_ellipse_detector_options
{
  color_channel_type gradient_channel = color_channel_min_inensity;
  c_simple_planetary_disk_detector_options planetary_disk_detector_options;
  double planetary_disk_tilt = 3; // [deg]
  double sigma_contour = 4;
  int nscale = 2;
  double neps = 1e-3;
  cv::Point2f offset;
  bool lmweighted = true;
};

bool serialize_base_saturn_ellipse_detector_options(c_config_setting section, bool save,
    c_saturn_ellipse_detector_options & opts);

inline bool save_settings(c_config_setting section, const c_saturn_ellipse_detector_options & opts)
{
  return serialize_base_saturn_ellipse_detector_options(section, true,
      const_cast<c_saturn_ellipse_detector_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_saturn_ellipse_detector_options * opts)
{
  return serialize_base_saturn_ellipse_detector_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType,
    c_saturn_ellipse_detector_options> & ctx)
{
  using S = c_saturn_ellipse_detector_options;

  //ctlbind(ctls, "method", ctx(&S::method), "");
  ctlbind(ctls, "gradient_channel", ctx(&S::gradient_channel), "");
  ctlbind(ctls, "sigma_contour", ctx(&S::sigma_contour), "");
  //ctlbind(ctls, "sigma_clouds", ctx(&S::sigma_clouds), "");
  ctlbind(ctls, "nscale", ctx(&S::nscale), "");
  ctlbind(ctls, "neps", ctx(&S::neps), "");
  //ctlbind(ctls, "gweighted", ctx(&S::gweighted), "");
  ctlbind(ctls, "lmweighted", ctx(&S::lmweighted), "");

  ctlbind(ctls, "tilt [deg]", ctx(&S::planetary_disk_tilt), "");
  ctlbind(ctls, "offset [px]", ctx(&S::offset), "");

  ctlbind_expandable_group(ctls, "planetary disk detection",
      [&, ctx = CTL_CONTEXT(ctx, planetary_disk_detector_options)]() {
        ctlbind(ctls, ctx);
      });
}

class c_saturn_ellipse_detector
{
public:
  static constexpr double k_saturn_axis_ratio = 0.898;

  void set_options(const c_saturn_ellipse_detector_options & opts);
  const c_saturn_ellipse_detector_options& options() const;

  // main methods
  bool detect(cv::InputArray _image, cv::InputArray mask = cv::noArray());
  void clear();

  // for debug / visualization
  const cv::Mat& grayscale_image() const;
  const cv::Mat& gradient_image() const;
  const cv::Mat1b& initial_mask() const;
  const cv::Mat1b& pca_mask() const;
  const cv::RotatedRect & pca_rect() const;
  const cv::Mat1b & skirt_mask() const;
  const cv::Mat1f & gx_image() const;
  const cv::Mat1f & gy_image() const;
  const cv::Mat1f & gr_image() const;
  const cv::Mat1f & grth_image() const;
  const cv::RotatedRect & skirt_roi() const;

  const cv::RotatedRect & final_planetary_disk_ellipse() const
  {
    return _final_planetary_disk_ellipse;
  }

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

  const cv::Mat & debugImage() const
  {
    return _debugImage;
  }

protected:
  bool detect_initial_mask(cv::InputArray input_image, cv::InputArray input_mask);
  bool detect_pca_rect();
  bool compute_radial_gradient(cv::InputArray input_image, cv::InputArray input_mask);

protected:
  c_saturn_ellipse_detector_options _opts;
  cv::Mat _grayscale_image;
  cv::Mat1f _gradient_image;
  cv::Mat1b _initial_mask;
  cv::Mat1b _pca_mask;
  cv::Mat1b _skirt_mask;
  cv::Mat1f _gx, _gy, _gr, _grth;
  cv::Mat _debugImage;

  double _apparent_axis_ratio = k_saturn_axis_ratio;
  cv::Point2f _detected_component_centroid;
  cv::Rect _detected_component_roi;
  cv::Rect _pca_roi;
  cv::RotatedRect _pca_rect;
  cv::RotatedRect _skirt_roi;
  cv::RotatedRect _final_planetary_disk_ellipse;

  cv::Point2d _center;
  cv::Vec3d _axes;
  cv::Vec3d _pose;
};

#endif /* __c_saturn_ellipse_detector_h__ */
