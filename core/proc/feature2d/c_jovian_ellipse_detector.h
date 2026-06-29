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
#include <core/ctrlbind/ctrlbind.h>
#include <core/settings.h>
#include "planetary-disk-detection.h"

enum JOVIAN_ELLIPSE_DETECTION_METHOD {
  JOVIAN_ELLIPSE_DETECTION_RADON_FFT,
  JOVIAN_ELLIPSE_DETECTION_STENSOR,
};

struct c_jovian_ellipse_detector_options
{
  JOVIAN_ELLIPSE_DETECTION_METHOD method = JOVIAN_ELLIPSE_DETECTION_RADON_FFT;
  c_simple_planetary_disk_detector_options planetary_disk_detector_options;
  double planetary_disk_tilt = 3; // [deg]
  double sigma_clouds = 3;
  int nscale = 2;
  int skirt_iterations = 2;
  cv::Point2f offset;
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
  ctlbind(ctls, "sigma_clouds", ctx(&S::sigma_clouds), "");
  ctlbind(ctls, "nscale", ctx(&S::nscale), "");
  ctlbind(ctls, "skirt_iterations", ctx(&S::skirt_iterations), "");
  ctlbind(ctls, "tilt [deg]", ctx(&S::planetary_disk_tilt), "");
  ctlbind(ctls, "offset [px]", ctx(&S::offset), "");

  ctlbind_expandable_group(ctls, "planetary disk detection",
      [&, ctx = CTL_CONTEXT(ctx, planetary_disk_detector_options)]() {
        ctlbind(ctls, ctx);
      });
}

class c_jovian_ellipse_detector
{
public:
  static constexpr double k_jovian_axis_ratio = 0.93512560845968779724;

  // main methods
  bool detect(cv::InputArray _image, cv::InputArray mask = cv::noArray());
  void clear();

  void set_options(const c_jovian_ellipse_detector_options & v)
  {
    _opts = v;
  }

  const c_jovian_ellipse_detector_options& options() const
  {
    return _opts;
  }

  void setEnableDebugImages(bool v)
  {
    _enableDebugImages = v;
  }

  bool enableDebugImages() const
  {
    return _enableDebugImages;
  }

  const cv::RotatedRect & finalPlanetaryDiskEllipse() const
  {
    return _finalPlanetaryDiskEllipse;
  }

  const cv::Mat1b& finalPlanetaryDiskMask() const
  {
    return _finalPlanetaryDiskMask;
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

  // for debug / visualization

  const cv::Rect & cropRC() const
  {
    return _cropRC;
  }

  const cv::Size cropSize() const
  {
    return _cropRC.size();
  }

  const std::vector<cv::Point3f> & edge_points() const
  {
    return _edge_points;
  }

  const cv::Mat1f & apodizationWindow() const
  {
    return _apodizationWindow;
  }

  const cv::Mat1f radonMagnitude() const
  {
    return _radonMagnitude;
  }

  const cv::Mat1f & vlap() const
  {
    return VLAP;
  }

  const cv::Mat& grayscaleImage() const
  {
    return _grayscaleInputImage;
  }

  const cv::Mat& grayscaleCrop() const
  {
    return _grayscaleImageCrop;
  }

  const cv::Mat1b& detectedPlanetaryDiskMask() const
  {
    return _planetaryDiskMask;
  }

  const cv::Mat1f & gxImage() const
  {
    return _gx;
  }

  const cv::Mat1f & gyImage() const
  {
    return _gy;
  }

  const cv::Mat1f & gradientImage() const
  {
    return _g;
  }

  const cv::Mat1f & radialGradientImage() const
  {
    return _gr;
  }

  const cv::Mat1f & radialGradientTopHatImage() const
  {
    return _grth;
  }

  const cv::Mat1b& skirtMask() const
  {
    return _skirtMask;
  }

  const cv::Mat1f & skirtPolar ()const
  {
    return _skirtPolar;
  }


protected:
  double compute_jovian_orientation_radon_fft();
  double compute_jovian_orientation_stensor();

protected:
  c_jovian_ellipse_detector_options _opts;
  bool _enableDebugImages = false;

  cv::Point2d _center;
  cv::Vec3d _axes;
  cv::Vec3d _pose;

  cv::Mat _grayscaleInputImage;
  cv::Mat1f _grayscaleImageCrop;
  cv::Mat1f _gx, _gy, _g, _gr, _grth;
  cv::Mat1b _planetaryDiskMask;
  cv::Mat1b _finalPlanetaryDiskMask;
  cv::Mat1b _skirtMask;
  std::vector<cv::Point3f> _edge_points;
  cv::Rect _cropRC;

  cv::Point2f _planetaryDiskGeometricalCenter;
  cv::Point2f _planetaryDiskCentroid;
  cv::Rect _planetaryDiskROI;
  cv::RotatedRect _finalPlanetaryDiskEllipse;

  cv::Mat1f _apodizationWindow;
  cv::Mat1f _radonMagnitude;
  cv::Mat1f VLAP;

  cv::Mat1f _skirtPolar;
};




#endif /* __c_jovian_ellipse_detector_h__ */

