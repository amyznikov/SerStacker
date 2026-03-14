/*
 * c_lunar_birdview_routine.cc
 *
 *  Created on: Mar 14, 2026
 *      Author: amyznikov
 */

#include "c_lunar_birdview_routine.h"


#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

namespace {
  struct c_lunar_birdview_options {
      double lon = 0;          // Crater longitude [deg]
      double lat = 0;          // Crater latitude [deg]
      double orientation = 0;  // Photo orientation angle [deg]
      int interpolation = cv::INTER_LINEAR;
      int borderMode = cv::BORDER_CONSTANT;
      cv::Scalar borderValue;
  };
}

static void mapToBirdView(cv::InputArray _src, cv::InputArray _src_mask,
    const c_lunar_birdview_options &opts,
    cv::OutputArray _dst, cv::OutputArray _dst_mask) {

  cv::Mat src = _src.getMat();
  if( src.empty() ) {
    return;
  }

  // Stretching factors for perspective compensation (spherical projection)
  // The further from the disk center (0,0), the greater the compression.
  const double kx = 1.0 / std::cos(opts.lon * CV_PI / 180.0);
  const double ky = 1.0 / std::cos(opts.lat * CV_PI / 180.0);

  // Rotation matrix around the image center
  const cv::Point2f center(src.cols / 2.0f, src.rows / 2.0f);
  const cv::Matx23d R = cv::getRotationMatrix2D(center, opts.orientation, 1.0);

  const cv::Matx33d H_rot(
      R(0, 0), R(0, 1), R(0, 2),
      R(1, 0), R(1, 1), R(1, 2),
      0.0, 0.0, 1.0);

  // Scale matrix
  // Stretch coordinates from the center to "unfold" the ellipse into a circle
  // Correcting the center after stretching (centering back)
  cv::Matx33d H_stretch(
      kx, 0.0, center.x * (1.0 - kx),
      0.0, ky, center.y * (1.0 - ky),
      0.0, 0.0, 1.0);

  // Homography matrix
  cv::Matx33d H_total = H_rot.inv() * H_stretch * H_rot;

  // Calculate new bounds so nothing gets cut off
  std::vector<cv::Point2f> corners = {
      { 0, 0 }, { (float) src.cols, 0 },
      { (float) src.cols, (float) src.rows }, { 0, (float) src.rows }
  };

  std::vector<cv::Point2f> dst_corners;
  cv::perspectiveTransform(corners, dst_corners, H_total);

  const cv::Rect bbox = cv::boundingRect(dst_corners);

  // Add an offset to the matrix so that the top left corner of bbox becomes (0,0)
  const cv::Matx33d T(
      1, 0, -bbox.x,
      0, 1, -bbox.y,
      0, 0, 1);

  H_total = T * H_total;

  // Final image transformation
  cv::warpPerspective(src, _dst, H_total, bbox.size(),
      opts.interpolation,
      opts.borderMode,
      opts.borderValue);

  // Processing the mask
  if( !_src_mask.empty() ) {
    cv::warpPerspective(_src_mask.getMat(), _dst_mask, H_total, bbox.size(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
  }
  else {
    cv::warpPerspective(cv::Mat1b(src.size(), 255), _dst_mask, H_total, bbox.size(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
  }
}


void c_lunar_birdview_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Lat [deg]", ctx(&this_class::_lat), "Selenographic Latitude in degreed");
  ctlbind(ctls, "Lon [deg]", ctx(&this_class::_lon), "Selenographic Longiture in degreed");
  ctlbind_slider_spinbox(ctls, "Orientation [deg]", ctx(&this_class::_rotation), -180, 180, 0.1, "Orientation angle in degrees");
  ctlbind(ctls, "interpolation", ctx(&this_class::_interpolation), "");
  ctlbind(ctls, "border mode", ctx(&this_class::_borderMode), "");
  ctlbind(ctls, "border value", ctx(&this_class::_borderValue), "");
}

bool c_lunar_birdview_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _lat);
    SERIALIZE_OPTION(settings, save, *this, _lon);
    SERIALIZE_OPTION(settings, save, *this, _rotation);
    SERIALIZE_OPTION(settings, save, *this, _interpolation);
    SERIALIZE_OPTION(settings, save, *this, _borderMode);
    SERIALIZE_OPTION(settings, save, *this, _borderValue);
    return true;
  }
  return false;
}

bool c_lunar_birdview_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Mat src = image.getMat();
  const cv::Mat msk = mask.getMat();

  c_lunar_birdview_options opts;
  opts.lon = _lon;
  opts.lat = _lat;
  opts.orientation = _rotation;
  opts.interpolation = _interpolation;
  opts.borderMode = _borderMode;
  opts.borderValue = _borderValue;

  mapToBirdView(src, msk, opts, image, mask);

  return true;
}
