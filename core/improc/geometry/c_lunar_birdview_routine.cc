/*
 * c_lunar_birdview_routine.cc
 *
 *  Created on: Mar 14, 2026
 *      Author: amyznikov
 */

#include "c_lunar_birdview_routine.h"
#include <core/debug.h>

namespace {
  struct c_lunar_birdview_options {
      double lon = 0;           // Crater longitude [deg]
      double lat = 0;           // Crater latitude [deg]
      double l = 0;             // Libration in longitude [deg]
      double b = 0;             // Libration in latitude [deg]
      double camera_rotation = 0;  // Photo orientation angle [deg]
      c_lunar_birdview_routine::ResizeMode resizeMode;
      int interpolation = cv::INTER_LINEAR;
      int borderMode = cv::BORDER_CONSTANT;
      cv::Scalar borderValue;
  };
}

template<>
const c_enum_member * members_of<c_lunar_birdview_routine::ResizeMode>()
{
  static const c_enum_member members[] = {
      {c_lunar_birdview_routine::ResizeModeKeep, "KEEP", ""},
      {c_lunar_birdview_routine::ResizeModeAdjust, "ADJUST", ""},
      {c_lunar_birdview_routine::ResizeModeCropVisible, "CropVisible", ""},
      {c_lunar_birdview_routine::ResizeModeCropVisible},
  };

  return members;
}

static cv::Matx33d getMoonBirdViewHomography(const c_lunar_birdview_options & opts,
    const cv::Point2d & c)
{
  const double lon = opts.lon * CV_PI / 180.0;
  const double lat = opts.lat * CV_PI / 180.0;
  const double l = opts.l * CV_PI / 180.0;
  const double b = opts.b * CV_PI / 180.0;
  const double camera_rotation = opts.camera_rotation *  CV_PI / 180.0;

  // 3x3 tilt rotation matrix to make horizontal the line from the center of the lunar disk to the crater
  // Scale ca^2 + sa^2 = 1, Ra — pure rotation.
  const double angle_a = std::atan2(sin(lat) * cos(b) - cos(lat) * sin(b) * cos(lon - l), cos(lat) * sin(lon - l));
  const double ca = std::cos(angle_a);
  const double sa = std::sin(angle_a);
  const cv::Matx33d Ra(
      ca, -sa,  c.x * (1.0 - ca) + c.y * sa,
      sa,  ca,  c.y * (1.0 - ca) - c.x * sa,
      0,   0,   1.0);

  // Camera rotation matrix to correct for camera orientation
  const double cc = std::cos(camera_rotation);
  const double sc = std::sin(camera_rotation);
  const cv::Matx33d Rc(
      cc, -sc,  c.x * (1.0 - cc) + c.y * sc,
      sc,  cc,  c.y * (1.0 - cc) - c.x * sc,
      0,   0,   1.0);

  // Total rotation matrix to make horizontal the line from the center of the lunar disk to the crater
  const cv::Matx33d Rt = Ra * Rc;

  // Compute stretch factor (will become in in horz direction after total rotation)
  const double cos_psi = std::max(0.05, std::sin(lat) * std::sin(b) + std::cos(lat) * std::cos(b) * std::cos(lon - l));
  const double s = 1./ cos_psi; // stretch factor
  const cv::Matx33d S(
      s,       0.0,     c.x * (1.0 - s),
      0.0,     1.0,     0.0,
      0.0,     0.0,     1.0
  );

  return Rt.inv() * S * Rt;
}

static void mapMoonToBirdView(cv::InputArray _src, cv::InputArray _src_mask,
    const c_lunar_birdview_options &opts,
    cv::OutputArray _dst, cv::OutputArray _dst_mask) {

  cv::Mat src = _src.getMat();
  if( src.empty() ) {
    return;
  }

  cv::Matx33d H = getMoonBirdViewHomography(opts,
      cv::Point2d(src.cols / 2, src.rows / 2));

  cv::Rect bbox(0, 0, src.cols, src.rows);

  switch ( opts.resizeMode ) {
    case c_lunar_birdview_routine::ResizeModeKeep:
      break;

    case c_lunar_birdview_routine::ResizeModeCropVisible: {
        std::vector<cv::Point2f> corners = {
            { 0, 0 }, { (float) src.cols, 0 },
            { (float) src.cols, (float) src.rows }, { 0, (float) src.rows }
        };

        std::vector<cv::Point2f> dst_corners;
        cv::perspectiveTransform(corners, dst_corners, H);

        cv::Point2f tl = dst_corners[0]; // Top-Left
        cv::Point2f tr = dst_corners[1]; // Top-Right
        cv::Point2f br = dst_corners[2]; // Bottom-Right
        cv::Point2f bl = dst_corners[3]; // Bottom-Left

        float inner_left = std::max(tl.x, bl.x);
        float inner_right = std::min(tr.x, br.x);
        float inner_top = std::max(tl.y, tr.y);
        float inner_bottom = std::min(bl.y, br.y);

        if( inner_right > inner_left && inner_bottom > inner_top ) {
          bbox = cv::Rect(
              (int) inner_left,
              (int) inner_top,
              (int) (inner_right - inner_left),
              (int) (inner_bottom - inner_top)
              );
        }
        else {
          bbox = cv::boundingRect(dst_corners);
        }

        bbox.width = std::min(bbox.width, 4 * src.cols);
        bbox.height = std::min(bbox.height, 4 * src.rows);

        // Add an offset to the matrix so that the top left corner of bbox becomes (0,0)
        cv::Matx33d T(1, 0, -bbox.x, 0, 1, -bbox.y, 0, 0, 1);
        H = T * H;

        bbox.x = 0;
        bbox.y = 0;
        break;
    }
    case c_lunar_birdview_routine::ResizeModeAdjust: {
      // Calculate new bounds so nothing gets cut off
      std::vector<cv::Point2f> corners = {
          { 0, 0 }, { (float) src.cols, 0 },
          { (float) src.cols, (float) src.rows }, { 0, (float) src.rows }
      };

      std::vector<cv::Point2f> dst_corners;
      cv::perspectiveTransform(corners, dst_corners, H);

      bbox = cv::boundingRect(dst_corners);

      bbox.width = std::min(bbox.width, 4 * src.cols);
      bbox.height = std::min(bbox.height, 4 * src.rows);

      // Add an offset to the matrix so that the top left corner of bbox becomes (0,0)
      const cv::Matx33d T( 1, 0, -bbox.x, 0, 1, -bbox.y, 0, 0, 1);
      H = T * H;

      bbox.x = 0;
      bbox.y = 0;
      break;
    }
  }

  // Final image transformation
  cv::warpPerspective(src, _dst, H, bbox.size(),
      opts.interpolation,
      opts.borderMode,
      opts.borderValue);

  // Processing the mask
  if( !_src_mask.empty() ) {
    cv::warpPerspective(_src_mask.getMat(), _dst_mask, H, bbox.size(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
  }
  else {
    cv::warpPerspective(cv::Mat1b(src.size(), 255), _dst_mask, H, bbox.size(),
        cv::INTER_NEAREST,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));
  }

}

void c_lunar_birdview_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Lon [deg]", ctx(&this_class::_lon), "Selenographic Longiture in degreed");
  ctlbind(ctls, "Lat [deg]", ctx(&this_class::_lat), "Selenographic Latitude in degreed");
  ctlbind(ctls, "Libr. l [deg]", ctx(&this_class::_l), "Libration in longitude in degreed");
  ctlbind(ctls, "Libr. b [deg]", ctx(&this_class::_b), "Libration in Latitude in degreed");
  ctlbind_slider_spinbox(ctls, "Camera Orientation [deg]", ctx(&this_class::_camera_rotation), -180, 180, 0.1, "Orientation angle in degrees");
  ctlbind(ctls, "Resize mode", ctx(&this_class::_resizeMode), "Don't crop bounding box");
  ctlbind(ctls, "interpolation", ctx(&this_class::_interpolation), "");
  ctlbind(ctls, "border mode", ctx(&this_class::_borderMode), "");
  ctlbind(ctls, "border value", ctx(&this_class::_borderValue), "");

}

bool c_lunar_birdview_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _lon);
    SERIALIZE_OPTION(settings, save, *this, _lat);
    SERIALIZE_OPTION(settings, save, *this, _l);
    SERIALIZE_OPTION(settings, save, *this, _b);
    SERIALIZE_OPTION(settings, save, *this, _camera_rotation);
    SERIALIZE_OPTION(settings, save, *this, _resizeMode);
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
  opts.lat = _lat;
  opts.lon = _lon;
  opts.l = _l;
  opts.b = _b;
  opts.camera_rotation = _camera_rotation;
  opts.interpolation = _interpolation;
  opts.borderMode = _borderMode;
  opts.borderValue = _borderValue;
  opts.resizeMode = _resizeMode;

  mapMoonToBirdView(src, msk, opts, image, mask);

  return true;
}
