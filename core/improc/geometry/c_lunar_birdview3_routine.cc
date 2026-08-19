/*
 * c_lunar_birdview3_routine.cc
 *
 *  Created on: Aug 19, 2026
 *      Author: amyznikov
 */

#include "c_lunar_birdview3_routine.h"

template<>
const c_enum_member * members_of<c_lunar_birdview3_routine::MapProjection>()
{
  static const c_enum_member members[] = {
      { c_lunar_birdview3_routine::MapOrthographic, "Orthographic", "" },
      { c_lunar_birdview3_routine::MapStereographic, "Stereographic", "" },
      { c_lunar_birdview3_routine::MapStereographic}
  };
  return members;
}

template<>
const c_enum_member * members_of<c_lunar_birdview3_routine::ResizeMode>()
{
  static const c_enum_member members[] = {
      {c_lunar_birdview3_routine::ResizeModeKeep, "KEEP", ""},
      {c_lunar_birdview3_routine::ResizeModeAdjust, "ADJUST", ""},
      {c_lunar_birdview3_routine::ResizeModeCropVisible, "CropVisible", ""},
      {c_lunar_birdview3_routine::ResizeModeCropVisible},
  };

  return members;
}

namespace {
using MapProjection = c_lunar_birdview3_routine::MapProjection;
using ResizeMode = c_lunar_birdview3_routine::ResizeMode;

struct c_lunar_birdview3_options
{
  cv::Point2d rpx[3];   // 3 reference craters pixel coordinates x;y
  cv::Point2d rse[3];   // 3 reference craters selenographic coordinates (lon,lat) [deg]
  double l = 0;             // Libration in longitude [deg]
  double b = 0;             // Libration in latitude [deg]
  //double camera_rotation = 0;  // Photo orientation angle [deg]
  MapProjection projection = c_lunar_birdview3_routine::MapStereographic;
  ResizeMode resizeMode = c_lunar_birdview3_routine::ResizeModeAdjust;
  int interpolation = cv::INTER_LINEAR;
  int borderMode = cv::BORDER_CONSTANT;
  cv::Scalar borderValue;
};

static bool createMoonRemapThreePoints(
    const c_lunar_birdview3_options & opts,
    const cv::Size &src_size,
    cv::Size &dst_size,
    cv::Mat2f &map)
{
  const double l = opts.l * CV_PI / 180.0;
  const double b = opts.b * CV_PI / 180.0;

  // Convert longitudes to the range [-180, 180]
  double lons[3];
  double lat0 = 0, lon0 = 0;

  for( int i = 0; i < 3; ++i ) {
    double lon = opts.rse[i].x;
    while (lon > 180.0) {
      lon -= 360.0;
    }
    while (lon < -180.0) {
      lon += 360.0;
    }
    lons[i] = lon;
    lon0 += lon;
    lat0 += opts.rse[i].y;
  }
  lon0 = (lon0 / 3.0) * CV_PI / 180.0;
  lat0 = (lat0 / 3.0) * CV_PI / 180.0;

  // Project 3 reference points onto the observer's visible disk
  cv::Point2f sph_pts[3];
  for( int i = 0; i < 3; ++i ) {
    double lon_p = lons[i] * CV_PI / 180.0;
    double lat_p = opts.rse[i].y * CV_PI / 180.0;
    double d_lon = lon_p - l;
    double cos_lat = std::cos(lat_p);
    double sin_lat = std::sin(lat_p);

    sph_pts[i].x = float(cos_lat * std::sin(d_lon));
    sph_pts[i].y = float(-(sin_lat * std::cos(b) - cos_lat * std::sin(b) * std::cos(d_lon)));
  }

  // Affine matrix T of the camera
  cv::Point2f img_pts[3];
  for(int i = 0; i < 3; ++i) {
      img_pts[i] = cv::Point2f(static_cast<float>(opts.rpx[i].x), static_cast<float>(opts.rpx[i].y));
  }

  cv::Mat src_tri(3, 1, CV_32FC2, sph_pts);
  cv::Mat dst_tri(3, 1, CV_32FC2, img_pts);
  cv::Mat T = cv::getAffineTransform(src_tri, dst_tri);

  double m00 = T.at<double>(0, 0); double m01 = T.at<double>(0, 1); double m02 = T.at<double>(0, 2);
  double m10 = T.at<double>(1, 0); double m11 = T.at<double>(1, 1); double m12 = T.at<double>(1, 2);

  double calculated_R = std::hypot(m00, m10);
  double map_scale = calculated_R; // Scale 1:1 in the projection center

  // Frame boundaries (BBOX)
  cv::Mat T_inv;
  cv::invertAffineTransform(T, T_inv);
  double im00 = T_inv.at<double>(0, 0); double im01 = T_inv.at<double>(0, 1); double im02 = T_inv.at<double>(0, 2);
  double im10 = T_inv.at<double>(1, 0); double im11 = T_inv.at<double>(1, 1); double im12 = T_inv.at<double>(1, 2);

  std::vector<cv::Point2f> corners = {
      {0, 0}, {static_cast<float>(src_size.width), 0},
      {static_cast<float>(src_size.width), static_cast<float>(src_size.height)}, {0, static_cast<float>(src_size.height)}
  };
  std::vector<cv::Point2f> map_corners;

  for (const auto& pt : corners) {
      // Convert source pixel to the coordinates of the visible disk of the Moon mx, my
      double mx = im00 * pt.x + im01 * pt.y + im02;
      double my = im10 * pt.x + im11 * pt.y + im12;

      // Recover true selenography (lon_p, lat_p) for the frame corner
      double center_sph_z2 = 1.0 - mx*mx - my*my;
      double mz = (center_sph_z2 > 0) ? std::sqrt(center_sph_z2) : 0.0;
      double y_sph_true = -my;
      double lat_p = std::asin(y_sph_true * std::cos(b) + mz * std::sin(b));
      double lon_p = l + std::atan2(mx, mz * std::cos(b) - y_sph_true * std::sin(b));

      // Project the angle into the coordinates of the target flat map
      double cos_c = std::sin(lat0) * std::sin(lat_p) + std::cos(lat0) * std::cos(lat_p) * std::cos(lon_p - lon0);
      cos_c = std::max(-1.0, std::min(1.0, cos_c));
      double c_dist = std::acos(cos_c);

      double k = 1.0;
      if (c_dist > 1e-7) {
          k = (opts.projection == c_lunar_birdview3_routine::MapOrthographic) ?
               (std::sin(c_dist) / c_dist) : (2.0 * std::tan(c_dist / 2.0) / c_dist);
      }

      double rx = k * std::cos(lat_p) * std::sin(lon_p - lon0);
      double ry = k * (std::cos(lat0) * std::sin(lat_p) - std::sin(lat0) * std::cos(lat_p) * std::cos(lon_p - lon0));

      // Convert map radians to the target screen's pixel grid.
      // The formula strictly duplicates the main loop logic (including the inversion of the ry sign)
      double map_x_pixels = rx * map_scale;
      double map_y_pixels = -ry * map_scale;
      map_corners.push_back(cv::Point2f(static_cast<float>(map_x_pixels), static_cast<float>(map_y_pixels)));
  }

  // minimum enclosing rectangle of useful scene
  cv::Rect bbox = cv::boundingRect(map_corners);

  // Prevent critical memory overrun (limited to 4 times the original frame)
  bbox.width = std::min(bbox.width, 4 * src_size.width);
  bbox.height = std::min(bbox.height, 4 * src_size.height);

  // Return the dynamic size to the calling code
  dst_size = bbox.size();

  // create remaps
  map.create(dst_size);

  for( int y = 0; y < dst_size.height; ++y ) {
    float * __restrict mp = (float * )(map[y]);

    // current screen pixel (x, y) back to map radians given bbox offset
    for( int x = 0; x < dst_size.width; ++x, mp += 2 ) {
      double rx = (x + bbox.x) / map_scale;
      double ry = -(y + bbox.y) / map_scale;

      double rho = std::hypot(rx, ry);
      double lon_p = lon0;
      double lat_p = lat0;

      if (rho > 1e-7) {
        double c = (opts.projection == c_lunar_birdview3_routine::MapOrthographic) ? std::asin(rho) : (2.0 * std::atan(rho / 2.0));
        lat_p = std::asin(std::cos(c) * std::sin(lat0) + (ry * std::sin(c) * std::cos(lat0)) / rho);
        lon_p = lon0 + std::atan2(rx * std::sin(c), rho * std::cos(lat0) * std::cos(c) - ry * std::sin(lat0) * std::sin(c));
      }

      // Project selenographic point onto the observer's visible disk
      double d_lon = lon_p - l;
      double cos_lat_p = std::cos(lat_p);
      double sin_lat_p = std::sin(lat_p);

      double mx = cos_lat_p * std::sin(d_lon);
      double my = -(sin_lat_p * std::cos(b) - cos_lat_p * std::sin(b) * std::cos(d_lon));
      double mz_sph = sin_lat_p * std::sin(b) + cos_lat_p * std::cos(b) * std::cos(d_lon);

      // Convert disk coordinates into source pixels via the affine matrix T
      if (mz_sph < 0) {
        mp[0] = -1.0f;
        mp[1] = -1.0f;
      }
      else {
        mp[0] = float(m00 * mx + m01 * my + m02);
        mp[1] = float(m10 * mx + m11 * my + m12);
      }
    }
  }

  return true;
}

}


void c_lunar_birdview3_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Projection", ctx(&this_class::_projection), "");
  ctlbind(ctls, "rpx0", ctx(&this_class::rpx0), "reference point0 pixel coordinates");
  ctlbind(ctls, "rse0 [deg]", ctx(&this_class::rse0), "reference point0 selenographic coordinates [deg]");
  ctlbind(ctls, "rpx1", ctx(&this_class::rpx1), "reference point1 pixel coordinates");
  ctlbind(ctls, "rse1 [deg]", ctx(&this_class::rse1), "reference point1 selenographic coordinates [deg]");
  ctlbind(ctls, "rpx2", ctx(&this_class::rpx2), "reference point0 pixe2 coordinates");
  ctlbind(ctls, "rse2 [deg]", ctx(&this_class::rse2), "reference point2 selenographic coordinates [deg]");
  ctlbind(ctls, "Libr. l [deg]", ctx(&this_class::_l), "Libration in longitude in degreed");
  ctlbind(ctls, "Libr. b [deg]", ctx(&this_class::_b), "Libration in Latitude in degreed");
  // ctlbind(ctls, "Resize mode", ctx(&this_class::_resizeMode), "Don't crop bounding box");
  ctlbind(ctls, "interpolation", ctx(&this_class::_interpolation), "");
  ctlbind(ctls, "border mode", ctx(&this_class::_borderMode), "");
  ctlbind(ctls, "border value", ctx(&this_class::_borderValue), "");

}

bool c_lunar_birdview3_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, rpx0);
    SERIALIZE_OPTION(settings, save, *this, rse0);
    SERIALIZE_OPTION(settings, save, *this, rpx1);
    SERIALIZE_OPTION(settings, save, *this, rse1);
    SERIALIZE_OPTION(settings, save, *this, rpx2);
    SERIALIZE_OPTION(settings, save, *this, rse2);
    SERIALIZE_OPTION(settings, save, *this, _l);
    SERIALIZE_OPTION(settings, save, *this, _b);
    SERIALIZE_OPTION(settings, save, *this, _projection);
    //  SERIALIZE_OPTION(settings, save, *this, _resizeMode);
    SERIALIZE_OPTION(settings, save, *this, _interpolation);
    SERIALIZE_OPTION(settings, save, *this, _borderMode);
    SERIALIZE_OPTION(settings, save, *this, _borderValue);

    return true;
  }
  return false;
}

bool c_lunar_birdview3_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Size src_size = image.size();
  const cv::Mat src = image.getMat();
  const cv::Mat msk = mask.getMat();

  c_lunar_birdview3_options opts;
  opts.rpx[0] = rpx0;
  opts.rse[0] = rse0;
  opts.rpx[1] = rpx1;
  opts.rse[1] = rse1;
  opts.rpx[2] = rpx2;
  opts.rse[2] = rse2;
  opts.l = _l;             // Libration in longitude [deg]
  opts.b = _b;             // Libration in latitude [deg]
  opts.projection = _projection;
  opts.resizeMode = c_lunar_birdview3_routine::ResizeModeAdjust;
  opts.interpolation = _interpolation;
  opts.borderMode = _borderMode;
  opts.borderValue = _borderValue;

  cv::Size dst_size = src_size * 2;

  cv::Mat2f rmap;

  const bool fOk =
      createMoonRemapThreePoints(
          opts,
          src_size,
          dst_size,
          rmap);

  if ( !fOk ) {
    CF_ERROR("createMoonMapsByThreePoints() fails");
    return false;
  }

  CF_DEBUG("src_size: %dx%d dst_size: %dx%d",
      src_size.width, src_size.height,
      dst_size.width, dst_size.height);

  if ( dst_size.width > 10000 || dst_size.height > 10000 ) {
    CF_ERROR("Too big dst size was blocked");
    return false;
  }

  cv::remap(image, image, rmap, cv::noArray(), _interpolation, _borderMode, _borderValue);
  if ( !mask.empty() ) {
    cv::remap(mask, mask, rmap, cv::noArray(), cv::INTER_NEAREST, cv::BORDER_CONSTANT);
  }

  return true;
}
