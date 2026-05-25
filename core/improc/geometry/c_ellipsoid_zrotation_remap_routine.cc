/*
 * c_ellipsoid_zrotation_remap_routine.cc
 *
 *  Created on: May 24, 2026
 *      Author: amyznikov
 */

#include "c_ellipsoid_zrotation_remap_routine.h"
#include <core/proc/feature2d/ellipsoid.h>
#include <core/proc/pose.h>

void c_ellipsoid_zrotation_remap_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "center", ctx(&this_class::_center), "Ellipsoid center [pix]");
  ctlbind(ctls, "axes [pix]", ctx(&this_class::_axes), "Ellipsoid axes [pix]");
  ctlbind(ctls, "pose [deg] ", ctx(&this_class::_pose), "Ellipsoid pose [deg]");
  ctlbind(ctls, "zrotation [deg] ", ctx(&this_class::_zrotation), "Z rotation [deg]");

  ctlbind_expandable_group(ctls, "Remap Options ",
      [&, ctx = CTL_CONTEXT(ctx, _remap)]() {
        ctlbind(ctls, "enable remap ", CTL_CONTEXT(ctx, enabled), "Enable remap");
        ctlbind(ctls, "display counter", CTL_CONTEXT(ctx, display_counter), "");
        ctlbind(ctls, "display weights", CTL_CONTEXT(ctx, display_weights), "");
        ctlbind(ctls, "display dist. transform", CTL_CONTEXT(ctx, display_distance_transform), "");

      });

  ctlbind_expandable_group(ctls, "Draw Options ",
      [&, ctx = CTL_CONTEXT(ctx, _draw_ellipoid)]() {
        ctlbind(ctls, "draw_ellipoid", CTL_CONTEXT(ctx, enabled), "Enable draw ellipsoid coordinate grid");
        ctlbind(ctls, "lat_step [deg]", CTL_CONTEXT(ctx, lat_step_deg), "Latitude step in degrees");
        ctlbind(ctls, "lon_step [deg]", CTL_CONTEXT(ctx, lon_step_deg), "Longitude step in degrees");
      });
}



bool c_ellipsoid_zrotation_remap_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_OPTION(settings, save, *this, _center);
    SERIALIZE_OPTION(settings, save, *this, _axes);
    SERIALIZE_OPTION(settings, save, *this, _pose);
    SERIALIZE_OPTION(settings, save, *this, _zrotation);

    if ( auto subsection = SERIALIZE_GROUP(settings, save, "remap") ) {
      SERIALIZE_OPTION(subsection, save, _remap, enabled);
      SERIALIZE_OPTION(subsection, save, _remap, display_counter);
      SERIALIZE_OPTION(subsection, save, _remap, display_weights);
      SERIALIZE_OPTION(subsection, save, _remap, display_distance_transform);
    }

    if ( auto subsection = SERIALIZE_GROUP(settings, save, "draw_ellipoid") ) {
      SERIALIZE_OPTION(subsection, save, _draw_ellipoid, enabled);
      SERIALIZE_OPTION(subsection, save, _draw_ellipoid, lat_step_deg);
      SERIALIZE_OPTION(subsection, save, _draw_ellipoid, lon_step_deg);
    }


    return true;
  }
  return false;
}

bool c_ellipsoid_zrotation_remap_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Size image_size = image.size();

  const cv::Vec3d target_pose((_pose(0) + _zrotation) * CV_PI / 180,
      _pose(1) * CV_PI / 180,
      _pose(2) * CV_PI / 180);

  const cv::Matx33d Rtarget =
      build_ellipsoid_rotation(target_pose);

  if( _remap.enabled ) {

    const cv::Vec3d initia_pose(_pose(0) * CV_PI / 180,
        _pose(1) * CV_PI / 180,
        _pose(2) * CV_PI / 180);

    const cv::Matx33d Rinitial =
        build_ellipsoid_rotation(initia_pose);

    cv::Mat2f rmap;
    cv::Mat1b rmask;
    cv::Mat1f wmap;
    cv::Mat1f counter;

    compute_ellipsoid_zrotation_remap(image.size(),
        _center,
        _axes,
        Rinitial,
        Rtarget,
        rmap,
        wmap,
        rmask,
        &counter);

    if( _remap.display_counter ) {
      counter.copyTo(image);
    }
    else if( _remap.display_weights ) {
      wmap.copyTo(image);
    }
    else if( _remap.display_distance_transform ) {
      cv::distanceTransform(rmask, image,
          cv::DIST_L2, cv::DIST_MASK_PRECISE,
          CV_32F);
    }

    else {
      cv::remap(image.getMat(), image, rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_TRANSPARENT);
    }

    rmask.copyTo(mask);
  }

  if ( _draw_ellipoid.enabled ) {

    double minv, maxv;
    cv::minMaxLoc(image, &minv, &maxv);
    if( !(maxv > minv) ) {
      maxv = 255;
    }

    draw_ellipoid(image,
        _center,
        _axes,
        Rtarget,
        _draw_ellipoid.lat_step_deg * CV_PI / 180,
        _draw_ellipoid.lon_step_deg * CV_PI / 180,
        cv::Scalar::all(maxv),
        1,
        cv::LINE_AA);
  }

//  const cv::RotatedRect _planetary_disk_ellipse =
//      ellipsoid_bbox(_center, A, B, C,
//          build_rotation2(_pose).t());
//
//  cv::Mat1b _planetary_disk_ellipse_mask = cv::Mat1b::zeros(image_size);
//
//  draw_ellipse(_planetary_disk_ellipse_mask,
//      _planetary_disk_ellipse,
//      cv::Scalar::all(255), -1,
//      cv::LINE_8);
//
//  cv::Mat2f _current_remap;
//  cv::Mat1b _current_bmask;
//  compute_ellipsoid_zrotation_remap(image_size,
//      _center,
//      _axes,
//      _pose * CV_PI / 180,
//      zrotation_deg * CV_PI / 180,
//      _current_remap,
//      _current_bmask);
//
//  cv::remap(image.getMat(), image,
//      _current_remap, cv::noArray(),
//      cv::INTER_LINEAR);
//
//  _current_bmask.copyTo(mask);


  ///////////

//  _current_bmask =
//      cv::Mat1b(_current_remap.size(),
//          (uint8_t)(0));
//
//  draw_ellipse(_current_bmask,
//      _planetary_disk_ellipse,
//      cv::Scalar::all(255),
//      -1,
//      cv::LINE_8);

  //  _current_bmask.convertTo(_current_wmask,
  //      CV_32F);

//  cv::distanceTransform(_current_bmask, _current_wmask,
//      cv::DIST_L2, cv::DIST_MASK_PRECISE,
//      _current_wmask.depth() );
//
//  cv::remap(_current_wmask, _current_wmask,
//      _current_remap, cv::noArray(),
//      cv::INTER_LINEAR,
//      cv::BORDER_CONSTANT);
//
//  double min, max;
//  cv::minMaxLoc(_current_wmask, &min, &max);
//  cv::multiply(_current_wmask, 1. / max, _current_wmask);

  return true;

}
