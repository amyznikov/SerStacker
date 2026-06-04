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
        ctlbind(ctls, "display weights", CTL_CONTEXT(ctx, display_weights), "");
        ctlbind(ctls, "display rmapx", CTL_CONTEXT(ctx, display_rmapx), "");
        ctlbind(ctls, "display rmapy", CTL_CONTEXT(ctx, display_rmapy), "");

      });

  ctlbind_expandable_group(ctls, "Draw Options ",
      [&, ctx = CTL_CONTEXT(ctx, _draw_ellipoid)]() {
        ctlbind(ctls, "draw grid", CTL_CONTEXT(ctx, enabled), "Enable draw ellipsoid coordinate grid");
        ctlbind(ctls, "lat_step [deg]", CTL_CONTEXT(ctx, lat_step_deg), "Latitude step in degrees");
        ctlbind(ctls, "lon_step [deg]", CTL_CONTEXT(ctx, lon_step_deg), "Longitude step in degrees");
      });

  ctlbind_command_button(ctls, "Paste pose", ctx, [](this_class * ths) {
    if ( const auto & cb = get_ctlbind_get_clipboard_text_callback() ) {
      bool have_center = false, have_axes = false, have_pose = false;
      cv::Point2d center;
      cv::Vec3d axes;
      cv::Vec3d pose;
      if ( parse_ellipsoid_from_string(cb(), &center, &axes, &pose, &have_center, &have_axes, &have_pose) ) {
        if ( have_center ) {
          ths->_center = center;
        }
        if ( have_axes ) {
          ths->_axes = axes;
        }
        if ( have_pose ) {
          ths->_pose = pose;
        }
        return true;
      }
    }
    return false;
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
      SERIALIZE_OPTION(subsection, save, _remap, display_weights);
      SERIALIZE_OPTION(subsection, save, _remap, display_rmapx);
      SERIALIZE_OPTION(subsection, save, _remap, display_rmapy);
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

    const cv::Vec3d initial_pose(_pose(0) * CV_PI / 180,
        _pose(1) * CV_PI / 180,
        _pose(2) * CV_PI / 180);

    const cv::Matx33d Rinitial =
        build_ellipsoid_rotation(initial_pose);

    cv::Mat2f rmap;
    cv::Mat1f wmap;
    cv::Mat1b rmask;

    compute_ellipsoid_zrotation_remap(image.size(),
        _center,
        _axes,
        Rinitial,
        Rtarget,
        rmap,
        wmap,
        rmask,
        1);

    if( _remap.display_weights ) {
      wmap.setTo(1, ~rmask);
      wmap.copyTo(image);
      rmask.copyTo(mask);
    }
    else if ( _remap.display_rmapx ) {
      cv::extractChannel(rmap, image, 0);
      rmask.copyTo(mask);
    }
    else if ( _remap.display_rmapy ) {
      cv::extractChannel(rmap, image, 1);
      rmask.copyTo(mask);
    }

    else {
      cv::remap(image.getMat(), image, rmap, cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      rmask.copyTo(mask);
    }

  }

  if ( _draw_ellipoid.enabled ) {

    double minv, maxv;
    cv::minMaxLoc(image, &minv, &maxv);
    if( !(maxv > minv) ) {
      maxv = 255;
    }

    draw_ellipsoid(image,
        _center,
        _axes,
        Rtarget,
        _draw_ellipoid.lat_step_deg * CV_PI / 180,
        _draw_ellipoid.lon_step_deg * CV_PI / 180,
        cv::Scalar::all(maxv),
        1,
        cv::LINE_AA);
  }

  return true;
}
