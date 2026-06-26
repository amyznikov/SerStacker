/*
 * c_planetary_disk_detection_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_planetary_disk_detection_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>

void c_planetary_disk_detection_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "gsigma", CTL_CONTEXT(ctx, gsigma), "");
  ctlbind(ctls, "se_radius", CTL_CONTEXT(ctx, se_radius), "");
  ctlbind(ctls, "updateROI", CTL_CONTEXT(ctx, updateROI), "");
}

bool c_planetary_disk_detection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, gsigma);
    SERIALIZE_OPTION(settings, save, *this, se_radius);
    SERIALIZE_OPTION(settings, save, *this, updateROI);
    return true;
  }
  return false;
}

bool c_planetary_disk_detection_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Point2f centroid;
  cv::Rect component_rect;
  cv::Mat cmponent_mask;
  cv::Point2f geometrical_center;

  bool fOk =
      simple_planetary_disk_detector(image, mask,
          gsigma, se_radius,
          &centroid,
          &component_rect,
          &cmponent_mask,
          &geometrical_center);

  if ( !fOk ) {
    CF_ERROR("simple_planetary_disk_detector() fails");
    return false;
  }

  mask.move(cmponent_mask);

  CF_DEBUG("\n"
      "centroid: %g;%g rect: %d;%d;%dx%d geometrical_center=%g;%g",
      centroid.x, centroid.y,
      component_rect.x, component_rect.y, component_rect.width, component_rect.height,
      geometrical_center.x, geometrical_center.y);

  if ( updateROI ) {
    ctlbind_update_roi(component_rect);
  }

  return fOk;
}

