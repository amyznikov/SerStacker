/*
 * c_tangential_transform_routine.cc
 *
 *  Created on: Jul 29, 2023
 *      Author: amyznikov
 */

#include "c_tangential_transform_routine.h"


 void c_tangential_transform_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "focus", ctx, &this_class::focus, &this_class::set_focus, "focal distance in pixels");
  ctlbind(ctls, "center", ctx, &this_class::center, &this_class::set_center, "optical center in pixels");
}

bool c_tangential_transform_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, focus);
    SERIALIZE_PROPERTY(settings, save, *this, center);
    return true;
  }
  return false;
}

bool c_tangential_transform_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() || !mask.empty() ) {

    const cv::Size image_size =
        image.empty() ? mask.size() : image.size();

    if( _rmap.empty() || _previous_image_size != image_size ) {

      if( _center.x == -1 && _center.y == -1 ) {
        _center.x = image_size.width / 2;
        _center.y = image_size.height / 2;
      }

      double al = atan2(0 - _center.x, _focus);
      double at = atan2(0 - _center.y, _focus);
      double ar = atan2(image_size.width - _center.x, _focus);
      double ab = atan2(image_size.height - _center.y, _focus);

      int l = cvRound(al * _focus);
      int t = cvRound(at * _focus);
      int r = cvRound(ar * _focus);
      int b = cvRound(ab * _focus);

      const cv::Size remap_size(r - l, b - t);

      _rmap.create(remap_size);

      for( int y = 0; y < remap_size.height; ++y ) {
        for( int x = 0; x < remap_size.width; ++x ) {

          double ax = (x + l) / _focus;
          double ay = (y + t) / _focus;

          _rmap[y][x][0] = tan(ax) * _focus + _center.x;
          _rmap[y][x][1] = tan(ay) * _focus + _center.y;
        }
      }
    }

    if( !image.empty() ) {
      cv::remap(image, image, _rmap, cv::noArray(), cv::INTER_LINEAR);
    }

    if( !mask.empty() ) {
      cv::remap(mask, mask, _rmap, cv::noArray(), cv::INTER_LINEAR);
      cv::compare(mask, 245, mask, cv::CMP_GE);
    }

    _previous_image_size = image_size;

  }

  return true;
}

