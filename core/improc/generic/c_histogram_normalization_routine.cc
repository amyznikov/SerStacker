/*
 * c_histogram_normalization_routine.cc
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#include "c_histogram_normalization_routine.h"
#include <core/proc/histogram.h>
#include <core/ssprintf.h>

void c_histogram_normalization_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "colorid", ctx(&this_class::colorid), "");
  ctlbind(ctls, ctx(&this_class::opts));
  //   ctlbind(ctls, "colorid", ctx(&this_class::_colorid), "");
  //   ctlbind(ctls, "normalize", ctx(&this_class::_normalization_type), "");
  //   ctlbind(ctls, "offset", ctx(&this_class::_offset), "");
  //   ctlbind(ctls, "stretch", ctx(&this_class::_stretch), "");
}

bool c_histogram_normalization_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, colorid);
    SERIALIZE_OPTION(settings, save, opts, norm_type);
    SERIALIZE_OPTION(settings, save, opts, stretch);
    SERIALIZE_OPTION(settings, save, opts, offset);
    SERIALIZE_OPTION(settings, save, opts, clipRange);
    return true;
  }
  return false;
}


// v' = (v - mv ) * stretch + offset
// v' = v* stretch + offset - mv * stretch
bool c_histogram_normalization_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !image.empty() ) {
    nomalizeImageHistogram(image.getMat(), mask.getMat(), image, opts, colorid);
    //nomalize_image_histogramm(image, mask, image, _normalization_type, _stretch, _offset, _colorid);
  }
  return true;
}
