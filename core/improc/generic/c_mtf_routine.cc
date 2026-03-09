/*
 * c_mtf_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_mtf_routine.h"

bool c_mtf_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, inputRange);
    SERIALIZE_OPTION(settings, save, *this, outputRange);
    SERIALIZE_OPTION(settings, save, *this, lclip);
    SERIALIZE_OPTION(settings, save, *this, hclip);
    SERIALIZE_OPTION(settings, save, *this, shadows);
    SERIALIZE_OPTION(settings, save, *this, highlights);
    SERIALIZE_OPTION(settings, save, *this, midtones);
    return true;
  }
  return false;
}

void c_mtf_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "clip input range", ctx(&this_class::inputRange), "");
  ctlbind(ctls, "stretch output range", ctx(&this_class::outputRange), "");
  ctlbind_slider_spinbox(ctls, "lclip", ctx(&this_class::shadows), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "shadows", ctx(&this_class::shadows), 0.0, 5.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "midtones", ctx(&this_class::midtones), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "highlights", ctx(&this_class::highlights), 0.0, 5.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "hclip", ctx(&this_class::shadows), 0.0, 1.0, 0.001, "");
}


bool c_mtf_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_mtf_options opts;

  opts.lclip = lclip;
  opts.hclip = hclip;
  opts.shadows = shadows;
  opts.highlights = highlights;
  opts.midtones = midtones;

  if( !(inputRange[1] > inputRange[0]) ) {
    double minv = 0, maxv = 1;
    cv::minMaxLoc(image, &minv, &maxv);
    inputRange[0] = minv, inputRange[1] = maxv;
  }

  if( !(outputRange[1] > outputRange[0]) ) {
    c_mtf::suggest_levels_range(image.depth(), &outputRange[0], &outputRange[1]);
  }

  mtf.set_input_range(inputRange[0], inputRange[1]);
  mtf.set_output_range(outputRange[0], outputRange[1]);
  mtf.set_opts(opts);

  return mtf.apply(image.getMat(), image);
}

