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
  ctlbind_slider_spinbox(ctls, "lclip", ctx(&this_class::lclip), 0.0, 1.0, 0.001, "");
  ctlbind_slider_spinbox(ctls, "shadows", ctx(&this_class::shadows), -2, 2, 0.01, "");
  ctlbind_slider_spinbox(ctls, "midtones", ctx(&this_class::midtones), 0.001, 0.999, 0.001, "");
  ctlbind_slider_spinbox(ctls, "highlights", ctx(&this_class::highlights), -2, 2, 0.01, "");
  ctlbind_slider_spinbox(ctls, "hclip", ctx(&this_class::hclip), 0.0, 1.0, 0.001, "");

  //ctlbind_menu_button(ctls, "options", ctx);
  ctlbind_command_button(ctls, "RESET", ctx,
      std::function([](this_class * _ths) {
        c_mtf_options opts = c_mtf_options();
        _ths->lclip = opts.lclip;
        _ths->hclip = opts.hclip;
        _ths->shadows = opts.shadows;
        _ths->midtones = opts.midtones;
        _ths->highlights = opts.highlights;
        return true;
      }), "");

}


bool c_mtf_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  c_mtf_options opts;
  double imin, imax, omin, omax;

  opts.lclip = lclip;
  opts.hclip = hclip;
  opts.shadows = shadows;
  opts.highlights = highlights;
  opts.midtones = midtones;

  if( (inputRange[1] > inputRange[0]) ) {
    imin = inputRange[0], imax = inputRange[1];
  }
  else {
    cv::minMaxLoc(image, &imin, &imax);
  }

  if( (outputRange[1] > outputRange[0]) ) {
    omin = outputRange[0], omax = outputRange[1];
  }
  else {
    c_mtf::suggest_levels_range(image.depth(), &omin, &omax);
  }

  mtf.set_input_range(imin, imax);
  mtf.set_output_range(omin, omax);
  mtf.set_opts(opts);

  return mtf.apply(image.getMat(), image);
}

