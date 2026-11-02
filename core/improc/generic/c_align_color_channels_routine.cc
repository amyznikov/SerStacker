/*
 * c_align_color_channels_routine.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "c_align_color_channels_routine.h"
#include <core/proc/reduce_channels.h>

static void serialize_transfrom_parameters(c_config_setting settings, bool save,
    c_align_color_channels & _algorithm, IMAGE_MOTION_TYPE motion_type)
{
  if ( save ) {
    c_config_setting section3 = settings.add_list("transforms");

    const auto & transforms = _algorithm.computed_transforms();
    for ( int i = 0, n = transforms.size(); i < n; ++i ) {
      c_image_transform_params opts;
      if ( transforms[i] ) {
        get_image_transform_params(transforms[i], opts);
      }
      serialize_image_transform_opts(section3.add_element(CONFIG_TYPE_GROUP), save, opts);
    }
  }
  else {
    c_config_setting section3 = settings["transforms"];
    if ( section3.isList() ) {

      c_image_transform_params opts[4];

      for ( int i = 0, n = section3.length(); i < n; ++i ) {
        serialize_image_transform_opts(section3[i], save, opts[i]);
      }

      _algorithm.setImageTransforms(motion_type, opts);
    }
  }
}

void c_align_color_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Reference channel:", CTL_CONTEXT(ctx, referenceChannel), "");
  base::getcontrols(ctls, ctx);

  ctlbind(ctls, "border_mode", CTL_CONTEXT(ctx, _opts.border_mode), "");
  ctlbind(ctls, "border_value", CTL_CONTEXT(ctx, _opts.border_value), "");

  ctlbind(ctls, "Estimate", CTL_CONTEXT(ctx, estimate), "");
  ctlbind_expandable_group(ctls, "Estimate", "Algorithm parameters");
    ctlbind(ctls, "motion_type", CTL_CONTEXT(ctx, _opts.motion_type), "");
    ctlbind(ctls, "normalization_level", CTL_CONTEXT(ctx, _opts.normalization_level), "");
    ctlbind(ctls, CTL_CONTEXT(ctx, _opts.ecch));
  ctlbind_end_group(ctls);

  ctlbind_menu_button(ctls, "Transform...", ctx);
  ctlbind_item(ctls, "Copy transform parameters to clipboard", ctx, &this_class::
      copy_transfrom_parameters_to_clipboard);
  ctlbind_item(ctls, "Paste transform parameters from clipboard", ctx, &this_class::
      paste_transfrom_parameters_from_clipboard);

}

bool c_align_color_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, referenceChannel);
    SERIALIZE_OPTION(settings, save, *this, estimate);
    serialize_align_color_channels_options(settings, save, _opts);
    serialize_transfrom_parameters(settings, save, _algorithm, _opts.motion_type);
    return true;
  }
  return false;
}



bool c_align_color_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( estimate || _algorithm.estimated_motion_type() != _opts.motion_type ) {
    const bool fOk = _algorithm.estimate(image, _ignore_mask ? cv::noArray() : mask, referenceChannel, _opts);
    if ( !fOk ) {
      CF_ERROR("_algorithm.align() fails");
      return false;
    }
  }
  return _algorithm.apply(image, _ignore_mask ? cv::noArray() : mask, _opts, image, mask);
}


bool c_align_color_channels_routine::copy_transfrom_parameters_to_clipboard()
{
  c_config cfg;
  c_config_setting root = cfg.root();

  c_config_setting section1 = root.add_group("c_align_color_channels");
  serialize_align_color_channels_options(section1.add_group("parameters"), true, _opts);
  serialize_transfrom_parameters(section1.add_list("transforms"), true, _algorithm, _opts.motion_type);

  ctlbind_copy_config_to_clipboard(cfg);

  return false;
}

bool c_align_color_channels_routine::paste_transfrom_parameters_from_clipboard()
{
  c_config cfg;
  if ( !ctlbind_paste_config_to_clipboard(cfg) ) {
    CF_ERROR("No c_config data in clipboard");
    return false;
  }

  c_config_setting root = cfg.root();
  c_config_setting section1 = root["c_align_color_channels"];
  if ( !section1.isGroup() ) {
    CF_ERROR("No c_align_color_channels group found in clipboard");
    return false;
  }

  c_config_setting section2 = section1["parameters"];
  if ( section2.isGroup() ) {
    serialize_align_color_channels_options(section2, false, _opts);
  }

  serialize_transfrom_parameters(section1["transforms"], false, _algorithm,
      _opts.motion_type);

  return true;
}
