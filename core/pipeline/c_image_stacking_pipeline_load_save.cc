/*
 * c_image_stacking_pipeline_load_save.cc
 *
 *  Created on: Aug 22, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <core/readdir.h>
#include <core/debug.h>

std::string c_image_stacks_collection :: default_config_filename_ =
    "~/.config/SerStacker/corrent_work.cfg";

const std::string & c_image_stacks_collection::default_config_filename()
{
  return default_config_filename_;
}

void c_image_stacks_collection::set_default_config_filename(const std::string & v)
{
  default_config_filename_ = v;
}


bool c_image_stacks_collection::save(const std::string & cfgfilename) const
{
  std::string filename;

  if ( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if ( !filename_.empty() ) {
    filename = filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if ( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for c_image_stacks_collection::save()");
    return false;
  }

  CF_DEBUG("Saving '%s' ...", filename.c_str());

  c_config cfg(filename);

  time_t t = time(0);

  if ( !save_settings(cfg.root(), "object_class", std::string("c_image_stacks_collection")) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }


  c_config_setting stacklist =
      cfg.root().add_list("stacks");

  for ( const c_image_stacking_options::ptr & stack : stacks_ ) {
    if ( !stack->serialize(stacklist.add_group()) ) {
      CF_ERROR("stack->save() fails for stack '%s'", stack->name().c_str());
    }
  }


  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  filename_ = filename;

  return true;
}

bool c_image_stacks_collection::load(const std::string & cfgfilename)
{
  std::string filename;

  if ( !cfgfilename.empty() ) {
    filename = cfgfilename;
  }
  else if ( !filename_.empty() ) {
    filename = filename_;
  }
  else {
    filename = default_config_filename_;
  }

  if ( (filename = expand_path(filename)).empty() ) {
    CF_ERROR("No output config file name specified for c_image_stacks_collection::save()");
    return false;
  }

  CF_DEBUG("Loading '%s' ...", filename.c_str());

  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return false;
  }


  std::string object_class;
  if ( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("load_settings(object_class) fails", filename.c_str());
    return false;
  }

  if ( object_class != "c_image_stacks_collection" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return false;
  }

  c_config_setting stacklist =
      cfg.root().get("stacks");

  if ( !stacklist || !stacklist.isList() ) {
    CF_FATAL("stacklist 'stacks' is not found in file '%s''",
        filename.c_str());
    return false;
  }

  const int n = stacklist.length();

  stacks_.clear(), stacks_.reserve(n);

  for ( int i = 0 ; i < n; ++i ) {

    c_config_setting stacklist_item =
        stacklist.get_element(i);

    if ( stacklist_item && stacklist_item.isGroup() ) {

      c_image_stacking_options::ptr stack =
          c_image_stacking_options::create();

      if ( !stack->deserialize(stacklist_item) ) {
        CF_ERROR("stack->deserialize() fails for element index %d", i);
      }
      else {
        stacks_.emplace_back(stack);
      }
    }
  }



  filename_ = filename;

  return true;
}


///////////////////////////////////////////////////////////////////////////////


bool c_image_stacking_options::serialize(c_config_setting settings) const
{
  settings.set("name", name_);

  input_options_.serialize(settings.add_group("input_options"));
  master_frame_options_.serialize(settings.add_group("master_frame_options"));
  roi_selection_options_.serialize(settings.add_group("roi_selection"));
  upscale_options_.serialize(settings.add_group("upscale_options"));
  frame_registration_options_.serialize(settings.add_group("frame_registration"));
  accumulation_options_.serialize(settings.add_group("accumulation_options"));
  output_options_.serialize(settings.add_group("output_options"));

  if ( input_sequence_ ) {
    input_sequence_->serialize(settings.add_group("input_sequence"));
  }

  return true;
}

bool c_image_stacking_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    CF_ERROR("No config group specified");
    return false;
  }

  if ( !settings.get("name", &name_) || name_.empty() ) {
    CF_ERROR("No stack name specified");
    return false;
  }

  input_options_.deserialize(settings["input_options"]);
  roi_selection_options_.deserialize(settings["roi_selection"]);
  upscale_options_.deserialize(settings["upscale_options"]);
  master_frame_options_.deserialize(settings["master_frame_options"]);
  frame_registration_options_.deserialize(settings["frame_registration"]);
  accumulation_options_.deserialize(settings["accumulation_options"]);
  output_options_.deserialize(settings["output_options"]);

  c_config_setting input_sequence_group =
      settings["input_sequence"];

  if ( input_sequence_group ) {
    input_sequence_ = c_input_sequence::create();
    input_sequence_->deserialize(input_sequence_group);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool c_input_options::serialize(c_config_setting settings) const
{
  settings.set("bad_pixel_mask", bad_pixel_mask_filename);
  settings.set("bad_pixels_marked_black", bad_pixels_marked_black);

  settings.set("remove_bad_pixels", filter_hot_pixels);
  settings.set("bad_pixels_variation_threshold", hot_pixels_variation_threshold);
  settings.set("enable_color_maxtrix", enable_color_maxtrix );
  settings.set("anscombe", anscombe);

  if ( input_frame_processor ) {
    settings.set("input_frame_processor", input_frame_processor->name());
  }

  return false;
}

bool c_input_options::deserialize(c_config_setting settings)
{
  std::string s;

  if ( !settings.isGroup() ) {
    return false;
  }

  settings.get("bad_pixel_mask", &bad_pixel_mask_filename);
  settings.get("bad_pixels_marked_black", &bad_pixels_marked_black);
  settings.get("remove_bad_pixels", &filter_hot_pixels);
  settings.get("bad_pixels_variation_threshold", &hot_pixels_variation_threshold);
  settings.get("enable_color_maxtrix", &enable_color_maxtrix );
  settings.get("anscombe", &anscombe);

  if ( settings.get("input_frame_processor", &s) && !s.empty() ) {
    input_frame_processor = c_image_processor_collection::default_instance()->get(s);
  }

  return true;
}
///////////////////////////////////////////////////////////////////////////////
bool c_roi_selection_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "roi_selection_method", method);
  save_settings(settings, "crop_size", crop_size);

  return true;
}

bool c_roi_selection_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "roi_selection_method", &method);
  load_settings(settings, "crop_size", &crop_size);


  return true;
}
///////////////////////////////////////////////////////////////////////////////
bool c_frame_upscale_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "upscale_option", upscale_option);
  save_settings(settings, "upscale_stage", upscale_stage);

  return false;
}

bool c_frame_upscale_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "upscale_option", &upscale_option);
  load_settings(settings, "upscale_stage", &upscale_stage);

  return true;
}
///////////////////////////////////////////////////////////////////////////////
bool c_master_frame_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "master_source_path", master_source_path);
  save_settings(settings, "master_frame_index",master_frame_index );
  save_settings(settings, "apply_input_frame_processor", apply_input_frame_processor);
  save_settings(settings, "generate_master_frame", generate_master_frame);
  save_settings(settings, "max_input_frames_to_generate_master_frame", max_input_frames_to_generate_master_frame);
  save_settings(settings, "eccflow_scale", eccflow_scale);
  save_settings(settings, "master_sharpen_factor", master_sharpen_factor);
  save_settings(settings, "accumulated_sharpen_factor", accumulated_sharpen_factor);
  save_settings(settings, "save_master_frame", save_master_frame);

  //save_settings(settings, "compensate_master_flow", compensate_master_flow);

  return false;
}

bool c_master_frame_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "master_source_path", &master_source_path);
  load_settings(settings, "master_frame_index", &master_frame_index );
  load_settings(settings, "apply_input_frame_processor", &apply_input_frame_processor);
  load_settings(settings, "generate_master_frame", &generate_master_frame);
  load_settings(settings, "max_input_frames_to_generate_master_frame", &max_input_frames_to_generate_master_frame);
  load_settings(settings, "eccflow_scale", &eccflow_scale);
  load_settings(settings, "master_sharpen_factor", &master_sharpen_factor);
  load_settings(settings, "accumulated_sharpen_factor", &accumulated_sharpen_factor);
  load_settings(settings, "save_master_frame", &save_master_frame);
  //load_settings(settings, "compensate_master_flow", &compensate_master_flow);

  return true;
}
///////////////////////////////////////////////////////////////////////////////
bool c_frame_accumulation_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "accumulation_method ", accumulation_method );
  return true;
}

bool c_frame_accumulation_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "accumulation_method ", &accumulation_method );

  return true;
}
///////////////////////////////////////////////////////////////////////////////

bool c_frame_registration_options::serialize(c_config_setting settings) const
{
  c_config_setting group;

  group = settings;
  save_settings(group, "registration_method", registration_method);
  save_settings(group, "accumulate_and_compensate_turbulent_flow", accumulate_and_compensate_turbulent_flow);
  save_settings(group, "motion_type", base_options.motion_type);
  save_settings(group, "registration_channel", base_options.registration_channel);
  save_settings(group, "interpolation_flags", base_options.interpolation_flags);
  save_settings(group, "remap_border_mode", base_options.remap_border_mode);
  save_settings(group, "remap_border_value", base_options.remap_border_value);
  save_settings(group, "feature_scale", base_options.feature_scale);
  save_settings(group, "enable_ecc", base_options.enable_ecc);
  save_settings(group, "enable_eccflow", base_options.enable_eccflow);

  group = settings.add_group("SURF");
  save_settings(group, "hessianThreshold", feature_options.hessianThreshold);
  save_settings(group, "nOctaves", feature_options.nOctaves);
  save_settings(group, "nOctaveLayers", feature_options.nOctaveLayers);
  save_settings(group, "extended", feature_options.extended);
  save_settings(group, "upright", feature_options.upright);

  group = settings.add_group("ecc");
  save_settings(group, "scale", base_options.ecc.scale);
  save_settings(group, "eps", base_options.ecc.eps);
  save_settings(group, "min_rho", base_options.ecc.min_rho);
  save_settings(group, "input_smooth_sigma", base_options.ecc.input_smooth_sigma);
  save_settings(group, "reference_smooth_sigma", base_options.ecc.reference_smooth_sigma);
  save_settings(group, "update_step_scale", base_options.ecc.update_step_scale);
  save_settings(group, "normalization_noise", base_options.ecc.normalization_noise);
  save_settings(group, "normalization_scale", base_options.ecc.normalization_scale);
  save_settings(group, "ecc.max_iterations", base_options.ecc.max_iterations);

  group = settings.add_group("eccflow");
  save_settings(group, "support_scale", base_options.eccflow.support_scale);
  save_settings(group, "normalization_scale", base_options.eccflow.normalization_scale);
  save_settings(group, "update_multiplier", base_options.eccflow.update_multiplier);
  save_settings(group, "max_iterations", base_options.eccflow.max_iterations);

  return true;
}

bool c_frame_registration_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  c_config_setting group = settings;
  load_settings(group, "registration_method", &registration_method);
  load_settings(group, "accumulate_and_compensate_turbulent_flow", &accumulate_and_compensate_turbulent_flow);
  load_settings(group, "motion_type", &base_options.motion_type);
  load_settings(group, "registration_channel", &base_options.registration_channel);
  load_settings(group, "interpolation_flags", &base_options.interpolation_flags);
  load_settings(group, "remap_border_mode", &base_options.remap_border_mode);
  load_settings(group, "remap_border_value", &base_options.remap_border_value);
  load_settings(group, "feature_scale", &base_options.feature_scale);
  load_settings(group, "enable_ecc", &base_options.enable_ecc);
  load_settings(group, "enable_eccflow", &base_options.enable_eccflow);

  if ( (group = settings["SURF"]).isGroup() ) {
    load_settings(group, "hessianThreshold", &feature_options.hessianThreshold);
    load_settings(group, "nOctaves", &feature_options.nOctaves);
    load_settings(group, "nOctaveLayers", &feature_options.nOctaveLayers);
    load_settings(group, "extended", &feature_options.extended);
    load_settings(group, "upright", &feature_options.upright);
  }

  if ( (group = settings["ecc"]).isGroup() ) {
    load_settings(group, "scale", &base_options.ecc.scale);
    load_settings(group, "eps", &base_options.ecc.eps);
    load_settings(group, "min_rho", &base_options.ecc.min_rho);
    load_settings(group, "input_smooth_sigma", &base_options.ecc.input_smooth_sigma);
    load_settings(group, "reference_smooth_sigma", &base_options.ecc.reference_smooth_sigma);
    load_settings(group, "update_step_scale", &base_options.ecc.update_step_scale);
    load_settings(group, "normalization_noise", &base_options.ecc.normalization_noise);
    load_settings(group, "normalization_scale", &base_options.ecc.normalization_scale);
    load_settings(group, "ecc.max_iterations", &base_options.ecc.max_iterations);
  }

  if ( (group = settings["eccflow"]).isGroup() ) {
    load_settings(group, "support_scale", &base_options.eccflow.support_scale);
    load_settings(group, "normalization_scale", &base_options.eccflow.normalization_scale);
    load_settings(group, "update_multiplier", &base_options.eccflow.update_multiplier);
    load_settings(group, "max_iterations", &base_options.eccflow.max_iterations);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool c_image_stacking_output_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "output_directory", output_directory);
  save_settings(settings, "output_aligned_video_filename", output_aligned_video_filename);
  save_settings(settings, "processed_frame_filename", processed_frame_filename);
  save_settings(settings, "write_aligned_video", write_aligned_video);
  save_settings(settings, "save_processed_frames", save_processed_frames);
  save_settings(settings, "dump_reference_data_for_debug", dump_reference_data_for_debug);
  save_settings(settings, "write_image_mask_as_alpha_channel", write_image_mask_as_alpha_channel);

  if ( frame_processor ) {
    save_settings(settings, "frame_processor", frame_processor->name() );
  }

  if ( accumuated_image_processor ) {
    save_settings(settings, "accumuated_image_processor", accumuated_image_processor->name());
  }

  return true;
}

bool c_image_stacking_output_options::deserialize(c_config_setting settings)
{
  std::string s;

  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "output_directory", &output_directory);
  load_settings(settings, "output_aligned_video_filename", &output_aligned_video_filename);
  load_settings(settings, "processed_frame_filename", &processed_frame_filename);
  load_settings(settings, "write_aligned_video", &write_aligned_video);
  load_settings(settings, "save_processed_frames", &save_processed_frames);
  load_settings(settings, "dump_reference_data_for_debug", &dump_reference_data_for_debug);
  load_settings(settings, "write_image_mask_as_alpha_channel", &write_image_mask_as_alpha_channel);


  if ( load_settings(settings, "frame_processor", &s) && !s.empty() ) {
    frame_processor = c_image_processor_collection::default_instance()->get(s);
  }

  if ( load_settings(settings, "accumuated_image_processor", &s) && !s.empty() ) {
    accumuated_image_processor = c_image_processor_collection::default_instance()->get(s);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
