/*
 * c_image_stacking_pipeline_load_save.cc
 *
 *  Created on: Aug 22, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/feature2d/feature2d_settings.h>
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

  // CF_DEBUG("Saving '%s' ...", filename.c_str());

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
    CF_ERROR("No output config file name specified for c_image_stacks_collection::load()");
    return false;
  }

  // CF_DEBUG("Loading '%s' ...", filename.c_str());

  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return false;
  }


  std::string object_class;
  if ( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("[%s] load_settings(object_class) fails", filename.c_str());
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

c_image_stacking_options::ptr c_image_stacking_options::load(const std::string & cfgfilename)
{
  if ( cfgfilename.empty() ) {
    CF_ERROR("c_image_stacking_options: No input file name specified");
    return nullptr;
  }

  const std::string filename =
      expand_path(cfgfilename);

  //CF_DEBUG("Saving '%s' ...",
  //    filename.c_str());

  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return nullptr;
  }


  std::string object_class;
  if ( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("[%s] load_settings(object_class) fails", filename.c_str());
    return nullptr;
  }

  if ( object_class != "c_image_stacking_options" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return nullptr;
  }

  c_config_setting root = cfg.root();
  if ( !root || !root.isGroup() ) {
    CF_FATAL("cfg.root() is not group in file '%s''",
        filename.c_str());
    return nullptr;
  }

  c_image_stacking_options::ptr obj =
      c_image_stacking_options::create("");

  if ( !obj->deserialize(root) ) {
    CF_FATAL("obj->deserialize() fails for file '%s''",
        filename.c_str());
    return nullptr;
  }

  return obj;
}

bool c_image_stacking_options::save(const std::string & cfgfilename) const
{
  if ( cfgfilename.empty() ) {
    CF_ERROR("c_image_stacking_options: No output file name specified");
    return false;
  }

  const std::string filename =
      expand_path(cfgfilename);

  // CF_DEBUG("Saving '%s' ...",
  //    filename.c_str());

  c_config cfg(filename);

  time_t t = time(0);

  if ( !save_settings(cfg.root(), "object_class", std::string("c_image_stacking_options")) ) {
    CF_FATAL("c_image_stacking_options: save_settings(object_class) fails");
    return false;
  }

  if ( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("c_image_stacking_options: save_settings(created) fails");
    return false;
  }

  if ( !serialize(cfg.root()) ) {
    CF_FATAL("c_image_stacking_options: serialize() fails");
    return false;
  }


  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  return true;
}


bool c_image_stacking_options::serialize(c_config_setting settings) const
{
  settings.set("name", name_);

  input_options_.serialize(settings.add_group("input_options"));
  frame_registration_options_.master_frame_options.serialize(settings.add_group("master_frame_options"));
  roi_selection_options_.serialize(settings.add_group("roi_selection"));
  upscale_options_.serialize(settings.add_group("upscale_options"));
  frame_registration_options_.serialize(settings.add_group("frame_registration"));
  accumulation_options_.serialize(settings.add_group("accumulation_options"));
  image_processing_options_.serialize(settings.add_group("image_processing_options"));
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
  frame_registration_options_.master_frame_options.deserialize(settings["master_frame_options"]);
  frame_registration_options_.deserialize(settings["frame_registration"]);
  accumulation_options_.deserialize(settings["accumulation_options"]);
  image_processing_options_.deserialize(settings["image_processing_options"]);
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
  settings.set("darkbayer", darkbayer_filename);
  settings.set("missing_pixel_mask", missing_pixel_mask_filename);
  settings.set("missing_pixels_marked_black", missing_pixels_marked_black);
  settings.set("inpaint_missing_pixels", inpaint_missing_pixels);
  settings.set("remove_bad_pixels", filter_bad_pixels);
  settings.set("drop_bad_asi_frames", drop_bad_asi_frames);
  settings.set("bad_pixels_variation_threshold", hot_pixels_variation_threshold);
  settings.set("enable_color_maxtrix", enable_color_maxtrix );
  settings.set("anscombe", anscombe);
  settings.set("start_frame_index", start_frame_index);
  settings.set("max_input_frames", max_input_frames);

  return false;
}

bool c_input_options::deserialize(c_config_setting settings)
{
  std::string s;

  if ( !settings.isGroup() ) {
    return false;
  }

  settings.get("darkbayer", &darkbayer_filename);
  settings.get("missing_pixel_mask", &missing_pixel_mask_filename);
  settings.get("missing_pixels_marked_black", &missing_pixels_marked_black);
  settings.get("inpaint_missing_pixels", &inpaint_missing_pixels);
  settings.get("remove_bad_pixels", &filter_bad_pixels);
  settings.get("drop_bad_asi_frames", &drop_bad_asi_frames);
  settings.get("bad_pixels_variation_threshold", &hot_pixels_variation_threshold);
  settings.get("enable_color_maxtrix", &enable_color_maxtrix );
  settings.get("anscombe", &anscombe);
  settings.get("start_frame_index", &start_frame_index);
  settings.get("max_input_frames", &max_input_frames);

  return true;
}
///////////////////////////////////////////////////////////////////////////////
bool c_roi_selection_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "roi_selection_method", method);
  save_settings(settings, "rectangle_roi_selection", rectangle_roi_selection);
  save_settings(settings, "planetary_disk_crop_size", planetary_disk_crop_size);
  save_settings(settings, "planetary_disk_gbsigma", planetary_disk_gbsigma);
  save_settings(settings, "planetary_disk_stdev_factor", planetary_disk_stdev_factor);

  return true;
}

bool c_roi_selection_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "roi_selection_method", &method);
  load_settings(settings, "rectangle_roi_selection", &rectangle_roi_selection);
  load_settings(settings, "planetary_disk_crop_size", &planetary_disk_crop_size);
  load_settings(settings, "planetary_disk_gbsigma", &planetary_disk_gbsigma);
  load_settings(settings, "planetary_disk_stdev_factor", &planetary_disk_stdev_factor);


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
  save_settings(settings, "master_selection_method", master_selection_method);
  save_settings(settings, "master_source_path", master_source_path);
  save_settings(settings, "master_frame_index",master_frame_index );
  save_settings(settings, "apply_input_frame_processors", apply_input_frame_processors);
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

  load_settings(settings, "master_selection_method", &master_selection_method);
  load_settings(settings, "master_source_path", &master_source_path);
  load_settings(settings, "master_frame_index", &master_frame_index );
  load_settings(settings, "apply_input_frame_processors", &apply_input_frame_processors);
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
  c_config_setting section;

  save_settings(settings, "accumulation_method", accumulation_method );


  section = settings.add_group("c_lpg_sharpness_measure");
  SAVE_PROPERTY(section, m_, k);
  SAVE_PROPERTY(section, m_, dscale);
  SAVE_PROPERTY(section, m_, uscale);
  SAVE_PROPERTY(section, m_, squared);
  SAVE_PROPERTY(section, m_, avgchannel);

  section = settings.add_group("c_laplacian_pyramid_focus_stacking");
  SAVE_OPTION(section, fs_, fusing_policy);
  SAVE_OPTION(section, fs_, inpaint_mask_holes);
  SAVE_OPTION(section, fs_, avgchannel);
  SAVE_OPTION(section, fs_, kradius);
  SAVE_OPTION(section, fs_, inpaint_mask_holes);

  return true;
}

bool c_frame_accumulation_options::deserialize(c_config_setting settings)
{
  c_config_setting section;

  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "accumulation_method", &accumulation_method );

  if( (section = settings["c_lpg_sharpness_measure"]).isGroup() ) {
    LOAD_PROPERTY(section, m_, k);
    LOAD_PROPERTY(section, m_, dscale);
    LOAD_PROPERTY(section, m_, uscale);
    LOAD_PROPERTY(section, m_, squared);
    LOAD_PROPERTY(section, m_, avgchannel);
  }

  if( (section = settings["c_laplacian_pyramid_focus_stacking"]).isGroup() ) {
    LOAD_OPTION(section, fs_, fusing_policy);
    LOAD_OPTION(section, fs_, inpaint_mask_holes);
    LOAD_OPTION(section, fs_, avgchannel);
    LOAD_OPTION(section, fs_, kradius);
    LOAD_OPTION(section, fs_, inpaint_mask_holes);
  }

  return true;
}
///////////////////////////////////////////////////////////////////////////////

bool c_frame_registration_options::serialize(c_config_setting settings) const
{
  c_config_setting section, subsection;

  section = settings;
  SAVE_OPTION(section, image_registration_options, enable_frame_registration);
  SAVE_OPTION(section, image_registration_options, motion_type);
  SAVE_OPTION(section, image_registration_options, registration_channel);
  SAVE_OPTION(section, image_registration_options, interpolation);
  SAVE_OPTION(section, image_registration_options, border_mode);
  SAVE_OPTION(section, image_registration_options, border_value);
  save_settings(section, "accumulate_and_compensate_turbulent_flow", accumulate_and_compensate_turbulent_flow);

  section = settings.add_group("feature_registration");
  SAVE_OPTION(section, image_registration_options.feature_registration, enabled);
  SAVE_OPTION(section, image_registration_options.feature_registration, scale);
  save_settings(section.add_group("sparse_feature_extractor"),
      image_registration_options.feature_registration.sparse_feature_extractor);
  save_settings(section.add_group("sparse_feature_matcher"),
      image_registration_options.feature_registration.sparse_feature_matcher);

  // save_settings(group, "align_planetary_disk_masks", planetary_disk_options.align_planetary_disk_masks);

  section = settings.add_group("ecc");
  SAVE_OPTION(section, image_registration_options.ecc, enabled);
  SAVE_OPTION(section, image_registration_options.ecc, scale);
  SAVE_OPTION(section, image_registration_options.ecc, eps);
  SAVE_OPTION(section, image_registration_options.ecc, min_rho);
  SAVE_OPTION(section, image_registration_options.ecc, input_smooth_sigma);
  SAVE_OPTION(section, image_registration_options.ecc, reference_smooth_sigma);
  SAVE_OPTION(section, image_registration_options.ecc, update_step_scale);
  SAVE_OPTION(section, image_registration_options.ecc, normalization_noise);
  SAVE_OPTION(section, image_registration_options.ecc, normalization_scale);
  SAVE_OPTION(section, image_registration_options.ecc, max_iterations);
  SAVE_OPTION(section, image_registration_options.ecc, enable_ecch);
  SAVE_OPTION(section, image_registration_options.ecc, ecch_minimum_image_size);
  SAVE_OPTION(section, image_registration_options.ecc, ecch_estimate_translation_first);
  SAVE_OPTION(section, image_registration_options.ecc, replace_planetary_disk_with_mask);
  SAVE_OPTION(section, image_registration_options.ecc, planetary_disk_mask_stdev_factor);


  section = settings.add_group("eccflow");
  SAVE_OPTION(section, image_registration_options.eccflow, enabled);
  SAVE_OPTION(section, image_registration_options.eccflow, update_multiplier);
  SAVE_OPTION(section, image_registration_options.eccflow, input_smooth_sigma);
  SAVE_OPTION(section, image_registration_options.eccflow, reference_smooth_sigma);
  SAVE_OPTION(section, image_registration_options.eccflow, max_iterations);
  SAVE_OPTION(section, image_registration_options.eccflow, support_scale);
  SAVE_OPTION(section, image_registration_options.eccflow, normalization_scale);

  section = settings.add_group("jovian_derotation");
  SAVE_OPTION(section, image_registration_options.jovian_derotation, enabled);
  //SAVE_OPTION(section, image_registration_options.jovian_derotation, align_planetary_disk_masks);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, min_rotation);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, max_rotation);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, max_pyramid_level);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, num_orientations);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, eccflow_support_scale);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, eccflow_normalization_scale);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, eccflow_max_pyramid_level);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, rotate_jovian_disk_horizontally);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, derotate_all_frames);
  SAVE_OPTION(section, image_registration_options.jovian_derotation, derotate_all_frames_max_context_size);
  SAVE_OPTION(section, image_registration_options.jovian_derotation.ellipse, stdev_factor);
  SAVE_OPTION(section, image_registration_options.jovian_derotation.ellipse, force_reference_ellipse);


  return true;
}

bool c_frame_registration_options::deserialize(c_config_setting settings)
{
  if ( !settings.isGroup() ) {
    return false;
  }

  c_config_setting section = settings;

  LOAD_OPTION(section, image_registration_options, enable_frame_registration);
  LOAD_OPTION(section, image_registration_options, motion_type);
  LOAD_OPTION(section, image_registration_options, registration_channel);
  LOAD_OPTION(section, image_registration_options, interpolation);
  LOAD_OPTION(section, image_registration_options, border_mode);
  LOAD_OPTION(section, image_registration_options, border_value);
  load_settings(section, "accumulate_and_compensate_turbulent_flow",
      &accumulate_and_compensate_turbulent_flow);


  if( (section = settings["feature_registration"]).isGroup() ) {
    LOAD_OPTION(section, image_registration_options.feature_registration, enabled);
    LOAD_OPTION(section, image_registration_options.feature_registration, scale);
    if( section["sparse_feature_extractor"] ) {
      load_settings(section["sparse_feature_extractor"],
          &image_registration_options.feature_registration.sparse_feature_extractor);
    }
    if( section["sparse_feature_matcher"] ) {
      load_settings(section.add_group("sparse_feature_matcher"),
          &image_registration_options.feature_registration.sparse_feature_matcher);
    }
  }

  // load_settings(ection, "align_planetary_disk_masks", &planetary_disk_options.align_planetary_disk_masks);

  if( (section = settings["ecc"]).isGroup() ) {
    LOAD_OPTION(section, image_registration_options.ecc, enabled);
    LOAD_OPTION(section, image_registration_options.ecc, scale);
    LOAD_OPTION(section, image_registration_options.ecc, eps);
    LOAD_OPTION(section, image_registration_options.ecc, min_rho);
    LOAD_OPTION(section, image_registration_options.ecc, input_smooth_sigma);
    LOAD_OPTION(section, image_registration_options.ecc, reference_smooth_sigma);
    LOAD_OPTION(section, image_registration_options.ecc, update_step_scale);
    LOAD_OPTION(section, image_registration_options.ecc, normalization_noise);
    LOAD_OPTION(section, image_registration_options.ecc, normalization_scale);
    LOAD_OPTION(section, image_registration_options.ecc, max_iterations);
    LOAD_OPTION(section, image_registration_options.ecc, enable_ecch);
    LOAD_OPTION(section, image_registration_options.ecc, ecch_minimum_image_size);
    LOAD_OPTION(section, image_registration_options.ecc, ecch_estimate_translation_first);
    LOAD_OPTION(section, image_registration_options.ecc, replace_planetary_disk_with_mask);
    LOAD_OPTION(section, image_registration_options.ecc, planetary_disk_mask_stdev_factor);
  }

  if( (section = settings["eccflow"]).isGroup() ) {
    LOAD_OPTION(section, image_registration_options.eccflow, enabled);
    LOAD_OPTION(section, image_registration_options.eccflow, update_multiplier);
    LOAD_OPTION(section, image_registration_options.eccflow, input_smooth_sigma);
    LOAD_OPTION(section, image_registration_options.eccflow, reference_smooth_sigma);
    LOAD_OPTION(section, image_registration_options.eccflow, max_iterations);
    LOAD_OPTION(section, image_registration_options.eccflow, support_scale);
    LOAD_OPTION(section, image_registration_options.eccflow, normalization_scale);
  }

  if( (section = settings["jovian_derotation"]).isGroup() ) {
    LOAD_OPTION(section, image_registration_options.jovian_derotation, enabled);
    //LOAD_OPTION(section, image_registration_options.jovian_derotation, align_planetary_disk_masks);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, min_rotation);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, max_rotation);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, max_pyramid_level);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, num_orientations);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, eccflow_support_scale);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, eccflow_normalization_scale);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, eccflow_max_pyramid_level);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, rotate_jovian_disk_horizontally);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, derotate_all_frames);
    LOAD_OPTION(section, image_registration_options.jovian_derotation, derotate_all_frames_max_context_size);
    LOAD_OPTION(section, image_registration_options.jovian_derotation.ellipse, stdev_factor);
    LOAD_OPTION(section, image_registration_options.jovian_derotation.ellipse, force_reference_ellipse);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool c_image_processing_options::serialize(c_config_setting settings) const
{
  if( input_image_processor ) {
    save_settings(settings, "input_image_processor", input_image_processor->name());
  }
  if( ecc_image_processor ) {
    save_settings(settings, "ecc_image_processor", ecc_image_processor->name());
  }
  if( aligned_image_processor ) {
    save_settings(settings, "aligned_image_processor", aligned_image_processor->name());
  }
  if ( incremental_frame_processor ) {
    save_settings(settings, "incremental_frame_processor", incremental_frame_processor->name());
  }
  if( accumulated_image_processor ) {
    save_settings(settings, "accumulated_image_processor", accumulated_image_processor->name());
  }
  return true;
}

bool c_image_processing_options::deserialize(c_config_setting settings)
{
  std::string s;

  if ( load_settings(settings, "input_image_processor", &s) && !s.empty() ) {
    input_image_processor = c_image_processor_collection::default_instance()->get(s);
  }
  if ( load_settings(settings, "ecc_image_processor", &s) && !s.empty() ) {
    ecc_image_processor = c_image_processor_collection::default_instance()->get(s);
  }
  if ( load_settings(settings, "aligned_image_processor", &s) && !s.empty() ) {
    aligned_image_processor = c_image_processor_collection::default_instance()->get(s);
  }
  if ( load_settings(settings, "incremental_frame_processor", &s) && !s.empty() ) {
    incremental_frame_processor = c_image_processor_collection::default_instance()->get(s);
  }
  if ( load_settings(settings, "accumulated_image_processor", &s) && !s.empty() ) {
    accumulated_image_processor = c_image_processor_collection::default_instance()->get(s);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
bool c_image_stacking_output_options::serialize(c_config_setting settings) const
{
  save_settings(settings, "output_directory", output_directory);

  save_settings(settings, "output_preprocessed_frames_filename", output_preprocessed_frames_filename);
  save_settings(settings, "output_aligned_frames_filename", output_aligned_frames_filename);
  save_settings(settings, "output_ecc_frames_filename", output_ecc_frames_filename);
  save_settings(settings, "output_postprocessed_frames_filename", output_postprocessed_frames_filename);
  save_settings(settings, "output_accumulation_masks_filename", output_accumulation_masks_filename);

  save_settings(settings, "save_preprocessed_frames", save_preprocessed_frames);
  save_settings(settings, "save_aligned_frames", save_aligned_frames);
  save_settings(settings, "save_ecc_frames", save_ecc_frames);
  save_settings(settings, "save_processed_aligned_frames", save_processed_aligned_frames);
  save_settings(settings, "save_incremental_frames", save_incremental_frames);
  save_settings(settings, "save_accumulation_masks", save_accumulation_masks);


  save_settings(settings, "dump_reference_data_for_debug", dump_reference_data_for_debug);
  save_settings(settings, "write_image_mask_as_alpha_channel", write_image_mask_as_alpha_channel);

  return true;
}

bool c_image_stacking_output_options::deserialize(c_config_setting settings)
{
  std::string s;

  if ( !settings.isGroup() ) {
    return false;
  }

  load_settings(settings, "output_directory", &output_directory);

  load_settings(settings, "output_preprocessed_frames_filename", &output_preprocessed_frames_filename);
  load_settings(settings, "output_aligned_frames_filename", &output_aligned_frames_filename);
  load_settings(settings, "output_ecc_frames_filename", &output_ecc_frames_filename);
  load_settings(settings, "output_postprocessed_frames_filename", &output_postprocessed_frames_filename);
  load_settings(settings, "output_accumulation_masks_filename", &output_accumulation_masks_filename);

  load_settings(settings, "save_preprocessed_frames", &save_preprocessed_frames);
  load_settings(settings, "save_aligned_frames", &save_aligned_frames);
  load_settings(settings, "save_ecc_frames", &save_ecc_frames);
  load_settings(settings, "save_processed_aligned_frames", &save_processed_aligned_frames);
  load_settings(settings, "save_incremental_frames", &save_incremental_frames);
  load_settings(settings, "save_accumulation_masks", &save_accumulation_masks);


  load_settings(settings, "dump_reference_data_for_debug", &dump_reference_data_for_debug);
  load_settings(settings, "write_image_mask_as_alpha_channel", &write_image_mask_as_alpha_channel);

  return true;
}

///////////////////////////////////////////////////////////////////////////////
