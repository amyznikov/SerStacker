/*
 * c_image_processor.cc
 *
 *  Created on: Feb 20, 2021
 *      Author: amyznikov
 */

#include "c_image_processor.h"

#include "generic/c_push_image_routine.h"
#include "generic/c_pop_image_routine.h"
#include "generic/c_push_global_routine.h"
#include "generic/c_pop_global_routine.h"
#include "generic/c_clear_globals_routine.h"
#include "generic/c_load_image_routine.h"

#include "generic/c_gradient_routine.h"
#include "generic/c_laplacian_routine.h"
#include "generic/c_hessian_routine.h"
#include "generic/c_ridgeness_routine.h"
#include "generic/c_radial_gradient_routine.h"
#include "generic/c_noisemap_routine.h"
#include "generic/c_auto_correlation_routine.h"
#include "generic/c_local_contrast_map_routine.h"
#include "generic/c_lpg_map_routine.h"
#include "generic/c_laplacian_map_routine.h"
#include "generic/c_harris_map_routine.h"
#include "generic/c_gabor_filter_routine.h"
#include "generic/c_absdiff_routine.h"
#include "generic/c_anscombe_routine.h"
#include "generic/c_mtf_routine.h"
#include "generic/c_range_normalize_routine.h"
#include "generic/c_rangeclip_routine.h"
#include "generic/c_pixel_func_routine.h"
#include "generic/c_image_calc_routine.h"
#include "generic/c_math_expression_routine.h"
#include "generic/c_threshold_routine.h"
#include "generic/c_normalize_mean_stdev_routine.h"


#include "generic/c_histogram_white_balance_routine.h"
#include "generic/c_color_saturation_routine.h"
#include "generic/c_color_transform_routine.h"
#include "generic/c_cvtcolor_routine.h"
#include "generic/c_desaturate_edges_routine.h"
#include "generic/c_extract_channel_routine.h"
#include "generic/c_histogram_normalization_routine.h"
#include "generic/c_scale_channels_routine.h"
#include "generic/c_color_diff_routine.h"
#include "generic/c_set_luminance_channel_routine.h"
#include "generic/c_color_balance_routine.h"
#include "generic/c_align_color_channels_routine.h"
#include "generic/c_autoclip_routine.h"
#include "generic/c_unsharp_mask_routine.h"
#include "generic/c_auto_unsharp_mask_routine.h"
#include "generic/c_type_convert_routine.h"
#include "generic/c_radial_polysharp_routine.h"
#include "generic/c_median_blur_routine.h"
#include "generic/c_median_hat_routine.h"
#include "generic/c_wmf_routine.h"
#include "generic/c_remove_sharp_artifacts_routine.h"
#include "generic/c_mean_curvature_blur_routine.h"
#include "generic/c_equalize_hist_routine.h"
#include "generic/c_linear_interpolation_inpaint_routine.h"
#include "generic/c_bilateral_filter_routine.h"
#include "generic/c_pnormalize_routine.h"
#include "generic/c_gaussian_blur_routine.h"
#include "generic/c_neighbourhood_average_routine.h"
#include "generic/c_morphology_routine.h"
#include "generic/c_fft_routine.h"
#include "generic/c_local_minmax_routine.h"


#include "camera_calibration/c_find_chessboard_corners_routine.h"
#include "camera_calibration/c_image_rectification_routine.h"
#include "camera_calibration/c_stereo_rectification_routine.h"

#include "pyramid/c_downstrike_routine.h"
#include "pyramid/c_upject_routine.h"
#include "pyramid/c_laplacian_pyramid_routine.h"
#include "pyramid/c_average_pyramid_inpaint_routine.h"
#include "pyramid/c_gaussian_pyramid_routine.h"
#include "pyramid/c_melp_pyramid_routine.h"
#include "pyramid/c_scale_gaussian_pyramid_layers_routine.h"
#include "pyramid/c_mpyramid_routine.h"
#include "pyramid/c_morph_gradient_pyramid_routine.h"
#include "pyramid/c_median_pyramid_routine.h"
#include "pyramid/c_radial_scale_routine.h"
#include "pyramid/c_resize_pyramid_routine.h"
#include "pyramid/c_amelp_pyramid_routine.h"

#include "feature2d/c_keypoins_detector_routine.h"
#include "feature2d/c_edgebox_routine.h"
#include "feature2d/c_selective_search_segmentation_routine.h"
#include "feature2d/c_fit_jovian_ellipse_routine.h"


#include "geometry/c_crop_image_routine.h"
#include "geometry/c_rotate_image_routine.h"
#include "geometry/c_affine_transform_routine.h"
#include "geometry/c_perspective_transform_routine.h"
#include "geometry/c_birdview_transform_routine.h"
#include "geometry/c_tangential_transform_routine.h"
#include "geometry/c_polar_warp_routine.h"
#include "geometry/c_unkanala_remap_routine.h"
#include "geometry/c_flip_image_routine.h"
#include "geometry/c_transpose_image_routine.h"
#include "geometry/c_resize_image_routine.h"

#include "quicktests/c_census_transfrom_routine.h"
#include "quicktests/c_homography_test_routine.h"
#include "quicktests/c_rotation_homography_test_routine.h"
#include "quicktests/c_sweepscan_routine.h"
#include "quicktests/c_melp_stereo_matcher_routine.h"
#include "quicktests/c_edge_test_routine.h"
#include "quicktests/c_dnn_test_routine.h"
#include "quicktests/c_blur_test_routine.h"
#include "quicktests/c_draw_saturn_ellipse_routine.h"
#include "quicktests/c_draw_jovian_ellipse_routine.h"

#include <atomic>

#include <core/readdir.h>
#include <core/debug.h>


static std::vector<const c_image_processor_routine::class_factory*> c_image_processor_routine_class_list_;

c_image_processor_routine::class_factory::class_factory(const std::string & _class_name,
    const std::string & _display_name,
    const std::string & _tooltip,
    const std::function<c_image_processor_routine::ptr()> & _create_instance) :
        class_name(_class_name), display_name(_display_name), tooltip(_tooltip), create_instance(_create_instance)
{
}


const std::vector<const c_image_processor_routine::class_factory*> & c_image_processor_routine::class_list()
{
  register_all();
  return c_image_processor_routine_class_list_;
}


const c_image_processor_routine::class_factory * c_image_processor_routine::classfactory() const
{
  return class_factory_;
}

void c_image_processor_routine::register_class_factory(const class_factory * class_factory)
{
  class_list_guard_lock guard_lock;

  std::vector<const c_image_processor_routine::class_factory*>::iterator ii =
      std::find(c_image_processor_routine_class_list_.begin(),
          c_image_processor_routine_class_list_.end(),
          class_factory);

  if ( ii == c_image_processor_routine_class_list_.end() ) {
    c_image_processor_routine_class_list_.emplace_back(class_factory);
  }
}

void c_image_processor_routine::register_all()
{
  static std::atomic<bool> registered(false);
  if ( !registered ) {

    registered = true;

    register_class_factory(c_push_image_routine::class_factory_instance());
    register_class_factory(c_pop_image_routine::class_factory_instance());
    register_class_factory(c_push_global_routine::class_factory_instance());
    register_class_factory(c_pop_global_routine::class_factory_instance());
    register_class_factory(c_clear_globals_routine::class_factory_instance());
    register_class_factory(c_load_image_routine::class_factory_instance());

    register_class_factory(c_laplacian_routine::class_factory_instance());
    register_class_factory(c_hessian_routine::class_factory_instance());
    register_class_factory(c_gradient_routine::class_factory_instance());
    register_class_factory(c_radial_gradient_routine::class_factory_instance());
    register_class_factory(c_ridgeness_routine::class_factory_instance());

    register_class_factory(c_align_color_channels_routine::class_factory_instance());
    register_class_factory(c_anscombe_routine::class_factory_instance());
    register_class_factory(c_autoclip_routine::class_factory_instance());
    register_class_factory(c_histogram_white_balance_routine::class_factory_instance());
    register_class_factory(c_mtf_routine::class_factory_instance());
    register_class_factory(c_noisemap_routine::class_factory_instance());
    register_class_factory(c_range_normalize_routine::class_factory_instance());
    register_class_factory(c_rangeclip_routine::class_factory_instance());
    register_class_factory(c_pixel_func_routine::class_factory_instance());
    register_class_factory(c_image_calc_routine::class_factory_instance());
    register_class_factory(c_math_expression_routine::class_factory_instance());
    register_class_factory(c_threshold_routine::class_factory_instance());

    register_class_factory(c_unsharp_mask_routine::class_factory_instance());
    register_class_factory(c_auto_unsharp_mask_routine::class_factory_instance());

    register_class_factory(c_scale_channels_routine::class_factory_instance());
    register_class_factory(c_color_diff_routine::class_factory_instance());
    register_class_factory(c_set_luminance_channel_routine::class_factory_instance());
    register_class_factory(c_color_balance_routine::class_factory_instance());

    register_class_factory(c_type_convert_routine::class_factory_instance());
    register_class_factory(c_color_saturation_routine::class_factory_instance());
    register_class_factory(c_average_pyramid_inpaint_routine::class_factory_instance());
    register_class_factory(c_linear_interpolation_inpaint_routine::class_factory_instance());
    register_class_factory(c_radial_polysharp_routine::class_factory_instance());
    register_class_factory(c_auto_correlation_routine::class_factory_instance());
    register_class_factory(c_gaussian_blur_routine::class_factory_instance());
    register_class_factory(c_neighbourhood_average_routine::class_factory_instance());


    register_class_factory(c_crop_image_routine::class_factory_instance());
    register_class_factory(c_rotate_image_routine::class_factory_instance());
    register_class_factory(c_affine_transform_routine::class_factory_instance());
    register_class_factory(c_perspective_transform_routine::class_factory_instance());
    register_class_factory(c_birdview_transform_routine::class_factory_instance());
    register_class_factory(c_tangential_transform_routine::class_factory_instance());
    register_class_factory(c_polar_warp_routine::class_factory_instance());
    register_class_factory(c_unkanala_remap_routine::class_factory_instance());
    register_class_factory(c_flip_image_routine::class_factory_instance());
    register_class_factory(c_transpose_image_routine::class_factory_instance());
    register_class_factory(c_resize_image_routine::class_factory_instance());

    register_class_factory(c_histogram_normalization_routine::class_factory_instance());

    register_class_factory(c_gaussian_pyramid_routine::class_factory_instance());
    register_class_factory(c_scale_gaussian_pyramid_layers_routine::class_factory_instance());
    register_class_factory(c_melp_pyramid_routine::class_factory_instance());
    register_class_factory(c_laplacian_pyramid_routine::class_factory_instance());
    register_class_factory(c_mpyramid_routine::class_factory_instance());
    register_class_factory(c_morph_gradient_pyramid_routine::class_factory_instance());
    register_class_factory(c_median_pyramid_routine::class_factory_instance());
    register_class_factory(c_radial_scale_routine::class_factory_instance());
    register_class_factory(c_resize_pyramid_routine::class_factory_instance());
    register_class_factory(c_amelp_pyramid_routine::class_factory_instance());

    register_class_factory(c_downstrike_routine::class_factory_instance());
    register_class_factory(c_upject_routine::class_factory_instance());
    register_class_factory(c_morphology_routine::class_factory_instance());

    register_class_factory(c_median_blur_routine::class_factory_instance());
    register_class_factory(c_median_hat_routine::class_factory_instance());
    register_class_factory(c_wmf_routine::class_factory_instance());

    register_class_factory(c_remove_sharp_artifacts_routine::class_factory_instance());
    register_class_factory(c_mean_curvature_blur_routine::class_factory_instance());
    register_class_factory(c_fit_jovian_ellipse_routine::class_factory_instance());
    register_class_factory(c_desaturate_edges_routine::class_factory_instance());
    register_class_factory(c_local_contrast_map_routine::class_factory_instance());
    register_class_factory(c_lpg_map_routine::class_factory_instance());
    register_class_factory(c_laplacian_map_routine::class_factory_instance());
    register_class_factory(c_harris_map_routine::class_factory_instance());
    register_class_factory(c_gabor_filter_routine::class_factory_instance());

    register_class_factory(c_normalize_mean_stdev_routine::class_factory_instance());

    register_class_factory(c_cvtcolor_routine::class_factory_instance());
    register_class_factory(c_equalize_hist_routine::class_factory_instance());
    register_class_factory(c_extract_channel_routine::class_factory_instance());
    register_class_factory(c_absdiff_routine::class_factory_instance());
    register_class_factory(c_find_chessboard_corners_routine::class_factory_instance());
    register_class_factory(c_bilateral_filter_routine::class_factory_instance());
    register_class_factory(c_color_transform_routine::class_factory_instance());
    register_class_factory(c_homography_test_routine::class_factory_instance());
    register_class_factory(c_rotation_homography_test_routine::class_factory_instance());

    register_class_factory(c_sweepscan_routine::class_factory_instance());
    register_class_factory(c_melp_stereo_matcher_routine::class_factory_instance());
    register_class_factory(c_edge_test_routine::class_factory_instance());
    register_class_factory(c_dnn_test_routine::class_factory_instance());
    register_class_factory(c_blur_test_routine::class_factory_instance());
    register_class_factory(c_draw_saturn_ellipse_routine::class_factory_instance());
    register_class_factory(c_draw_jovian_ellipse_routine::class_factory_instance());

    register_class_factory(c_image_rectification_routine::class_factory_instance());
    register_class_factory(c_stereo_rectification_routine::class_factory_instance());
    register_class_factory(c_pnormalize_routine::class_factory_instance());
    register_class_factory(c_census_transfrom_routine::class_factory_instance());
    register_class_factory(c_fft_routine::class_factory_instance());
    register_class_factory(c_local_minmax_routine::class_factory_instance());

    register_class_factory(c_keypoins_detector_routine::class_factory_instance());
    register_class_factory(c_edgebox_routine::class_factory_instance());
    register_class_factory(c_selective_search_segmentation_routine::class_factory_instance());

  }
}

c_image_processor::c_image_processor(const std::string & objname, const std::string & filename ):
    name_(objname), filename_(filename)
{
}

c_image_processor::sptr c_image_processor::create(const std::string & objname)
{
  return sptr(new this_class(objname));
}

c_image_processor::sptr c_image_processor::load(const std::string & filename)
{
  c_config cfg(filename);

  if ( !cfg.read() ) {
    CF_FATAL("cfg.read('%s') fails", filename.c_str());
    return nullptr;
  }


  std::string object_class;
  if ( !::load_settings(cfg.root(), "object_class", &object_class) ) {
    CF_FATAL("load_settings(object_class) fails for %s", filename.c_str());
    return nullptr;
  }

  if ( object_class != "c_image_processor" ) {
    CF_FATAL("Incorrect object_class='%s' from file '%s'",
        object_class.c_str(), filename.c_str());
    return nullptr;
  }

  c_config_setting root = cfg.root().get("c_image_processor");
  if ( !root || !root.isGroup() ) {
    CF_FATAL("group 'c_image_processor' is not found in file '%s''",
        filename.c_str());
    return nullptr;
  }

  c_image_processor::sptr obj = deserialize(root);
  if ( obj ) {
    obj->filename_ = filename;
  }

  return obj;
}

c_image_processor::sptr c_image_processor::deserialize(c_config_setting settings)
{
  std::string objname;

  if ( !settings.get("name", &objname) || objname.empty() ) {
    CF_ERROR("c_image_processor: settings.get('name') fails");
    return nullptr;
  }

  sptr obj = create(objname);
  if ( !obj ) {
    CF_ERROR("c_image_processor::create(objname=%s) fails", objname.c_str());
    return nullptr;
  }

  c_config_setting chain =
      settings["chain"];

  if ( chain && !chain.isList() ) {
    CF_ERROR("c_image_processor::load(objname=%s): 'chain' item must be libconfig list", objname.c_str());
    return nullptr;
  }

  if ( chain ) {

    const int chain_length = chain.length();

    obj->reserve(chain_length);

    for ( int i = 0; i < chain_length; ++i ) {

      c_config_setting group =
          chain.get_element(i);

      if ( !group || !group.isGroup() ) {
        CF_ERROR("c_image_processor:  chain.get_element(objname=%s, index=%d, type=GROUP) fails", objname.c_str(), i);
        continue;
      }

      c_image_processor_routine::ptr routine =
          c_image_processor_routine::create(group);

      if ( !routine ) {
        CF_ERROR("c_image_processor_routine::create(objname=%s, chain index=%d) fails", objname.c_str(), i);
        continue;
      }

      obj->emplace_back(routine);
    }
  }

  return obj;
}


bool c_image_processor::save(const std::string & path_or_filename,
    const std::string & objname,
    bool incude_disabled_functions) const
{
  std::string filename;

  if ( path_or_filename.empty() ) {
    if ( !filename_.empty() ) {
      filename = filename_;
    }
    else {
      filename = ssprintf("%s/%s.cfg",
          c_image_processor_collection::default_processor_collection_path().c_str(),
          name_.c_str());
    }
  }
  else if ( path_or_filename[path_or_filename.length()-1] == '/' || is_directory(path_or_filename)) {
    filename = ssprintf("%s/%s.cfg", path_or_filename.c_str(), name_.c_str());
  }
  else if ( get_file_suffix(path_or_filename).empty() ) {
    filename = ssprintf("%s/%s.cfg", path_or_filename.c_str(), name_.c_str());
  }
  else {
    filename = path_or_filename;
  }

  if ( filename.empty() ) {
    CF_ERROR("can not decide file name to save config file for '%s'", name_.c_str());
    return false;
  }

  if ( filename[0] == '~' ) {
    filename = expand_path(filename);
  }

  c_config cfg(filename);

  time_t t = time(0);

  if ( !save_settings(cfg.root(), "object_class", std::string("c_image_processor")) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !save_settings(cfg.root(), "created", asctime(localtime(&t))) ) {
    CF_FATAL("save_settings() fails");
    return false;
  }

  if ( !serialize(cfg.root().add_group("c_image_processor"), objname, incude_disabled_functions)) {
    CF_FATAL("save(c_image_processor) fails");
    return false;
  }

  if ( !cfg.write() ) {
    CF_FATAL("cfg.write('%s') fails", cfg.filename().c_str());
    return false;
  }

  filename_ = filename;


  return true;
}

bool c_image_processor::serialize(c_config_setting settings, const std::string & objname, bool incude_disabled_functions) const
{
  if ( !settings ) {
    CF_ERROR("c_image_processor::save() gets invalid argument: c_config_setting = null");
    return false;
  }

  settings.set("name",  objname.empty() ? name_ : objname);

  c_config_setting chain =
      settings.add_list("chain");

  for( const c_image_processor_routine::ptr &routine : *this ) {
    if( routine ) {
      if( routine->enabled() || incude_disabled_functions ) {
        if( !routine->serialize(chain.add_element(CONFIG_TYPE_GROUP), true) ) {
          return false;
        }
      }
    }
  }

  return true;
}

c_image_processor::iterator c_image_processor::find(const c_image_processor_routine::ptr & p)
{
  return std::find(begin(), end(), p);
}

c_image_processor::const_iterator c_image_processor::find(const c_image_processor_routine::ptr & p) const
{
  return std::find(begin(), end(), p);
}

bool c_image_processor::process(cv::InputOutputArray image, cv::InputOutputArray mask) const
{
  c_image_processor::edit_lock lock(this);

  c_image_processor_routine::clear_artifacts();

  for ( const c_image_processor_routine::ptr & routine : *this ) {
    if ( routine && routine->enabled() ) {

      try {

        routine->emit_preprocess_notify(image, mask);

        if ( true ) {

          std::lock_guard<std::mutex> lock(routine->mutex());

          if ( !routine->process(image, mask) ) {

            CF_ERROR("[%s] processor->process() fails",
                routine->class_name().c_str());
          }
        }

        routine->emit_postprocess_notify(image, mask);
      }

      catch( const cv::Exception &e ) {
        CF_ERROR("OpenCV Exception in '%s' : %s\n"
            "%s() : %d\n"
            "file : %s\n",
            routine->class_name().c_str(),
            e.err.c_str(), ///< error description
            e.func.c_str(),///< function name. Available only when the compiler supports getting it
            e.line,///< line number in the source file where the error has occurred
            e.file.c_str()///< source file name where the error has occurred
            );
      }

      catch( const std::exception &e ) {

        CF_ERROR("std::exception in '%s': %s\n",
            routine->class_name().c_str(),
            e.what());
      }

      catch( ... ) {
        CF_ERROR("unknown exception in '%s'",
            routine->class_name().c_str());
      }
    }
  }

  c_image_processor_routine::clear_artifacts();

  return true;
}




c_image_processor_routine::ptr c_image_processor_routine::create(c_config_setting settings)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine::create(c_config_setting): settings is null");
    return nullptr;
  }

  std::string objname;
  if ( !settings.get("routine", &objname) || objname.empty() ) {
    CF_ERROR("No 'routine' string found");
    return nullptr;
  }

  c_image_processor_routine::ptr routine =
      c_image_processor_routine::create(objname);

  if( !routine ) {
    CF_ERROR("c_image_processor_routine::create(routine=%s) fails", objname.c_str());
    return nullptr;
  }

  if ( !routine->serialize(settings, false) ) {
    CF_ERROR("routine->serialize(routine=%s, c_config_setting, save=false) fails", objname.c_str());
    return nullptr;
  }

  return routine;
}

c_image_processor_routine::ptr c_image_processor_routine::create(const std::string & processor_name)
{
  register_all();

  if ( processor_name.empty() ) {
    return nullptr;
  }

  class_list_guard_lock lock;

  const char * cname = processor_name.c_str();
  for ( const class_factory * f : c_image_processor_routine_class_list_ ) {
    if ( strcasecmp(cname, f->class_name.c_str()) == 0 ) {

      c_image_processor_routine::ptr p =
          f->create_instance();

      if ( !p ) {
        CF_ERROR("factory->create_instance(class_name='%s') fails", cname);
      }
      else if ( !p->initialize() ) {
        CF_ERROR("p->initialize() fails for object class '%s'", cname);
        p.reset();
      }

      return p;
    }
  }

  return nullptr;
}


//bool c_image_processor_routine::deserialize(c_config_setting settings)
//{
//  if ( !settings ) {
//    CF_ERROR("c_image_processor_routine::load(c_config_setting) : settings is null");
//    return false;
//  }
//
//  settings.get("enabled", &enabled_);
//
//  return true;
//}

void c_image_processor_routine::parameter_changed()
{
}

bool c_image_processor_routine::serialize(c_config_setting settings, bool save)
{
  if ( !settings ) {
    CF_ERROR("c_image_processor_routine settings is null");
    return false;
  }

  if ( save ) {
    settings.set("routine", class_name());
    settings.set("display_name", display_name());
    settings.set("tooltip", tooltip());
    settings.set("enabled", enabled());
    settings.set("ignore_mask", ignore_mask());
  }
  else {
    settings.get("enabled", &enabled_);
    settings.get("ignore_mask", &ignore_mask_);
  }

  return true;
}


bool c_image_processor_collection::save(const std::string & output_path) const
{
  const std::string output_directory =
      expand_path(output_path.empty() ?
        default_processor_collection_path_ :
        output_path);

  if ( !create_path(output_directory) ) {
    CF_ERROR("create_path(output_directory=%s) fails: %s",
        output_directory.c_str(),
        strerror(errno));
    return  false;
  }

  for ( const c_image_processor::sptr & processor : *this ) {
    if ( processor && !processor->name().empty() ) {

      const std::string filename =
          ssprintf("%s/%s.cfg", output_directory.c_str(),
              processor->name().c_str());

      if ( !processor->save(filename) ) {
        CF_ERROR("processor->save(filename=%s) fails", filename.c_str());
      }
    }
  }

  return true;
}

bool c_image_processor_collection::load(const std::string & input_directrory)
{
  std::vector<std::string> filenames;

  if ( !readdir(&filenames, input_directrory, "*.cfg", true, DT_REG) ) {
    CF_ERROR("readdir(%s) fails: %s", input_directrory.c_str(), strerror(errno));
    return false;
  }

  for ( const std::string & filename : filenames ) {
    c_image_processor::sptr processor = c_image_processor::load(filename);
    if ( !processor ) {
      CF_ERROR("c_image_processor::load(filename='%s') fails", filename.c_str());
    }
    else {
      emplace_back(processor);
    }
  }

  if ( size() > 1 ) {
    std::sort(begin(), end(),
        [](const c_image_processor::sptr & prev, const c_image_processor::sptr & next) {
          return prev->name() < next->name();
        });
  }

  return true;
}


std::string c_image_processor_collection::default_processor_collection_path_ =
    "~/.config/SerStacker/image_processors";

c_image_processor_collection::sptr c_image_processor_collection::default_instance_ =
    c_image_processor_collection::create();

c_image_processor_collection::sptr c_image_processor_collection::default_instance()
{
  return default_instance_;
}

c_image_processor_collection::sptr c_image_processor_collection::create()
{
  return sptr(new this_class());
}

c_image_processor_collection::sptr c_image_processor_collection::create(c_config_setting settings)
{
  sptr obj(new this_class());
  if ( obj->deserialize(settings) ) {
    return obj;
  }
  return nullptr;
}

bool c_image_processor_collection::deserialize(c_config_setting settings)
{
  clear();

  if ( !settings ) {
    return false;
  }

  c_config_setting processors_list_item =
      settings["processors"];

  if ( !processors_list_item || !processors_list_item.isList() ) {
    return false;
  }

  const int num_processors = processors_list_item.length();

  reserve(num_processors);

  for ( int i = 0; i < num_processors; ++i ) {

    c_config_setting processor_item =
        processors_list_item.get_element(i);

    if ( processor_item && processor_item.isGroup() ) {
      c_image_processor::sptr processor = c_image_processor::deserialize(processor_item);
      if ( processor ) {
        emplace_back(processor);
      }
    }
  }

  if ( size() > 1 ) {
    std::sort(begin(), end(),
        [](const c_image_processor::sptr & prev, const c_image_processor::sptr & next) {
          return prev->name() < next->name();
        });
  }

  return true;
}

bool c_image_processor_collection::serialize(c_config_setting settings) const
{
  if ( !settings ) {
    return false;
  }

  c_config_setting processors_list_item =
      settings.add_list("processors");

  for ( const c_image_processor::sptr & processor : *this ) {
    if ( processor && !processor->serialize(processors_list_item.add_element(CONFIG_TYPE_GROUP)) ) {
      return false;
    }
  }

  return true;
}

c_image_processor_collection::iterator c_image_processor_collection::find(const std::string & name)
{
  return std::find_if(begin(), end(),
      [name]( const c_image_processor::sptr & p) {
        return p->name() == name;
      });
}

c_image_processor_collection::const_iterator c_image_processor_collection::find(const std::string & name) const
{
  return std::find_if(begin(), end(),
      [name]( const c_image_processor::sptr & p) {
        return p->name() == name;
      });
}

c_image_processor_collection::iterator c_image_processor_collection::find(const c_image_processor::sptr & p)
{
  return std::find(begin(), end(), p);
}

c_image_processor_collection::const_iterator c_image_processor_collection::find(const c_image_processor::sptr & p)  const
{
  return std::find(begin(), end(), p);
}

c_image_processor::sptr c_image_processor_collection::get(const std::string & name) const
{
  const_iterator pos = find(name);
  return pos == end() ? nullptr : *pos;
}



const std::string & c_image_processor_collection::default_processor_collection_path()
{
  return default_processor_collection_path_;
}

void c_image_processor_collection::set_default_processor_collection_path(const std::string & v)
{
  default_processor_collection_path_ = v;
}
