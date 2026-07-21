/*
 * c_running_average_pipeline.cc
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#include "c_running_average_pipeline.h"
#include <core/proc/unsharp_mask.h>
#include <core/io/load_image.h>

c_running_average_pipeline::c_running_average_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const std::string& c_running_average_pipeline::get_class_name() const
{
  return class_name();
}

const std::string& c_running_average_pipeline::class_name()
{
  static const std::string _classname = "running_average";
  return _classname;
}

const std::string& c_running_average_pipeline::tooltip()
{
  static const std::string _tooltip =
      "<strong>c_running_average_pipeline.</strong><br>"
          "test for running average registered frames<br>";
  return _tooltip;
}

bool c_running_average_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if( !base::serialize(settings, save) ) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_image_stacking_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "registration_options")) ) {

    SERIALIZE_OPTION(section, save, _registration_options, motion_type);
    SERIALIZE_OPTION(section, save, _registration_options, enable_star_registration);
    SERIALIZE_OPTION(section, save, _registration_options, enable_ecc_registration);
    SERIALIZE_OPTION(section, save, _registration_options, enable_eccflow_registration);

    if ( auto group = SERIALIZE_GROUP(section, save, "star_registration") ) {
      if( auto g = SERIALIZE_GROUP(group, save, "star_detection") ) {
        serialize_simple_star_detector_options(g, save, _registration_options.star_detection);
      }
      if( auto g = SERIALIZE_GROUP(group, save, "triangle_extractor") ) {
        serialize_triangle_extractor_options(g, save, _registration_options.triangle_extractor);
      }
      if( auto g = SERIALIZE_GROUP(group, save, "triangle_matcher") ) {
        serialize_triangle_matcher_options(g, save, _registration_options.triangle_matcher);
      }
    }

    if ( auto group = SERIALIZE_GROUP(section, save, "ecch") ) {
      serialize_ecch_options(group, save, _registration_options.ecch);
    }

    if ( auto group = SERIALIZE_GROUP(section, save, "eccflow") ) {
      serialize_eccflow_options(group, save, _registration_options.eccflow);
    }
  }

  if( (section = SERIALIZE_GROUP(settings, save, "average_options")) ) {
    SERIALIZE_OPTION(section, save, _average_options, running_weight);
    if( auto group = SERIALIZE_GROUP(section, save, "lpg") ) {
      serialize_lpg_options(group, save, _average_options.lpg);
    }
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, _output_options, default_display_type);
    SERIALIZE_OPTION(section, save, _output_options, output_directory);

    SERIALIZE_OPTION(section, save, _output_options, save_accumulated_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_accumulated_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_reference_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "output_reference_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_reference_video_options);
    }


    SERIALIZE_OPTION(section, save, _output_options, display_scale);
  }

  return true;
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_input_options> & ctx)
{
  using S = c_running_average_input_options;
  ctlbind(ctls, as_base<c_image_stacking_pipeline_base_input_options>(ctx));
  //ctlbind(ctls, "ecc_image_processor", ctx(&S::ecc_image_processor));
}


template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_registration_options> & ctx)
{
  using S = c_running_average_registration_options;

  ctlbind(ctls, "motion_type", ctx(&S::motion_type), "");

  ctlbind_expandable_group(ctls, "Stars", "");
    ctlbind(ctls, "Enable star registration", ctx(&S::enable_star_registration), "");
    ctlbind_expandable_group(ctls, "Star Detection", "");
      ctlbind(ctls, ctx(&S::star_detection));
    ctlbind_end_group(ctls);
    ctlbind_expandable_group(ctls, "Triangle extraction", "");
      ctlbind(ctls, ctx(&S::triangle_extractor));
    ctlbind_end_group(ctls);
    ctlbind_expandable_group(ctls, "Triangle Matching", "");
      ctlbind(ctls, ctx(&S::triangle_matcher));
    ctlbind_end_group(ctls);
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "ECC", "");
    ctlbind(ctls, "Enable ecc", ctx(&S::enable_ecc_registration), "");
    ctlbind(ctls, ctx(&S::ecch));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "ECCFLOW", "");
    ctlbind(ctls, "Enable eccflow", ctx(&S::enable_eccflow_registration), "");
    ctlbind(ctls, ctx(&S::eccflow));
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_update_options> & ctx)
{
  using S = c_running_average_update_options;

  //ctlbind(ctls, "reference running_weight", ctx(&S::reference_weight), ""); // , "", (_this->_registration_options.double_align_moode));
  ctlbind(ctls, "running_weight", ctx(&S::running_weight), "");
  ctlbind_expandable_group(ctls, "Shapness measure", "");
    ctlbind(ctls, ctx(&S::lpg));
  ctlbind_end_group(ctls);
}

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_running_average_output_options> & ctx)
{
  using S = c_running_average_output_options;

  ctlbind(ctls, "display_type", ctx(&S::default_display_type), "");
  ctlbind(ctls, "display_scale", ctx(&S::display_scale), "");
  ctlbind_browse_for_directory(ctls, "output_directory", ctx(&S::output_directory), "");

  ctlbind_expandable_group(ctls, "Save accumulated video");
    ctlbind(ctls, "save_accumulated_video", ctx(&S::save_accumulated_video), "");
    ctlbind(ctls, ctx(&S::output_accumulated_video_options));//  (_this->_output_options.save_accumulated_video));
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Save reference video", "");
    ctlbind(ctls, "save_reference_video", ctx(&S::save_reference_video), "");
    ctlbind(ctls, ctx(&S::output_reference_video_options)); //  (_this->_output_options.save_reference_video));
  ctlbind_end_group(ctls);
}

const c_ctlist<c_running_average_pipeline> & c_running_average_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "Input options", "");
      ctlbind(ctls, ctx(&this_class::_input_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Image registration", "");
      ctlbind(ctls, ctx(&this_class::_registration_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Average options", "");
      ctlbind(ctls, ctx(&this_class::_average_options));
    ctlbind_end_group(ctls);

    ctlbind_expandable_group(ctls, "Output options", "");
      ctlbind(ctls, ctx(&this_class::_output_options));
    ctlbind_end_group(ctls);
  }

  return ctls;
}

bool c_running_average_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  return _average.compute(display_frame, display_mask, _input_bpp > 0 ? 1 << _input_bpp : 1, -1, true);
}

bool c_running_average_pipeline::copy_parameters(const c_image_processing_pipeline::sptr & dst) const
{
  if ( !base::copy_parameters(dst) ) {
    CF_ERROR("c_running_average_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  p->_input_options = this->_input_options;
  p->_registration_options = this->_registration_options;
  p->_average_options = this->_average_options;
  p->_output_options = this->_output_options;
  p->_ecch.copy_parameters(_ecch);
  //p->ecc_.copy_parameters(ecc_);
  p->_eccflow.copy_parameters(_eccflow);

  return true;
}


bool c_running_average_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
    CF_ERROR("base::initialize() fails");
    return false;
  }

  _output_path = create_output_path(_output_options.output_directory);

  _average.clear();
  _current_image.release();
  _current_mask.release();
  _image_transform.reset();
  _star_extractor.clear();
  _triangle_extractor.clear();

  if ( !_input_options.darkbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.darkbayer_filename, _darkbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.darkbayer_filename.c_str());
      return false;
    }
  }

  if ( !_input_options.flatbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(_input_options.flatbayer_filename, _flatbayer, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.flatbayer_filename.c_str());
      return false;
    }
  }



  if ( !_input_options.missing_pixel_mask_filename.empty() ) {

    if ( !load_image(_input_options.missing_pixel_mask_filename, _missing_pixel_mask) ) {
      CF_ERROR("load_image('%s') fails.", _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( _missing_pixel_mask.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          _input_options.missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !_input_options.missing_pixels_marked_black ) {
      cv::invert(_missing_pixel_mask, _missing_pixel_mask);
    }
  }

  const bool enable_registration =
      _registration_options.enable_star_registration ||
          _registration_options.enable_ecc_registration ||
          _registration_options.enable_eccflow_registration;


  if ( enable_registration ) {
    _image_transform = create_image_transform(_registration_options.motion_type);

    if ( _registration_options.enable_star_registration ) {
      _star_extractor.set_options(_registration_options.star_detection);
      _triangle_extractor.set_options(_registration_options.triangle_extractor);
      _triangle_matcher.set_options(_registration_options.triangle_matcher);
    }

    if ( _registration_options.enable_ecc_registration ) {
      _ecch.set_image_transform(_image_transform.get());
      _ecch.set_options(_registration_options.ecch);
    }

    if( _registration_options.enable_eccflow_registration ) {
      _eccflow.set_options(_registration_options.eccflow);
    }
  }

  CF_DEBUG("Output path='%s'", this->_output_path.c_str());


  return true;
}

void c_running_average_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();

  _ecch.clear();
  _image_transform.reset();
  //_average.clear();

  _current_image.release();
  _current_mask.release();
}

bool c_running_average_pipeline::run_pipeline()
{
  if ( !start_pipeline(_input_options.start_frame_index, _input_options.max_input_frames) ) {
    CF_ERROR("ERROR: start_pipeline() fails");
    return false;
  }

  set_status_msg("RUNNING ...");

  for( ; _processed_frames < _total_frames; ++_processed_frames, ++_accumulated_frames, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    const bool fOk =
        read_input_frame(_input_sequence, _input_options,
            _current_image, _current_mask,
            false,
            false);

    if( !fOk ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if ( _current_image.empty() ) {
      // in case of corrupted ASI frame detection the read_input_frame() returns true with empty output image.
      continue;
    }

    _input_bpp = _input_sequence->bpp();

    if( _input_options.input_image_processor && !_input_options.input_image_processor->empty() ) {
      if( !_input_options.input_image_processor->process(_current_image, _current_mask) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame1() fails");
      return false;
    }
  }

  return true;
}

bool c_running_average_pipeline::process_current_frame()
{
  bool has_updates = true;

  const bool enable_registration =
      _registration_options.enable_star_registration ||
          _registration_options.enable_ecc_registration ||
          _registration_options.enable_eccflow_registration;

  if( !enable_registration || _average.accumulated_frames() < 1 ) {

    lock_guard lock(mutex());

    if ( !_average.add(_current_image, _current_mask, _average_options.running_weight) ) {
      CF_ERROR("average_add() fails");
      return false;
    }
  }
  else {

    static const auto mkgrayscale = [](const cv::Mat & src, cv::Mat & dst) {
      if( src.channels() != 1 ) {
        cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
      }
      else if ( &src != &dst ) {
        dst = src;
      }
    };

    cv::Mat current_image, current_mask, reference_image, reference_mask;
    cv::Mat2f rmap;

    if ( !_average.compute(reference_image, reference_mask, 1, -1, true) ) {
      CF_ERROR("_average.compute() fails");
      return !canceled();
    }

    mkgrayscale(reference_image, reference_image);
    mkgrayscale(_current_image, current_image);
    current_mask = _current_mask;

    if( _registration_options.enable_star_registration ) {

      std::vector<cv::KeyPoint> current_keypoints, reference_keypoints;
      cv::Mat current_descriptors, reference_descriptors;
      std::vector<cv::DMatch> triangle_matches;

      _star_extractor.detect(reference_image, reference_keypoints, reference_mask);
      if ( reference_keypoints.size() < 3 ) {
        CF_ERROR("_star_extractor.detect(reference_image) fails: reference_keypoints.size=%zu", reference_keypoints.size());
        return !canceled();
      }

      _triangle_extractor.compute(reference_image, reference_keypoints, reference_descriptors);
      _triangle_matcher.train(reference_keypoints, reference_descriptors);


      _star_extractor.detect(current_image, current_keypoints, current_mask);
      if ( reference_keypoints.size() < 3 ) {
        CF_ERROR("_star_extractor.detect(current_image) fails: current_keypoints.size=%zu", current_keypoints.size());
        return !canceled();
      }

      _triangle_extractor.compute(current_image, current_keypoints, current_descriptors);
      _triangle_matcher.match(current_keypoints, current_descriptors, triangle_matches);
      if ( triangle_matches.size() < 1 ) {
        CF_ERROR("_triangle_matcher.match() fails: triangle_matches.size()=%zu", triangle_matches.size());
        return !canceled();
      }

      const bool transformEstimated =
          estimate_image_transform(_image_transform.get(),
              current_keypoints, reference_keypoints, triangle_matches,
              _registration_options.transform_estimation);

      if( !transformEstimated ) {
        CF_ERROR("estimate_image_transform() fails");
        return !canceled();
      }
    }

    if( _registration_options.enable_ecc_registration ) {

      _ecch.set_reference_image(reference_image, reference_mask);
      if( !_ecch.align(current_image, current_mask) ) {
        CF_ERROR("_ecch.align() fails");
        return false;
      }

      CF_DEBUG("ECCH: %d iterations eps=%g", _ecch.num_iterations(),  _ecch.eps() );
    }

    _image_transform->create_remap(reference_image.size(), rmap);
    if( _registration_options.enable_eccflow_registration ) {
      _eccflow.set_reference_image(reference_image, reference_mask);
      if( !_eccflow.compute(current_image, rmap, current_mask) ) {
        CF_ERROR("_eccflow.compute() fails");
        return false;
      }
    }

    if ( true ) {
      lock_guard lock(mutex());
      if ( !_average.add(_current_image, _current_mask, _average_options.running_weight, &rmap) ) {
        CF_ERROR("average_add() fails");
        return false;
      }
    }
  }

  return true;
}


void c_running_average_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  c_lpg_sharpness_measure::create_map(src, dst, _average_options.lpg);
  if( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }
}


