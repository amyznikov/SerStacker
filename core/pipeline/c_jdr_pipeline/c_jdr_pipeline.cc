/*
 * c_jdr_pipeline.cc
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#include "c_jdr_pipeline.h"
#include <core/proc/sharpness_measure/c_laplacian_sharpness_measure.h>

template<>
const c_enum_member* members_of<c_jdr_pipeline::STACKING_STAGE>()
{
  static const c_enum_member members[] = {
      { c_jdr_pipeline::stacking_stage_idle, "idle", "idle" },
      { c_jdr_pipeline::stacking_stage_initialize, "initialize", "initialize" },
      { c_jdr_pipeline::stacking_stage_select_master_frame_index, "select_master_frame_index", "select master frame index" },
      { c_jdr_pipeline::stacking_stage_generate_reference_frame, "generate_reference_frame", "generate reference frame" },
      { c_jdr_pipeline::stacking_stage_in_progress, "stacking_in_progress", "stacking in progress" },
      { c_jdr_pipeline::stacking_stage_finishing, "finishing", "finishing" },
      { c_jdr_pipeline::stacking_stage_idle },
  };

  return members;
}


c_jdr_pipeline::c_jdr_pipeline(const std::string & name, const c_input_sequence::sptr & input_sequence) :
  base(name, input_sequence)
{
  _master_options.master_selection.input_sequence = input_sequence.get();
  if( input_sequence && !input_sequence->sources().empty() ) {
    set_master_source(input_sequence->source(input_sequence->sources().size() / 2)->filename());
    set_master_frame_index(0);
  }
}

c_jdr_pipeline::~c_jdr_pipeline()
{

}

const std::string & c_jdr_pipeline::get_class_name() const
{
  return class_name();
}

const std::string & c_jdr_pipeline::class_name()
{
  static const std::string _classname = "jdr";
  return _classname;
}

const std::string & c_jdr_pipeline::tooltip()
{
  static const std::string _tooltip =
      "<strong>c_jdr_pipeline.</strong><br>"
      "<br>";
  return _tooltip;
}

const c_ctlist<c_jdr_pipeline::this_class> & c_jdr_pipeline::getcontrols()
{
  static c_ctlist<this_class> ctls;
  if ( ctls.empty() ) {
    c_ctlbind_context<this_class> ctx;

    ctlbind_expandable_group(ctls, "1. Input options",
        [&, cctx = ctx(&this_class::_input_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(cctx));
        });

    ctlbind_expandable_group(ctls, "2. ROI Selection",
        [&, cctx = ctx(&this_class::_roi_selection_options)]() {
          ctlbind(ctls, cctx);
        });

    ctlbind_expandable_group(ctls, "3. Master Options",
        [&, cctx = ctx(&this_class::_master_options)]() {
          ctlbind(ctls, CTL_CONTEXT(cctx, master_selection));
        });

    ctlbind_expandable_group(ctls, "4. Reference Frame Options ",
        [&, cctx = ctx(&this_class::_reference_frame_options)]() {
          ctlbind(ctls, "generate_reference_frame", CTL_CONTEXT(cctx, generate_reference_frame));
          ctlbind_browse_for_file(ctls, "reference file name", CTL_CONTEXT(cctx, reference_file_name));
          //ctlbind(ctls, "preprocess input frames", CTL_CONTEXT(cctx, input_image_preprocessor));
        });

    ctlbind_expandable_group(ctls, "5. Stack Options",
        [&, cctx = ctx(&this_class::_stack_options)]() {
        });

    ctlbind_expandable_group(ctls, "6. Output options",
        [&, cctx = ctx(&this_class::_output_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(cctx));
        });
  }

  return ctls;
}

bool c_jdr_pipeline::serialize(c_config_setting settings, bool save)
{
  static const auto get_group =
      [](c_config_setting setting, bool save, const std::string & name) {
        return save ? setting.add_group(name) : setting[name];
      };

  if ( !base::serialize(settings, save) ) {
    CF_ERROR("base::serialize(save=%d) fails", save);
    return false;
  }

  return true;
}


bool c_jdr_pipeline::copy_parameters(const c_image_processing_pipeline::sptr & dst) const
{
  if ( !base::copy_parameters(dst) ) {
    CF_ERROR("c_jdr_pipeline: base::copyParameters() fails");
    return false;
  }

  this_class::sptr p = std::dynamic_pointer_cast<this_class>(dst);
  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  const std::string backup_master_source_fiename = p->_master_options.master_selection.master_fiename;
  const int backup_master_frame_index = p->_master_options.master_selection.master_frame_index;


  p->_input_options = this->_input_options;
  p->_roi_selection_options = this->_roi_selection_options;
//  p->_upscale_options = this->_upscale_options;
  p->_master_options = this->_master_options;
//  p->_stacking_options = this->_stacking_options ;
//  p->_output_options = this->_output_options;
//  p->_image_processing_options = this->_image_processing_options;
//  p->_camera_intrinsics = this->_camera_intrinsics;

  p->_master_options.master_selection.master_fiename = backup_master_source_fiename;
  p->_master_options.master_selection.master_frame_index = backup_master_frame_index;

  return true;

}

bool c_jdr_pipeline::has_master_frame() const
{
  return true;
}

void c_jdr_pipeline::set_master_source(const std::string & master_source_path)
{
  _master_options.master_selection.master_fiename = master_source_path;
}

std::string c_jdr_pipeline::master_source() const
{
  return _master_options.master_selection.master_fiename;
}

void c_jdr_pipeline::set_master_frame_index(int v)
{
  _master_options.master_selection.master_selection_method = master_frame_specific_index;
  _master_options.master_selection.master_frame_index = v;
}

int c_jdr_pipeline::master_frame_index() const
{
  return _master_options.master_selection.master_frame_index;
}

bool c_jdr_pipeline::get_display_image(cv::OutputArray outputImage, cv::OutputArray outputMask)
{
  if ( outputImage.needed() ) {
    _current_master_frame_candidate.copyTo(outputImage);
  }
  if ( outputMask.needed() ) {
    _current_master_frame_candidate_mask.copyTo(outputMask);
  }
  return true;

//  switch (_pipeline_stage) {
//    case stacking_stage_idle:
//      if ( outputImage.needed() ) {
//      }
//      if ( outputMask.needed() ) {
//      }
//      break;
//    case stacking_stage_generate_reference_frame:
//      if ( outputImage.needed() ) {
//        _current_master_frame_candidate.copyTo(outputImage);
//      }
//      if ( outputMask.needed() ) {
//        _current_master_frame_candidate_mask.copyTo(outputMask);
//      }
//      return true;
//    default:
//      break;
//  }
//
//  return false;
}

void c_jdr_pipeline::set_pipeline_stage(int newstage)
{
  const auto oldstage = _pipeline_stage;
  if( newstage != oldstage ) {
    _pipeline_stage = newstage;
    on_status_update();
  }
}


bool c_jdr_pipeline::initialize_pipeline()
{
  if( !base::initialize_pipeline() ) {
    CF_ERROR("c_jdr_pipeline: base::initialize() fails");
    return false;
  }

//  set_pipeline_stage(stacking_stage_initialize);
  _output_path = create_output_path(_output_options.output_directory);

//
//
//  if (true ) {
//
//    lock_guard lock(mutex());
//
//    // ecc_normalization_noise_ = 0;
//
//    _output_file_name_postfix.clear();
//    _output_file_name.clear();
//
//    _roi_selection.reset();
//    _frame_registration.reset();
//    _frame_accumulation.reset();
//    _flow_accumulation.reset();
//    _generating_master_frame = false;
//    _master_options.registration.feature_registration.estimate_options.epipolar_derotation.camera_intrinsics = _camera_intrinsics;
//    _stacking_options.registration.feature_registration.estimate_options.epipolar_derotation.camera_intrinsics = _camera_intrinsics;
//  }
//
//  _anscombe.set_method(input_options().anscombe);
//
//  if ( roi_selection_options().method != roi_selection_none ) {
//    if ( !(_roi_selection = create_roi_selection()) ) {
//      set_status_msg("ERROR: create_roi_selection() fails");
//      return false;
//    }
//  }
//
//  if ( !input_options().darkbayer_filename.empty() ) {
//    cv::Mat ignored_optional_mask;
//    if ( !load_image(input_options().darkbayer_filename, _darkbayer, ignored_optional_mask) ) {
//      CF_ERROR("load_image('%s') fails.", input_options().darkbayer_filename.c_str());
//      return false;
//    }
//  }
//
//  if ( !input_options().flatbayer_filename.empty() ) {
//    cv::Mat ignored_optional_mask;
//    if ( !load_image(input_options().flatbayer_filename, _flatbayer, ignored_optional_mask) ) {
//      CF_ERROR("load_image('%s') fails.", input_options().flatbayer_filename.c_str());
//      return false;
//    }
//  }
//
//
//
//  if ( !input_options().missing_pixel_mask_filename.empty() ) {
//
//    if ( !load_image(input_options().missing_pixel_mask_filename, _missing_pixel_mask) ) {
//      CF_ERROR("load_image('%s') fails.", input_options().missing_pixel_mask_filename.c_str());
//      return false;
//    }
//
//    if ( _missing_pixel_mask.type() != CV_8UC1 ) {
//      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
//          input_options().missing_pixel_mask_filename.c_str());
//      return false;
//    }
//
//    if ( !input_options().missing_pixels_marked_black ) {
//      cv::invert(_missing_pixel_mask, _missing_pixel_mask);
//    }
//  }
//
//  CF_DEBUG("Output path='%s'", this->_output_path.c_str());
//
//
  return true;
}

void c_jdr_pipeline::cleanup_pipeline()
{
  //set_pipeline_stage(stacking_stage_finishing);
  base::cleanup_pipeline();

  //_roi_selection.reset();

  if ( true ) {
    //_frame_registration.reset();
    //_flow_accumulation.reset();
  }

  //set_pipeline_stage(stacking_stage_idle);
}

bool c_jdr_pipeline::run_pipeline()
{
  CF_DEBUG("ENTER");

  if ( !create_reeference_frame() ) {
    CF_ERROR("create_reeference_frame() fails");
    return false;
  }


  CF_DEBUG("LEAVE");
  return true;
}



#if 0
static int select_master_frame(const c_input_sequence::sptr & input_sequence, const c_master_frame_selection_options & opts,
    const std::function<bool()> & canceled )
{
  INSTRUMENT_REGION("");

  int selected_master_frame_index = 0;

//  _selected_master_frame.release();
//  _selected_master_frame_mask.release();

  switch (opts.master_selection_method) {

    case master_frame_specific_index:
      selected_master_frame_index = opts.master_frame_index;
      break;

    case master_frame_middle_index:
      selected_master_frame_index = input_sequence->size() / 2;
      break;

    case master_frame_best_of_100_in_middle: {

      c_laplacian_sharpness_measure measure(2, cv::Size(5, 5));

      constexpr int max_frames_to_scan = 2000;

      CF_DEBUG("Scan %d frames around of middle %d",
          max_frames_to_scan, input_sequence->size() / 2);

      int start_pos, end_pos, backup_current_pos;

      if( input_sequence->size() <= max_frames_to_scan ) {
        start_pos = 0;
        end_pos = input_sequence->size();
      }
      else {
        start_pos = input_sequence->size() / 2 - max_frames_to_scan / 2;
        end_pos = std::min(input_sequence->size(), start_pos + max_frames_to_scan / 2);
      }

      //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
      input_sequence->set_auto_apply_color_matrix(false);

      backup_current_pos = input_sequence->current_pos();
      input_sequence->seek(start_pos);

      cv::Mat image, mask, dogs;
      int current_index, best_index = 0;
      double current_metric, best_metric = 0;

      int total_frames = end_pos - start_pos;
      int processed_frames = 0;
      int accumulated_frames = 0;

      on_frame_processed();

      for( current_index = 0; processed_frames < total_frames;
          processed_frames = ++current_index, on_frame_processed() ) {

        if ( canceled && canceled() ) {
          CF_DEBUG("cancel requested");
          return -1;
        }

        if( input_sequence->is_bad_frame_index(input_sequence->current_pos()) ) {
          CF_DEBUG("Skip frame %d as blacklisted", input_sequence->current_pos());
          input_sequence->seek(input_sequence->current_pos() + 1);
          continue;
        }

        if( !read_input_frame(input_sequence, image, mask, false, false) ) {
          CF_ERROR("read_input_frame() fails");
          return false;
        }

        current_metric =
            measure.compute(image,
                mask)[0];

        if( current_metric > best_metric ) {

          best_metric = current_metric;
          best_index = current_index;

          if( true ) {
            lock_guard lock(mutex());
            image.copyTo(_selected_master_frame);
            mask.copyTo(_selected_master_frame_mask);
          }

          set_status_msg(ssprintf("SELECT REFERENCE FRAME...\n"
              "BEST: INDEX=%d METRIC: %g",
              best_index + start_pos,
              best_metric));
        }
      }

      selected_master_frame_index = best_index + start_pos;
      input_sequence->seek(backup_current_pos);

      break;
    }
  }

  return selected_master_frame_index;
}
#endif


bool c_jdr_pipeline::create_reeference_frame()
{
  if( !_reference_frame_options.generate_reference_frame ) {
    CF_DEBUG("NOT CREATIMG REFERENCE FRAME");
  }
  else {
    CF_DEBUG("CREATIMG REFERENCE FRAME");

    int master_source_index = -1;
    int master_frame_index = -1;
    bool is_external_master_file = false;
    std::string master_filename;

    c_input_sequence::sptr master_sequence =
        select_master_source(_master_options.master_selection,
            _input_sequence,
            &master_source_index);

    if( !master_sequence ) {
      CF_ERROR("select_master_source() fails");
      return false;
    }

    is_external_master_file = _input_sequence != master_sequence;

    CF_DEBUG("master_source_index: %d master_frame_index: %d is_external_master_file=%d",
        master_source_index,
        master_frame_index,
        is_external_master_file);

    if ( !master_sequence->source(master_source_index)->enabled() ) {
      CF_FATAL("ERROR: master_source_index=%d is NOT enabled in input_sequence",
          master_source_index);
      return false;
    }

    if ( canceled() ) {
      return false;
    }

    master_filename = master_sequence->source(master_source_index)->filename();
    if ( !master_sequence->is_open() && !master_sequence->open() ) {
      CF_FATAL("ERROR: Can not open master input source '%s'",
          master_filename.c_str());
      return false;
    }

    set_pipeline_stage(stacking_stage_select_master_frame_index);

    master_frame_index = base::select_master_frame(master_sequence, _input_options, _master_options.master_selection);

    if ( canceled() ) {
      return false;
    }

    if ( master_frame_index <  0 ) {
      CF_ERROR("select_master_frame() fails");
      return false;
    }

    CF_DEBUG("master_frame_index=%d", master_frame_index);
  }

  return true;
}
