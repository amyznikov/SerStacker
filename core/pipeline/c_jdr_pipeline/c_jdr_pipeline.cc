/*
 * c_jdr_pipeline.cc
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#include "c_jdr_pipeline.h"

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
  _reference_frame_options.master_selection.input_sequence = input_sequence.get();
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
        [&, ctx = ctx(&this_class::_input_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_input_options>(ctx));
        });

    ctlbind_expandable_group(ctls, "2. ROI Selection",
        [&, ctx = ctx(&this_class::_roi_selection_options)]() {
          ctlbind(ctls, ctx);
        });

    ctlbind_expandable_group(ctls, "3. Reference Frame Options ",
        [&, ctx = ctx(&this_class::_reference_frame_options)]() {

          ctlbind_browse_for_file(ctls, "Reference file name", CTL_CONTEXT(ctx, reference_file_name));
          ctlbind(ctls, "Generate reference frame", CTL_CONTEXT(ctx, generate_reference_frame));

          ctlbind_expandable_group(ctls, "Master Frame Selection",
              [&, ctx = CTL_CONTEXT(ctx, master_selection)]() {
                ctlbind(ctls, ctx);
              });

          ctlbind_expandable_group(ctls, "Reference Frame Generate Options",
              [ctx = CTL_CONTEXT(ctx, generate_opts)]() {
                ctlbind(ctls, "motion_type", CTL_CONTEXT(ctx, motion_type), "");
                ctlbind(ctls, "input_image_preprocessor", CTL_CONTEXT(ctx, input_image_preprocessor), "");
                ctlbind(ctls, "reference_channel", CTL_CONTEXT(ctx, reference_channel), "");
                ctlbind_expandable_group(ctls, "ECCH",
                    [ctx = CTL_CONTEXT(ctx, ecch_opts)]() {
                      ctlbind(ctls, ctx);
                    });
              });
        });

    ctlbind_expandable_group(ctls, "4. Jovian Ellipse Estimation Options",
        [ctx = CTL_CONTEXT(ctx, _ellipse_estimation_options)]() {
          ctlbind(ctls, CTL_CONTEXT(ctx, jovian_ellipse_detector_options));

//          ctlbind_menu_button(ctls, "Options >>", ctx);
//          ctlbind_item(ctls, "Copy parameters to clipboard", ctx, [](const auto * obj) {
//            return ctlbind_copy_config_to_clipboard("c_jovian_ellipse_detector_options",
//                obj->jovian_ellipse_detector_options), false;
//          });
//          ctlbind_item(ctls, "Paste parameters from clipboard", ctx, [](auto * obj) {
//              return ctlbind_paste_config_from_clipboard("c_jovian_ellipse_detector_options",
//                  &obj->jovian_ellipse_detector_options);
//            });
        });

    ctlbind_expandable_group(ctls, "5. Stack Options",
        [&, ctx = ctx(&this_class::_stack_options)]() {
          ctlbind(ctls, "input_image_preprocessor", CTL_CONTEXT(ctx, input_image_preprocessor));
        });

    ctlbind_expandable_group(ctls, "6. Output options",
        [&, ctx = ctx(&this_class::_output_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));

          ctlbind_expandable_group(ctls, "Save Aligned Frames", "");
          ctlbind(ctls, "save_aligned_frames",CTL_CONTEXT(ctx, save_aligned_frames), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_aligned_frames_opts));
          ctlbind_end_group(ctls);

          ctlbind_expandable_group(ctls, "Save Derotated Frames", "");
          ctlbind(ctls, "save_derotated_frames",CTL_CONTEXT(ctx, save_derotated_frames), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_derotated_frames_opts));
          ctlbind_end_group(ctls);

          ctlbind_expandable_group(ctls, "Save accumulation weights", "");
          ctlbind(ctls, "save_accumulation_weights",CTL_CONTEXT(ctx, save_accumulation_weights), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_accumulation_weights_opts));
          ctlbind_end_group(ctls);
        });
  }

  return ctls;
}

bool c_jdr_pipeline::serialize(c_config_setting settings, bool save)
{
  if ( !base::serialize(settings, save) ) {
    CF_ERROR("base::serialize(save=%d) fails", save);
    return false;
  }

  if( auto input_opts = SERIALIZE_GROUP(settings, save, "input_opts") ) {
    serialize_base_input_options(input_opts, save, _input_options);
  }

  if( auto roi_opts = SERIALIZE_GROUP(settings, save, "roi_opts") ) {
    serialize_base_roi_selection_options(roi_opts, save, _roi_selection_options);
  }

  if( auto reference_frame_opts = SERIALIZE_GROUP(settings, save, "reference_frame_opts") ) {
    auto & opts = _reference_frame_options;

    if( auto master_selection_opts = SERIALIZE_GROUP(reference_frame_opts, save, "master_selection") ) {
      serialize_base_master_frame_selection_options(master_selection_opts, save, opts.master_selection);
    }

    SERIALIZE_OPTION(reference_frame_opts, save, opts, reference_file_name);
    SERIALIZE_OPTION(reference_frame_opts, save, opts, generate_reference_frame);

    if( auto generate_opts = SERIALIZE_GROUP(reference_frame_opts, save, "generate_opts") ) {
      SERIALIZE_OPTION(generate_opts, save, opts.generate_opts, reference_channel);
      SERIALIZE_OPTION(generate_opts, save, opts.generate_opts, motion_type);
      SERIALIZE_OPTION(generate_opts, save, opts.generate_opts, input_image_preprocessor);
      if( auto ecch_opts = SERIALIZE_GROUP(generate_opts, save, "ecch") ) {
        serialize_ecch_options(ecch_opts, save, opts.generate_opts.ecch_opts);
      }
    }
  }

  if( auto ellipse_opts = SERIALIZE_GROUP(settings, save, "ellipse_opts") ) {
    if( auto jovian_ellipse_detector_opts = SERIALIZE_GROUP(ellipse_opts, save, "jovian_ellipse_detector") ) {
      serialize_base_jovian_ellipse_detector_options(jovian_ellipse_detector_opts, save,
          _ellipse_estimation_options.jovian_ellipse_detector_options);
    }
  }


  if( auto stack_opts = SERIALIZE_GROUP(settings, save, "stack_opts") ) {
    SERIALIZE_OPTION(stack_opts, save, _stack_options, input_image_preprocessor);
  }

  if( auto output_opts = SERIALIZE_GROUP(settings, save, "output_opts") ) {
    serialize_base_output_options(output_opts, save, _output_options);

    SERIALIZE_OPTION(output_opts, save, _output_options, save_aligned_frames);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_aligned_frames_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_aligned_frames_opts);
    }

    SERIALIZE_OPTION(output_opts, save, _output_options, save_derotated_frames);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_derotated_frames_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_derotated_frames_opts);
    }

    SERIALIZE_OPTION(output_opts, save, _output_options, save_accumulation_weights);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_accumulation_weights_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_accumulation_weights_opts);
    }

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

  const std::string backup_master_source_fiename = p->_reference_frame_options.master_selection.master_fiename;
  const int backup_master_frame_index = p->_reference_frame_options.master_selection.master_frame_index;

  p->_input_options = this->_input_options;
  p->_roi_selection_options = this->_roi_selection_options;
  p->_reference_frame_options = this->_reference_frame_options;
  p->_ellipse_estimation_options = this->_ellipse_estimation_options;
  p->_stack_options = this->_stack_options ;
  p->_output_options = this->_output_options;

  p->_reference_frame_options.master_selection.master_fiename = backup_master_source_fiename;
  p->_reference_frame_options.master_selection.master_frame_index = backup_master_frame_index;

  return true;

}

bool c_jdr_pipeline::has_master_frame() const
{
  return true;
}

void c_jdr_pipeline::set_master_source(const std::string & master_source_path)
{
  _reference_frame_options.master_selection.master_fiename = master_source_path;
}

std::string c_jdr_pipeline::master_source() const
{
  return _reference_frame_options.master_selection.master_fiename;
}

void c_jdr_pipeline::set_master_frame_index(int v)
{
  _reference_frame_options.master_selection.master_selection_method = master_frame_specific_index;
  _reference_frame_options.master_selection.master_frame_index = v;
}

int c_jdr_pipeline::master_frame_index() const
{
  return _reference_frame_options.master_selection.master_frame_index;
}

bool c_jdr_pipeline::get_display_image(cv::OutputArray outputImage, cv::OutputArray outputMask)
{
  switch (_pipeline_stage) {
    case stacking_stage_idle:
      if ( outputImage.needed() ) {
      }
      if ( outputMask.needed() ) {
      }
      break;

    case stacking_stage_select_master_frame_index: {
      CF_DEBUG("stacking_stage_select_master_frame_index");
      if ( outputImage.needed() ) {
        _current_master_frame_candidate.copyTo(outputImage);
      }
      if ( outputMask.needed() ) {
        _current_master_frame_candidate_mask.copyTo(outputMask);
      }
      return true;
    }

    case stacking_stage_generate_reference_frame: {
      CF_DEBUG("stacking_stage_generate_reference_frame : accumulated_frames=%d ", _reference_frame_avg.accumulated_frames());
      return _reference_frame_avg.compute(outputImage, outputMask);
    }

    case stacking_stage_in_progress: {
      bool draw_ellipsoid = true;
      if( !draw_ellipsoid ) {
        _current_aligned_frame.copyTo(outputImage);
        _current_aligned_mask.copyTo(outputMask);
      }
      else {

        cv::Mat display;

        double minv, maxv;
        cv::minMaxLoc(_current_aligned_frame, &minv, &maxv);
        if( !(maxv > minv) ) {
          maxv = 255;
        }

        _current_aligned_frame.copyTo(display);
        draw_ellipoid(display,
            _jovian_ellipse_detector.center(),
            _jovian_ellipse_detector.axes(),
            build_ellipsoid_rotation(_jovian_ellipse_detector.pose()) ,
            30 * CV_PI / 180,
            30 * CV_PI / 180,
            cv::Scalar::all(maxv * 1.01),
            1,
            cv::LINE_8);

        display.copyTo(outputImage);
        _current_aligned_mask.copyTo(outputMask);
      }

      return true;
    }
    default:
      CF_DEBUG("BAD _pipeline_stage=%d", _pipeline_stage);
      break;
  }

  return false;
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
  _frame_average.clear();

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

bool c_jdr_pipeline::open_output_writers()
{
  if( _output_options.save_aligned_frames ) {

    const bool fOK =
        add_output_writer(_current_aligned_frame_writer,
            _output_options.save_aligned_frames_opts,
            "aligned", ".avi");
    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          _current_aligned_frame_writer.cfilename());
      return false;
    }
  }


  if( _output_options.save_derotated_frames ) {

    const bool fOK =
        add_output_writer(_current_derotated_frame_writer,
            _output_options.save_derotated_frames_opts,
            "derotated", ".avi");
    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          _current_derotated_frame_writer.cfilename());
      return false;
    }
  }

  if( _output_options.save_accumulation_weights ) {

    const bool fOK =
        add_output_writer(_accumulation_weights_writer,
            _output_options.save_accumulation_weights_opts,
            "accweights", ".ser");
    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          _accumulation_weights_writer.cfilename());
      return false;
    }
  }

  return true;
}

bool c_jdr_pipeline::run_pipeline()
{
  CF_DEBUG("ENTER");

  if ( !create_reference_frame() ) {
    CF_ERROR("create_reference_frame() fails");
    return false;
  }

  if ( !estimate_jovian_ellipse() ) {
    CF_ERROR("estimate_jovian_ellipse() fails");
    return false;
  }

  if ( _stack_options.derotate_all_frames ) {
  }

  if ( !derotate_jovian_frames() ) {
    CF_ERROR("derotate_jovian_frames() fails");
    return false;
  }

  CF_DEBUG("LEAVE");
  return true;
}

bool c_jdr_pipeline::create_reference_frame()
{
  int master_source_index = -1;
  int master_frame_index = -1;
  bool is_external_master_file = false;
  std::string master_filename;

  c_input_sequence::sptr master_sequence =
      select_master_source(_reference_frame_options.master_selection,
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

  master_frame_index =
      select_master_frame(master_sequence, master_source_index, _input_options,
          _reference_frame_options.master_selection);

  if ( canceled() ) {
    return false;
  }

  if ( master_frame_index < 0 ) {
    CF_ERROR("select_master_frame() fails");
    return false;
  }

  CF_DEBUG("master_frame_index=%d", master_frame_index);

  if ( !master_sequence->seek(master_frame_index) ) {
    CF_ERROR("master_sequence->seek(master_frame_index=%d) fails", master_frame_index);
    return false;
  }

  if ( !read_input_frame(master_sequence, _input_options, _master_frame, _master_mask, is_external_master_file, false) ) {
    CF_ERROR("read_input_frame() fails");
    return false;
  }

  _master_ts = master_sequence->has_last_ts() ? master_sequence->last_ts() : 0;
  CF_DEBUG("_master_ts=%lf [ms]", _master_ts);

  CF_DEBUG("master_frame: %dx%d channels=%d depth=%d",
      _master_frame.cols, _master_frame.rows,
      _master_frame.channels(), _master_frame.depth());

  CF_DEBUG("master_mask : %dx%d channels=%d depth=%d",
      _master_mask.cols, _master_mask.rows,
      _master_mask.channels(), _master_mask.depth());


  if( !_reference_frame_options.generate_reference_frame ) {
    CF_DEBUG("NOT GENERATING REFERENCE FRAME");

    const std::string reference_file_name =
        generate_output_filename(_reference_frame_options.reference_file_name,
            "_reference",
            ".tiff");
    if( !load_image(reference_file_name, _reference_frame, _reference_mask) ) {
      CF_ERROR("load_image('%s') fails\n"
          "Check if file exists or (Re)Generate new reference frame",
          reference_file_name.c_str());
      return false;
    }

    CF_DEBUG("reference frame: %s\n"
        "image: %dx%d channels=%d depth=%d\n"
        "mask : %dx%d channels=%d depth=%d",
        reference_file_name.c_str(),
        _reference_frame.cols, _reference_frame.rows, _reference_frame.channels(), _reference_frame.depth(),
        _reference_mask.cols, _reference_mask.rows, _reference_mask.channels(), _reference_mask.depth());
  }
  else {
    CF_DEBUG("GENERATING REFERENCE FRAME");

    synchronized([this]() {_reference_frame_avg.clear();});
    set_pipeline_stage(stacking_stage_generate_reference_frame);


    const int master_sequence_size = master_sequence->size();
    const int max_input_frames = _input_options.max_input_frames < 0 ? master_sequence_size : std::clamp(_input_options.max_input_frames, 0, master_sequence_size);
    const int start_frame_index = std::clamp(_input_options.start_frame_index, 0, master_sequence_size - 1);
    const int end_frame_index = std::min(start_frame_index + max_input_frames, max_input_frames);

    CF_DEBUG("start_frame_index=%d end_frame_index=%d / %d", start_frame_index, end_frame_index, master_sequence_size);

    if ( !master_sequence->seek(start_frame_index) ) {
      CF_ERROR("master_sequence->seek(start_frame_index=%d) fails", start_frame_index);
      return false;
    }

    const c_image_transform::sptr transform =
        create_image_transform(_reference_frame_options.generate_opts.motion_type);
    if( !transform ) {
      CF_ERROR("create_image_transform(motion_type = %d (%s) ) fails",
          (int )(_reference_frame_options.generate_opts.motion_type),
          toCString(_reference_frame_options.generate_opts.motion_type));
      return false;
    }

    const color_channel_type & reference_channel =
        _reference_frame_options.generate_opts.reference_channel;

    cv::Mat current_frame, current_grayscale_frame;
    cv::Mat current_mask;
    cv::Mat2f rmap;

    c_ecch ecch(transform.get(), _reference_frame_options.generate_opts.ecch_opts);

    _master_frame.copyTo(current_frame);
    _master_mask.copyTo(current_mask);
    if( const auto & proc = _reference_frame_options.generate_opts.input_image_preprocessor ) {
      if( !proc->process(current_frame, current_mask) ) {
        CF_ERROR("input_image_preprocessor->process(master_frame, master_mask) fails");
        return false;
      }
    }

    const std::string output_reference_master_frame_file_name = generate_output_filename("reference_master_frame", "", ".tiff");
    if( !save_image(current_frame, current_mask, output_reference_master_frame_file_name) ) {
      CF_ERROR("save_image(%s) fails", output_reference_master_frame_file_name.c_str());
      return false;
    }
    CF_ERROR("SAVED reference_master_frame=%s", output_reference_master_frame_file_name.c_str());


    if ( current_frame.channels() == 1 ) {
      current_grayscale_frame = current_frame;
    }
    else if ( !extract_channel(current_frame, current_grayscale_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
      CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
      return false;
    }


    if ( !ecch.set_reference_image(current_grayscale_frame, current_mask) ) {
      CF_ERROR("ecch.set_reference_image() fails");
      return false;
    }

    _processed_frames = 0;
    _accumulated_frames = 0;
    _total_frames = end_frame_index - start_frame_index;
    current_grayscale_frame.release();

    for ( int i = start_frame_index; i < end_frame_index; ++i, ++_processed_frames, on_frame_processed() ) {

      if( is_bad_frame_index(master_sequence->current_pos()) ) {
        CF_DEBUG("Skip frame %d as blacklisted", master_sequence->current_pos());
        master_sequence->seek(master_sequence->current_pos() + 1);
        continue;
      }

      if ( !read_input_frame(master_sequence, _input_options, current_frame, current_mask, is_external_master_file, false) ) {
        CF_ERROR("read_input_frame(master_sequence->current_pos()=%d) fails", master_sequence->current_pos());
        return false;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      CF_DEBUG("[F %d] frame: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d", i,
          current_frame.cols, current_frame.rows, current_frame.channels(), current_frame.depth(),
          current_mask.cols, current_mask.rows, current_mask.channels(), current_mask.depth());

      if( const auto & proc = _reference_frame_options.generate_opts.input_image_preprocessor ) {
        if( !proc->process(current_frame, current_mask) ) {
          CF_ERROR("input_image_preprocessor->process(current_frame, current_mask) fails");
          return false;
        }
        CF_DEBUG("[F %d] PREPROCESSED", i);
      }

      if ( current_frame.channels() == 1 ) {
        current_grayscale_frame = current_frame;
      }
      else if ( !extract_channel(current_frame, current_grayscale_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
        CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
        return false;
      }

      if( !ecch.align(current_grayscale_frame, current_mask) ) {
        CF_ERROR("[F %d] ecch.align() fails", i);
        return false;
      }

      CF_DEBUG("[F %d] ALIGNED", i);

      if ( !ecch.create_remap(rmap) ) {
        CF_ERROR("[F %d] ecch.create_remap() fails", i);
        return false;
      }

      cv::remap(current_frame, current_frame,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REPLICATE);
      cv::remap(current_mask.empty() ? cv::Mat1b(current_frame.size(), uint8_t(255)) : current_mask, current_mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
      cv::compare(current_mask, 250, current_mask,
          cv::CMP_GE);

      CF_DEBUG("[F %d] REMAPPED mask nnz = %d ", i, cv::countNonZero(current_mask));

      if( true ) {
        lock_guard lock(mutex());
        if( !_reference_frame_avg.add(current_frame, current_mask) ) {
          CF_ERROR("[F %d]  _reference_frame_avg.add() fails", i);
          return false;
        }
        ++_accumulated_frames;
      }

      CF_DEBUG("[F %d] ACCUMULATED", i);
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !_reference_frame_avg.compute(current_frame, current_mask) ) {
        CF_ERROR("_reference_frame_avg.compute() fails");
        return false;
      }
    }

    const std::string output_file_name =
        generate_output_filename(_reference_frame_options.reference_file_name,
            "_reference",
            ".tiff");

    if ( !save_image(current_frame, current_mask, output_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_file_name.c_str());
      return false;
    }


    _reference_frame = std::move(current_frame);
    _reference_mask = std::move(current_mask);

    CF_DEBUG("reference frame: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d",
        _reference_frame.cols, _reference_frame.rows, _reference_frame.channels(), _reference_frame.depth(),
        _reference_mask.cols, _reference_mask.rows, _reference_mask.channels(), _reference_mask.depth());

    CF_DEBUG("Saved as %s", output_file_name.c_str());
  }

  return true;
}

bool c_jdr_pipeline::estimate_jovian_ellipse()
{
  if ( _reference_frame.empty() ) {
    CF_ERROR("APP BUG: No jovian reference frame available");
    return false;
  }

  _jovian_ellipse_detector.set_options(_ellipse_estimation_options.jovian_ellipse_detector_options);
  if( !_jovian_ellipse_detector.detect_jovian_ellipse(_reference_frame, _reference_mask) ) {
    CF_ERROR("_jovian_ellipse_detector.detect_jovian_ellipse() fails");
    return false;
  }

  _jovian_ellipse_detector.disk_mask().copyTo(_reference_planetary_disk_mask);
  if ( true ) {
    const std::string output_planetary_disk_mask_file_name = generate_output_filename("reference_planetary_disk_mask", "", ".png");
    if( !save_image(_reference_planetary_disk_mask, cv::noArray(), output_planetary_disk_mask_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_planetary_disk_mask_file_name.c_str());
      return false;
    }
    CF_DEBUG("Saved %s", output_planetary_disk_mask_file_name.c_str());
  }


  _jovian_derotation_remap.set_reference_pose(_reference_frame.size(),
      _jovian_ellipse_detector.center(),
      _jovian_ellipse_detector.axes(),
      _jovian_ellipse_detector.pose());


  if ( true ) {
    // Create ellipse 2D illustration image
    cv::Mat display;
    double minv = 0, maxv = 1;
    cv::minMaxLoc(_reference_frame, &minv, &maxv, nullptr, nullptr);

    if ( _reference_frame.channels() == 3 ) {
      _reference_frame.convertTo(display, CV_8UC3, 255./maxv);
    }
    else {
      cv::cvtColor(_reference_frame, display, cv::COLOR_GRAY2BGR);
      display.convertTo(display, CV_8UC3, 255./maxv);
    }

    static const auto drawRotatedRect =
        [](cv::InputOutputArray image, const cv::RotatedRect & rc,
        const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0)
    {
      cv::rectangle(image, ellipse_bounding_box(rc), cv::Scalar(0, 0, 200), 1);
      cv::Point2f pts[4];
      rc.points(pts);
      for( int i = 0; i < 4; i++ ) {
        cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
      }
      cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
      cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
    };


    drawRotatedRect(display, _jovian_ellipse_detector.final_planetary_disk_ellipse(), CV_RGB(0, 255, 0), 1);
    display.setTo(cv::Scalar::all(255), _jovian_ellipse_detector.disk_edge());
    cv::ellipse(display, _jovian_ellipse_detector.final_planetary_disk_ellipse(), CV_RGB(0, 0, 255), 1);

    const std::string output_display_file_name =
        generate_output_filename("jovian_ellipse_fit", "", ".png");
    if( !save_image(display, cv::noArray(), output_display_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_display_file_name.c_str());
      return false;
    }
    CF_DEBUG("Saved %s", output_display_file_name.c_str());
  }

  if ( true ) {
    // Create ellipsoid 3D illustration image
    cv::Mat display, mask;
    double minv = 0, maxv = 1;

    _master_frame.copyTo(display);
    _master_mask.copyTo(mask);

    if( const auto & preproc = _stack_options.input_image_preprocessor ) {
      preproc->process(display, mask);
    }

    cv::minMaxLoc(display, &minv, &maxv, nullptr, nullptr);

    if ( display.channels() == 3 ) {
      display.convertTo(display, CV_8UC3, 255./maxv);
    }
    else {
      cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
      display.convertTo(display, CV_8UC3, 255./maxv);
    }

    draw_ellipoid(display,
        _jovian_derotation_remap.center(),
        _jovian_derotation_remap.axes(),
        _jovian_derotation_remap.Rtarget(),
        30 * CV_PI / 180,
        30 * CV_PI / 180,
        cv::Scalar::all(255),
        1,
        cv::LINE_AA);

    const std::string output_display_file_name = generate_output_filename("jovian_ellipsoid_fit", "", ".png");
    if( !save_image(display, cv::noArray(), output_display_file_name) ) {
      CF_ERROR("save_image('%s') fails", output_display_file_name.c_str());
      return false;
    }
    CF_DEBUG("Saved %s", output_display_file_name.c_str());
  }

  return true;
}

bool c_jdr_pipeline::derotate_jovian_frames()
{
  const int input_sequence_size = _input_sequence->size();
  const int max_input_frames = _input_options.max_input_frames < 0 ? input_sequence_size : std::clamp(_input_options.max_input_frames, 0, input_sequence_size);
  const int start_frame_index = std::clamp(_input_options.start_frame_index, 0, input_sequence_size - 1);
  const int end_frame_index = std::min(start_frame_index + max_input_frames, max_input_frames);

  CF_DEBUG("start_frame_index=%d end_frame_index=%d / %d", start_frame_index, end_frame_index, input_sequence_size);

  if ( !_input_sequence->seek(start_frame_index) ) {
    CF_ERROR("_input_sequence->seek(start_frame_index=%d) fails", start_frame_index);
    return false;
  }

  static const auto preproc_align_and_remap =
      [](const c_image_processor::sptr & proc, c_ecch & ecch,
          cv::Mat & current_frame, cv::Mat & current_mask,
          color_channel_type reference_channel) -> bool {

      cv::Mat grayscale_frame;
      cv::Mat2f rmap;

      if ( proc && !proc->process(current_frame, current_mask)) {
        CF_ERROR("proc->process(current_frame, current_mask) fails");
        return false;
      }

      if ( current_frame.channels() == 1 ) {
        grayscale_frame = current_frame;
      }
      else if ( !extract_channel(current_frame, grayscale_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
        CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
        return false;
      }

      if ( !ecch.align(grayscale_frame, current_mask) ) {
        CF_ERROR("ecch.align() fails");
        return false;
      }

      if ( !ecch.create_remap(rmap) ) {
        CF_ERROR("ecch.create_remap() fails");
        return false;
      }

      cv::remap(current_frame, current_frame,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_REPLICATE);

      cv::remap(current_mask, current_mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);

      cv::compare(current_mask, 250, current_mask,
          cv::CMP_GE);

      return true;
    };


  const color_channel_type & reference_channel = color_channel_gray;

  const c_image_transform::sptr transform =
      create_image_transform(_reference_frame_options.generate_opts.motion_type);
  if( !transform ) {
    CF_ERROR("create_image_transform(motion_type = %d (%s) ) fails",
        (int )(_reference_frame_options.generate_opts.motion_type),
        toCString(_reference_frame_options.generate_opts.motion_type));
    return false;
  }

  set_pipeline_stage(stacking_stage_in_progress);

  const c_image_processor::sptr preproc = _stack_options.input_image_preprocessor;

  c_ecch ecch(transform.get(), _reference_frame_options.generate_opts.ecch_opts);

  cv::Mat current_frame;
  cv::Mat grayscale_frame;
  cv::Mat current_mask;
  cv::Mat2f rmap;

  if ( _reference_frame.channels() == 1 ) {
    grayscale_frame = _reference_frame;
  }
  else if ( !extract_channel(_reference_frame, grayscale_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
    CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
    return false;
  }
  if ( !ecch.set_reference_image(grayscale_frame, _reference_mask) ) {
    CF_ERROR("ecch.set_reference_image() fails");
    return false;
  }

  _master_frame.copyTo(current_frame);
  _master_mask.copyTo(current_mask);
  if( !preproc_align_and_remap(preproc, ecch, current_frame, current_mask, reference_channel) ) {
    CF_ERROR("preproc_align_and_remap(_master_frame) fails");
    return false;
  }
  const std::string output_master_frame_file_name = generate_output_filename("master_frame", "", ".tiff");
  if( !save_image(current_frame, current_mask, output_master_frame_file_name) ) {
    CF_ERROR("save_image(%s) fails", output_master_frame_file_name.c_str());
    return false;
  }
  CF_ERROR("SAVED reference_master_frame=%s", output_master_frame_file_name.c_str());

  _processed_frames = 0;
  _accumulated_frames = 0;
  _total_frames = end_frame_index - start_frame_index;

  if ( !open_output_writers() ) {
    CF_ERROR("open_output_writers() fails");
    return false;
  }

  for ( int i = start_frame_index; i < end_frame_index; ++i, ++_processed_frames, on_frame_processed() ) {

    if( is_bad_frame_index(_input_sequence->current_pos()) ) {
      CF_DEBUG("Skip frame %d as blacklisted", _input_sequence->current_pos());
      _input_sequence->seek(_input_sequence->current_pos() + 1);
      continue;
    }

    if ( !read_input_frame(_input_sequence, _input_options, current_frame, current_mask, false, false) ) {
      CF_ERROR("read_input_frame(_input_sequence->current_pos()=%d) fails", _input_sequence->current_pos());
      return false;
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    CF_DEBUG("[F %d] frame: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d", i,
        current_frame.cols, current_frame.rows, current_frame.channels(), current_frame.depth(),
        current_mask.cols, current_mask.rows, current_mask.channels(), current_mask.depth());

    if( !preproc_align_and_remap(preproc, ecch, current_frame, current_mask, reference_channel) ) {
      CF_ERROR("[F %d] preproc_align_and_remap(current_frame) fails", i);
      return false;
    }

    CF_DEBUG("[F %d] PREPROCESSED", i);

    if ( _current_aligned_frame_writer.is_open() ) {
      if ( !_current_aligned_frame_writer.write(current_frame, current_mask) ) {
        CF_ERROR("[F %d] _current_aligned_frame_writer.write() fails for %s", i, _current_aligned_frame_writer.cfilename());
        return false;
      }
    }

    if( _input_sequence->has_last_ts() ) {

      const double ts = _input_sequence->last_ts();
      const double dt = ts - _master_ts;
      CF_DEBUG("[F %d] ts = %lf [ms] dt = %lf [ms] ", i, ts, dt);

      _jovian_derotation_remap.compute_derotation_for_time(-dt, 1. / (1. + std::abs(dt) / 1200000.));

      cv::remap(current_frame, current_frame,
          _jovian_derotation_remap.rmap(), cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_TRANSPARENT);

      if ( _current_derotated_frame_writer.is_open() ) {
        if ( !_current_derotated_frame_writer.write(current_frame, current_mask) ) {
          CF_ERROR("[F %d] _current_derotated_frame_writer.write() fails for %s", i, _current_derotated_frame_writer.cfilename());
          return false;
        }
      }

      CF_DEBUG("[F %d] DEROTATED", i);

      cv::Mat1f current_weights;
      _jovian_derotation_remap.wmap().copyTo(current_weights);
      current_weights.setTo(1,~ _jovian_derotation_remap.rmask());
      cv::medianBlur(current_weights, current_weights, 3);

      if ( !current_mask.empty() ) {
        current_weights.setTo(0, ~current_mask);
      }
      _frame_average.add(current_frame, current_weights);
      CF_DEBUG("[F %d] ACCUMULATED", i);

      if ( _accumulation_weights_writer.is_open() ) {
        if ( !_accumulation_weights_writer.write(current_weights) ) {
          CF_ERROR("[F %d] _accumulation_weights_writer.write() fails for %s", i, _accumulation_weights_writer.cfilename());
          return false;
        }
      }
    }

    synchronized([&]() {
      current_frame.copyTo(_current_aligned_frame);
      current_mask.copyTo(_current_aligned_mask);
    });

    ++_accumulated_frames;
  }

  if ( _frame_average.accumulated_frames() > 0 )  {

    cv::Mat avg, mask;

    if ( !_frame_average.compute(avg, mask) ) {
      CF_ERROR("_frame_average.compute() fails");
      return false;
    }

    const std::string output_avg_file_name = generate_output_filename("averaged", "", ".tiff");
    if( !save_image(avg, mask, output_avg_file_name) ) {
      CF_ERROR("save_image(%s) fails", output_avg_file_name.c_str());
      return false;
    }

    CF_ERROR("SAVED output_avg_file_name=%s", output_avg_file_name.c_str());
  }

  return true;
}

