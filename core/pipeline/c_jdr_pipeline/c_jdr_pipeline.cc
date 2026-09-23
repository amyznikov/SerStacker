/*
 * c_jdr_pipeline.cc
 *
 *  Created on: Mar 17, 2026
 *      Author: amyznikov
 */

#include "c_jdr_pipeline.h"
#include <core/proc/divide.h>
#include <core/proc/multiply.h>

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

namespace {

//static void makeFFImage(cv::InputArray _src, cv::OutputArray _dst, int maxLvl = 3, double alpha = 1e-2)
//{
//  const cv::Size srcSize = _src.size();
//  cv::Mat src;
//
//  if ( _src.channels() == 1 ) {
//    src = _src.getMat();
//  }
//  else {
//    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
//  }
//
//  cv::Mat blured;
//
//  for ( int i = 0; i < maxLvl; ++i ) {
//    cv::medianBlur(i == 0 ? src : blured, blured, 5);
//    cv::pyrDown(blured, blured);
//  }
//
//  // Don't use cv::resize() as it will always create crucial interpolation artifacts !
//  for ( int i = 0; i < maxLvl; ++i ) {
//    cv::pyrUp(blured, blured);
//  }
//
//  cv::add(blured, alpha, _dst);
//}

static void makeFFImage(cv::InputArray _src, cv::OutputArray _dst, int maxLvl = 3, double alpha = 5e-2)
{
  const cv::Size srcSize = _src.size();
  cv::Mat src;
  cv::Mat blured;
  cv::Mat num, den;
  double minv, maxv;

  if ( _src.channels() == 1 ) {
    src = _src.getMat();
  }
  else {
    cv::cvtColor(_src, src, cv::COLOR_BGR2GRAY);
  }

  for ( int i = 0; i < maxLvl; ++i ) {
    cv::medianBlur(i == 0 ? src : blured, blured, 5);
    cv::pyrDown(blured, blured);
  }

  // blured = (blured^2 + alpha * max_val^2) / (blured + reg)

  cv::minMaxLoc(blured, &minv, &maxv);
  cv::add(blured.mul(blured), alpha * maxv * maxv, num);
  cv::add(blured, alpha * maxv, den);
  cv::divide(num, den, blured);

  for ( int i = 0; i < maxLvl; ++i ) {
    cv::pyrUp(blured, blured);
  }

}

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

//    ctlbind_expandable_group(ctls, "2. ROI Selection",
//        [&, ctx = ctx(&this_class::_roi_selection_options)]() {
//          ctlbind(ctls, ctx);
//        });
//
    ctlbind_expandable_group(ctls, "3. Reference Frame Options ",
        [&, ctx = ctx(&this_class::_reference_frame_options)]() {

          ctlbind_browse_for_existing_file(ctls, "Reference file name", CTL_CONTEXT(ctx, reference_file_name));
          ctlbind(ctls, "Generate reference frame", CTL_CONTEXT(ctx, generate_reference_frame));

          ctlbind_expandable_group(ctls, "Master Frame Selection",
              [&, ctx = CTL_CONTEXT(ctx, master_selection)]() {
                ctlbind(ctls, ctx);
              });

          ctlbind_expandable_group(ctls, "Reference Frame Generate Options",
              [ctx = CTL_CONTEXT(ctx, generate_opts)]() {
                ctlbind(ctls, "motion_type", CTL_CONTEXT(ctx, motion_type), "");
                ctlbind(ctls, "image preprocessor", CTL_CONTEXT(ctx, image_preprocessor), "");
                ctlbind(ctls, "reference_channel", CTL_CONTEXT(ctx, reference_channel), "");
                ctlbind_expandable_group(ctls, "ECCH",
                    [ctx = CTL_CONTEXT(ctx, ecch_opts)]() {
                      ctlbind(ctls, ctx);
                    });
              });
        });

    ctlbind_expandable_group(ctls, "4. Planet Ellipse Estimation Options",
        [&, ctx = CTL_CONTEXT(ctx, _ellipse_estimation_options)]() {
          ctlbind(ctls, "Auto pose estimation", CTL_CONTEXT(ctx, auto_pose_estimation));
          ctlbind(ctls, "Update estimated pose", CTL_CONTEXT(ctx, update_estimated_pose));

          ctlbind_expandable_group(ctls, "Planet pose",
              [&, ctx = CTL_CONTEXT(ctx, pose)]() {
                ctlbind(ctls, "Center [pix]:", CTL_CONTEXT(ctx, center), "Jovian ellipse center in pixels");
                ctlbind(ctls, "Axes [pix]:", CTL_CONTEXT(ctx, axes), "Jovian ellipse axes in pixels");
                ctlbind(ctls, "Orientation [deg]:", CTL_CONTEXT(ctx, orientation), "Jovian ellipsoid orientation in degrees");
              });

          ctlbind_expandable_group(ctls, "Jovian ellipsoid detection and pose estimation",
              [&, ctx]() {
                ctlbind(ctls, CTL_CONTEXT(ctx, ellipse_detector_options));
              });

          ctlbind_menu_button(ctls, "Options >>", ctx);
          ctlbind_item(ctls, "Copy ellipse detector options", ctx, [](const auto * obj) {
            return ctlbind_copy_config_to_clipboard("c_jovian_ellipse_detector_options",
                obj->ellipse_detector_options), false;
          });
          ctlbind_item(ctls, "Paste ellipse detector options", ctx, [](auto * obj) {
              return ctlbind_paste_config_from_clipboard("c_jovian_ellipse_detector_options",
                  &obj->ellipse_detector_options);
            });
          ctlbind_item(ctls, "Copy pose", ctx, [](const auto * obj) {
            if ( const auto & cb = get_ctlbind_copy_to_clipboard_callback() ) {
              cb(serialize_ellipsoid_to_string(obj->pose.center, obj->pose.axes, obj->pose.orientation));
            }
            return false;
          });
          ctlbind_item(ctls, "Paste pose", ctx, [](auto * obj) {
            if ( const auto & cb = get_ctlbind_get_clipboard_text_callback() ) {
              if ( parse_ellipsoid_from_string(cb(), &obj->pose.center, &obj->pose.axes, &obj->pose.orientation) ) {
                return true;
              }
            }
            return false;
          });
        });

    ctlbind_expandable_group(ctls, "5. Stack Options",
        [&, ctx = ctx(&this_class::_stack_options)]() {
          ctlbind(ctls, "enable_image_stacking", CTL_CONTEXT(ctx, enable_image_stacking));
          ctlbind(ctls, "derotate_all_frames", CTL_CONTEXT(ctx, derotate_all_frames));
          ctlbind(ctls, "derotate_context_size", CTL_CONTEXT(ctx, derotate_context_size));
          ctlbind(ctls, "input_image_preprocessor", CTL_CONTEXT(ctx, input_image_preprocessor));
          ctlbind(ctls, "wts [s]", CTL_CONTEXT(ctx, wts), "w = 1. / (1. + std::abs(dt) / wts)");
          ctlbind(ctls, "enable_weighted_average", CTL_CONTEXT(ctx, enable_weighted_average));

          ctlbind_expandable_group(ctls, "LPG Options", [&, ctx = CTL_CONTEXT(ctx,lpg)] () {
            ctlbind(ctls, ctx);
            ctlbind_command_button(ctls, "Copy LPG options", ctx, [](const auto * obj) {
              return ctlbind_copy_config_to_clipboard("c_lpg_options", *obj), false;
            });
            ctlbind_command_button(ctls, "Paste LPG options", ctx, [](auto * obj) {
              return ctlbind_paste_config_from_clipboard("c_lpg_options", obj);
            });
          });

          ctlbind_expandable_group(ctls, "Remap Options", [&, ctx = CTL_CONTEXT(ctx,remap)] () {
                ctlbind(ctls, ctx);
              });
        });

    ctlbind_expandable_group(ctls, "6. Output options",
        [&, ctx = ctx(&this_class::_output_options)]() {
          ctlbind(ctls, as_base<c_image_processing_pipeline_output_options>(ctx));

          ctlbind_expandable_group(ctls, "Save Aligned Frames", "");
          ctlbind(ctls, "save_aligned_frames",CTL_CONTEXT(ctx, save_aligned_frames), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_aligned_frames_opts));
          ctlbind_end_group(ctls);

          ctlbind_expandable_group(ctls, "Save Derotated Avg Frames", "");
          ctlbind(ctls, "save_derotated_avg_frames",CTL_CONTEXT(ctx, save_derotated_avg_frames), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_derotated_avg_frames_opts));
          ctlbind_end_group(ctls);

          ctlbind_expandable_group(ctls, "Save Derotated Avg Weights", "");
          ctlbind(ctls, "save_derotated_avg_weights",CTL_CONTEXT(ctx, save_derotated_avg_weights), "");
          ctlbind(ctls,CTL_CONTEXT(ctx, save_derotated_avg_weights_opts));
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

//  if( auto roi_opts = SERIALIZE_GROUP(settings, save, "roi_opts") ) {
//    serialize_base_roi_selection_options(roi_opts, save, _roi_selection_options);
//  }

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
      SERIALIZE_OPTION(generate_opts, save, opts.generate_opts, image_preprocessor);
      if( auto ecch_opts = SERIALIZE_GROUP(generate_opts, save, "ecch") ) {
        serialize_ecch_options(ecch_opts, save, opts.generate_opts.ecch_opts);
      }
    }
  }

  if( auto ellipse_opts = SERIALIZE_GROUP(settings, save, "ellipse_opts") ) {
    SERIALIZE_OPTION(ellipse_opts, save, _ellipse_estimation_options, auto_pose_estimation);
    SERIALIZE_OPTION(ellipse_opts, save, _ellipse_estimation_options, update_estimated_pose);
    if( auto pose_opts = SERIALIZE_GROUP(ellipse_opts, save, "pose") ) {
      SERIALIZE_OPTION(pose_opts, save, _ellipse_estimation_options.pose, center);
      SERIALIZE_OPTION(pose_opts, save, _ellipse_estimation_options.pose, axes);
      SERIALIZE_OPTION(pose_opts, save, _ellipse_estimation_options.pose, orientation);
    }
    if( auto jovian_ellipse_detector_opts = SERIALIZE_GROUP(ellipse_opts, save, "jovian_ellipse_detector") ) {
      serialize_base_jovian_ellipse_detector_options(jovian_ellipse_detector_opts, save,
          _ellipse_estimation_options.ellipse_detector_options);
    }
  }


  if( auto stack_opts = SERIALIZE_GROUP(settings, save, "stack_opts") ) {
    SERIALIZE_OPTION(stack_opts, save, _stack_options, enable_image_stacking);
    SERIALIZE_OPTION(stack_opts, save, _stack_options, derotate_context_size);
    SERIALIZE_OPTION(stack_opts, save, _stack_options, input_image_preprocessor);
    SERIALIZE_OPTION(stack_opts, save, _stack_options, wts);
    SERIALIZE_OPTION(stack_opts, save, _stack_options, derotate_all_frames);

    SERIALIZE_OPTION(stack_opts, save, _stack_options, enable_weighted_average);
    if( auto lpg_opts = SERIALIZE_GROUP(stack_opts, save, "lpg") ) {
      SERIALIZE_OPTION(lpg_opts , save, _stack_options, lpg);
    }

    if( auto remap_opts = SERIALIZE_GROUP(stack_opts, save, "remap") ) {
      SERIALIZE_OPTION(remap_opts , save, _stack_options.remap, jovian_rotation_period_sec);
    }

  }

  if( auto output_opts = SERIALIZE_GROUP(settings, save, "output_opts") ) {
    serialize_base_output_options(output_opts, save, _output_options);

    SERIALIZE_OPTION(output_opts, save, _output_options, save_aligned_frames);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_aligned_frames_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_aligned_frames_opts);
    }

    SERIALIZE_OPTION(output_opts, save, _output_options, save_derotated_avg_frames);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_derotated_avg_frames_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_derotated_avg_frames_opts);
    }

    SERIALIZE_OPTION(output_opts, save, _output_options, save_derotated_avg_weights);
    if( auto group = SERIALIZE_GROUP(output_opts, save, "save_derotated_avg_weights_opts") ) {
      SERIALIZE_OPTION(group, save, _output_options, save_derotated_avg_weights_opts);
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
  const c_jdr_pipeline_ellipsoid_pose backup_pose = p->_ellipse_estimation_options.pose;

  p->_input_options = this->_input_options;
  // p->_roi_selection_options = this->_roi_selection_options;
  p->_reference_frame_options = this->_reference_frame_options;
  p->_ellipse_estimation_options = this->_ellipse_estimation_options;
  p->_stack_options = this->_stack_options ;
  p->_output_options = this->_output_options;

  p->_reference_frame_options.master_selection.master_fiename = backup_master_source_fiename;
  p->_reference_frame_options.master_selection.master_frame_index = backup_master_frame_index;
  p->_ellipse_estimation_options.pose = backup_pose;

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
      bool enable_draw_ellipsoid = true;
      if( !enable_draw_ellipsoid ) {
        _current_display_frame.copyTo(outputImage);
        _current_display_mask.copyTo(outputMask);
      }
      else {

        cv::Mat display;

        double minv, maxv;
        cv::minMaxLoc(_current_display_frame, &minv, &maxv);
        if( !(maxv > minv) ) {
          maxv = 255;
        }

        if ( _current_display_mask.empty() ) {
          _current_display_frame.copyTo(display);
        }
        else {
          _current_display_frame.copyTo(display,  _current_display_mask > 0 );
        }

        draw_ellipsoid(display,
            _planetary_disk_pose.center,
            _planetary_disk_pose.axes,
            build_ellipsoid_rotation(_planetary_disk_pose.orientation) ,
            30 * CV_PI / 180,
            30 * CV_PI / 180,
            cv::Scalar::all(maxv * 1.01),
            1,
            cv::LINE_8);

        display.copyTo(outputImage);
        _current_display_mask.copyTo(outputMask);
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
  _reference_frame_avg.clear();
  _average.clear();
  _ellipse_detector.clear();
  _ellipsoid_derotation_remap.set_opts(_stack_options.remap);

  return true;
}

void c_jdr_pipeline::cleanup_pipeline()
{
  //set_pipeline_stage(stacking_stage_finishing);
  base::cleanup_pipeline();
  _ellipse_detector.clear();

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
        add_output_writer(_aligned_frames_writer,
            _output_options.save_aligned_frames_opts,
            ".aligned", ".avi");
    if( !fOK ) {
      CF_ERROR("Can not open _aligned_frames_writer '%s'",
          _aligned_frames_writer.cfilename());
      return false;
    }
  }

  if( _output_options.save_derotated_frames ) {

    const bool fOK =
        add_output_writer(_derotated_frames_writer,
            _output_options.save_derotated_frames_opts,
            ".derotated", ".avi");
    if( !fOK ) {
      CF_ERROR("Can not open _derotated_frames_writer '%s'",
          _derotated_frames_writer.cfilename());
      return false;
    }
  }

  if( _output_options.save_accumulation_weights ) {

    const bool fOK =
        add_output_writer(_accumulation_weights_writer,
            _output_options.save_accumulation_weights_opts,
            ".acc_weights", ".ser");
    if( !fOK ) {
      CF_ERROR("Can not open _accumulation_weights_writer '%s'",
          _accumulation_weights_writer.cfilename());
      return false;
    }
  }

  if( _output_options.save_derotated_avg_frames ) {
    const bool fOK =
        add_output_writer(_derotated_avg_frames_writer,
            _output_options.save_derotated_avg_frames_opts,
            ".avg", _stack_options.derotate_all_frames ? ".ser" : ".tiff");
    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          _derotated_avg_frames_writer.cfilename());
      return false;
    }
  }

  if( _output_options.save_derotated_avg_weights ) {
    const bool fOK =
        add_output_writer(_derotated_avg_weights_writer,
            _output_options.save_derotated_avg_weights_opts,
            ".avg_weights", _stack_options.derotate_all_frames ? ".ser" : ".tiff");
    if( !fOK ) {
      CF_ERROR("Can not open _derotated_avg_weights_writer '%s'",
          _derotated_avg_weights_writer.cfilename());
      return false;
    }
  }

  return true;
}

bool c_jdr_pipeline::preprocess(const c_image_processor::sptr & proc,
    cv::Mat & current_frame, cv::Mat & current_mask) const
{
  if( proc && !proc->process(current_frame, current_mask) ) {
    CF_ERROR("proc->process(current_frame, current_mask) fails");
    return false;
  }
  return true;
}

bool c_jdr_pipeline::align_to_reference(const c_image_processor::sptr & proc, c_ecch & ecchr,
    cv::Mat & current_frame, cv::Mat & current_mask) const
{
  const color_channel_type reference_channel =
      _reference_frame_options.generate_opts.reference_channel;

  cv::Mat grayscale_frame;
  cv::Mat2f rmap;

  if( current_frame.channels() == 1 ) {
    grayscale_frame = current_frame;
  }
  else if( !extract_channel(current_frame, grayscale_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
    CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
    return false;
  }

  if( !ecchr.align(grayscale_frame, current_mask) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  if( !ecchr.create_remap(rmap) ) {
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
}

bool c_jdr_pipeline::run_pipeline()
{
  CF_DEBUG("ENTER");

  // It is important to distinct between 'Master Frame' and 'Reference Frame'.
  // As input raw stacks i input sequence are not aligned, the 'Reference Frame'
  // is created first to provode single uniform reference image coordinate system
  // for all input frames. Each of inpurt raw stacks is firstly aligned to this reference
  // before the derotation and average.
  // The Reference frame is also used to detect planetary ellipsoid fit with
  // orientation estimation based on RADON FFT.

  // Create reference frame based on user-selected selected 'Reference Master Stack'
  if ( !create_reference_frame() ) {
    CF_ERROR("create_reference_frame() fails");
    return false;
  }

  // Use 'Reference frame' generated above to detect planetary disk ellipsoid
  if ( !estimate_planetary_disk_ellipse() ) {
    CF_ERROR("estimate_planetary_disk_ellipse() fails");
    return false;
  }

  if ( _stack_options.enable_image_stacking ) {

    const int input_sequence_size = _input_sequence->size();
    const int max_input_frames = _input_options.max_input_frames < 0 ? input_sequence_size : std::clamp(_input_options.max_input_frames, 0, input_sequence_size);
    const int stack_context_size = _stack_options.derotate_context_size < 0 ?  input_sequence_size :
        std::clamp(_stack_options.derotate_context_size, 1, input_sequence_size);

    int start_frame_index, end_frame_index;

    // User may request to compute only single derotated average, or, alternatively
    // compute derotated average for each of frames in input sequence (to create a planetary disk rotation video)
    if ( !_stack_options.derotate_all_frames ) {
      start_frame_index = _reference_master_pos;
      end_frame_index = _reference_master_pos + 1;
    }
    else {
      start_frame_index = std::clamp(_input_options.start_frame_index, 0, input_sequence_size - 1);
      end_frame_index = std::min(start_frame_index + max_input_frames, max_input_frames);
    }

    CF_DEBUG("start_frame_index=%d end_frame_index=%d / %d", start_frame_index, end_frame_index, input_sequence_size);

    const color_channel_type reference_channel = _reference_frame_options.generate_opts.reference_channel;
    const c_image_processor::sptr preproc = _stack_options.input_image_preprocessor;

    const c_image_transform::sptr transform =
        create_image_transform(_reference_frame_options.generate_opts.motion_type);
    if( !transform ) {
      CF_ERROR("create_image_transform(motion_type = %d (%s) ) fails",
          (int )(_reference_frame_options.generate_opts.motion_type),
          toCString(_reference_frame_options.generate_opts.motion_type));
      return false;
    }

    c_ecch ecchr(transform.get(), _reference_frame_options.generate_opts.ecch_opts);
    if ( !ecchr.set_reference_image(_reference_frame, _reference_mask) ) {
      CF_ERROR("ecch.set_reference_image(_reference_frame) fails");
      return false;
    }

    cv::Mat current_master_frame, current_master_mask, current_master_ff;

    for ( int i = start_frame_index; i < end_frame_index; ++i ) {

      if ( !_input_sequence->seek(i) ) {
        CF_ERROR("_input_sequence->seek(i=%d) fails", i);
        return false;
      }

      if( is_bad_frame_index(_input_sequence->current_pos()) ) {
        CF_DEBUG("Skip frame %d as blacklisted", _input_sequence->current_pos());
        continue;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if ( !read_input_frame(_input_sequence, _input_options, current_master_frame, current_master_mask, false, false) ) {
        CF_ERROR("read_input_frame(_input_sequence->current_pos()=%d) fails", _input_sequence->current_pos());
        return false;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !_input_sequence->has_last_ts() ) {
        CF_ERROR("[F %d] Frame has no time stamp. Can not derotate", i);
        continue;
      }

      if( !preprocess(preproc, current_master_frame, current_master_mask) ) {
        CF_ERROR("[F%d] preprocess(current_master_frame) fails", _input_sequence->current_pos() - 1);
        continue;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( current_master_frame.channels() > 1 ) {
        const bool fOK =
            extract_channel(current_master_frame, current_master_frame,
                cv::noArray(), cv::noArray(),
                reference_channel);
        if( !fOK ) {
          CF_ERROR("extract_channel(reference_channel=%d (%s)) fails",
              reference_channel, toCString(reference_channel));
          return false;
        }

        if ( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }

      if( !align_to_reference(preproc, ecchr, current_master_frame, current_master_mask) ) {
        CF_ERROR("[F%d] preproc_and_align_to_reference(current_master_frame) fails. Ignored",
            _input_sequence->current_pos() - 1);
        continue;
      }

      makeFFImage(current_master_frame, current_master_ff);
      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      const double current_master_ts = 1e-3 * _input_sequence->last_ts();
      const int current_master_pos = i;
      const int stack_begin = std::clamp(current_master_pos - stack_context_size, 0, input_sequence_size);
      const int stack_end = std::clamp(current_master_pos + stack_context_size, 0, input_sequence_size);

      CF_DEBUG("[MF %d] frame: _master_ts=%lf %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d\n"
          "stack_begin=%d end_frame_index=%d / %d", i, current_master_ts,
          current_master_frame.cols, current_master_frame.rows, current_master_frame.channels(), current_master_frame.depth(),
          current_master_mask.cols, current_master_mask.rows, current_master_mask.channels(), current_master_mask.depth(),
          stack_begin, stack_end, input_sequence_size);

      bool fOK =
          derotate_and_average_frames(stack_begin, stack_end,
              ecchr, current_master_ff,
              current_master_ts,
              current_master_pos);
      if ( !fOK ) {
        CF_ERROR("[MF %d] derotate_jovian_frames() fails", i);
        return false;
      }
    }
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

  cv::Mat master_frame, master_mask;

  if ( !read_input_frame(master_sequence, _input_options, master_frame, master_mask, is_external_master_file, false) ) {
    CF_ERROR("read_input_frame() fails");
    return false;
  }

  _reference_master_ts = master_sequence->has_last_ts() ? 1e-3 * master_sequence->last_ts() : 0;
  _reference_master_pos = master_frame_index;
  CF_DEBUG("_master_pos=%d _master_ts=%lf [ms]", _reference_master_pos, _reference_master_ts);

  CF_DEBUG("master_frame: %dx%d channels=%d depth=%d",
      master_frame.cols, master_frame.rows,
      master_frame.channels(), master_frame.depth());

  CF_DEBUG("master_mask : %dx%d channels=%d depth=%d",
      master_mask.cols, master_mask.rows,
      master_mask.channels(), master_mask.depth());


  const std::string reference_file_name =
      (!_reference_frame_options.reference_file_name.empty()) ? _reference_frame_options.reference_file_name :
          generate_output_filename(_reference_frame_options.reference_file_name,
              ".reference",
              ".tiff");

  if( !_reference_frame_options.generate_reference_frame ) {
    CF_DEBUG("NOT GENERATING REFERENCE FRAME");

    if( !load_image(reference_file_name, _reference_frame, _reference_mask) ) {
      CF_ERROR("load_image('%s') fails\n"
          "Check if file exists or (Re)Generate new reference frame",
          reference_file_name.c_str());
      return false;
    }

    if ( _reference_frame.channels() > 1 ) {
      const color_channel_type reference_channel = _reference_frame_options.generate_opts.reference_channel;
      if ( !extract_channel(_reference_frame, _reference_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
        CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
        return false;
      }
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

    const c_image_transform::sptr transform = create_image_transform(_reference_frame_options.generate_opts.motion_type);
    if( !transform ) {
      CF_ERROR("create_image_transform(motion_type = %d (%s) ) fails",
          (int )(_reference_frame_options.generate_opts.motion_type),
          toCString(_reference_frame_options.generate_opts.motion_type));
      return false;
    }

    const color_channel_type & reference_channel = _reference_frame_options.generate_opts.reference_channel;

    cv::Mat current_frame;
    cv::Mat current_mask;
    cv::Mat2f rmap;

    c_ecch ecch(transform.get(), _reference_frame_options.generate_opts.ecch_opts);

    if( const auto & proc = _reference_frame_options.generate_opts.image_preprocessor ) {
      if( !proc->process(master_frame, master_mask) ) {
        CF_ERROR("input_image_preprocessor->process(master_frame, master_mask) fails");
        return false;
      }
    }

    const std::string output_reference_master_frame_filename =
        generate_output_filename("", ".reference_master_frame", ".tiff");
    if( !save_image(master_frame, master_mask, output_reference_master_frame_filename) ) {
      CF_ERROR("save_image(%s) fails", output_reference_master_frame_filename.c_str());
      return false;
    }
    CF_ERROR("SAVED reference_master_frame=%s", output_reference_master_frame_filename.c_str());

    if ( master_frame.channels() != 1 ) {
      if ( !extract_channel(master_frame, master_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
        CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
        return false;
      }
    }

    if ( !ecch.set_reference_image(master_frame, master_mask) ) {
      CF_ERROR("ecch.set_reference_image() fails");
      return false;
    }

    _processed_frames = 0;
    _accumulated_frames = 0;
    _total_frames = end_frame_index - start_frame_index;

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

      if( const auto & proc = _reference_frame_options.generate_opts.image_preprocessor ) {
        if( !proc->process(current_frame, current_mask) ) {
          CF_ERROR("input_image_preprocessor->process(current_frame, current_mask) fails");
          return false;
        }
        CF_DEBUG("[F %d] PREPROCESSED", i);
      }

      if ( current_frame.channels() > 1 ) {
        if ( !extract_channel(current_frame, current_frame, cv::noArray(), cv::noArray(), reference_channel) ) {
          CF_ERROR("extract_channel(reference_channel=%d (%s)) fails", reference_channel, toCString(reference_channel));
          return false;
        }
      }

      if( !ecch.align(current_frame, current_mask) ) {
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
      if( !_reference_frame_avg.compute(_reference_frame, _reference_mask) ) {
        CF_ERROR("_reference_frame_avg.compute(_reference_frame) fails");
        return false;
      }
    }

    CF_DEBUG("reference frame: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d",
        _reference_frame.cols, _reference_frame.rows, _reference_frame.channels(), _reference_frame.depth(),
        _reference_mask.cols, _reference_mask.rows, _reference_mask.channels(), _reference_mask.depth());

    if ( !save_image(_reference_frame, _reference_mask, reference_file_name) ) {
      CF_ERROR("save_image('%s') fails", reference_file_name.c_str());
      return false;
    }

    CF_DEBUG("Saved as %s", reference_file_name.c_str());
  }

  return true;
}

bool c_jdr_pipeline::estimate_planetary_disk_ellipse()
{
  if ( _reference_frame.empty() ) {
    CF_ERROR("APP BUG: No reference frame was created");
    return false;
  }

  if ( !_ellipse_estimation_options.auto_pose_estimation ) {
    _planetary_disk_pose.center = _ellipse_estimation_options.pose.center;
    _planetary_disk_pose.axes = _ellipse_estimation_options.pose.axes;
    _planetary_disk_pose.orientation = _ellipse_estimation_options.pose.orientation * CV_PI / 180;
  }
  else {
    _ellipse_detector.set_options(_ellipse_estimation_options.ellipse_detector_options);
    if( !_ellipse_detector.detect(_reference_frame, _reference_mask) ) {
      CF_ERROR("_ellipse_detector.detect() fails");
      return false;
    }

    _planetary_disk_pose.center = _ellipse_detector.center();
    _planetary_disk_pose.axes = _ellipse_detector.axes();
    _planetary_disk_pose.orientation = _ellipse_detector.pose();

    if ( _ellipse_estimation_options.update_estimated_pose ) {
      _ellipse_estimation_options.pose.center = _planetary_disk_pose.center;
      _ellipse_estimation_options.pose.axes = _planetary_disk_pose.axes;
      _ellipse_estimation_options.pose.orientation = _planetary_disk_pose.orientation * 180 / CV_PI;
      on_parameters_update();
    }

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

      drawRotatedRect(display, _ellipse_detector.finalPlanetaryDiskEllipse(), CV_RGB(0, 255, 0), 1);
      cv::ellipse(display, _ellipse_detector.finalPlanetaryDiskEllipse(), CV_RGB(0, 0, 255), 1);

      const std::string output_display_filename =
          generate_output_filename("", ".jovian_ellipse_fit", ".png");
      if( !save_image(display, cv::noArray(), output_display_filename) ) {
        CF_ERROR("save_image('%s') fails", output_display_filename.c_str());
        return false;
      }
      CF_DEBUG("Saved %s", output_display_filename.c_str());
    }

    _ellipse_detector.clear();
  }

  draw_ellipsoid_mask(_reference_planetary_disk_mask, _reference_frame.size(),
      _planetary_disk_pose.center, _planetary_disk_pose.axes,
      _planetary_disk_pose.orientation);

  if ( true ) {
    const std::string output_planetary_disk_mask_filename =
        generate_output_filename("", ".reference_planetary_disk_mask", ".png");
    if( !save_image(_reference_planetary_disk_mask, cv::noArray(), output_planetary_disk_mask_filename) ) {
      CF_ERROR("save_image('%s') fails", output_planetary_disk_mask_filename.c_str());
      return false;
    }
    CF_DEBUG("Saved %s", output_planetary_disk_mask_filename.c_str());
  }

  _ellipsoid_derotation_remap.set_reference_pose(_reference_frame.size(),
      _planetary_disk_pose.center,
      _planetary_disk_pose.axes,
      _planetary_disk_pose.orientation);


  if ( true ) {
    // Create ellipsoid 3D illustration image
    cv::Mat display, mask;

    double minv = 0, maxv = 1;
    cv::minMaxLoc(_reference_frame, &minv, &maxv, nullptr, nullptr);

    if ( _reference_frame.channels() == 3 ) {
      _reference_frame.convertTo(display, CV_8UC3, 255./maxv);
    }
    else {
      cv::cvtColor(_reference_frame, display, cv::COLOR_GRAY2BGR);
      display.convertTo(display, CV_8UC3, 255./maxv);
    }

    draw_ellipsoid(display,
        _ellipsoid_derotation_remap.center(),
        _ellipsoid_derotation_remap.axes(),
        _ellipsoid_derotation_remap.Rtarget(),
        30 * CV_PI / 180,
        30 * CV_PI / 180,
        cv::Scalar::all(255),
        1,
        cv::LINE_AA);

    const std::string output_display_filename =
        generate_output_filename("", ".ellipsoid_fit", ".png");
    if( !save_image(display, cv::noArray(), output_display_filename) ) {
      CF_ERROR("save_image('%s') fails", output_display_filename.c_str());
      return false;
    }
    CF_DEBUG("Saved %s", output_display_filename.c_str());
  }

  return true;
}

bool c_jdr_pipeline::derotate_and_average_frames(int start_frame_index, int end_frame_index,
    c_ecch & ecchr, const cv::Mat & current_master_ff,
    double current_master_ts,
    int current_master_pos)
{
  _average.clear();

  if ( !_input_sequence->seek(start_frame_index) ) {
    CF_ERROR("_input_sequence->seek(start_frame_index=%d) fails", start_frame_index);
    return false;
  }

  set_pipeline_stage(stacking_stage_in_progress);

  const color_channel_type & reference_channel = _reference_frame_options.generate_opts.reference_channel;
  const c_image_processor::sptr preproc = _stack_options.input_image_preprocessor;

  cv::Mat current_frame, current_mask, current_ff;
  cv::Mat1f current_weights;//, current_wmap, reference_wmap;
  cv::Mat2f rmap;

  _processed_frames = 0;
  _accumulated_frames = 0;
  _total_frames = end_frame_index - start_frame_index;

  if ( !open_output_writers() ) {
    CF_ERROR("open_output_writers() fails");
    return false;
  }

//  _ellipsoid_derotation_remap.compute_derotation_for_time(0, 1);
//  _ellipsoid_derotation_remap.wmap().copyTo(reference_wmap);

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

    CF_DEBUG("[F %d (MF %d)] frame: %dx%d channels=%d depth=%d mask: %dx%d channels=%d depth=%d", i, current_master_pos,
        current_frame.cols, current_frame.rows, current_frame.channels(), current_frame.depth(),
        current_mask.cols, current_mask.rows, current_mask.channels(), current_mask.depth());

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if( !_input_sequence->has_last_ts() ) {
      CF_ERROR("[F %d] Frame has no time stamp. Can not derotate", i);
      continue;
    }

    if( !preprocess(preproc, current_frame, current_mask) ) {
      CF_ERROR("[F %d] preprocess(current_frame) fails", i);
      continue;
    }

    if( !align_to_reference(preproc, ecchr, current_frame, current_mask) ) {
      CF_ERROR("[F%d] preproc_and_align_to_reference(current_frame) fails. Ignored", i);
      continue;
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( _aligned_frames_writer.is_open() ) {
      if ( !_aligned_frames_writer.write(current_frame, current_mask) ) {
        CF_ERROR("[F %d] _current_aligned_frame_writer.write() fails for %s", i, _aligned_frames_writer.cfilename());
        return false;
      }
    }

    const double ts = 1e-3 * _input_sequence->last_ts();
    const double dt = ts - current_master_ts;
    const double wts = _stack_options.wts > 0 ? _stack_options.wts : 360.;
    const double w = 1. / (1. + std::abs(dt) / wts);
    CF_DEBUG("[F %d (MF %d)] ts = %lf [s] dt = %lf [s] w=%g ", i, current_master_pos, ts, dt, w);

    _ellipsoid_derotation_remap.compute_derotation_for_time(-dt, w);

    //current_weights = cv::Mat1f::ones(current_frame.size());
    current_mask.convertTo(current_weights, CV_32F, 1./255);
    current_weights.setTo(0, _ellipsoid_derotation_remap.rmask());
    const auto & wmap = _ellipsoid_derotation_remap.wmap();
    wmap.copyTo(current_weights, wmap > 1e-4);
    cv::GaussianBlur(current_weights, current_weights, cv::Size(), 1, 1,
        cv::BORDER_REPLICATE);

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !current_master_ff.empty() ) {
      makeFFImage(current_frame, current_ff);
      divideImages(current_frame, current_ff, current_frame);
      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    cv::remap(current_frame, current_frame,
        _ellipsoid_derotation_remap.rmap(), cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_TRANSPARENT);

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !current_master_ff.empty() ) {
      multiplyImages(current_frame, current_master_ff, current_frame);
    }

//    if ( !current_mask.empty() ) {
//      current_weights.setTo(0, ~current_mask);
//    }

    if( !_average.add(current_frame, current_weights) ) {
      CF_ERROR("[F%d] _average.add(current_frame) fails", i);
      return false;
    }

    if ( _derotated_frames_writer.is_open() ) {
      if ( !_derotated_frames_writer.write(current_frame, current_mask) ) {
        CF_ERROR("[F %d] _derotated_frame_writer.write() fails for %s", i, _derotated_frames_writer.cfilename());
        return false;
      }
    }

    if ( _accumulation_weights_writer.is_open() ) {
      if ( !_accumulation_weights_writer.write(current_weights) ) {
        CF_ERROR("[F %d] _accumulation_weights_writer.write() fails for %s", i, _accumulation_weights_writer.cfilename());
        return false;
      }
    }

    synchronized([&]() {
      current_frame.copyTo(_current_display_frame);
      current_mask.copyTo(_current_display_mask);
    });

    ++_accumulated_frames;
  }

  CF_DEBUG("_average.accumulated_frames()=%d", _average.accumulated_frames());

  if ( _average.accumulated_frames() > 0 )  {

    cv::Mat avg, mask;

    if ( !_average.compute(avg, mask) ) {
      CF_ERROR("_frame_average.compute() fails");
      return false;
    }

    if ( _derotated_avg_frames_writer.is_open() ) {
      if ( !_derotated_avg_frames_writer.write(avg, mask, current_master_pos) ) {
        CF_ERROR("[MF %d] _derotated_avg_frames_writer.write() fails for %s", current_master_pos,
            _derotated_avg_frames_writer.cfilename());
        return false;
      }
    }

    if ( _derotated_avg_weights_writer.is_open() ) {
      if ( !_derotated_avg_weights_writer.write(_average.counter(), mask, current_master_pos) ) {
        CF_ERROR("[MF %d] _derotated_avg_weights_writer.write() fails for %s", current_master_pos,
            _derotated_avg_weights_writer.cfilename());
        return false;
      }
    }
  }

  return true;
}

