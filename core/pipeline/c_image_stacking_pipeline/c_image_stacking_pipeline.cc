/*
 * c_stacking_pipeline.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/sharpness_measure/c_laplacian_sharpness_measure.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/extract_channel.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/autoclip.h>
#include <core/proc/flow2HSV.h>
#include <core/proc/normalize.h>
#include <core/proc/inpaint.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/fft.h>
#include <core/proc/focus.h>
#include <core/io/save_image.h>
#include <core/io/load_image.h>
#include <core/readdir.h>
#include <core/get_time.h>
#include <core/ssprintf.h>
#include <tbb/tbb.h>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<>
const c_enum_member * members_of<roi_selection_method>()
{
  static const c_enum_member members[] = {
      { roi_selection_none, "none", },
      { roi_selection_planetary_disk, "planetary_disk", },
      { roi_selection_rectange_crop, "rectangle", },
      { roi_selection_none, },
  };
  return members;
}

template<>
const c_enum_member* members_of<frame_accumulation_method>()
{
  static const c_enum_member members[] = {

      { frame_accumulation_average, "average",
          "Simple average" },

      { frame_accumulation_weighted_average, "weighted_average",
          "Weighted average with weights proportional to the smoothed sum of squared laplacian and gradient" },

      { frame_accumulation_bayer_average, "bayer_average",
          "Experimental code for bayer pattern average" },

      { frame_accumulation_focus_stack, "focus_stack",
          "Focus stacking based on paper of Wang and Chang 2011" },

      { frame_accumulation_fft, "fft",
          "Stupid experiments with fft-based stacking " },

      { frame_accumulation_none, "None", },

      { frame_accumulation_none, },
  };

  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_stage>()
{
  static const c_enum_member members[] = {
      { frame_upscale_after_align , "after_align", },
      { frame_upscale_before_align , "before_align", },
      { frame_upscale_stage_unknown },
  };
  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_option>()
{
  static const c_enum_member members[] = {
      {frame_upscale_none, "none", },
      {frame_upscale_pyrUp, "x2.0", },
      {frame_upscale_x15, "x1.5", },
      {frame_upscale_x30, "x3.0", },
      {frame_upscale_none},
  };
  return members;
}

template<>
const c_enum_member* members_of<STACKING_STAGE>()
{
  static const c_enum_member members[] = {
      { stacking_stage_idle, "idle", "idle" },
      { stacking_stage_initialize, "initialize", "initialize" },
      { stacking_stage_select_master_frame_index, "select_master_frame_index", "select master frame index" },
      { stacking_stage_generate_reference_frame, "generate_reference_frame", "generate reference frame" },
      { stacking_stage_in_progress, "stacking_in_progress", "stacking in progress" },
      { stacking_stage_finishing, "finishing", "finishing" },
      { stacking_stage_idle },
  };

  return members;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

bool multiply_weights(cv::InputArray mask, cv::InputArray weights, cv::OutputArray dst, double scale = 1, int dtype = -1)
{
  INSTRUMENT_REGION("");

  if( mask.channels() == weights.channels() ) {
    cv::multiply(mask, weights, dst, scale, dtype);
  }
  else if( mask.channels() == 1 ) {
    std::vector<cv::Mat> channels;
    cv::split(weights, channels);
    for( int c = 0, cn = weights.channels(); c < cn; ++c ) {
      cv::multiply(mask, channels[c], channels[c], scale, dtype);
    }
    cv::merge(channels, dst);
  }
  else if( weights.channels() == 1 ) {
    std::vector<cv::Mat> channels;
    cv::split(mask, channels);
    for( int c = 0, cn = mask.channels(); c < cn; ++c ) {
      cv::multiply(channels[c], weights, channels[c], scale, dtype);
    }
    cv::merge(channels, dst);
  }
  else {
    CF_ERROR("Unsupported combination of mask (%d) and weights (%d) channels",
        mask.channels(), weights.channels());
    return false;
  }

  return true;
}


static cv::Mat2f flow2remap(const cv::Mat2f &uv, const cv::Mat1b & mask)
{
  cv::Mat2f rmap(uv.size());

  typedef tbb::blocked_range<int> range;

  if( mask.empty() /*|| cv::countNonZero(mask) == mask.size().area()*/ ) {

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&rmap, &uv](const range & r) {
          const int nx = rmap.cols;
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < nx; ++x ) {
              rmap[y][x][0] = x - uv[y][x][0];
              rmap[y][x][1] = y - uv[y][x][1];
            }
          }
        });
  }
  else {

    tbb::parallel_for(range(0, rmap.rows, 256),
        [&rmap, &uv, &mask](const range & r) {
          const int nx = rmap.cols;
          for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
            for ( int x = 0; x < nx; ++x ) {
              if ( mask[y][x] ) {
                rmap[y][x][0] = x - uv[y][x][0];
                rmap[y][x][1] = y - uv[y][x][1];
              }
              else {
                rmap[y][x][0] = x;
                rmap[y][x][1] = y;
              }
            }
          }
        });
  }
  return rmap;
}

static cv::Mat2f compute_turbulent_flow(const c_image_transform * current_transform, const cv::Mat2f & current_remap )
{
  INSTRUMENT_REGION("");

  cv::Mat2f uv;

  const cv::Vec2f T =
      current_transform->translation();

  uv.create(current_remap.size());

  typedef tbb::blocked_range<int> range;
  tbb::parallel_for(range(0, current_remap.rows, 256),
      [&current_remap, &uv, T](const range & r) {

        const int nx = current_remap.cols;

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          for ( int x = 0; x < nx; ++x ) {

            if ( current_remap[y][x][0] >= 0 && current_remap[y][x][1] >= 0 ) {
              uv[y][x][0] = current_remap[y][x][0] - x - T[0];
              uv[y][x][1] = current_remap[y][x][1] - y - T[1];
            }
            else {
              uv[y][x][0] = 0;
              uv[y][x][1] = 0;
            }

          }
        }
      });

  return uv;
}


c_frame_registration::ecc_image_preprocessor_function create_ecc_image_preprocessor(
    const c_image_processing_options & image_processing_options)
{
  c_frame_registration::ecc_image_preprocessor_function ecc_preprocessor;

  if( image_processing_options.ecc_image_processor ) {

    const c_image_processor::sptr &processor =
        image_processing_options.ecc_image_processor;

    if( processor ) {

      ecc_preprocessor =
          [processor](cv::InputOutputArray image, cv::InputOutputArray mask) {
            processor->process(image, mask);
          };
    }
  }

  return ecc_preprocessor;
}

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_image_stacking_pipeline::c_image_stacking_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
  if( input_sequence && input_sequence->sources().size() > 0 ) {
    set_master_source(input_sequence->source(0)->filename());
    set_master_frame_index(0);
  }
}

c_image_stacking_pipeline::~c_image_stacking_pipeline()
{
  cancel(true);
}

std::string c_image_stacking_pipeline::output_file_name() const
{
  return _output_file_name;
}

const c_anscombe_transform & c_image_stacking_pipeline::anscombe() const
{
  return anscombe_;
}

c_image_stacking_input_options & c_image_stacking_pipeline::input_options()
{
  return _input_options;
}

const c_image_stacking_input_options & c_image_stacking_pipeline::input_options() const
{
  return _input_options;
}

c_frame_upscale_options & c_image_stacking_pipeline::upscale_options()
{
  return _upscale_options;
}

const c_frame_upscale_options & c_image_stacking_pipeline::upscale_options() const
{
  return _upscale_options;
}

c_roi_selection_options & c_image_stacking_pipeline::roi_selection_options()
{
  return _roi_selection_options;
}

const c_roi_selection_options & c_image_stacking_pipeline::roi_selection_options() const
{
  return _roi_selection_options;
}

c_roi_selection::ptr c_image_stacking_pipeline::create_roi_selection() const
{
  switch ( _roi_selection_options.method ) {
  case roi_selection_planetary_disk :
    return c_planetary_disk_selection::create(_roi_selection_options.planetary_disk_crop_size,
        _roi_selection_options.planetary_disk_gbsigma,
        _roi_selection_options.planetary_disk_stdev_factor,
        _roi_selection_options.se_close_size);
  case roi_selection_rectange_crop :
    return c_roi_rectangle_selection::create(_roi_selection_options.rectangle_roi_selection);

  default :
    break;
  }
  return nullptr;
}

c_frame_registration::sptr c_image_stacking_pipeline::create_frame_registration(const c_image_registration_options & options) const
{
  return c_frame_registration::sptr(new c_frame_registration(options));
}

//c_frame_accumulation_options & c_image_stacking_pipeline::accumulation_options()
//{
//  return accumulation_options_;
//}
//
//const c_frame_accumulation_options & c_image_stacking_pipeline::accumulation_options() const
//{
//  return accumulation_options_;
//}

c_frame_accumulation::ptr c_image_stacking_pipeline::create_frame_accumulation(const c_frame_accumulation_options & opts) const
{
  switch (opts.accumulation_method) {
    case frame_accumulation_average:
      return c_frame_accumulation::ptr(new c_frame_weigthed_average());
    case frame_accumulation_weighted_average:
      return c_frame_accumulation::ptr(new c_frame_weigthed_average(opts.max_weights_ratio));
    case frame_accumulation_focus_stack:
      return c_frame_accumulation::ptr(new c_laplacian_pyramid_focus_stacking(opts.fs));
    case frame_accumulation_fft:
      return c_frame_accumulation::ptr(new c_frame_accumulation_with_fft());
    case frame_accumulation_bayer_average:
      return c_frame_accumulation::ptr(new c_bayer_average());
    default:
      break;
  }
  return nullptr;
}

c_image_stacking_output_options & c_image_stacking_pipeline::output_options()
{
  return _output_options;
}

const c_image_stacking_output_options & c_image_stacking_pipeline::output_options() const
{
  return _output_options;
}

c_image_processing_options & c_image_stacking_pipeline::image_processing_options()
{
  return _image_processing_options;
}

const c_image_processing_options & c_image_stacking_pipeline::image_processing_options() const
{
  return _image_processing_options;
}

std::string c_image_stacking_pipeline::generate_output_file_name() const
{
  std::string output_file_name =
      _output_options.output_file_name;

  if( output_file_name.empty() ) {

    output_file_name =
        ssprintf("%s/%s%s.32F.tiff",
            output_path_.c_str(),
            csequence_name(),
            output_file_name_postfix_.c_str());
  }
  else {

    std::string path, name, suffix;

    split_pathfilename(_output_options.output_file_name, &path, &name, &suffix);

    if( path.empty() ) {
      path = output_path_;
    }
    else if( !is_absolute_path(path) ) {
      path = ssprintf("%s/%s", output_path_.c_str(), path.c_str());
    }

    if( name.empty() ) {
      name = ssprintf("%s%s", csequence_name(),
          output_file_name_postfix_.c_str());
    }

    if( suffix.empty() || suffix.back() == '.' ) {
      suffix = ".tiff";
    }

    output_file_name =
        ssprintf("%s/%s%s",
            path.c_str(),
            name.c_str(),
            suffix.c_str());
  }


  return output_file_name;
}

bool c_image_stacking_pipeline::initialize_pipeline()
{
  set_pipeline_stage(stacking_stage_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_image_stacking_pipeline: base::initialize() fails" );
    return false;
  }

  output_path_ =
      create_output_path(_output_options.output_directory);


  if (true ) {

    lock_guard lock(mutex());

    missing_pixel_mask_.release();

    // ecc_normalization_noise_ = 0;

    output_file_name_postfix_.clear();
    _output_file_name.clear();

    roi_selection_.reset();
    frame_registration_.reset();
    frame_accumulation_.reset();
    flow_accumulation_.reset();
    _generating_master_frame = false;
  }

  anscombe_.set_method(input_options().anscombe);

  if ( roi_selection_options().method != roi_selection_none ) {
    if ( !(roi_selection_ = create_roi_selection()) ) {
      set_status_msg("ERROR: create_roi_selection() fails");
      return false;
    }
  }

  if ( !input_options().darkbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(input_options().darkbayer_filename, darkbayer_, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", input_options().darkbayer_filename.c_str());
      return false;
    }
  }

  if ( !input_options().flatbayer_filename.empty() ) {
    cv::Mat ignored_optional_mask;
    if ( !load_image(input_options().flatbayer_filename, flatbayer_, ignored_optional_mask) ) {
      CF_ERROR("load_image('%s') fails.", input_options().flatbayer_filename.c_str());
      return false;
    }
  }



  if ( !input_options().missing_pixel_mask_filename.empty() ) {

    if ( !load_image(input_options().missing_pixel_mask_filename, missing_pixel_mask_) ) {
      CF_ERROR("load_image('%s') fails.", input_options().missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( missing_pixel_mask_.type() != CV_8UC1 ) {
      CF_ERROR("Invalid bad pixels mask %s : \nMust be CV_8UC1 type",
          input_options().missing_pixel_mask_filename.c_str());
      return false;
    }

    if ( !input_options().missing_pixels_marked_black ) {
      cv::invert(missing_pixel_mask_, missing_pixel_mask_);
    }
  }

  CF_DEBUG("Output path='%s'", this->output_path_.c_str());


  return true;
}


void c_image_stacking_pipeline::cleanup_pipeline()
{
  set_pipeline_stage(stacking_stage_finishing);
  base::cleanup_pipeline();

  roi_selection_.reset();

  if ( true ) {
    frame_registration_.reset();
    flow_accumulation_.reset();
  }

  missing_pixel_mask_.release();
  darkbayer_.release();
  flatbayer_.release();

  set_pipeline_stage(stacking_stage_idle);
}


bool c_image_stacking_pipeline::run_pipeline()
{
  bool fOk = false;

  CF_ERROR("FIXME: Not saving pipeline config !");
  //  save(ssprintf("%s/%s.cfg",
  //      output_path_.c_str(),
  //      cname()));

//  const bool do_planetary_disk_derotation =
//      _stacking_options.registration.enabled &&
//      _stacking_options.registration.planetary_disk_derotation.derotation_type != planetary_disk_derotation_disabled;
//
//  if ( do_planetary_disk_derotation ) {
//    if( !(fOk = run_planetary_disk_derotation()) ) {
//      CF_ERROR("run_planetary_disk_derotation() fails");
//    }
//  }
//  else {
//    if( !(fOk = run_image_stacking()) ) {
//      CF_ERROR("run_image_stacking() fails");
//    }
//  }

  if( !(fOk = run_image_stacking()) ) {
    CF_ERROR("run_image_stacking() fails");
  }

  return fOk;
}
//
//bool c_image_stacking_pipeline::run_planetary_disk_derotation()
//{
//  CF_DEBUG("Starting '%s: %s' ...",
//      csequence_name(), cname());
//
//
//  static const auto drawRotatedRectange =
//      [](cv::InputOutputArray image, const cv::RotatedRect & rc,
//          const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0) {
//            cv::Point2f pts[4];
//            rc.points(pts);
//
//            cv::ellipse(image, rc, color, thickness, lineType);
//
//            for( int i = 0; i < 4; i++ ) {
//              cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
//            }
//
//            cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
//            cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
//          };
//
//
//  cv::Mat master_frame;
//  cv::Mat master_mask;
//
//  if( !create_reference_frame(master_frame, master_mask) ) {
//    CF_FATAL("create_reference_frame() fails");
//    return false;
//  }
//
//  if( _master_options.save_master_frame ) {
//
//    const std::string filename =
//        ssprintf("%s/%s-jovian-master.tiff",
//            output_path_.c_str(),
//            csequence_name());
//
//    if( !write_image(filename, _output_options, master_frame, master_mask) ) {
//      CF_FATAL("write_image('%s') fails", filename.c_str());
//      return false;
//    }
//  }
//
//  if ( !(frame_registration_ = create_frame_registration(_stacking_options.registration)) ) {
//    CF_FATAL("create_frame_registration() fails");
//    return false;
//  }
//
//  if( _stacking_options.accumulation.accumulation_method != frame_accumulation_none ) {
//    lock_guard lock(mutex());
//    if( !(frame_accumulation_ = create_frame_accumulation(_stacking_options.accumulation)) ) {
//      CF_ERROR("ERROR: create_frame_accumulation(stack_accumulation_options_) fails");
//      return false;
//    }
//  }
//
//
//  switch (_stacking_options.registration.planetary_disk_derotation.derotation_type) {
//    case planetary_disk_derotation_jovian: {
//
//      c_jovian_derotation & jovian_derotation =
//          frame_registration_->jovian_derotation();
//
//      if( _output_options.debug_frame_registration ) {
//        jovian_derotation.set_debug_path(ssprintf("%s/debug/jovian_derotation",
//            output_path_.c_str()));
//      }
//
//      if( !jovian_derotation.detect_jovian_ellipse(master_frame, master_mask) ) {
//        CF_FATAL("jovian_derotation.setup_jovian_ellipse() fails");
//        return false;
//      }
//
//      break;
//    }
//
//    case planetary_disk_derotation_saturn: {
//
//      c_saturn_derotation & saturn_derotation =
//          frame_registration_->saturn_derotation();
//
//      saturn_derotation.set_detector_options(_stacking_options.registration.planetary_disk_derotation.saturn_derotation.detector_options);
//
//      if( !saturn_derotation.detect(master_frame, master_mask) ) {
//        CF_FATAL("saturn_derotation.detect_saturn() fails");
//        return false;
//      }
//
//      break;
//    }
//
//    default:
//      CF_ERROR("Invalid derotation_type=%d requested", _stacking_options.registration.planetary_disk_derotation.derotation_type);
//      return false;
//  }
//
//
//
//
//  if( !input_sequence_->is_open() && !input_sequence_->open() ) {
//    CF_ERROR("input_sequence_->open() fails");
//    return false;
//  }
//
//
//  std::vector<int> reference_frames;
//
//  if ( !_stacking_options.registration.planetary_disk_derotation.jovian_derotation.derotate_all_frames ) {
//
//    const int master_source_index =
//        input_sequence_->indexof(_master_options.master_selection.master_fiename);
//
//    if( master_source_index < 0 ) {
//      CF_ERROR("ERROR: Master source '%s' is outside of input sequence",
//          _master_options.master_selection.master_fiename.c_str());
//      return false;
//    }
//
//    reference_frames.emplace_back(input_sequence_->global_pos(master_source_index,
//        _master_options.master_selection.master_frame_index));
//  }
//  else {
//
//    const int start_pos =
//        std::max(0, _input_options.start_frame_index);
//
//    const int end_pos =
//        _input_options.max_input_frames > 0 ?
//            start_pos + _input_options.max_input_frames :
//            INT_MAX;
//
//    for( int i = 0, n = input_sequence_->sources().size(); i < n; ++i ) {
//
//      const c_input_source::sptr &source =
//          input_sequence_->source(i);
//
//      if( source->enabled() ) {
//
//        for( int j = 0, m = source->size(); j < m; ++j ) {
//
//          const int global_pos =
//              input_sequence_->global_pos(i, j);
//
//          if( global_pos >= start_pos && global_pos < end_pos && !is_bad_frame_index(global_pos) ) {
//            reference_frames.emplace_back(global_pos);
//          }
//        }
//      }
//    }
//  }
//
//  c_translation_image_transform image_transform;
//  //c_translation_ecc_motion_model model(&image_transform);
//
////  c_euclidean_image_transform image_transform;
////  c_euclidean_ecc_motion_model model(&image_transform);
//
////  c_affine_image_transform image_transform;
////  c_affine_ecc_motion_model model(&image_transform);
//
//  //c_ecc_forward_additive ecc(&model);
//  //c_ecc_forward_additive ecc(&image_transform);
//
//
//  c_ecch ecch(&image_transform, ECC_ALIGN_LM);
//
//  ecch.set_epsx(0.1);
//  ecch.set_minimum_image_size(16);
//  ecch.set_maxlevel(-1);
//
//  if( !ecch.set_reference_image(master_frame, master_mask) ) {
//    CF_FATAL("ecch.set_reference_image() fails");
//    return false;
//  }
//
//  cv::Mat reference_frame, reference_mask;
//
//  const color_channel_type master_channel =
//      _stacking_options.registration.ecc_registration_channel;
//
//  const int context_size =
//      std::min(input_sequence_->size(),
//          std::max(_stacking_options.registration.planetary_disk_derotation.jovian_derotation.max_context_size, 1));
//
//  const bool save_raw_bayer_image =
//      _stacking_options.accumulation.accumulation_method ==
//          frame_accumulation_bayer_average;
//
//
//  cv::Mat2f current_remap;
//
//  for( int i = 0, n = reference_frames.size(); i < n; ++i ) {
//
//    if( !input_sequence_->seek(reference_frames[i]) ) {
//      CF_ERROR("input_sequence_->seek(pos=%d) fails", reference_frames[i]);
//      return false;
//    }
//
//    const int cpos =
//        input_sequence_->current_pos();
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    if( !read_input_frame(input_sequence_, reference_frame, reference_mask, false, save_raw_bayer_image) ) {
//      CF_ERROR("read_input_frame(pos=%d) fails", cpos);
//      continue;
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    if ( !select_image_roi(roi_selection_, reference_frame, reference_mask, reference_frame, reference_mask) ) {
//      CF_FATAL("select_image_roi(reference_frame) fails");
//      return false;
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    if( _input_options.input_image_processor ) {
//      // lock_guard lock(mutex());
//      if( !_input_options.input_image_processor->process(reference_frame, reference_mask) ) {
//        CF_ERROR("input_image_processor->process(reference_frame) fails");
//        return false;
//      }
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    if( reference_frame.channels() != 1 ) {
//      if( !extract_channel(reference_frame, reference_frame, reference_mask, reference_mask, master_channel) ) {
//        CF_ERROR("extract_channel(pos=%d, master_channel='%s') fails", cpos, toCString(master_channel));
//        return false;
//      }
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    image_transform.reset();
//
//    if ( !ecch.align(reference_frame, reference_mask) ) {
//      CF_ERROR("ecch.align(pos=%d) fails", cpos);
//      continue;
//    }
//
//    CF_DEBUG("frame %d: %d/%d iterations eps=%g/%g", cpos,
//        ecch.num_iterations(), ecch.max_iterations(),
//        ecch.eps(), ecch.epsx());
//
//
//    current_remap =
//        ecch.create_remap();
//
//    cv::remap(reference_frame, reference_frame,
//        current_remap, cv::noArray(),
//        cv::INTER_LINEAR,
//        cv::BORDER_REFLECT101);
//
//    cv::remap(reference_mask, reference_mask,
//        current_remap, cv::noArray(),
//        cv::INTER_LINEAR,
//        cv::BORDER_REFLECT101);
//
//    cv::compare(reference_mask, 250, reference_mask,
//        cv::CMP_GE);
//
//    if( _output_options.debug_frame_registration ) {
//
//      const std::string filename =
//          ssprintf("%s/debug/jovian_derotation/reference.%03d.tiff",
//              output_path_.c_str(),
//              cpos);
//
//      cv::Mat tmp;
//
//      cv::cvtColor(reference_frame, tmp, cv::COLOR_GRAY2BGR);
//
//      switch (_stacking_options.registration.planetary_disk_derotation.derotation_type) {
//        case planetary_disk_derotation_jovian: {
//          drawRotatedRectange(tmp, frame_registration_->jovian_derotation().planetary_disk_ellipse(), CV_RGB(0, 1, 0));
//          break;
//        }
//        case planetary_disk_derotation_saturn: {
//          CF_DEBUG("FIXME: planetary_disk_derotation_saturn still not implemented");
//          drawRotatedRectange(tmp, frame_registration_->saturn_derotation().planetary_disk_ellipse(), CV_RGB(0, 1, 0));
//          break;
//        }
//      }
//
//
//      if( !save_image(tmp, reference_mask, filename) ) {
//        CF_ERROR("save_image(%s) fails", filename.c_str());
//        return false;
//      }
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    frame_registration_->set_reference_timestamp(input_sequence_->last_ts(),
//        input_sequence_->has_last_ts());
//
//    // CF_DEBUG("setup_frame_registration()");
//    if ( !setup_frame_registration(frame_registration_, reference_frame, reference_mask) ) {
//      CF_ERROR("setup_frame_registration(pos=%d) fails", cpos);
//      return false;
//    }
//
//
//
//    if ( frame_accumulation_ ) {
//      frame_accumulation_->clear();
//    }
//
//
//    int startpos, endpos;
//
//    if( (startpos = cpos - context_size / 2) < 0 ) {
//      startpos = 0;
//      endpos = std::min(startpos + context_size, input_sequence_->size());
//    }
//    else if( (endpos = cpos + context_size / 2) > input_sequence_->size() ) {
//      endpos = input_sequence_->size();
//      startpos = std::max(0, endpos - context_size);
//    }
//    else {
//      endpos = startpos + context_size;
//    }
//
//    CF_DEBUG("cpos=%d startpos=%d endpos=%d",
//        cpos, startpos, endpos);
//
//    if ( canceled() ) {
//      return false;
//    }
//
//    if ( !process_input_sequence(input_sequence_, startpos, endpos) ) {
//      CF_ERROR("process_input_sequence() fails");
//      return false;
//    }
//
//    if ( frame_accumulation_ ) {
//
//      cv::Mat accumulated_image;
//      cv::Mat1b accumulated_mask;
//
//      if ( true ) {
//        lock_guard lock(mutex());
//
//        if ( !frame_accumulation_->compute(accumulated_image, accumulated_mask) ) {
//          CF_ERROR("ERROR: frame_accumulation_->compute() fails");
//          return false;
//        }
//
//  #if 0
//        linear_interpolation_inpaint(accumulated_image,
//            accumulated_mask,
//            accumulated_image);
//  #else
//        average_pyramid_inpaint(accumulated_image,
//            accumulated_mask,
//            accumulated_image);
//  #endif
//      }
//
//      std::string filename =
//          ssprintf("%s/jovian/%s%s.%03d.32F.tiff",
//              output_path_.c_str(),
//              csequence_name(),
//              output_file_name_postfix_.c_str(),
//              cpos);
//
//      CF_DEBUG("Saving '%s'", filename.c_str());
//      if( !write_image(_output_file_name = filename, _output_options, accumulated_image, accumulated_mask) ) {
//        CF_ERROR("write_image('%s') fails", filename.c_str());
//        return false;
//      }
//    }
//
//    if ( canceled() ) {
//      return false;
//    }
//  }
//
//
//  CF_DEBUG("Finished");
//
//  return true;
//}

bool c_image_stacking_pipeline::run_image_stacking()
{
  std::string output_file_name;
  cv::Mat2f upscaled_remap;
  cv::Mat tmp;
  bool fOk;


  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  /////////////////////////////////////////////////////////////////////////////

  if( _stacking_options.registration.enabled || _master_options.generate_master_frame ) {

    // SETUP FRAME REGISTRATION

    cv::Mat reference_frame, reference_mask;
    double reference_timestamp = 0;

    if( !create_reference_frame(reference_frame, reference_mask, &reference_timestamp) ) {
      CF_ERROR("create_reference_frame() fails");
      return false;
    }

    if ( !_stacking_options.registration.enabled ) {
      frame_registration_.reset();
    }
    else {

      if ( !(frame_registration_ = create_frame_registration(_stacking_options.registration)) ) {
        CF_FATAL("create_frame_registration() fails");
        return false;
      }

      if ( !setup_frame_registration(frame_registration_, reference_frame, reference_mask) ) {
        CF_ERROR("setup_frame_registration() fails");
        return false;
      }

      frame_registration_->set_reference_timestamp(reference_timestamp, true);
    }
  }

  _generating_master_frame = false;

  if ( !_master_options.generate_master_frame || !_master_options.stop_after_master_frame_generation ) {


    if( _stacking_options.accumulation.accumulation_method != frame_accumulation_none ) {
      lock_guard lock(mutex());
      if( !(frame_accumulation_ = create_frame_accumulation(_stacking_options.accumulation)) ) {
        CF_ERROR("ERROR: create_frame_accumulation(stack_accumulation_options_) fails");
        return false;
      }
    }

    //////////////////

    set_pipeline_stage(stacking_stage_in_progress);


    if ( !input_sequence_->open() ) {
      set_status_msg("ERROR: input_sequence->open() fails");
      return false;
    }

    CF_DEBUG("input_sequence->size()=%d", input_sequence_->size());

    set_status_msg("RUNNING ...");



    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);


    if ( !(fOk = process_input_sequence(input_sequence_, start_pos, end_pos)) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    set_pipeline_stage(stacking_stage_finishing);
    set_status_msg("FINISHING ...");


    // Read accumulators back

    if ( frame_accumulation_ ) {

      cv::Mat accumulated_image;
      cv::Mat1b accumulated_mask;

      if ( true ) {
        lock_guard lock(mutex());

        if ( !frame_accumulation_->compute(accumulated_image, accumulated_mask) ) {
          CF_ERROR("ERROR: frame_accumulation_->compute() fails");
          return false;
        }

  #if 1
        linear_interpolation_inpaint(accumulated_image,
            accumulated_mask,
            accumulated_image);
  #else
        average_pyramid_inpaint(accumulated_image,
            accumulated_mask,
            accumulated_image);
  #endif
      }


      if ( anscombe_.method() != anscombe_none ) {
        anscombe_.inverse(accumulated_image,
            accumulated_image);
      }





      // Fix turbulent flow
  //    if ( flow_accumulation_ ) {
  //
  //      cv::Mat accumulated_flow;
  //      double sharpenScale = 0;
  //
  //      if ( !flow_accumulation_->compute(accumulated_flow) ) {
  //        CF_ERROR("ERROR: flow_accumulation_->compute() fails");
  //      }
  //      else {
  //
  //
  //        if ( master_frame_options_.accumulated_sharpen_factor > 0 ) {
  //          if ( frame_accumulation_->accumulated_frames() > 1 ) {
  //
  //            if ( output_options_.dump_reference_data_for_debug ) {
  //              write_image(ssprintf("%s/%s-before-sharpen.tiff",
  //                  output_path_.c_str(), csequence_name()),
  //                  output_options_,
  //                  accumulated_image,
  //                  accumulated_mask);
  //            }
  //
  //            sharpenScale =
  //                master_frame_options().accumulated_sharpen_factor
  //                    * sqrt(frame_accumulation_->accumulated_frames());
  //
  //            fftSharpenR1(accumulated_image, accumulated_image,
  //                sharpenScale, false);
  //
  //          }
  //        }
  //
  //        if ( accumulated_flow.size() != accumulated_image.size() ) {
  //          upscale_optflow(upscale_options_.upscale_option,
  //              accumulated_flow,
  //              accumulated_flow);
  //        }
  //
  //        if ( output_options_.dump_reference_data_for_debug ) {
  //
  //          save_image(flow_accumulation_->counter(),
  //              ssprintf("%s/%s-masterflow-counter.tiff", output_path_.c_str(),
  //                  csequence_name()));
  //
  //          save_image(accumulated_flow,
  //              ssprintf("%s/%s-masterflow.flo", output_path_.c_str(),
  //                  csequence_name()));
  //
  //          write_image(ssprintf("%s/%s-before-flow_compensation.tiff",
  //              output_path_.c_str(), csequence_name()),
  //              output_options_,
  //              accumulated_image,
  //              accumulated_mask);
  //        }
  //
  //        accumulated_flow =
  //            flow2remap(accumulated_flow,
  //                accumulated_mask);
  //
  //        // FIXME: BORDER !!!
  //        cv::remap(accumulated_image, accumulated_image, accumulated_flow,
  //            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  //
  //        if( !accumulated_mask.empty() ) {
  //
  //          cv::remap(accumulated_mask, accumulated_mask, accumulated_flow,
  //              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  //
  //          cv::compare(accumulated_mask, 255, accumulated_mask, cv::CMP_GE);
  //        }
  //
  //        if ( sharpenScale > 0 ) {
  //
  //          fftSharpenR1(accumulated_image, accumulated_image,
  //              -sharpenScale, false);
  //
  //        }
  //      }
  //    }


      output_file_name =
          generate_output_file_name();

      CF_DEBUG("Saving '%s'", output_file_name.c_str());
      if ( !write_image(_output_file_name = output_file_name, _output_options, accumulated_image, accumulated_mask) ) {
        CF_ERROR("write_image('%s') fails", output_file_name.c_str());
      }


      if ( _image_processing_options.accumulated_image_processor ) {

        if ( !_image_processing_options.accumulated_image_processor->process(accumulated_image, accumulated_mask) ) {
          CF_ERROR("post-processor '%s' fails", _image_processing_options.accumulated_image_processor->cname());
        }
        else {

          output_file_name =
              ssprintf("%s/%s%s.32F.PP.tiff",
                  output_path_.c_str(),
                  csequence_name(),
                  output_file_name_postfix_.c_str());

          CF_DEBUG("Saving '%s'", output_file_name.c_str());
          if ( !write_image(_output_file_name = output_file_name, _output_options, accumulated_image, accumulated_mask) ) {
            CF_ERROR("write_image('%s') fails", output_file_name.c_str());
          }
        }
      }

    }


    //////////////////
  }

  set_status_msg("FINISHED");

  return true;
}



bool c_image_stacking_pipeline::setup_frame_registration(const c_frame_registration::sptr & frame_registration,
    cv::Mat & reference_frame, cv::Mat & reference_mask)
{

  const c_image_registration_options & registration_options =
      frame_registration->options();

  const c_image_stacking_master_options & master_options =
      _master_options;

  if( upscale_required(frame_upscale_before_align, false) ) {

    upscale_image(upscale_options().upscale_option,
        reference_frame, reference_mask,
        reference_frame, reference_mask);
  }

  if( _output_options.debug_frame_registration ) {
    frame_registration_->set_debug_path(ssprintf("%s/debug/reference_frame",
        output_path_.c_str()));
  }

  if( _image_processing_options.ecc_image_processor && master_options.apply_input_image_processor  ) {
    frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(_image_processing_options));
  }

  if ( !frame_registration_->setup_reference_frame(reference_frame, reference_mask) ) {
    CF_ERROR("ERROR: frame_registration_->setup_reference_frame() fails");
    return false;
  }

  if( _image_processing_options.ecc_image_processor && !master_options.apply_input_image_processor ) {
    frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(_image_processing_options));
  }

  if( _output_options.debug_frame_registration ) {
    frame_registration_->set_debug_path("");
  }

  if ( _output_options.debug_frame_registration && frame_registration_->options().eccflow.enabled  ) {

    const c_eccflow &eccflow =
        frame_registration_->eccflow();

    const std::vector<c_eccflow::pyramid_entry> & pyramid =
        eccflow.current_pyramid();

    const std::string debugpath =
        ssprintf("%s/eccdebug", output_path_.c_str());

    for ( uint i = 0, n = pyramid.size(); i < n; ++i ) {
      save_image(pyramid[i].reference_image, ssprintf("%s/eccref.%02u.tiff", debugpath.c_str(), i));
    }
  }


  if( registration_options.accumulate_and_compensate_turbulent_flow ) {
    if( registration_options.motion_type > IMAGE_MOTION_EUCLIDEAN ||
        registration_options.eccflow.enabled ) {
      flow_accumulation_.reset(new c_frame_weigthed_average());
    }
  }

  return true;
}

//bool c_image_stacking_pipeline::setup_frame_accumulation()
//{
//  if ( accumulation_options_.accumulation_method != frame_accumulation_none ) {
//
//    lock_guard lock(mutex());
//
//    if ( !(frame_accumulation_ = create_frame_accumulation()) ) {
//      CF_ERROR("ERROR: create_frame_accumulation() fails");
//      return false;
//    }
//  }
//
//  return true;
//}

bool c_image_stacking_pipeline::create_reference_frame(cv::Mat & reference_frame, cv::Mat & reference_mask, double * reference_timestamp)
{
  set_status_msg("SELECT REFERENCE FRAME ...");

  c_input_sequence::sptr master_sequence;

  int master_source_index = -1;
  int master_frame_index = -1;
  int max_frames_to_stack = 0;

  bool is_external_master_file = false;

  const c_image_stacking_master_options & master_options =
      _master_options;

  std::string master_filename =
      master_options.master_selection.master_fiename;

  _generating_master_frame = true;

  CF_DEBUG("master_options.master_fiename=%s", master_filename.c_str());

  if ( master_filename.empty() ) {
    master_filename =
        input_sequence_->
            source(master_source_index = 0)->filename();
  }
  else {

    std::vector<c_input_source::sptr>::const_iterator source_pos =
        std::find_if(this->input_sequence()->sources().begin(), this->input_sequence()->sources().end(),
            [&master_options](const c_input_source::sptr & s ) -> bool {
              return s->filename() == master_options.master_selection.master_fiename;
            });

    if ( source_pos != this->input_sequence_->sources().end() ) {
      master_source_index = source_pos - this->input_sequence_->sources().begin();
    }
  }

  if ( master_source_index >= 0 ) {
    master_sequence = this->input_sequence();
    // is_external_master_file = false;
  }
  else if ( !(master_sequence = c_input_sequence::create(master_filename)) ) {
    CF_ERROR("ERROR: c_input_sequence::create(master_file_name_=%s) fails", master_filename.c_str());
    return false;
  }
  else {
    is_external_master_file = true;
    master_source_index = 0;
  }

  if ( master_source_index >= (int) master_sequence->sources().size() ) {
    CF_FATAL("ERROR: master_source_index=%d exceeds input_sequence->sources().size()=%zu",
        master_source_index, master_sequence->sources().size());
    return false;
  }

  if ( !master_sequence->source(master_source_index)->enabled() ) {
    CF_FATAL("ERROR: master_source_index=%d is NOT enabled in input_sequence",
        master_source_index);
    return false;
  }

  if ( !master_sequence->open() ) {
    CF_FATAL("ERROR: Can not open master input source '%s'",
        master_filename.c_str());
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  set_pipeline_stage(stacking_stage_select_master_frame_index);

  master_frame_index =
      select_master_frame(master_sequence);

  if ( canceled() ) {
    return false;
  }

  if ( master_frame_index <  0 ) {
    CF_ERROR("select_master_frame() fails");
    return false;
  }


  set_pipeline_stage(stacking_stage_generate_reference_frame);

  set_status_msg("CREATE REFERENCE FRAME...");

  if ( master_frame_index < 0 ) {
    master_frame_index = 0;
  }
  else if ( master_frame_index >= master_sequence->source(master_source_index)->size() ) {
    CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
        master_frame_index, master_filename.c_str());
    return false;
  }

  int master_frame_in_source_index =
      master_frame_index;

  int master_frame_pos =
      master_sequence->global_pos(
          master_source_index,
          master_frame_index);

  CF_DEBUG("master_source_index=%d master_frame_in_source_index=%d master_frame_pos_=%d",
      master_source_index, master_frame_in_source_index, master_frame_pos);


  if ( master_options.generate_master_frame ) {
    max_frames_to_stack =
        master_options.max_frames_to_generate_master_frame;
  }

  CF_DEBUG("master_frame_pos=%d max_frames_to_stack=%d",
      master_frame_pos, max_frames_to_stack);

  if( !create_reference_frame(master_sequence, is_external_master_file, master_frame_pos,
      max_frames_to_stack, reference_frame, reference_mask,
      reference_timestamp) ) {
    CF_FATAL("ERROR: create_reference_frame() fails");
    return false;
  }

  _generating_master_frame = false;

  CF_DEBUG("Reference frame : %dx%d depth=%d channels=%d",
      reference_frame.cols, reference_frame.rows,
      reference_frame.depth(),
      reference_frame.channels());

  CF_DEBUG("Reference mask : %dx%d depth=%d channels=%d",
      reference_mask.cols, reference_mask.rows,
      reference_mask.depth(),
      reference_mask.channels());

//  ecc_normalization_noise_ =
//      compute_image_noise(reference_frame, reference_mask,
//          master_options.registration.registration_channel);

//  if ( master_options.save_master_frame ) {
//    write_image(ssprintf("%s/%s-master.tiff", output_path_.c_str(), csequence_name()),
//        output_options_,
//        reference_frame,
//        reference_mask);
//  }


  return true;
}

bool c_image_stacking_pipeline::create_reference_frame(const c_input_sequence::sptr & input_sequence, bool is_external_master_file,
    int master_frame_pos, int max_frames_to_stack,
    cv::Mat & reference_frame,
    cv::Mat & reference_mask,
    double * output_reference_timestamp)
{
  INSTRUMENT_REGION("");

  const c_image_stacking_master_options & master_options =
      _master_options;

  if( !input_sequence->seek(master_frame_pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(master_frame_pos=%d) fails", master_frame_pos);
    return false;
  }

  if( canceled() ) {
    return false;
  }

  const bool save_raw_bayer_image =
      _master_options.accumulation.accumulation_method ==
          frame_accumulation_bayer_average;

  if( !read_input_frame(input_sequence, reference_frame, reference_mask, is_external_master_file, save_raw_bayer_image) ) {
    CF_FATAL("read_input_frame(reference_frame) fails for master_frame_pos=%d",
        master_frame_pos);
    return false;
  }

  * output_reference_timestamp =
      input_sequence->last_ts();

  if( canceled() ) {
    return false;
  }

  if( reference_frame.empty() ) {
    CF_ERROR("read_input_frame(reference_frame) returns empty frame for master_frame_pos=%d",
        master_frame_pos);
    return false;
  }


  if( !select_image_roi(roi_selection_, reference_frame, reference_mask, reference_frame, reference_mask) ) {
    CF_FATAL("select_image_roi(reference_frame) fails");
    return false;
  }

  if( canceled() ) {
    return false;
  }

  if( master_options.apply_input_image_processor && _input_options.input_image_processor ) {
    // lock_guard lock(mutex());
    if( !_input_options.input_image_processor->process(reference_frame, reference_mask) ) {
      CF_ERROR("input_image_processor->process(reference_frame) fails");
      return false;
    }
  }

  if( canceled() ) {
    return false;
  }

  if( reference_frame.channels() > 1 && master_options.master_channel != color_channel_dont_change ) {

    if( !extract_channel(reference_frame, reference_frame, cv::noArray(), cv::noArray(),
        master_options.master_channel) ) {
      CF_ERROR("extract_channel(master_channel=%d) fails", master_options.master_channel);
      return false;
    }

    if( canceled() ) {
      set_status_msg("canceled");
      return false;
    }
  }

  if( max_frames_to_stack < 2 || input_sequence->size() < 2 ) {
    // Use single frame as reference
  }
  else {

    // Generate from sequence

    if( !(frame_registration_ = create_frame_registration(master_options.registration)) ) {
      CF_FATAL("create_frame_registration(master_registration_options) fails");
      return false;
    }

    if( _image_processing_options.ecc_image_processor && master_options.apply_input_image_processor ) {
      frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(_image_processing_options));
    }

    if( !frame_registration_->setup_reference_frame(reference_frame, reference_mask) ) {
      CF_FATAL("frame_registration_->setup_referece_frame() fails");
      return false;
    }

    if( _image_processing_options.ecc_image_processor && !master_options.apply_input_image_processor ) {
      frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(_image_processing_options));
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !(frame_accumulation_ = create_frame_accumulation(_master_options.accumulation)) ) {
        CF_ERROR("create_frame_accumulation(master_accumulation_options_) fails");
        return false;
      }
    }

    if( canceled() ) {
      return false;
    }

    const int start_frame_index =
        std::max(0, _input_options.start_frame_index);

    int startpos, endpos;

    if( start_frame_index + max_frames_to_stack >= input_sequence->size() ) {
      startpos = start_frame_index;
      endpos = input_sequence->size();
    }
    else {

      startpos =
          std::max(start_frame_index,
              master_frame_pos - max_frames_to_stack / 2);

      if( (endpos = startpos + max_frames_to_stack) >= input_sequence->size() ) {
        startpos = std::max(start_frame_index, input_sequence->size() - max_frames_to_stack);
        endpos = input_sequence->size();
      }
    }

    if( max_frames_to_stack > input_sequence->size() ) {
      max_frames_to_stack = input_sequence->size();
    }

    CF_DEBUG("input_options.start_frame_index=%d \n"
        "master_frame_pos=%d \n"
        "max_frames_to_stack=%d \n"
        "input_sequence->size()=%d\n"
        "startpos=%d\n"
        "endpos=%d\n",
        _input_options.start_frame_index,
        master_frame_pos,
        max_frames_to_stack,
        input_sequence->size(),
        startpos,
        endpos);

    if( !process_input_sequence(input_sequence, startpos, endpos) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    // Reset master index indicator because master frame was generated from a sequence, not just a single frame

    if( canceled() ) {
      return false;
    }

    // Read accumulators back
    if( true ) {
      lock_guard lock(mutex());

      if( frame_accumulation_->accumulated_frames() < 1 ) {
        CF_ERROR("ERROR: No frames accumulated for reference frame");
        return false;
      }

      if( !frame_accumulation_->compute(reference_frame, reference_mask) ) {
        CF_ERROR("ERROR: frame_accumulation_->compute() fails");
        return false;
      }

      linear_interpolation_inpaint(reference_frame,
          reference_mask,
          reference_frame);

    }

    if( canceled() ) {
      return false;
    }
  }

  if( master_options.generate_master_frame && master_options.save_master_frame ) {
    write_image(ssprintf("%s/%s-master.tiff", output_path_.c_str(), csequence_name()),
        _output_options,
        reference_frame,
        reference_mask);
  }

  if( true ) {
    lock_guard lock(mutex());

    flow_accumulation_.reset();
    frame_accumulation_.reset();
    frame_registration_.reset();

    if( master_options.unsharp_sigma > 0 && master_options.unsharp_alpha > 0 ) {

      unsharp_mask(reference_frame, reference_frame,
          master_options.unsharp_sigma,
          master_options.unsharp_alpha);
    }

  }

  return true;
}


bool c_image_stacking_pipeline::process_input_sequence(const c_input_sequence::sptr & input_sequence, int startpos, int endpos)
{
  INSTRUMENT_REGION("");

  cv::Mat current_frame, current_mask, current_weights;
  cv::Mat2f current_remap;

  const c_image_registration_options & registration_options =
      _generating_master_frame ? _master_options.registration :
          _stacking_options.registration;

  const c_frame_accumulation_options & accopts =
      _generating_master_frame ? _master_options.accumulation :
          _stacking_options.accumulation;

  const bool save_raw_bayer_image =
      accopts.accumulation_method == frame_accumulation_bayer_average;

  if ( !input_sequence->seek(startpos) ) {
    CF_ERROR("input_sequence->seek(startpos=%d) fails", startpos);
    return false;
  }

  if ( input_sequence->is_live() ) {
    total_frames_ = INT_MAX;
  }
  else {
    total_frames_ = endpos - startpos;
    if ( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1", total_frames_);
      return false;
    }
  }

  processed_frames_ = 0;
  accumulated_frames_ = 0;


  CF_DEBUG("total_frames_=%d", total_frames_);

  for ( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    double t0, t1, start_time, total_time;
    double time_read = 0;
    double time_upscale = 0;
    double time_register = 0;
    double time_remap = 0;
    double time_accumulate = 0;
    double time_select_roi = 0;

    CF_DEBUG("_generating_master_frame=%d", _generating_master_frame);

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    t0 = start_time = get_realtime_ms();

    if( is_bad_frame_index(input_sequence->current_pos()) ) {
      CF_DEBUG("Skip frame %d as blacklisted", input_sequence->current_pos());
      input_sequence->seek(input_sequence->current_pos() + 1);
      continue;
    }

    if( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !read_input_frame(input_sequence, current_frame, current_mask, false, save_raw_bayer_image) ) {
      set_status_msg("read_input_frame() fails");
      break;
    }

    time_read = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( current_frame.empty() ) {
      continue;
    }

    if ( !select_image_roi(roi_selection_, current_frame, current_mask, current_frame, current_mask) ) {
      continue;
    }

    time_select_roi = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if( _input_options.input_image_processor ) {
      if( !_generating_master_frame || _master_options.apply_input_image_processor ) {
        // lock_guard lock(mutex());
        if( !_input_options.input_image_processor->process(current_frame, current_mask) ) {
          CF_ERROR("input_image_processor->process(current_frame) fails");
          return false;
        }
      }
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if( _output_options.save_preprocessed_frames ) {

      if( !save_preprocessed_video(current_frame, current_mask, input_sequence->current_pos() - 1) ) {
        CF_ERROR("save_preprocessed_video() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if( _generating_master_frame && current_frame.channels() > 1 ) {

      if( _master_options.master_channel != color_channel_dont_change ) {

        const bool fOk =
            extract_channel(current_frame, current_frame,
                cv::noArray(), cv::noArray(),
                _master_options.master_channel);

        if( !fOk ) {
          CF_ERROR("extract_channel(master_channel=%d) fails",
              _master_options.master_channel);
          return false;
        }

        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }
    }



    if( weights_required(accopts) ) {
      compute_weights(accopts, current_frame, current_mask, current_weights);
      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if ( upscale_required(frame_upscale_before_align, _generating_master_frame) ) {

      upscale_image(_upscale_options.upscale_option,
          current_frame, current_mask,
          current_frame, current_mask);

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !current_weights.empty() ) {
        upscale_image(_upscale_options.upscale_option,
            current_weights, cv::noArray(),
            current_weights, cv::noArray());
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }


    time_upscale = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    /////////////////////////////////////

    if ( frame_registration_ ) {

      frame_registration_->set_current_timestamp(input_sequence->last_ts(),
          input_sequence->has_last_ts());

      if( _output_options.debug_frame_registration ) {

        if( _output_options.debug_frame_registration_frame_indexes.empty() ) {
          frame_registration_->set_debug_path(ssprintf("%s/debug/registration-%d",
              output_path_.c_str(), input_sequence->current_pos() - 1));
        }
        else {

          const std::vector<int>::const_iterator pos =
              std::find(_output_options.debug_frame_registration_frame_indexes.begin(),
                  _output_options.debug_frame_registration_frame_indexes.end(),
                  input_sequence->current_pos() - 1);

          if( pos == _output_options.debug_frame_registration_frame_indexes.end() ) {
            frame_registration_->set_debug_path("");
          }
          else {
            frame_registration_->set_debug_path(ssprintf("%s/debug/registration-%d", output_path_.c_str(), *pos));
          }
        }

        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }


      const bool registered =
          frame_registration_->register_frame(current_frame,
              current_mask);

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if ( !save_sparse_matches_video(input_sequence->current_pos() - 1) )  {
        CF_ERROR("save_sparse_matches_video() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !registered ) {
        CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + startpos);
        continue;
      }

      if ( !save_sparse_matches_blend_video(input_sequence->current_pos() - 1) ) {
        CF_ERROR("[F %6d] save_sparse_matches_video() fails\n",
            processed_frames_ + startpos);
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }


      if( flow_accumulation_ ) {


        const cv::Mat2f turbulence =
            compute_turbulent_flow(frame_registration_->image_transform().get(),
                frame_registration_->current_remap());

        if( turbulence.empty() ) {
          CF_ERROR("compute_turbulence_flow() fails");
        }
        else {

//          if( flow_accumulation_->accumulated_frames() < 1 ) {
//
//            const bool fOk =
//                flow_accumulation_->initialze(turbulence.size(),
//                    turbulence.type(),
//                    CV_8UC1);
//
//            if( !fOk ) {
//              CF_ERROR("flow_accumulation_->initialze() fails");
//              break;
//            }
//          }

//          CF_DEBUG("H: turbulence=%dx%d", turbulence.cols, turbulence.rows);
          if( !flow_accumulation_->add(turbulence) ) {
            CF_ERROR("flow_accumulation_->add(turbulence) fails");
            break;
          }
        }

        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }

      if( !upscale_required(frame_upscale_after_align, _generating_master_frame) ) {
        current_remap = frame_registration_->current_remap();
      }
      else {
        current_remap.release();
        upscale_remap(_upscale_options.upscale_option,
            frame_registration_->current_remap(),
            current_remap);
      }


      frame_registration_->custom_remap(current_remap,
          current_frame, current_frame,
          current_mask, current_mask,
          registration_options.interpolation,
          _generating_master_frame ?
              ECC_BORDER_REFLECT101 :
              registration_options.border_mode,
          registration_options.border_value);

      if( !current_weights.empty() ) {

        frame_registration_->custom_remap(current_remap,
            current_weights, current_weights,
            cv::noArray(), cv::noArray(),
            registration_options.interpolation,
            ECC_BORDER_CONSTANT);
      }


      if( !save_aligned_video(current_frame, current_mask, input_sequence->current_pos() - 1) ) {
        CF_ERROR("save_aligned_frame() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !save_ecc_video(input_sequence->current_pos() - 1) ) {
        CF_ERROR("save_ecc_video() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !save_eccflow_video(input_sequence->current_pos() - 1) ) {
        CF_ERROR("save_eccflow_video() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( _image_processing_options.aligned_image_processor ) {
        if( !_image_processing_options.aligned_image_processor->process(current_frame, current_mask) ) {
          CF_ERROR("aligned_image_processor->process() fails");
          return false;
        }
        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }

    }

    time_register = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    /////////////////////////////////////////////////////////////////////////////////

    if( !current_weights.empty() ) {

      if ( !current_mask.empty() ) {
        multiply_weights(current_mask, current_weights, current_weights,
            current_mask.depth() == CV_8U ?
                1. / 255 : 1,
            CV_32F);
      }

      current_mask = current_weights;
    }
    /////////////////////////////////////////////////////////////////////////////////

    if( !save_accumulation_masks_video(current_frame, current_mask, input_sequence->current_pos() - 1) ) {
      CF_ERROR("save_accumulation_masks_video() fails");
      return false;
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    /////////////////////////////////////////////////////////////////////////////////
    if ( frame_accumulation_ ) {

      if( c_bayer_average *bayer_average = dynamic_cast<c_bayer_average*>(frame_accumulation_.get()) ) {

        lock_guard lock(mutex());

        if( bayer_average->accumulated_frames() < 1 ) {
          bayer_average->set_bayer_pattern(raw_bayer_colorid_);
          // bayer_average->initialze(raw_bayer_image_.size());
        }


        static const cv::Mat2f empty_remap;
        bayer_average->set_remap(frame_registration_ ? frame_registration_->current_remap() : empty_remap);
        bayer_average->add(raw_bayer_image_, current_mask);

      }
      else {

        lock_guard lock(mutex());

//        if( frame_accumulation_->accumulated_frames() < 1 ) {
//
//          const bool fok =
//              frame_accumulation_->initialze(current_frame.size(),
//                  current_frame.type(),
//                  current_mask.empty() ? CV_8UC1 :
//                      current_mask.type());
//
//          if( !fok ) {
//            CF_ERROR("frame_accumulation_->initialze() fails");
//            return false;
//          }
//        }

        if ( !frame_accumulation_->add(current_frame, current_mask) ) {
          CF_ERROR("frame_accumulation_->add(current_frame) fails");
          return false;
        }

      }

      accumulated_frames_ =
          frame_accumulation_->accumulated_frames();

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }


      if( _output_options.save_incremental_frames ) {

        cv::Mat accumulated_frame, accumulated_mask;

        if( true ) {
          lock_guard lock(mutex());
          if( !frame_accumulation_->compute(accumulated_frame, accumulated_mask) ) {
            CF_ERROR("ERROR: frame_accumulation->compute() fails");
          }
        }

        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }

        if( !accumulated_frame.empty() ) {

          const c_image_processor::sptr &proc =
              _image_processing_options.incremental_frame_processor;

          if( proc && !proc->process(accumulated_frame, accumulated_mask) ) {
            CF_ERROR("image processor '%s' fails", proc->cname());
            return false;
          }

          if( !save_incremental_video(accumulated_frame, accumulated_mask, input_sequence->current_pos() - 1) ) {
            CF_ERROR("save_incremental_video() fails");
            return false;
          }

          if( canceled() ) {
            set_status_msg("canceled");
            break;
          }
        }
      }
    }

    /////////////////////////////////////


    time_accumulate = (t1 = get_realtime_ms()) - t0, t0 = t1;
    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    ///////////////

    total_time = get_realtime_ms() - start_time;

    CF_DEBUG("[F %d / %5d / %5d / %6d] OK  %g ms\n"
        "read    : %g ms\n"
        "roi     : %g ms\n"
        "upscale : %g ms\n"
        "register: %g ms\n"
        "remap   : %g ms\n"
        "process : %g ms\n"
        "-----------\n\n\n",
        processed_frames_ + startpos, processed_frames_, accumulated_frames_, total_frames_, total_time,

        time_read,
        time_select_roi,
        time_upscale,
        time_register,
        time_remap,
        time_accumulate);

    on_status_update();
  }

  CF_DEBUG("Leave");

  return true;
}


bool c_image_stacking_pipeline::read_input_frame(const c_input_sequence::sptr & input_sequence,
    cv::Mat & output_image, cv::Mat & output_mask,
    bool is_external_master_frame,
    bool save_raw_bayer_image) const
{
  INSTRUMENT_REGION("");

  //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

  if( !is_external_master_frame ) {

    if( !darkbayer_.empty() ) {

      if( darkbayer_.size() != output_image.size() || darkbayer_.channels() != output_image.channels() ) {
        CF_FATAL("darkbayer (%dx%d*%d) and input frame (%dx%d*%d) not match",
            darkbayer_.cols, darkbayer_.rows, darkbayer_.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::subtract(output_image, darkbayer_,
          output_image);
    }

    if( !flatbayer_.empty() ) {

      if( flatbayer_.size() != output_image.size() || flatbayer_.channels() != output_image.channels() ) {
        CF_FATAL("flatbayer_ (%dx%d*%d) and input frame (%dx%d*%d) not match",
            flatbayer_.cols, flatbayer_.rows, flatbayer_.channels(),
            output_image.cols, output_image.rows, output_image.channels());
        return false;
      }

      if( output_image.depth() != CV_32F ) {
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      cv::divide(output_image, flatbayer_,
          output_image, output_image.depth());
    }

    if ( _input_options.enable_bground_normalization ) {
      nomalize_image_histogramm(output_image, output_mask, output_image,
          _input_options.background_normalization_options,
          input_sequence->colorid());
    }
  }

  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if( _input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }

    if ( _input_options.filter_bad_pixels ) {
      remove_bad_pixels(output_image, _input_options, false);
    }

    if( output_image.depth() != CV_32F ) {
      output_image.convertTo(output_image, CV_32F,
          1. / ((1 << input_sequence->bpp())));
    }

  }
  else {

    if( _input_options.detect_bad_asi_frames ) {

      cv::Mat tmp;
      if ( !extract_bayer_planes(output_image, tmp, input_sequence->colorid()) ) {
        CF_ERROR("extract_bayer_planes() fails");
        output_image.release();
        return false;
      }

      CF_DEBUG("Check for corrupted_asi_frame");

      if ( is_corrupted_asi_frame(tmp) ) {
        CF_ERROR("CORRUPTED ASI FRAME DETECTED");
        output_image.release();
        return true; // return true with empty output image
      }

      CF_DEBUG("Check OK");

    }

    const DEBAYER_ALGORITHM algo =
        _input_options.debayer_method;

    if ( save_raw_bayer_image /*accumulation_options_.accumulation_method == frame_accumulation_bayer_average*/ ) {

      raw_bayer_colorid_ =
          input_sequence->colorid();

      if( output_image.depth() == CV_32F ) {
        output_image.copyTo(raw_bayer_image_);
      }
      else {
        output_image.convertTo(raw_bayer_image_, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      if( _input_options.filter_bad_pixels && _input_options.bad_pixels_variation_threshold > 0 ) {
        if( !bayer_denoise(raw_bayer_image_, _input_options.bad_pixels_variation_threshold) ) {
          CF_ERROR("bayer_denoise() fails");
          return false;
        }
      }
    }


    switch (algo) {

      case DEBAYER_DISABLE:
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, input_sequence->colorid(), algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
//        if( _input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
//          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
//          output_image.release();
//          return true; // return true with empty output image
//        }
        if ( _input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, _input_options, true);
        }
        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }
        if( _input_options.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if( _input_options.filter_bad_pixels ) {
          remove_bad_pixels(output_image, _input_options, true);
        }

        if( output_image.depth() != CV_32F ) {
          output_image.convertTo(output_image, CV_32F,
              1. / ((1 << input_sequence->bpp())));
        }

        if ( !nninterpolation(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("nninterpolation() fails");
          return false;
        }

        break;

      default:
        CF_ERROR("APP BUG: unknown debayer algorithm %d ('%s') specified",
            algo, toCString(algo));
        return false;
    }
  }

  if( _input_options.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
    cv::transform(output_image, output_image,
        input_sequence->color_matrix());
  }

  if ( anscombe_.method() != anscombe_none ) {
    anscombe_.apply(output_image, output_image);
  }

  if ( !missing_pixel_mask_.empty() ) {

    if ( output_image.size() != missing_pixel_mask_.size() ) {

      CF_ERROR("Invalid input: "
          "frame and bad pixel mask sizes not match:\n"
          "frame size: %dx%d\n"
          "mask size : %dx%d",
          output_image.cols, output_image.rows,
          missing_pixel_mask_.cols, missing_pixel_mask_.rows);

      return false;
    }

    if ( output_mask.empty() ) {
      missing_pixel_mask_.copyTo(output_mask);
    }
    else {
      cv::bitwise_and(output_mask, missing_pixel_mask_,
          output_mask);
    }
  }

  if ( !output_mask.empty() && _input_options.inpaint_missing_pixels ) {
#if 1
    linear_interpolation_inpaint(output_image, output_mask,
        output_image);
#else
    average_pyramid_inpaint(output_image, output_mask,
        output_image);
#endif
  }


  return true;
}

bool c_image_stacking_pipeline::select_image_roi(const c_roi_selection::ptr & roi_selection,
    const cv::Mat & src, const cv::Mat & srcmask,
    cv::Mat & dst, cv::Mat & dstmask)
{
  if ( !roi_selection ) {
    if ( &src != &dst  ) {
      dst = src;
    }
    if ( &srcmask != &dstmask ) {
      dstmask = srcmask;
    }
    return true;
  }

  INSTRUMENT_REGION("");

  cv::Rect ROI;

  if ( !roi_selection->select(src, srcmask, ROI) || ROI.empty() ) {
    CF_ERROR("roi_selection->select_roi() fails");
    return false;
  }


  dst = src(ROI);

  if ( !srcmask.empty() ) {
    dstmask = srcmask(ROI);
  }
  else {
    dstmask.release();
  }

  return true;
}


void c_image_stacking_pipeline::remove_bad_pixels(cv::Mat & image,
    const c_image_stacking_input_options & input_optons,
    bool isbayer)
{
  INSTRUMENT_REGION("");

  cv::Mat medianImage, variationImage, meanVariationImage;
  double minimal_mean_variation_for_very_smooth_images;

  // threshold = estimate_noise(image);
  if ( image.depth() == CV_32F || image.depth() == CV_64F ) {
    minimal_mean_variation_for_very_smooth_images = 1e-3;
  }
  else {
    minimal_mean_variation_for_very_smooth_images = 1;
  }

  cv::medianBlur(image, medianImage, isbayer ? 3 : 5);
  cv::absdiff(image, medianImage, variationImage);

  static float K[3*3] = {
      1./8, 1./8, 1./8,
      1./8, 0.0,  1./8,
      1./8, 1./8, 1./8,
  };

  static const cv::Mat1f SE(3,3, K);

  cv::filter2D(variationImage, meanVariationImage, -1, SE);
  cv::max(meanVariationImage, minimal_mean_variation_for_very_smooth_images, meanVariationImage);

  medianImage.copyTo(image, variationImage > input_optons.bad_pixels_variation_threshold * meanVariationImage);
}


void c_image_stacking_pipeline::upscale_remap(enum frame_upscale_option scale,
    cv::InputArray srcmap,
    cv::OutputArray dstmap)
{
  INSTRUMENT_REGION("");

  switch (scale) {
  case frame_upscale_x15:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3 / 2, srcmap.rows() * 3 / 2);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      cv::pyrUp(srcmap, dstmap);
    }
    break;
  }
  case frame_upscale_x30:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3, srcmap.rows() * 3);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    break;
  }
  default:
    if( !srcmap.empty() && dstmap.needed() ) {
      srcmap.copyTo(dstmap);
    }
    break;
  }
}

void c_image_stacking_pipeline::upscale_optflow(enum frame_upscale_option scale,
    cv::InputArray srcmap,
    cv::OutputArray dstmap)
{
  INSTRUMENT_REGION("");

  switch (scale) {
  case frame_upscale_x15:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3 / 2, srcmap.rows() * 3 / 2);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR);
      cv::multiply(dstmap, 3.0 / 2.0, dstmap);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      cv::pyrUp(srcmap, dstmap);
      cv::multiply(dstmap, 2.0, dstmap);
    }
    break;
  }
  case frame_upscale_x30:
  {
    if( !srcmap.empty() && dstmap.needed() ) {
      const cv::Size dstsize(srcmap.cols() * 3, srcmap.rows() * 3);
      cv::resize(srcmap, dstmap, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
      cv::multiply(dstmap, 3.0, dstmap);
    }
    break;
  }
  default:
    if( !srcmap.empty() && dstmap.needed() ) {
      srcmap.copyTo(dstmap);
    }
    break;
  }
}


void c_image_stacking_pipeline::upscale_image(enum frame_upscale_option scale,
    cv::InputArray src, cv::InputArray srcmask,
    cv::OutputArray dst, cv::OutputArray dstmask)
{
  INSTRUMENT_REGION("");

  switch (scale) {
  case frame_upscale_x15:
  {
    const cv::Size dstsize(src.cols() * 3 / 2, src.rows() * 3 / 2);
    if( !src.empty() && dst.needed() ) {
      cv::resize(src, dst, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::resize(srcmask, dstmask, dstsize, 0, 0, cv::INTER_LINEAR);
    }
    break;
  }
  case frame_upscale_pyrUp:
  {
    if( !src.empty() && dst.needed() ) {
      cv::pyrUp(src, dst);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::pyrUp(srcmask, dstmask);
    }
    break;
  }
  case frame_upscale_x30:
  {
    const cv::Size dstsize(src.cols() * 3, src.rows() * 3);
    if( !src.empty() && dst.needed() ) {
      cv::resize(src, dst, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      cv::resize(srcmask, dstmask, dstsize, 0, 0, cv::INTER_LINEAR_EXACT);
    }
    break;
  }
  default:
    if( !src.empty() && dst.needed() ) {
      src.copyTo(dst);
    }
    if( !srcmask.empty() && dstmask.needed() ) {
      srcmask.copyTo(dstmask);
    }
    return;
  }

  if( !srcmask.empty() && dstmask.needed() ) {
    cv::compare(dstmask.getMatRef(), 255, dstmask.getMatRef(), cv::CMP_GE);
  }

}

bool c_image_stacking_pipeline::weights_required(const c_frame_accumulation_options & opts) const
{
  return opts.accumulation_method == frame_accumulation_weighted_average &&
      opts.lpg.dscale >= 0 && opts.lpg.k >= 0;
}

void c_image_stacking_pipeline::compute_weights(const c_frame_accumulation_options & opts, const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  c_lpg_sharpness_measure::create_map(src, dst, opts.lpg);
  if ( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }
}

void c_image_stacking_pipeline::compute_relative_weights(const cv::Mat & wc, const cv::Mat & mc, const cv::Mat & wref, cv::Mat & wrel)
{
  INSTRUMENT_REGION("");

  cv::divide(wc, wref, wrel);
  if ( mc.size() == wrel.size() ) {
    wrel.setTo(0, ~mc);
  }

}


double c_image_stacking_pipeline::compute_image_noise(const cv::Mat & image, const cv::Mat & mask,
    color_channel_type channel)
{
  INSTRUMENT_REGION("");

  double estimated_noise_level;

  if ( image.channels() == 1 ) {
    estimated_noise_level = estimate_noise(image, cv::noArray(), mask)[0];
  }
  else {
    cv::Mat tmp;
    extract_channel(image, tmp, cv::noArray(), cv::noArray(), channel >= 0 ? channel : color_channel_gray);
    estimated_noise_level = estimate_noise(tmp, cv::noArray(), mask)[0];
  }

  return estimated_noise_level;
}

bool c_image_stacking_pipeline::get_display_image(cv::OutputArray dst, cv::OutputArray dst_mask)
{
  lock_guard lock(mutex());

  switch (pipeline_stage_) {
    case stacking_stage_idle:
      break;

    case stacking_stage_initialize:
      break;

    case stacking_stage_select_master_frame_index:
      if( dst.needed() ) {
        selected_master_frame_.copyTo(dst);
      }

      if( dst_mask.needed() ) {
        selected_master_frame_mask_.copyTo(dst_mask);
      }

      return true;

    case stacking_stage_generate_reference_frame:
      case stacking_stage_in_progress:
      case stacking_stage_finishing:
      if( frame_accumulation_ && frame_accumulation_->accumulated_frames() > 0 ) {
        if( !frame_accumulation_->compute(dst, dst_mask) ) {
          CF_ERROR("frame_accumulation_->compute() fails");
          return false;
        }
        if( !dst.empty() && anscombe().method() != anscombe_none ) {
          anscombe().inverse(dst.getMatRef(), dst.getMatRef());
        }
        return true;
      }
      break;
  }

  return false;
}



bool c_image_stacking_pipeline::upscale_required(frame_upscale_stage current_stage, bool generating_master_frame) const
{
  if ( generating_master_frame ) {
    return false;
  }

  const c_frame_upscale_options & opts =
      upscale_options();

  return opts.upscale_option != frame_upscale_none  && current_stage == opts.upscale_stage;
}



bool c_image_stacking_pipeline::write_image(const std::string & output_file_name,
    const c_image_stacking_output_options & output_options,
    const cv::Mat & output_image, const cv::Mat & output_mask)
{
  INSTRUMENT_REGION("");

  cv::Mat image_to_write;

  if ( !output_options.write_image_mask_as_alpha_channel || output_mask.empty() || (output_image.channels() != 3 && output_image.channels() != 1)  ) {
    image_to_write = output_image;
  }
  else if ( !mergebgra(output_image, output_mask, image_to_write) ) {
    CF_ERROR("ERROR: mergebgra() fails");
    return false;
  }

  CF_DEBUG("image_to_write: %dx%d channels=%d depth=%d output_mask: %dx%d channels=%d depth=%d",
      image_to_write.cols, image_to_write.rows, image_to_write.channels(), image_to_write.depth(),
      output_mask.cols, output_mask.rows, output_mask.channels(), output_mask.depth());

  return save_image(image_to_write, output_file_name);
}


int c_image_stacking_pipeline::select_master_frame(const c_input_sequence::sptr & input_sequence)
{
  INSTRUMENT_REGION("");

  int selected_master_frame_index = 0;

  selected_master_frame_.release();
  selected_master_frame_mask_.release();

  switch (_master_options.master_selection.master_selection_method) {

    case master_frame_specific_index:
      selected_master_frame_index = _master_options.master_selection.master_frame_index;
      break;

    case master_frame_middle_index:
      selected_master_frame_index = input_sequence->size() / 2;
      break;

    case master_frame_best_of_100_in_middle: {

//      c_lpg_sharpness_measure measure;
//
//      measure.set_k(6);
//      measure.set_dscale(1);
//      measure.set_uscale(6);
//      measure.set_avgchannel(true);
//      measure.set_squared(false);

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

      total_frames_ = end_pos - start_pos;
      processed_frames_ = 0;
      accumulated_frames_ = 0;

      on_frame_processed();

      for( current_index = 0; processed_frames_ < total_frames_;
          processed_frames_ = ++current_index, on_frame_processed() ) {

        if ( canceled() ) {
          CF_DEBUG("cancel requested");
          return -1;
        }

        if( is_bad_frame_index(input_sequence->current_pos()) ) {
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
            image.copyTo(selected_master_frame_);
            mask.copyTo(selected_master_frame_mask_);
          }

          set_status_msg(ssprintf("SELECT REFERENCE FRAME...\n"
              "BEST: INDEX=%d METRIC: %g",
              best_index + start_pos,
              best_metric));

          //  on_selected_master_frame_changed();
        }

//////
      }

      selected_master_frame_index = best_index + start_pos;
      input_sequence->seek(backup_current_pos);

      break;
    }
  }

  return selected_master_frame_index;
}


bool c_image_stacking_pipeline::serialize(c_config_setting settings, bool save)
{
  static const auto get_group =
      [](c_config_setting setting, bool save, const std::string & name) {
        return save ? setting.add_group(name) : setting[name];
      };

  if ( !base::serialize(settings, save) ) {
    CF_ERROR("base::serialize(save=%d) fails", save);
    return false;
  }

  c_config_setting section, subsection, subsubsection;

  if( (section = get_group(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
    SERIALIZE_OPTION(section, save, _input_options, anscombe);
  }

  if( (section = get_group(settings, save, "roi")) ) {
    SERIALIZE_OPTION(section, save, _roi_selection_options, method);
    SERIALIZE_OPTION(section, save, _roi_selection_options, rectangle_roi_selection);
    SERIALIZE_OPTION(section, save, _roi_selection_options, planetary_disk_crop_size);
    SERIALIZE_OPTION(section, save, _roi_selection_options, planetary_disk_gbsigma);
    SERIALIZE_OPTION(section, save, _roi_selection_options, planetary_disk_stdev_factor);
    SERIALIZE_OPTION(section, save, _roi_selection_options, se_close_size);
  }

  // c_frame_upscale_options upscale_options_;
  if( (section = get_group(settings, save, "upscale")) ) {
    SERIALIZE_OPTION(section, save, _upscale_options, upscale_option);
    SERIALIZE_OPTION(section, save, _upscale_options, upscale_stage);
  }

  if( (section = get_group(settings, save, "master_frame_options")) ) {

    c_image_stacking_master_options & opts =
        _master_options;


    SERIALIZE_OPTION(section, save, opts, apply_input_image_processor);
    SERIALIZE_OPTION(section, save, opts, generate_master_frame);
    SERIALIZE_OPTION(section, save, opts, stop_after_master_frame_generation);
    SERIALIZE_OPTION(section, save, opts, save_master_frame);
    SERIALIZE_OPTION(section, save, opts, max_frames_to_generate_master_frame);
    SERIALIZE_OPTION(section, save, opts, unsharp_sigma);
    SERIALIZE_OPTION(section, save, opts, unsharp_alpha);
    SERIALIZE_OPTION(section, save, opts, master_channel);

    if( (subsection = get_group(section, save, "master_frame_selection")) ) {
      SERIALIZE_OPTION(subsection, save, opts.master_selection, master_selection_method);
      SERIALIZE_OPTION(subsection, save, opts.master_selection, master_fiename);
      SERIALIZE_OPTION(subsection, save, opts.master_selection, master_frame_index);
    }

    if( (subsection = get_group(section, save, "registration")) ) {

      SERIALIZE_OPTION(subsection, save, opts.registration, enabled);
      SERIALIZE_OPTION(subsection, save, opts.registration, motion_type);
      SERIALIZE_OPTION(subsection, save, opts.registration, accumulate_and_compensate_turbulent_flow);
      SERIALIZE_OPTION(subsection, save, opts.registration, ecc_registration_channel);
      SERIALIZE_OPTION(subsection, save, opts.registration, interpolation);
      SERIALIZE_OPTION(subsection, save, opts.registration, border_mode);
      SERIALIZE_OPTION(subsection, save, opts.registration, border_value);


      if( (subsubsection = get_group(subsection, save, "feature_registration")) ) {

        c_feature_registration_options & feature_registration =
            opts.registration.feature_registration;

        SERIALIZE_OPTION(subsubsection, save, feature_registration, enabled);
        SERIALIZE_OPTION(subsubsection, save, feature_registration, scale);
        SERIALIZE_OPTION(subsection, save, feature_registration, registration_channel);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_detector"), save,
            feature_registration.sparse_feature_extractor_and_matcher, detector);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_descriptor"), save,
            feature_registration.sparse_feature_extractor_and_matcher, descriptor);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_matcher"), save,
            feature_registration.sparse_feature_extractor_and_matcher, matcher);

        SERIALIZE_OPTION(get_group(subsubsection, save, "estimate_options"), save,
            feature_registration, estimate_options);
      }

      if( (subsubsection = get_group(subsection, save, "ecc")) ) {

        c_ecc_registration_options & ecc =
            opts.registration.ecc;

        SERIALIZE_OPTION(subsubsection, save, ecc, enabled);
        SERIALIZE_OPTION(subsubsection, save, ecc, scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, eps);
        SERIALIZE_OPTION(subsubsection, save, ecc, min_rho);
        SERIALIZE_OPTION(subsubsection, save, ecc, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, update_step_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_noise);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecc_method);
        SERIALIZE_OPTION(subsubsection, save, ecc, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_minimum_image_size);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_max_level);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_estimate_translation_first);
        SERIALIZE_OPTION(subsubsection, save, ecc, replace_planetary_disk_with_mask);
        SERIALIZE_OPTION(subsubsection, save, ecc, planetary_disk_mask_stdev_factor);
        SERIALIZE_OPTION(subsubsection, save, ecc, se_close_size);
      }

      if( (subsubsection = get_group(subsection, save, "eccflow")) ) {

        c_eccflow_registration_options & eccflow =
            opts.registration.eccflow;

        SERIALIZE_OPTION(subsubsection, save, eccflow, enabled);
        SERIALIZE_OPTION(subsubsection, save, eccflow, update_multiplier);
        SERIALIZE_OPTION(subsubsection, save, eccflow, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, eccflow, support_scale);
        SERIALIZE_OPTION(subsubsection, save, eccflow, min_image_size);
        SERIALIZE_OPTION(subsubsection, save, eccflow, max_pyramid_level);
        SERIALIZE_OPTION(subsubsection, save, eccflow, noise_level);
        SERIALIZE_OPTION(subsubsection, save, eccflow, scale_factor);
        SERIALIZE_OPTION(subsubsection, save, eccflow, downscale_method);
      }
    }
  }

  if( (section = get_group(settings, save, "stacking_options")) ) {

    c_image_stacking_options & opts =
        _stacking_options;

    SERIALIZE_OPTION(section, save, opts, unsharp_sigma);
    SERIALIZE_OPTION(section, save, opts, unsharp_alpha);

    // c_image_registration_options registration;
    if( (subsection = get_group(section, save, "registration")) ) {

      SERIALIZE_OPTION(subsection, save, opts.registration, enabled);
      SERIALIZE_OPTION(subsection, save, opts.registration, motion_type);
      SERIALIZE_OPTION(subsection, save, opts.registration, accumulate_and_compensate_turbulent_flow);
      SERIALIZE_OPTION(subsection, save, opts.registration, ecc_registration_channel);
      SERIALIZE_OPTION(subsection, save, opts.registration, interpolation);
      SERIALIZE_OPTION(subsection, save, opts.registration, border_mode);
      SERIALIZE_OPTION(subsection, save, opts.registration, border_value);


      if( (subsubsection = get_group(subsection, save, "feature_registration")) ) {

        c_feature_registration_options & feature_registration =
            opts.registration.feature_registration;

        SERIALIZE_OPTION(subsubsection, save, feature_registration, enabled);
        SERIALIZE_OPTION(subsubsection, save, feature_registration, scale);
        SERIALIZE_OPTION(subsection, save, feature_registration, registration_channel);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_detector"), save,
            feature_registration.sparse_feature_extractor_and_matcher, detector);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_descriptor"), save,
            feature_registration.sparse_feature_extractor_and_matcher, descriptor);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_matcher"), save,
            feature_registration.sparse_feature_extractor_and_matcher, matcher);

        SERIALIZE_OPTION(get_group(subsubsection, save, "estimate_options"), save,
            feature_registration, estimate_options);
      }

      if( (subsubsection = get_group(subsection, save, "ecc")) ) {

        c_ecc_registration_options & ecc =
            opts.registration.ecc;

        SERIALIZE_OPTION(subsubsection, save, ecc, enabled);
        SERIALIZE_OPTION(subsubsection, save, ecc, scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, eps);
        SERIALIZE_OPTION(subsubsection, save, ecc, min_rho);
        SERIALIZE_OPTION(subsubsection, save, ecc, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, update_step_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_noise);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecc_method);
        SERIALIZE_OPTION(subsubsection, save, ecc, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_minimum_image_size);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_max_level);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_estimate_translation_first);
        SERIALIZE_OPTION(subsubsection, save, ecc, replace_planetary_disk_with_mask);
        SERIALIZE_OPTION(subsubsection, save, ecc, planetary_disk_mask_stdev_factor);
        SERIALIZE_OPTION(subsubsection, save, ecc, se_close_size);

      }

      if( (subsubsection = get_group(subsection, save, "eccflow")) ) {

        c_eccflow_registration_options & eccflow =
            opts.registration.eccflow;

        SERIALIZE_OPTION(subsubsection, save, eccflow, enabled);
        SERIALIZE_OPTION(subsubsection, save, eccflow, update_multiplier);
        SERIALIZE_OPTION(subsubsection, save, eccflow, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, eccflow, support_scale);
        SERIALIZE_OPTION(subsubsection, save, eccflow, min_image_size);
        SERIALIZE_OPTION(subsubsection, save, eccflow, max_pyramid_level);
        SERIALIZE_OPTION(subsubsection, save, eccflow, noise_level);
        SERIALIZE_OPTION(subsubsection, save, eccflow, scale_factor);
        SERIALIZE_OPTION(subsubsection, save, eccflow, downscale_method);
      }

      SERIALIZE_OPTION(subsection, save, opts.registration.planetary_disk_derotation, derotation_type);

      if( (subsubsection = get_group(subsection, save, "jovian_derotation")) ) {

        struct c_jovian_derotation_options & jovian_derotation =
            opts.registration.planetary_disk_derotation.jovian_derotation;

        //SERIALIZE_OPTION(subsubsection, save, jovian_derotation, enabled);
//        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, min_rotation);
//        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, max_rotation);
//        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, max_pyramid_level);
//        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, num_orientations);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, derotate_all_frames);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, max_context_size);

        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, gbsigma);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, stdev_factor);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, pca_blur);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, se_close_radius);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, equatorial_radius);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, pose);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, center);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, offset);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.detector_options, auto_location);
      }

      if( (subsubsection = get_group(subsection, save, "saturn_derotation")) ) {

        struct c_saturn_derotation_options & saturn_derotation =
            opts.registration.planetary_disk_derotation.saturn_derotation;

        //SERIALIZE_OPTION(subsubsection, save, jovian_derotation, enabled);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, min_rotation);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, max_rotation);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, max_pyramid_level);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, num_orientations);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, derotate_all_frames);
//        SERIALIZE_OPTION(subsubsection, save, saturn_derotation, max_context_size);
        SERIALIZE_OPTION(subsubsection, save, saturn_derotation.detector_options, stdev_factor);
        SERIALIZE_OPTION(subsubsection, save, saturn_derotation.detector_options, se_close_radius);
        //SERIALIZE_OPTION(subsubsection, save, jovian_derotation.ellipse, force_reference_ellipse);
      }

    }
  }

  // c_frame_accumulation_options accumulation_options_;
  if( (section = get_group(settings, save, "master_accumulation")) ) {

    c_frame_accumulation_options & opts =
        _master_options.accumulation;

    SERIALIZE_OPTION(section, save, opts, accumulation_method);
    SERIALIZE_OPTION(section, save, opts, max_weights_ratio);

    if( (subsection = get_group(section, save, "c_lpg_sharpness_measure")) ) {

      c_lpg_options &m =
          opts.lpg;

      SERIALIZE_OPTION(subsection, save, m, k);
      SERIALIZE_OPTION(subsection, save, m, p);
      SERIALIZE_OPTION(subsection, save, m, dscale);
      SERIALIZE_OPTION(subsection, save, m, uscale);
      SERIALIZE_OPTION(subsection, save, m, avgchannel);
    }

    if( (subsection = get_group(section, save, "c_laplacian_pyramid_focus_stacking")) ) {

      c_laplacian_pyramid_focus_stacking::options &fs =
          opts.fs;

      SERIALIZE_OPTION(subsection, save, fs, fusing_policy);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
      SERIALIZE_OPTION(subsection, save, fs, avgchannel);
      SERIALIZE_OPTION(subsection, save, fs, kradius);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
    }
  }

  if( (section = get_group(settings, save, "stack_accumulation")) ) {

    c_frame_accumulation_options & opts =
        _stacking_options.accumulation;

    SERIALIZE_OPTION(section, save, opts, accumulation_method);
    SERIALIZE_OPTION(section, save, opts, max_weights_ratio);

    if( (subsection = get_group(section, save, "c_lpg_sharpness_measure")) ) {

      c_lpg_options &m =
          opts.lpg;

      SERIALIZE_OPTION(subsection, save, m, k);
      SERIALIZE_OPTION(subsection, save, m, p);
      SERIALIZE_OPTION(subsection, save, m, dscale);
      SERIALIZE_OPTION(subsection, save, m, uscale);
      SERIALIZE_OPTION(subsection, save, m, avgchannel);
    }

    if( (subsection = get_group(section, save, "c_laplacian_pyramid_focus_stacking")) ) {

      c_laplacian_pyramid_focus_stacking::options &fs =
          opts.fs;

      SERIALIZE_OPTION(subsection, save, fs, fusing_policy);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
      SERIALIZE_OPTION(subsection, save, fs, avgchannel);
      SERIALIZE_OPTION(subsection, save, fs, kradius);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
    }
  }

  // c_image_stacking_output_options output_options_;
  if( (section = get_group(settings, save, "output")) ) {

    SERIALIZE_OPTION(section, save, _output_options, output_directory);
    SERIALIZE_OPTION(section, save, _output_options, output_file_name);

    SERIALIZE_OPTION(section, save, _output_options, save_preprocessed_frames);
    if( (subsection = get_group(section, save, "output_preprocessed_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_preprocessed_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_aligned_frames);
    if( (subsection = get_group(section, save, "output_aligned_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_aligned_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_ecc_frames);
    if( (subsection = get_group(section, save, "output_ecc_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_ecc_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_accumulation_masks);
    if( (subsection = get_group(section, save, "output_acc_masks_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_acc_masks_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_incremental_frames);
    if( (subsection = get_group(section, save, "output_incremental_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_incremental_video_options);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_eccflow_frames);
    if( (subsection = get_group(section, save, "output_eccflow_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_eccflow_options);
      SERIALIZE_OPTION(subsection, save, _output_options.output_eccflow_options, hsv_max_motion);
    }

    SERIALIZE_OPTION(section, save, _output_options, save_sparse_matches_video);
    if( (subsection = get_group(section, save, "output_sparse_matches_video_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_sparse_matches_video_options);
    }


    SERIALIZE_OPTION(section, save, _output_options, save_sparse_match_blend_frames);
    if( (subsection = get_group(section, save, "output_sparse_match_blend_options")) ) {
      SERIALIZE_OPTION(subsection, save, _output_options, output_sparse_match_blend_options);
    }


    SERIALIZE_OPTION(section, save, _output_options, dump_reference_data_for_debug);
    SERIALIZE_OPTION(section, save, _output_options, write_image_mask_as_alpha_channel);
  }

  if( (section = get_group(settings, save, "image_processing")) ) {
    if( save ) {

      if( _image_processing_options.ecc_image_processor ) {
        save_settings(settings, "ecc_image_processor",
            _image_processing_options.ecc_image_processor->name());
      }
      if( _image_processing_options.aligned_image_processor ) {
        save_settings(settings, "aligned_image_processor",
            _image_processing_options.aligned_image_processor->name());
      }
      if( _image_processing_options.incremental_frame_processor ) {
        save_settings(settings, "incremental_frame_processor",
            _image_processing_options.incremental_frame_processor->name());
      }
      if( _image_processing_options.accumulated_image_processor ) {
        save_settings(settings, "accumulated_image_processor",
            _image_processing_options.accumulated_image_processor->name());
      }
    }
    else {

      std::string s;

      if( load_settings(settings, "ecc_image_processor", &s) && !s.empty() ) {
        _image_processing_options.ecc_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "aligned_image_processor", &s) && !s.empty() ) {
        _image_processing_options.aligned_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "incremental_frame_processor", &s) && !s.empty() ) {
        _image_processing_options.incremental_frame_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "accumulated_image_processor", &s) && !s.empty() ) {
        _image_processing_options.accumulated_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
    }
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_image_stacking_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
      PIPELINE_CTL(ctrls, _input_options.anscombe, "anscombe", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "ROI options", "");
    PIPELINE_CTL(ctrls, _roi_selection_options.method, "ROI selection:", "");
    PIPELINE_CTLC(ctrls, _roi_selection_options.planetary_disk_crop_size, "Crop Size", "", (_this->_roi_selection_options.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, _roi_selection_options.planetary_disk_gbsigma, "gbsigma", "", (_this->_roi_selection_options.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, _roi_selection_options.planetary_disk_stdev_factor, "Stdev factor", "", (_this->_roi_selection_options.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, _roi_selection_options.se_close_size, "Stdev factor", "", (_this->_roi_selection_options.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, _roi_selection_options.rectangle_roi_selection, "Rectangle:", "", (_this->_roi_selection_options.method == roi_selection_rectange_crop));
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Upscale options", "");
    PIPELINE_CTL(ctrls, _upscale_options.upscale_option, "Upscale:", "");
    PIPELINE_CTLC(ctrls, _upscale_options.upscale_stage, "Stage:", "", (_this->_upscale_options.upscale_option != frame_upscale_none));
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Frame Registration", "");

      PIPELINE_CTL_GROUP(ctrls, "Master Frame", "");

        PIPELINE_CTL_MASTER_FRAME_SELECTION(ctrls, _master_options.master_selection, true);
        PIPELINE_CTL(ctrls, _master_options.apply_input_image_processor, "apply_input_image_processor", "");
        PIPELINE_CTL(ctrls, _master_options.master_channel, "master_channel", "");
        PIPELINE_CTL(ctrls, _master_options.unsharp_sigma, "unsharp_sigma", "Set positive value to apply unsharp mask to master frame");
        PIPELINE_CTL(ctrls, _master_options.unsharp_alpha, "unsharp_alpha", "Set positive value to apply unsharp mask to master frame");

        PIPELINE_CTL(ctrls, _master_options.generate_master_frame, "Generate Master Frame:", "");
        PIPELINE_CTLC(ctrls, _master_options.save_master_frame, "Save Generated Master Frame:", "",
            _this->_master_options.generate_master_frame);
        PIPELINE_CTLC(ctrls, _master_options.stop_after_master_frame_generation, "Stop after master generation:", "Finish pipeline after master frame generation",
            _this->_master_options.generate_master_frame);


        PIPELINE_CTL_GROUPC(ctrls, "Master Frame Generation", "", _this->_master_options.generate_master_frame);


          PIPELINE_CTL(ctrls, _master_options.registration.motion_type, "Motion Type:", "");
          PIPELINE_CTL(ctrls, _master_options.registration.interpolation, "Interpolation:", "");
          PIPELINE_CTL(ctrls, _master_options.registration.border_mode, "Border Mode:", "");
          PIPELINE_CTL(ctrls, _master_options.registration.border_value, "Border Value:", "");
          PIPELINE_CTL(ctrls, _master_options.registration.ecc_registration_channel, "ECC Registration Channel:", "");
          PIPELINE_CTL(ctrls, _master_options.registration.accumulate_and_compensate_turbulent_flow, "Compute and Compensate Turbulent Flow:", "");
          PIPELINE_CTL(ctrls, _master_options.max_frames_to_generate_master_frame, "Max Input frames:", "Max input frames used to generate master frame");

          PIPELINE_CTL_GROUPC(ctrls, "Camera Matrix", "", (_this->_master_options.registration.motion_type == IMAGE_MOTION_EPIPOLAR_DEROTATION));
            PIPELINE_CTL_CAMERA_INTRINSICS(ctrls, _master_options.registration.feature_registration.estimate_options.epipolar_derotation.camera_intrinsics);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "Feature Registration Options", "");
          PIPELINE_CTL(ctrls, _master_options.registration.feature_registration.enabled, "Enable Feature Registration", "");
            PIPELINE_CTL_FEATURE_REGISTRATION_OPTIONS(ctrls, _master_options.registration.feature_registration, _this->_master_options.registration.feature_registration.enabled);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "ECC Registration Options", "");
          PIPELINE_CTL(ctrls, _master_options.registration.ecc.enabled, "Enable ECC Registration", "");
            PIPELINE_CTL_ECC_REGISTRATION_OPTIONS(ctrls, _master_options.registration.ecc, _this->_master_options.registration.ecc.enabled);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "ECC Flow Registration Options", "");
            PIPELINE_CTL(ctrls, _master_options.registration.eccflow.enabled, "Enable ECC Flow Registration", "");
            PIPELINE_CTL_ECCFLOW_REGISTRATION_OPTIONS(ctrls, _master_options.registration.eccflow, _this->_master_options.registration.eccflow.enabled);
          PIPELINE_CTL_END_GROUP(ctrls);

        PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);


      PIPELINE_CTL_GROUP(ctrls, "Stack Registration", "");

        PIPELINE_CTL(ctrls, _stacking_options.registration.enabled, "Enable Stack Registration:", "");

        PIPELINE_CTL_GROUPC(ctrls, "Stack Registration Options", "",  _this->_stacking_options.registration.enabled);

          PIPELINE_CTL(ctrls, _stacking_options.registration.motion_type, "Motion Type:", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.interpolation, "Interpolation:", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.border_mode, "Border Mode:", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.border_value, "Border Value:", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.ecc_registration_channel, "ECC Registration Channel:", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.accumulate_and_compensate_turbulent_flow, "Compute And Compensate Turbulent Flow:", "");

          PIPELINE_CTL_GROUPC(ctrls, "Camera Matrix", "", (_this->_stacking_options.registration.motion_type == IMAGE_MOTION_EPIPOLAR_DEROTATION));
            PIPELINE_CTL_CAMERA_INTRINSICS(ctrls, _stacking_options.registration.feature_registration.estimate_options.epipolar_derotation.camera_intrinsics);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "Feature Registration Options", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.feature_registration.enabled, "Enable Feature Registration", "");
            PIPELINE_CTL_FEATURE_REGISTRATION_OPTIONS(ctrls, _stacking_options.registration.feature_registration, _this->_stacking_options.registration.feature_registration.enabled);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "ECC Registration Options", "");
          PIPELINE_CTL(ctrls, _stacking_options.registration.ecc.enabled, "Enable ECC Registration", "");
            PIPELINE_CTL_ECC_REGISTRATION_OPTIONS(ctrls, _stacking_options.registration.ecc, _this->_stacking_options.registration.ecc.enabled);
          PIPELINE_CTL_END_GROUP(ctrls);

          PIPELINE_CTL_GROUP(ctrls, "Planetary Disk Derotation", "");
            PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.derotation_type, "Planetary Disk Derotation Type", "");

            PIPELINE_CTL_GROUPC(ctrls, "Jovian Derotation Options", "", (_this->_stacking_options.registration.planetary_disk_derotation.derotation_type == planetary_disk_derotation_jovian));
            //PIPELINE_CTL_JOVIAN_DEROTATION_OPTIONS(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation, (_this->_stacking_options.registration.planetary_disk_derotation.derotation_type == planetary_disk_derotation_jovian));
//              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.min_rotation, "min_rotation", "");
//              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.max_rotation, "max_rotation", "");
//              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.max_pyramid_level, "max_pyramid_level", "");
//              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.num_orientations, "", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.derotate_all_frames, "derotate_all_frames", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.max_context_size, "max_context_size", "");

              //PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.gbsigma, "gbsigma", "");
              //PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.stdev_factor, "stdev_factor", "");
              //PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.pca_blur, "pca_blur", "");
              //PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.se_close_radius, "se_close_radius", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.equatorial_radius, "equatorial_radius", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.center, "center", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.offset, "offset", "");
              PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.pose, "pose", "");
              //PIPELINE_CTL(ctrls, _stacking_options.registration.planetary_disk_derotation.jovian_derotation.detector_options.auto_location, "", "");

            PIPELINE_CTL_END_GROUP(ctrls);

            PIPELINE_CTL_GROUP(ctrls, "Saturn Derotation Options", "");
            PIPELINE_CTL_SATURN_DEROTATION_OPTIONS(ctrls, _stacking_options.registration.planetary_disk_derotation.saturn_derotation, (_this->_stacking_options.registration.planetary_disk_derotation.derotation_type == planetary_disk_derotation_saturn));
            PIPELINE_CTL_END_GROUP(ctrls);
          PIPELINE_CTL_END_GROUP(ctrls); // Planetary Disk Derotation

          PIPELINE_CTL_GROUP(ctrls, "ECC Flow Registration Options", "");
            PIPELINE_CTL(ctrls, _stacking_options.registration.eccflow.enabled, "Enable ECC Flow Registration", "");
            PIPELINE_CTL_ECCFLOW_REGISTRATION_OPTIONS(ctrls, _stacking_options.registration.eccflow, _this->_stacking_options.registration.eccflow.enabled);
          PIPELINE_CTL_END_GROUP(ctrls); // ECC Flow Registration Options

        PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Frame accumulation", "");

    if ( true ) {
      PIPELINE_CTL_GROUPC(ctrls, "Master Frame accumulation", "", _this->_master_options.generate_master_frame);

      PIPELINE_CTL(ctrls, _master_options.accumulation.accumulation_method, "Acc. Method", "");
      PIPELINE_CTLC(ctrls, _master_options.accumulation.max_weights_ratio, "max_weights_ratio", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));

      PIPELINE_CTL_GROUP(ctrls, "weighted_average", "");
      PIPELINE_CTLC(ctrls, _master_options.accumulation.lpg.k, "K", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.lpg.p, "p", "power", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.lpg.dscale, "dscale", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.lpg.uscale, "uscale", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.lpg.avgchannel, "avgchannel", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "focus_stack", "");
      PIPELINE_CTLC(ctrls, _master_options.accumulation.fs.fusing_policy, "fusing_policy", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.fs.inpaint_mask_holes, "inpaint_mask_holes", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.fs.kradius, "kradius", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.fs.ksigma, "ksigma", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _master_options.accumulation.fs.avgchannel, "avgchannel", "", (_this->_master_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);
    }

    if ( true ) {
      PIPELINE_CTL_GROUP(ctrls, "Stack Frame accumulation", "");

      PIPELINE_CTL(ctrls, _stacking_options.accumulation.accumulation_method, "Acc. Method", "");
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.max_weights_ratio, "max_weights_ratio", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));

      PIPELINE_CTL_GROUP(ctrls, "weighted_average", "");
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.lpg.k, "K", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.lpg.p, "p", "power", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.lpg.dscale, "dscale", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.lpg.uscale, "uscale", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.lpg.avgchannel, "avgchannel", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "focus_stack", "");
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.fs.fusing_policy, "fusing_policy", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.fs.inpaint_mask_holes, "inpaint_mask_holes", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.fs.kradius, "kradius", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.fs.ksigma, "ksigma", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, _stacking_options.accumulation.fs.avgchannel, "avgchannel", "", (_this->_stacking_options.accumulation.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_END_GROUP(ctrls);
    }

    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
      // PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.input_image_processor, "input_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _image_processing_options.ecc_image_processor, "ecc_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _image_processing_options.aligned_image_processor, "aligned_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _image_processing_options.incremental_frame_processor, "incremental_frame_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, _image_processing_options.accumulated_image_processor, "accumulated_image_processor", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, _output_options.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, _output_options.output_directory, "output_directory", "");
    PIPELINE_CTL(ctrls, _output_options.output_file_name, "output_file_name", "");

    PIPELINE_CTL(ctrls, _output_options.write_image_mask_as_alpha_channel, "write_image_mask_as_alpha_channel", "");
    PIPELINE_CTL(ctrls, _output_options.dump_reference_data_for_debug, "dump_reference_data_for_debug", "");
    PIPELINE_CTL(ctrls, _output_options.debug_frame_registration, "debug_frame_registration", "");
    PIPELINE_CTLC(ctrls, _output_options.debug_frame_registration_frame_indexes, "debug_frame_registration_frame_indexes", "", (_this->_output_options.debug_frame_registration));

    PIPELINE_CTL_GROUP(ctrls, "Save Preprocessed Frames", "");
    PIPELINE_CTL(ctrls, _output_options.save_preprocessed_frames, "save_preprocessed_frames", "");
    PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_preprocessed_video_options, (_this->_output_options.save_preprocessed_frames));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save Aligned Frames", "");
    PIPELINE_CTL(ctrls, _output_options.save_aligned_frames, "save_aligned_frames", "");
    PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_aligned_video_options, (_this->_output_options.save_aligned_frames));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save ECC Frames", "");
      PIPELINE_CTL(ctrls, _output_options.save_ecc_frames, "save_ecc_frames", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_ecc_video_options, (_this->_output_options.save_ecc_frames));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save ACC Masks", "");
      PIPELINE_CTL(ctrls, _output_options.save_accumulation_masks, "save_accumulation_masks", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_acc_masks_video_options, (_this->_output_options.save_accumulation_masks));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save Incremental Frames", "");
      PIPELINE_CTL(ctrls, _output_options.save_incremental_frames, "save_incremental_frames", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_incremental_video_options, (_this->_output_options.save_incremental_frames));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save eccflow maps", "");
      PIPELINE_CTL(ctrls, _output_options.save_eccflow_frames, "save_eccflow_maps", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_eccflow_options, (_this->_output_options.save_eccflow_frames));
      PIPELINE_CTLC(ctrls,  _output_options.output_eccflow_options.hsv_max_motion, "hsv_max_motion", "", (_this->_output_options.save_eccflow_frames));
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "Save sparse matches video", "");
      PIPELINE_CTL(ctrls, _output_options.save_sparse_matches_video, "save_sparse_matches_video", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_sparse_matches_video_options, (_this->_output_options.save_sparse_matches_video));
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_GROUP(ctrls, "Save sparse match blends", "");
      PIPELINE_CTL(ctrls, _output_options.save_sparse_match_blend_frames, "save sparse match blends", "");
      PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, _output_options.output_sparse_match_blend_options, (_this->_output_options.save_sparse_match_blend_frames));
    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_END_GROUP(ctrls);
    ////////
  }

  return ctrls;
}

bool c_image_stacking_pipeline::has_master_frame() const
{
  return true;
}

void c_image_stacking_pipeline::set_master_source(const std::string & master_source_path)
{
  _master_options.master_selection.master_fiename = master_source_path;
}

std::string c_image_stacking_pipeline::master_source() const
{
  return _master_options.master_selection.master_fiename;
}

void c_image_stacking_pipeline::set_master_frame_index(int v)
{
  _master_options.master_selection.master_selection_method = master_frame_specific_index;
  _master_options.master_selection.master_frame_index = v;
}

int c_image_stacking_pipeline::master_frame_index() const
{
  return _master_options.master_selection.master_frame_index;
}

bool c_image_stacking_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_image_stacking_pipeline::base::copyParameters() fails");
    return false;
  }

  this_class::sptr p =
      std::dynamic_pointer_cast<this_class>(dst);

  if( !p ) {
    CF_ERROR("std::dynamic_pointer_cast<this_class=%s>(dst) fails",
        get_class_name().c_str());
    return false;
  }

  const std::string backup_master_source_fiename =
      p->_master_options.master_selection.master_fiename;

  const int backup_master_frame_index =
      p->_master_options.master_selection.master_frame_index;


  p->_input_options = this->_input_options;
  p->_roi_selection_options = this->_roi_selection_options;
  p->_upscale_options = this->_upscale_options;
  p->_master_options = this->_master_options;
  p->_stacking_options = this->_stacking_options ;
  p->_output_options = this->_output_options;
  p->_image_processing_options = this->_image_processing_options;

  p->_master_options.master_selection.master_fiename =
      backup_master_source_fiename;

  p->_master_options.master_selection.master_frame_index =
      backup_master_frame_index;

  return true;
}

bool c_image_stacking_pipeline::save_preprocessed_video(const cv::Mat & current_frame, const cv::Mat & current_mask, int seqindex)
{
  if ( !_output_options.save_preprocessed_frames ) {
    return true;
  }

  c_output_frame_writer & writer =
      preprocessed_video_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_preprocessed_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("preproc%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  return writer.write(current_frame, current_mask,
      _output_options.write_image_mask_as_alpha_channel,
      seqindex);
}


bool c_image_stacking_pipeline::save_sparse_matches_blend_video(int seqindex)
{
  if ( !_output_options.save_sparse_match_blend_frames || !frame_registration_ ) {
    return true;
  }

  const c_sparse_feature_extractor_and_matcher::sptr & feature_matcher =
      frame_registration_->sparse_feature_extractor_and_matcher();

  if( !feature_matcher ) {
    return true;
  }

  const cv::Mat & referece_image =
      frame_registration_->reference_feature_image();

  const cv::Mat & current_image =
      frame_registration_->current_feature_image();

  if ( !referece_image.empty() && current_image.size() == referece_image.size() ) {

    const std::vector<cv::Point2f> & matched_reference_positions =
        feature_matcher->matched_reference_positions();

    const std::vector<cv::Point2f> & matched_current_positions =
        feature_matcher->matched_current_positions();


    c_output_frame_writer & writer =
        sparse_match_blend_writer_;

    const c_output_frame_writer_options & output_opts =
        _output_options.output_sparse_match_blend_options;

    const bool fOK =
        add_output_writer(writer,
            output_opts,
            ssprintf("sparse-matches%s",_generating_master_frame ? "-master" : ""),
            ".avi");

    if( !fOK ) {
      CF_ERROR("Can not open output writer '%s'",
          writer.filename().c_str());
      return false;
    }


    cv::Mat display;
    cv::addWeighted(referece_image, 0.5, current_image, 0.5, 0, display);

    if ( display.channels() == 1 ) {
      cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);
    }

    const size_t n = matched_reference_positions.size();
    for( size_t i = 0; i < n; ++i ) {

      const cv::Point2f & rp =
          matched_reference_positions[i];

      const cv::Point2f & cp =
          matched_current_positions[i];

      cv::line(display, rp, cp, CV_RGB(32, 255, 32), 1,
          cv::LINE_AA);

    }

    if( !writer.write(display, cv::noArray(), false, seqindex) ) {
      CF_ERROR("writer.write() fails: %s",
          writer.filename().c_str());
      return false;
    }

  }

  return true;
}


bool c_image_stacking_pipeline::save_ecc_video(int seqindex)
{
  if ( !_output_options.save_ecc_frames || !frame_registration_) {
    return true;
  }

  const cv::Mat & current_frame =
      frame_registration_->current_ecc_image();

  if ( current_frame.empty() ) {
    return true;
  }

  const cv::Mat & current_mask =
      frame_registration_->current_ecc_mask();

  c_output_frame_writer & writer =
      ecc_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_ecc_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("ecc%s",_generating_master_frame ? "-master" : ""),
          ".ser");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  return writer.write(current_frame,current_mask,
      _output_options.write_image_mask_as_alpha_channel,
      seqindex);
}

bool c_image_stacking_pipeline::save_eccflow_video(int seqindex)
{
  if ( !_output_options.save_eccflow_frames || !frame_registration_ ) {
    return true;
  }

  const cv::Mat2f & uv =
      frame_registration_->eccflow().current_uv();

  if ( uv.empty() ) {
    return true;
  }

  c_output_frame_writer & writer =
      eccflow_writer_;

  const c_eccflow_output_frame_writer_options & output_opts =
      _output_options.output_eccflow_options;


  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("uv%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  cv::Mat display;

  if ( strcasecmp(get_file_suffix(writer.filename()).c_str(), ".flo") == 0 ) {
    display = uv;
  }
  else {
    flow2HSV(uv, display, output_opts.hsv_max_motion);
  }

  if( !writer.write(display, cv::noArray(), false, seqindex) ) {
    CF_ERROR("writer.write() fails: %s",
        writer.filename().c_str());
    return false;
  }

  return true;
}

bool c_image_stacking_pipeline::save_aligned_video(const cv::Mat & current_frame, const cv::Mat & current_mask, int seqindex)
{
  if( !_output_options.save_aligned_frames ) {
    return true;
  }

  c_output_frame_writer & writer =
      aligned_video_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_aligned_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("algned%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }


  CF_DEBUG("current_frame: %dx%d depth=%d channels=%d. current_mask: %dx%d depth=%d channels=%d. ",
      current_frame.cols, current_frame.rows, current_frame.depth(), current_frame.channels(),
      current_mask.cols, current_mask.rows, current_mask.depth(), current_mask.channels());

  return writer.write(current_frame, current_mask,
      _output_options.write_image_mask_as_alpha_channel,
      seqindex);
}


bool c_image_stacking_pipeline::save_incremental_video(const cv::Mat & current_frame, const cv::Mat & current_mask, int seqindex)
{
  if ( !_output_options.save_incremental_frames ) {
    return true;
  }

  c_output_frame_writer & writer =
      incremental_video_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_incremental_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("incremental%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  return writer.write(current_frame, current_mask,
      _output_options.write_image_mask_as_alpha_channel,
      seqindex);
}

bool c_image_stacking_pipeline::save_accumulation_masks_video(const cv::Mat & current_frame, const cv::Mat & current_mask, int seqindex)
{
  if ( !_output_options.save_accumulation_masks ) {
    return true;
  }

  c_output_frame_writer & writer =
      accumulation_masks_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_acc_masks_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("acc_masks%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  if ( !current_mask.empty() ) {
    if ( !writer.write(current_mask, cv::noArray(), false, seqindex) ) {
      CF_ERROR("writer.write() fails");
      return false;
    }
  }
  else {
    if ( !writer.write(cv::Mat1b(current_frame.size(), 255), cv::noArray(), false, seqindex) ) {
      CF_ERROR("writer.write() fails");
      return false;
    }
  }

  return true;
}


bool c_image_stacking_pipeline::save_sparse_matches_video(int seqindex)
{
  if ( !_output_options.save_sparse_matches_video || !frame_registration_->options().feature_registration.enabled ) {
    return true;
  }

  const c_sparse_feature_extractor_and_matcher::sptr & sm =
      frame_registration_->sparse_feature_extractor_and_matcher();

  if ( !sm ) {
    return true;
  }

  const cv::Mat & current_feature_image =
      frame_registration_->current_feature_image();

  const cv::Mat & reference_feature_image =
      frame_registration_->reference_feature_image();

  const std::vector<cv::Point2f> & cps =
      sm->matched_current_positions();

  const std::vector<cv::Point2f> & rps =
      sm->matched_reference_positions();

  const double scale =
      frame_registration_->options().feature_registration.scale;

  const cv::Size sizes[2] = {
      current_feature_image.size(),
      reference_feature_image.size(),
  };

  const int max_width =
      std::max(sizes[0].width, sizes[1].width);

  const cv::Size output_frame_size(2 * max_width,
      sizes[0].height+ sizes[1].height);

  cv::Mat3b output_frame(output_frame_size,
      cv::Vec3b(0, 0, 0));

  const cv::Rect ROI[4] = {
      cv::Rect(0, 0, sizes[0].width, sizes[0].height),
      cv::Rect(max_width, 0, sizes[0].width, sizes[0].height),

      cv::Rect(0, sizes[0].height, sizes[1].width, sizes[1].height),
      cv::Rect(max_width, sizes[0].height, sizes[1].width, sizes[1].height),
  };

  cv::Mat3b panes[4] = {
      output_frame(ROI[0]), output_frame(ROI[1]),
      output_frame(ROI[2]), output_frame(ROI[3]),
  };

  if ( current_feature_image.channels() == 3 ) {
    current_feature_image.copyTo(panes[0]);
    current_feature_image.copyTo(panes[1]);
  }
  else {
    cv::cvtColor(current_feature_image, panes[0], cv::COLOR_GRAY2BGR);
    cv::cvtColor(current_feature_image, panes[1], cv::COLOR_GRAY2BGR);
  }

  if ( reference_feature_image.channels() == 3 ) {
    reference_feature_image.copyTo(panes[2]);
    reference_feature_image.copyTo(panes[3]);
  }
  else {
    cv::cvtColor(reference_feature_image, panes[2], cv::COLOR_GRAY2BGR);
    cv::cvtColor(reference_feature_image, panes[3], cv::COLOR_GRAY2BGR);
  }

  const size_t num_matches =
      cps.size();

  for( size_t i = 0; i < num_matches; ++i ) {

    const cv::Point2f cp =
        cps[i];

    const cv::Point2f rp =
        rps[i];

    cv::line(output_frame, cv::Point2f(cp.x + ROI[0].x, cp.y + ROI[0].y),
        cv::Point2f(rp.x + ROI[2].x, rp.y + ROI[2].y),
        CV_RGB(rand() % 255, rand() % 255, rand() % 255), 1,
        cv::LINE_AA);
  }

  c_output_frame_writer & writer =
      sparse_matches_video_writer_;

  const c_output_frame_writer_options & output_opts =
      _output_options.output_sparse_matches_video_options;

  const bool fOK =
      add_output_writer(writer,
          output_opts,
          ssprintf("sparse_matches%s",_generating_master_frame ? "-master" : ""),
          ".avi");

  if( !fOK ) {
    CF_ERROR("Can not open output writer '%s'",
        writer.filename().c_str());
    return false;
  }

  if( !writer.write(output_frame, cv::noArray(), false, seqindex) ) {
    CF_ERROR("writer.write() fails for '%s'", writer.filename().c_str());
    return false;
  }

  return true;
}
