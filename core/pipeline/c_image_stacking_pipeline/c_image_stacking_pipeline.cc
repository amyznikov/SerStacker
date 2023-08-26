/*
 * c_stacking_pipeline.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/image_registration/c_translation_ecc_motion_model.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/extract_channel.h>
#include <core/proc/unsharp_mask.h>
#include <core/proc/autoclip.h>
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
  static constexpr c_enum_member members[] = {
      { roi_selection_none, "none", },
      { roi_selection_planetary_disk, "planetary_disk", },
      { roi_selection_rectange_crop, "rectangle", },
      { roi_selection_none, nullptr, },
  };
  return members;
}

template<>
const c_enum_member* members_of<frame_accumulation_method>()
{
  static constexpr c_enum_member members[] = {

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

      { frame_accumulation_none, nullptr, },
  };

  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_stage>()
{
  static constexpr c_enum_member members[] = {
      { frame_upscale_after_align , "after_align", },
      { frame_upscale_before_align , "before_align", },
      { frame_upscale_stage_unknown , nullptr, },
  };
  return members;
}

template<>
const c_enum_member * members_of<frame_upscale_option>()
{
  static constexpr c_enum_member members[] = {
      {frame_upscale_none, "none", },
      {frame_upscale_pyrUp, "x2.0", },
      {frame_upscale_x15, "x1.5", },
      {frame_upscale_x30, "x3.0", },
      {frame_upscale_none, nullptr, },
  };
  return members;
}

template<>
const c_enum_member* members_of<STACKING_STAGE>()
{
  static constexpr c_enum_member members[] = {
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
  return output_file_name_;
}

const c_anscombe_transform & c_image_stacking_pipeline::anscombe() const
{
  return anscombe_;
}

c_image_stacking_input_options & c_image_stacking_pipeline::input_options()
{
  return input_options_;
}

const c_image_stacking_input_options & c_image_stacking_pipeline::input_options() const
{
  return input_options_;
}

c_frame_upscale_options & c_image_stacking_pipeline::upscale_options()
{
  return upscale_options_;
}

const c_frame_upscale_options & c_image_stacking_pipeline::upscale_options() const
{
  return upscale_options_;
}

c_sparse_feature_extractor_options & c_image_stacking_pipeline::sparse_feature_extractor_options()
{
  return image_registration_options_.feature_registration.sparse_feature_extractor;
}

const c_sparse_feature_extractor_options & c_image_stacking_pipeline::sparse_feature_extractor_options() const
{
  return image_registration_options_.feature_registration.sparse_feature_extractor;
}

c_sparse_feature_detector_options & c_image_stacking_pipeline::sparse_feature_detector_options()
{
  return sparse_feature_extractor_options().detector;
}

const c_sparse_feature_detector_options & c_image_stacking_pipeline::sparse_feature_detector_options() const
{
  return sparse_feature_extractor_options().detector;
}

c_sparse_feature_descriptor_options & c_image_stacking_pipeline::sparse_feature_descriptor_options()
{
  return sparse_feature_extractor_options().descriptor;
}

const c_sparse_feature_descriptor_options & c_image_stacking_pipeline::sparse_feature_descriptor_options() const
{
  return sparse_feature_extractor_options().descriptor;
}

c_master_frame_options & c_image_stacking_pipeline::master_frame_options()
{
  return image_registration_options_.master_frame_options;
}

const c_master_frame_options & c_image_stacking_pipeline::master_frame_options() const
{
  return image_registration_options_.master_frame_options;
}

c_roi_selection_options & c_image_stacking_pipeline::roi_selection_options()
{
  return roi_selection_options_;
}

const c_roi_selection_options & c_image_stacking_pipeline::roi_selection_options() const
{
  return roi_selection_options_;
}

c_roi_selection::ptr c_image_stacking_pipeline::create_roi_selection() const
{
  switch ( roi_selection_options_.method ) {
  case roi_selection_planetary_disk :
    return c_planetary_disk_selection::create(roi_selection_options_.planetary_disk_crop_size,
        roi_selection_options_.planetary_disk_gbsigma,
        roi_selection_options_.planetary_disk_stdev_factor);
  case roi_selection_rectange_crop :
    return c_roi_rectangle_selection::create(roi_selection_options_.rectangle_roi_selection);

  default :
    break;
  }
  return nullptr;
}



c_image_registration_options & c_image_stacking_pipeline::registration_options()
{
  return image_registration_options_;
}

const c_image_registration_options & c_image_stacking_pipeline::registration_options() const
{
  return image_registration_options_;
}

c_frame_registration::sptr c_image_stacking_pipeline::create_frame_registration(const c_image_registration_options & options) const
{
  return c_frame_registration::sptr(new c_frame_registration(options));
}

c_frame_registration::sptr c_image_stacking_pipeline::create_frame_registration() const
{
  return create_frame_registration(image_registration_options_);
}

c_frame_accumulation_options & c_image_stacking_pipeline::accumulation_options()
{
  return accumulation_options_;
}

const c_frame_accumulation_options & c_image_stacking_pipeline::accumulation_options() const
{
  return accumulation_options_;
}

c_frame_accumulation::ptr c_image_stacking_pipeline::create_frame_accumulation() const
{
  switch (accumulation_options_.accumulation_method) {
    case frame_accumulation_average:
      return c_frame_accumulation::ptr(new c_frame_weigthed_average());
    case frame_accumulation_weighted_average:
      return c_frame_accumulation::ptr(new c_frame_weigthed_average());
    case frame_accumulation_focus_stack:
      return c_frame_accumulation::ptr(new c_laplacian_pyramid_focus_stacking(accumulation_options_.fs));
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
  return output_options_;
}

const c_image_stacking_output_options & c_image_stacking_pipeline::output_options() const
{
  return output_options_;
}

c_image_processing_options & c_image_stacking_pipeline::image_processing_options()
{
  return image_processing_options_;
}

const c_image_processing_options & c_image_stacking_pipeline::image_processing_options() const
{
  return image_processing_options_;
}

bool c_image_stacking_pipeline::initialize_pipeline()
{
  set_pipeline_stage(stacking_stage_initialize);

  if ( !base::initialize_pipeline() ) {
    CF_ERROR("c_image_stacking_pipeline: base::initialize() fails" );
    return false;
  }

  output_path_ =
      create_output_path(output_options_.output_directory);


  if (true ) {

    lock_guard lock(accumulator_lock_);

    missing_pixel_mask_.release();

    ecc_normalization_noise_ = 0;

    output_file_name_postfix_.clear();
    output_file_name_.clear();

    roi_selection_.reset();
    frame_registration_.reset();
    frame_accumulation_.reset();
    flow_accumulation_.reset();
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
    lock_guard lock(registration_lock_);
    frame_registration_.reset();
    flow_accumulation_.reset();
  }

  if ( true ) {
    lock_guard lock(accumulator_lock_);
    frame_accumulation_.reset();
    sharpness_norm_accumulation_.reset();
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


  const bool do_jovian_derotation =
      image_registration_options_.enable_frame_registration &&
      image_registration_options_.jovian_derotation.enabled;

  if ( do_jovian_derotation ) {
    if( !(fOk = run_jovian_derotation()) ) {
      CF_ERROR("run_jovian_derotation() fails");
    }
  }
  else {
    if( !(fOk = run_image_stacking()) ) {
      CF_ERROR("run_image_stacking() fails");
    }
  }

  return fOk;
}

bool c_image_stacking_pipeline::run_jovian_derotation()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  static const auto drawRotatedRectange =
      [](cv::InputOutputArray image, const cv::RotatedRect & rc,
          const cv::Scalar color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0) {
            cv::Point2f pts[4];
            rc.points(pts);

            cv::ellipse(image, rc, color, thickness, lineType);

            for( int i = 0; i < 4; i++ ) {
              cv::line(image, pts[i], pts[(i + 1) % 4], color, thickness, lineType, shift);
            }

            cv::line(image, (pts[0] + pts[1]) * 0.5, (pts[2] + pts[3]) * 0.5, color, thickness, lineType, shift);
            cv::line(image, (pts[1] + pts[2]) * 0.5, (pts[0] + pts[3]) * 0.5, color, thickness, lineType, shift);
          };


  cv::Mat master_frame;
  cv::Mat master_mask;

  // make copy of image registration options
  c_image_registration_options registration_options =
      image_registration_options_;

  registration_options.jovian_derotation.enabled = false;
  registration_options.eccflow.enabled = false;

  CF_DEBUG("call create_reference_frame()");

  if( !create_reference_frame(registration_options, master_frame, master_mask) ) {
    CF_FATAL("create_reference_frame() fails");
    return false;
  }

  if( registration_options.master_frame_options.save_master_frame ) {

    const std::string filename =
        ssprintf("%s/%s-jovian-master.tiff",
            output_path_.c_str(),
            csequence_name());

    if( !write_image(filename, output_options_, master_frame, master_mask) ) {
      CF_FATAL("write_image('%s') fails", filename.c_str());
      return false;
    }
  }

  registration_options.jovian_derotation.enabled = true;
  registration_options.eccflow.enabled = false;

  if ( !(frame_registration_ = create_frame_registration(registration_options)) ) {
    CF_FATAL("create_frame_registration() fails");
    return false;
  }


  if ( !setup_frame_accumulation() ) {
    CF_ERROR("setup_frame_accumulation()() fails");
    return false;
  }

  c_jovian_derotation & jovian_derotation =
      frame_registration_->jovian_derotation();

  if( output_options_.debug_frame_registration ) {
    jovian_derotation.set_debug_path(ssprintf("%s/debug/jovian_derotation",
        output_path_.c_str()));
  }

  if( !jovian_derotation.setup_jovian_ellipse(master_frame, master_mask) ) {
    CF_FATAL("jovian_derotation.setup_jovian_ellipse() fails");
    return false;
  }

  std::vector<int> reference_frames;

  if ( !registration_options.jovian_derotation.derotate_all_frames ) {
    reference_frames.emplace_back(registration_options.master_frame_options.master_frame_index);
  }
  else {

    for( int i = 0, n = input_sequence_->sources().size(); i < n; ++i ) {

      const c_input_source::sptr &source =
          input_sequence_->source(i);

      if( source->enabled() ) {

        for( int j = 0, m = source->size(); j < m; ++j ) {

          const int global_pos =
              input_sequence_->global_pos(i, j);

          if( !is_bad_frame_index(global_pos) ) {
            reference_frames.emplace_back(global_pos);
          }
        }
      }
    }
  }

  c_translation_image_transform image_transform;
  c_translation_ecc_motion_model model(&image_transform);
  c_ecc_forward_additive ecc(&model);
  c_ecch ecch(&ecc);

  ecc.set_max_eps(0.1);
  ecch.set_minimum_image_size(16);

  if( !ecch.set_reference_image(master_frame, master_mask) ) {
    CF_FATAL("ecch.set_reference_image() fails");
    return false;
  }

  cv::Mat reference_frame, reference_mask;

  const color_channel_type master_channel =
      registration_options.registration_channel;


  const int context_size =
      std::min(input_sequence_->size(),
          std::max(registration_options.jovian_derotation.derotate_all_frames_max_context_size, 1));

  for( int i = 0, n = reference_frames.size(); i < n; ++i ) {

    if( !input_sequence_->seek(reference_frames[i]) ) {
      CF_ERROR("input_sequence_->seek(pos=%d) fails", reference_frames[i]);
      return false;
    }

    const int cpos =
        input_sequence_->current_pos();

    // CF_DEBUG("input_sequence_->seek(pos=%d) OK", cpos);

    if( !read_input_frame(input_sequence_, reference_frame, reference_mask, false) ) {
      CF_ERROR("read_input_frame(pos=%d) fails", cpos);
      continue;
    }

    if( image_processing_options_.input_image_processor ) {
      if( !image_processing_options_.input_image_processor->process(reference_frame, reference_mask) ) {
        CF_ERROR("input_image_processor->process(pos=%d) fails", cpos);
        continue;
      }
    }

    if( reference_frame.channels() != 1 ) {
      if( !extract_channel(reference_frame, reference_frame, reference_mask, reference_mask, master_channel) ) {
        CF_ERROR("extract_channel(pos=%d, master_channel='%s') fails", cpos, toString(master_channel));
        return false;
      }
    }

    image_transform.set_translation(0, 0);

    if ( !ecch.align(reference_frame, reference_mask) ) {
      CF_ERROR("ecch.align(pos=%d) fails", cpos);
      continue;
    }

    CF_DEBUG("frame %d: rho=%g %d/%d iterations eps=%g/%g", cpos,
        ecc.rho(),
        ecc.num_iterations(), ecc.max_iterations(),
        ecc.eps(), ecc.max_eps());

    cv::remap(reference_frame, reference_frame,
        ecc.current_remap(), cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::remap(reference_mask, reference_mask,
        ecc.current_remap(), cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_REFLECT101);

    cv::compare(reference_mask, 250, reference_mask,
        cv::CMP_GE);

    if( output_options_.debug_frame_registration ) {

      const std::string filename =
          ssprintf("%s/debug/jovian_derotation/reference.%03d.tiff",
              output_path_.c_str(),
              cpos);

      cv::Mat tmp;

      cv::cvtColor(reference_frame, tmp, cv::COLOR_GRAY2BGR);
      drawRotatedRectange(tmp, jovian_derotation.jovian_ellipse(), CV_RGB(0, 1, 0));

      if( !save_image(tmp, reference_mask, filename) ) {
        CF_ERROR("save_image(%s) fails", filename.c_str());
        return false;
      }
    }

    CF_DEBUG("setup_frame_registration()");
    if ( !setup_frame_registration(frame_registration_, reference_frame, reference_mask) ) {
      CF_ERROR("setup_frame_registration(pos=%d) fails", cpos);
      return false;
    }

    if ( frame_accumulation_ ) {
      frame_accumulation_->clear();
    }


    int startpos, endpos;

    if( (startpos = cpos - context_size / 2) < 0 ) {
      startpos = 0;
      endpos = std::min(startpos + context_size, input_sequence_->size());
    }
    else if( (endpos = cpos + context_size / 2) > input_sequence_->size() ) {
      endpos = input_sequence_->size();
      startpos = std::max(0, endpos - context_size);
    }
    else {
      endpos = startpos + context_size;
    }

    CF_DEBUG("cpos=%d startpos=%d endpos=%d", cpos, startpos, endpos);

    if ( !process_input_sequence(input_sequence_, startpos, endpos, false) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    if ( frame_accumulation_ ) {

      cv::Mat accumulated_image;
      cv::Mat1b accumulated_mask;

      if ( true ) {
        lock_guard lock(accumulator_lock_);

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

      std::string filename =
          ssprintf("%s/jovian/%s%s.%03d.32F.tiff",
              output_path_.c_str(),
              csequence_name(),
              output_file_name_postfix_.c_str(),
              cpos);

      CF_DEBUG("Saving '%s'", filename.c_str());
      if( !write_image(output_file_name_ = filename, output_options_, accumulated_image, accumulated_mask) ) {
        CF_ERROR("write_image('%s') fails", filename.c_str());
        return false;
      }
    }

  }


  CF_DEBUG("Finished");

  return true;
}

bool c_image_stacking_pipeline::run_image_stacking()
{
  std::string output_file_name;
  cv::Mat2f upscaled_remap;
  cv::Mat tmp;
  bool fOk;


  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  /////////////////////////////////////////////////////////////////////////////

  if( image_registration_options_.enable_frame_registration ) {

    // SETUP FRAME REGISTRATION

    cv::Mat reference_frame, reference_mask;

    if( !create_reference_frame(image_registration_options_, reference_frame, reference_mask) ) {
      CF_ERROR("create_reference_frame() fails");
      return false;
    }

    if ( !(frame_registration_ = create_frame_registration(image_registration_options_)) ) {
      CF_FATAL("create_frame_registration() fails");
      return false;
    }

    if ( !setup_frame_registration(frame_registration_, reference_frame, reference_mask) ) {
      CF_ERROR("setup_frame_registration() fails");
      return false;
    }

  }


  /////////////////////////////////////////////////////////////////////////////

  if ( !setup_frame_accumulation() ) {
    CF_ERROR("setup_frame_accumulation()() fails");
    return false;
  }


  /////////////////////////////////////////////////////////////////////////////

  set_pipeline_stage(stacking_stage_in_progress);

  if ( !input_sequence_->open() ) {
    set_status_msg("ERROR: input_sequence->open() fails");
    return false;
  }

  CF_DEBUG("input_sequence->size()=%d", input_sequence_->size());

  set_status_msg("RUNNING ...");



  const int start_pos =
      std::max(input_options_.start_frame_index, 0);

  const int end_pos =
      input_options_.max_input_frames < 1 ?
          input_sequence_->size() :
          std::min(input_sequence_->size(),
              input_options_.start_frame_index + input_options_.max_input_frames);

  if ( !(fOk = process_input_sequence(input_sequence_, start_pos, end_pos, false)) ) {
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
      lock_guard lock(accumulator_lock_);

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
    if ( flow_accumulation_ ) {

      cv::Mat accumulated_flow;
      double sharpenScale = 0;

      if ( !flow_accumulation_->compute(accumulated_flow) ) {
        CF_ERROR("ERROR: flow_accumulation_->compute() fails");
      }
      else {


        if ( master_frame_options().accumulated_sharpen_factor > 0 ) {
          if ( frame_accumulation_->accumulated_frames() > 1 ) {

            if ( output_options_.dump_reference_data_for_debug ) {
              write_image(ssprintf("%s/%s-before-sharpen.tiff",
                  output_path_.c_str(), csequence_name()),
                  output_options_,
                  accumulated_image,
                  accumulated_mask);
            }

            sharpenScale =
                master_frame_options().accumulated_sharpen_factor
                    * sqrt(frame_accumulation_->accumulated_frames());

            fftSharpenR1(accumulated_image, accumulated_image,
                sharpenScale, false);

          }
        }

        if ( accumulated_flow.size() != accumulated_image.size() ) {
          upscale_optflow(upscale_options_.upscale_option,
              accumulated_flow,
              accumulated_flow);
        }

        if ( output_options_.dump_reference_data_for_debug ) {

          save_image(flow_accumulation_->counter(),
              ssprintf("%s/%s-masterflow-counter.tiff", output_path_.c_str(),
                  csequence_name()));

          save_image(accumulated_flow,
              ssprintf("%s/%s-masterflow.flo", output_path_.c_str(),
                  csequence_name()));

          write_image(ssprintf("%s/%s-before-flow_compensation.tiff",
              output_path_.c_str(), csequence_name()),
              output_options_,
              accumulated_image,
              accumulated_mask);
        }

        accumulated_flow =
            flow2remap(accumulated_flow,
                accumulated_mask);

        // FIXME: BORDER !!!
        cv::remap(accumulated_image, accumulated_image, accumulated_flow,
            cv::noArray(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

        if( !accumulated_mask.empty() ) {

          cv::remap(accumulated_mask, accumulated_mask, accumulated_flow,
              cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

          cv::compare(accumulated_mask, 255, accumulated_mask, cv::CMP_GE);
        }

        if ( sharpenScale > 0 ) {

          fftSharpenR1(accumulated_image, accumulated_image,
              -sharpenScale, false);

        }
      }
    }


    if ( output_file_name.empty() ) {

      output_file_name =
          ssprintf("%s/%s%s.32F.tiff",
              output_path_.c_str(),
              csequence_name(),
              output_file_name_postfix_.c_str());
    }




    CF_DEBUG("Saving '%s'", output_file_name.c_str());
    if ( !write_image(output_file_name_ = output_file_name, output_options_, accumulated_image, accumulated_mask) ) {
      CF_ERROR("write_image('%s') fails", output_file_name.c_str());
    }


    if ( image_processing_options_.accumulated_image_processor ) {

      if ( !image_processing_options_.accumulated_image_processor->process(accumulated_image, accumulated_mask) ) {
        CF_ERROR("post-processor '%s' fails", image_processing_options_.accumulated_image_processor->cname());
      }
      else {

        output_file_name =
            ssprintf("%s/%s%s.32F.PP.tiff",
                output_path_.c_str(),
                csequence_name(),
                output_file_name_postfix_.c_str());

        CF_DEBUG("Saving '%s'", output_file_name.c_str());
        if ( !write_image(output_file_name_ = output_file_name, output_options_, accumulated_image, accumulated_mask) ) {
          CF_ERROR("write_image('%s') fails", output_file_name.c_str());
        }
      }
    }

  }


  //////////////////

  set_status_msg("FINISHED");

  return true;
}



bool c_image_stacking_pipeline::setup_frame_registration(const c_frame_registration::sptr & frame_registration,
    cv::Mat & reference_frame, cv::Mat & reference_mask)
{

  const c_image_registration_options & registration_options =
      frame_registration->options();

  const c_master_frame_options & master_options =
      registration_options.master_frame_options;

  if( upscale_required(frame_upscale_before_align, false) ) {

    upscale_image(upscale_options().upscale_option,
        reference_frame, reference_mask,
        reference_frame, reference_mask);
  }

  if( output_options_.debug_frame_registration ) {
    frame_registration_->set_debug_path(ssprintf("%s/debug/reference_frame",
        output_path_.c_str()));
  }

  if( image_processing_options_.ecc_image_processor && master_options.apply_input_frame_processors  ) {
    frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(image_processing_options_));
  }

  if ( !frame_registration_->setup_reference_frame(reference_frame, reference_mask) ) {
    CF_ERROR("ERROR: frame_registration_->setup_reference_frame() fails");
    return false;
  }

  if( image_processing_options_.ecc_image_processor && !master_options.apply_input_frame_processors ) {
    frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(image_processing_options_));
  }

  if( output_options_.debug_frame_registration ) {
    frame_registration_->set_debug_path("");
  }

  if ( output_options_.debug_frame_registration && frame_registration_->options().eccflow.enabled  ) {

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

bool c_image_stacking_pipeline::setup_frame_accumulation()
{
  if ( accumulation_options_.accumulation_method != frame_accumulation_none ) {

    lock_guard lock(accumulator_lock_);

    if ( !(frame_accumulation_ = create_frame_accumulation()) ) {
      CF_ERROR("ERROR: create_frame_accumulation() fails");
      return false;
    }
  }

  return true;
}

bool c_image_stacking_pipeline::create_reference_frame(const c_image_registration_options & registration_options,
    cv::Mat & reference_frame, cv::Mat & reference_mask)
{
  set_status_msg("SELECT REFERENCE FRAME ...");

  c_input_sequence::sptr master_sequence;

  int master_source_index = -1;
  int master_frame_index = -1;
  int max_frames_to_stack = 0;

  bool is_external_master_file = false;

  //  cv::Mat reference_frame;
  //  cv::Mat reference_mask;

  const c_master_frame_options & master_options =
      registration_options.master_frame_options;

  std::string master_filename =
      master_options.master_fiename;

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
              return s->filename() == master_options.master_fiename;
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

  if( !create_reference_frame(registration_options, master_sequence, is_external_master_file,
      master_frame_pos, max_frames_to_stack, reference_frame, reference_mask) ) {
    CF_FATAL("ERROR: create_reference_frame() fails");
    return false;
  }

  CF_DEBUG("Reference frame : %dx%d depth=%d channels=%d",
      reference_frame.cols, reference_frame.rows,
      reference_frame.depth(),
      reference_frame.channels());

  CF_DEBUG("Reference mask : %dx%d depth=%d channels=%d",
      reference_mask.cols, reference_mask.rows,
      reference_mask.depth(),
      reference_mask.channels());

  ecc_normalization_noise_ =
      compute_image_noise(reference_frame, reference_mask,
          registration_options.registration_channel);

  if ( master_options.save_master_frame ) {
    write_image(ssprintf("%s/%s-master.tiff", output_path_.c_str(), csequence_name()),
        output_options_,
        reference_frame,
        reference_mask);
  }

  return true;
}

bool c_image_stacking_pipeline::create_reference_frame(const c_image_registration_options & registration_options,
    const c_input_sequence::sptr & input_sequence, bool is_external_master_file,
    int master_frame_pos, int max_frames_to_stack,
    cv::Mat & reference_frame,
    cv::Mat & reference_mask)
{
  INSTRUMENT_REGION("");

  const c_master_frame_options & master_options =
      registration_options.master_frame_options;

  if ( !input_sequence->seek(master_frame_pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(master_frame_pos=%d) fails", master_frame_pos);
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  CF_DEBUG("XXX: is_external_master_file=%d master_frame_pos=%d input_sequence->current_pos=%d",
      is_external_master_file,
      master_frame_pos,
      input_sequence->current_pos());

  if ( !read_input_frame(input_sequence, reference_frame, reference_mask, is_external_master_file) ) {
    CF_FATAL("read_input_frame(reference_frame) fails for master_frame_pos=%d",
        master_frame_pos);
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if ( reference_frame.empty() ) {
    CF_ERROR("read_input_frame(reference_frame) returns empty frame for master_frame_pos=%d",
        master_frame_pos);
    return false;
  }

  if ( !select_image_roi(roi_selection_, reference_frame, reference_mask, reference_frame, reference_mask) ) {
    CF_FATAL("select_image_roi(reference_frame) fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if( image_processing_options_.input_image_processor ) {
    if( master_options.apply_input_frame_processors ) { // !is_external_master_file ||
      if( !image_processing_options_.input_image_processor->process(reference_frame, reference_mask) ) {
        CF_ERROR("input_image_processor->process(reference_frame) fails");
        return false;
      }
    }
  }

  if ( canceled() ) {
    return false;
  }

  if ( reference_frame.channels() > 1 ) {

    const color_channel_type master_channel =
        image_registration_options_.registration_channel;

    if ( !extract_channel(reference_frame, reference_frame, cv::noArray(), cv::noArray(), master_channel) ) {
      CF_ERROR("extract_channel(master_channel=%d) fails",master_channel);
      return false;
    }

    if ( canceled() ) {
      return false;
    }
  }

  // setup master index indicator
 //this->master_frame_index_ = master_frame_index;

 if ( max_frames_to_stack < 2 || input_sequence->size() < 2 ) {

    // Use single frame as reference

    reference_sharpness_ =
        c_sharpness_norm_measure().measure(
            reference_frame,
            reference_mask);

  }
  else {

    // Generate from sequence

    c_image_registration_options master_registration_options =
        registration_options;

//    if ( master_options.feature_scale > 0 ) {
//      master_registration_options.feature_registration.enabled = true;
//      master_registration_options.feature_registration.scale = master_options.feature_scale;
//    }
//    else {
//      master_registration_options.feature_registration.enabled = false;
//      master_registration_options.feature_registration.scale = 0;
//    }
//
//    if( master_options.ecc_scale > 0 ) {
//      master_registration_options.ecc.enabled = true;
//      master_registration_options.ecc.scale = master_options.feature_scale;
//    }
//    else {
//      master_registration_options.ecc.enabled = false;
//      master_registration_options.ecc.scale = 0;
//    }

    if( master_options.eccflow_scale > 1 ) {
      master_registration_options.eccflow.enabled = true;
      master_registration_options.eccflow.support_scale = master_options.eccflow_scale;
    }
    else {
      master_registration_options.eccflow.enabled = false;
      master_registration_options.eccflow.support_scale = 0;
    }

    CF_DEBUG("eccflow.enabled=%d ecc.enabled=%d feature_registration.enabled=%d",
        master_registration_options.eccflow.enabled,
        master_registration_options.ecc.enabled,
        master_registration_options.feature_registration.enabled);

    if ( !(frame_registration_ = create_frame_registration(master_registration_options)) ) {
      CF_FATAL("create_frame_registration(master_registration_options) fails");
      return false;
    }

    // disable jovian derotation for master frame generator
    frame_registration_->options().jovian_derotation.enabled = false;

    if( image_processing_options_.ecc_image_processor && master_options.apply_input_frame_processors  ) {
      frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(image_processing_options_));
    }

    if ( !frame_registration_->setup_reference_frame(reference_frame, reference_mask) ) {
      CF_FATAL("frame_registration_->setup_referece_frame() fails");
      return false;
    }

    if( image_processing_options_.ecc_image_processor && !master_options.apply_input_frame_processors  ) {
      frame_registration_->set_ecc_image_preprocessor(create_ecc_image_preprocessor(image_processing_options_));
    }

//    if ( weights_required() ) {
//      compute_weights(reference_frame, reference_mask,
//          reference_weights_);
//    }

    if ( true ) {
      lock_guard lock(accumulator_lock_);

      frame_accumulation_.reset(new c_frame_weigthed_average());

      if ( master_options.master_sharpen_factor > 0 ) {
        sharpness_norm_accumulation_.reset(new c_sharpness_norm_measure());
      }
    }

    if ( canceled() ) {
      return false;
    }

    const int start_frame_index =
        std::max(0, input_options_.start_frame_index);

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


    if ( max_frames_to_stack > input_sequence->size() ) {
      max_frames_to_stack = input_sequence->size();
    }

    CF_DEBUG("input_options.start_frame_index=%d \n"
        "master_frame_pos=%d \n"
        "max_frames_to_stack=%d \n"
        "input_sequence->size()=%d\n"
        "startpos=%d\n"
        "endpos=%d\n",
        input_options_.start_frame_index,
        master_frame_pos,
        max_frames_to_stack,
        input_sequence->size(),
        startpos,
        endpos);

    if ( !process_input_sequence(input_sequence, startpos, endpos, true) ) {
      CF_ERROR("process_input_sequence() fails");
      return false;
    }

    // Reset master index indicator because master frame was generated from a sequence, not just a single frame

    if ( canceled() ) {
      return false;
    }

    // Read accumulators back
    if ( true ) {
      lock_guard lock(accumulator_lock_);

      if ( frame_accumulation_->accumulated_frames() < 1 ) {
        CF_ERROR("ERROR: No frames accumulated for reference frame");
        return false;
      }

      if ( !frame_accumulation_->compute(reference_frame, reference_mask) ) {
        CF_ERROR("ERROR: frame_accumulation_->compute() fails");
        return false;
      }
#if 1
      linear_interpolation_inpaint(reference_frame,
          reference_mask,
          reference_frame);
#else
      average_pyramid_inpaint(reference_frame,
          reference_mask,
          reference_frame);
#endif

      // sharpen reference image
      if ( sharpness_norm_accumulation_ ) {

        reference_sharpness_ =
            sharpness_norm_accumulation_->average();

        const double current_sharpeness =
            sharpness_norm_accumulation_->measure(reference_frame,
                reference_mask);

        const double alpha =
            1 - master_options.master_sharpen_factor * current_sharpeness / reference_sharpness_;

        //        CF_DEBUG("XX MASTER: current sharpeness = %g averaged sharpeness = %g alpha=%g",
        //            current_sharpeness,
        //            reference_sharpness_,
        //            alpha);

        if ( output_options().dump_reference_data_for_debug ) {

          save_image(reference_frame,
              ssprintf("%s/%s-initial_reference_frame.tiff",
                  output_path_.c_str(),
                  csequence_name()));
        }

        if ( alpha < 1 ) {

          unsharp_mask(reference_frame, reference_frame,
              sharpness_norm_accumulation_->sigma(),
              alpha, 0, 1);

          if ( output_options().dump_reference_data_for_debug ) {

            const double current_sharpeness =
                sharpness_norm_accumulation_->measure(reference_frame,
                    reference_mask);

//            CF_DEBUG("AFTER UNSHARP: sharpeness = %g",
//                current_sharpeness);

            if ( !master_options.save_master_frame ) {

              save_image(reference_frame, ssprintf("%s/%s-reference_frame_after_sharpenning.tiff",
                  output_path_.c_str(),
                  csequence_name()));

            }
          }
        }
      }

    }

    if ( canceled() ) {
      return false;
    }
  }


  if ( true ) {
    lock_guard lock(accumulator_lock_);
    sharpness_norm_accumulation_.reset();
    flow_accumulation_.reset();
    frame_accumulation_.reset();
    frame_registration_.reset();
  }

  return true;
}


bool c_image_stacking_pipeline::process_input_sequence(const c_input_sequence::sptr & input_sequence, int startpos, int endpos,
    bool generating_master_frame )
{
  INSTRUMENT_REGION("");

  cv::Mat current_frame, current_mask, current_weights;
  cv::Mat2f current_remap;

  c_output_frame_writer output_preprocessed_frames_writer;
  c_output_frame_writer output_aligned_frames_writer;
  c_output_frame_writer output_ecc_writer;
  c_output_frame_writer output_postprocessed_frames_writer;
  c_output_frame_writer output_accumulation_masks_writer;
  c_output_frame_writer output_incremental_frame_writer;

  if ( !input_sequence->seek(startpos) ) {
    CF_ERROR("input_sequence->seek(startpos=%d) fails", startpos);
    return false;
  }

  total_frames_ = endpos - startpos;

  if ( total_frames_ < 1 ) {
    CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1", total_frames_);
    return false;
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

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    INSTRUMENT_REGION("body");

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

    if ( !read_input_frame(input_sequence, current_frame, current_mask, false) ) {
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

    if( image_processing_options_.input_image_processor ) {
      if( !generating_master_frame || image_registration_options_.master_frame_options.apply_input_frame_processors ) {
        if( !image_processing_options_.input_image_processor->process(current_frame, current_mask) ) {
          CF_ERROR("input_image_processor->process(current_frame) fails");
          continue;
        }
      }
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if( output_options_.save_preprocessed_frames && !generating_master_frame ) {

      if( !save_preprocessed_frame(current_frame, current_mask,
          output_preprocessed_frames_writer,
          input_sequence->current_pos() - 1) ) {
        CF_ERROR("save_preprocessed_frame() fails");
        return false;
      }

      if( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    // generated master frame msut be always single-channel (grayscale)
    if ( generating_master_frame && current_frame.channels() > 1 ) {

      bool fOk =
          extract_channel(current_frame, current_frame,
              cv::noArray(), cv::noArray(),
              image_registration_options_.registration_channel);

      if ( !fOk ) {
        CF_ERROR("extract_channel(registration_channel=%d) fails",
            image_registration_options_.registration_channel);
        return false;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if( weights_required() ) {
      compute_weights(current_frame, current_mask, current_weights);
      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if ( upscale_required(frame_upscale_before_align, generating_master_frame) ) {

      upscale_image(upscale_options_.upscale_option,
          current_frame, current_mask,
          current_frame, current_mask);

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }

      if( !current_weights.empty() ) {
        upscale_image(upscale_options_.upscale_option,
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

    if ( sharpness_norm_accumulation_ ) {
      sharpness_norm_accumulation_-> add(current_frame,
          current_mask);
    }

    /////////////////////////////////////
    if ( frame_registration_ ) {

      if( output_options_.debug_frame_registration ) {

        if( output_options_.debug_frame_registration_frame_indexes.empty() ) {
          frame_registration_->set_debug_path(ssprintf("%s/debug/registration-%d",
              output_path_.c_str(), input_sequence->current_pos() - 1));
        }
        else {

          const std::vector<int>::const_iterator pos =
              std::find(output_options_.debug_frame_registration_frame_indexes.begin(),
                  output_options_.debug_frame_registration_frame_indexes.end(),
                  input_sequence->current_pos() - 1);

          if( pos == output_options_.debug_frame_registration_frame_indexes.end() ) {
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

      if( !frame_registration_->register_frame(current_frame, current_mask) ) {
        CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + startpos);
        continue;
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

          if( flow_accumulation_->accumulated_frames() < 1 ) {

            const bool fOk =
                flow_accumulation_->initialze(turbulence.size(),
                    turbulence.type(),
                    CV_8UC1);

            if( !fOk ) {
              CF_ERROR("flow_accumulation_->initialze() fails");
              break;
            }
          }

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

      if( !upscale_required(frame_upscale_after_align, generating_master_frame) ) {
        current_remap = frame_registration_->current_remap();
      }
      else {
        current_remap.release();
        upscale_remap(upscale_options_.upscale_option,
            frame_registration_->current_remap(),
            current_remap);
      }


      frame_registration_->custom_remap(current_remap,
          current_frame, current_frame,
          current_mask, current_mask,
          image_registration_options_.interpolation,
          generating_master_frame ?
              ECC_BORDER_REFLECT101 :
              image_registration_options_.border_mode,
          image_registration_options_.border_value);

        if( !current_weights.empty() ) {

          frame_registration_->custom_remap(current_remap,
              current_weights, current_weights,
              cv::noArray(), cv::noArray(),
              image_registration_options_.interpolation,
              ECC_BORDER_CONSTANT);
        }


      if ( !generating_master_frame ) {

        if( !save_aligned_frame(current_frame, current_mask,
            output_aligned_frames_writer,
            input_sequence->current_pos() - 1) ) {

          CF_ERROR("save_aligned_frame() fails");
          return false;
        }

        if ( canceled() ) {
          set_status_msg("canceled");
          break;
        }

        if ( !frame_registration_->current_ecc_image().empty() ) {

          if ( !save_ecc_frame(frame_registration_->current_ecc_image(),
              frame_registration_->current_ecc_mask(),
              output_ecc_writer,
              input_sequence->current_pos() - 1) ) {

            CF_ERROR("save_ecc_frame() fails");
            return false;
          }
        }

        if ( canceled() ) {
          set_status_msg("canceled");
          break;
        }
      }

      if( image_processing_options_.aligned_image_processor ) {

        image_processing_options_.aligned_image_processor->process(current_frame, current_mask);
        if ( canceled() ) {
          set_status_msg("canceled");
          break;
        }

        if( output_options_.save_processed_aligned_frames ) {

          if( !save_postprocessed_frame(current_frame, current_mask,
              output_postprocessed_frames_writer,
              input_sequence->current_pos() - 1) ) {

            CF_ERROR("save_postprocessed_frame() fails");
            return false;
          }

          if ( canceled() ) {
            set_status_msg("canceled");
            break;
          }
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

    if ( output_options_.save_accumulation_masks ) {

      if( !save_accumulation_mask(current_frame, current_mask,
          output_accumulation_masks_writer,
          input_sequence->current_pos() - 1) ) {

        CF_ERROR("save_accumulation_mask() fails");
        return false;
      }

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if ( frame_accumulation_ ) {

      if( c_bayer_average *bayer_average = dynamic_cast<c_bayer_average*>(frame_accumulation_.get()) ) {

        lock_guard lock(accumulator_lock_);

        if( bayer_average->accumulated_frames() < 1 ) {
          bayer_average->set_bayer_pattern(raw_bayer_colorid_);
          bayer_average->initialze(raw_bayer_image_.size());
        }


        static const cv::Mat2f empty_remap;
        bayer_average->set_remap(frame_registration_ ? frame_registration_->current_remap() : empty_remap);
        bayer_average->add(raw_bayer_image_, current_mask);

      }
      else {

        lock_guard lock(accumulator_lock_);

        if( frame_accumulation_->accumulated_frames() < 1 ) {

          const bool fok =
              frame_accumulation_->initialze(current_frame.size(),
                  current_frame.type(),
                  current_mask.empty() ? CV_8UC1 :
                      current_mask.type());

          if( !fok ) {
            CF_ERROR("frame_accumulation_->initialze() fails");
            return false;
          }
        }

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

      if( !generating_master_frame && output_options_.save_incremental_frames ) {

        cv::Mat accumulated_frame, accumulated_mask;

        if( true ) {
          lock_guard lock(accumulator_lock_);
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
              image_processing_options_.incremental_frame_processor;

          if( proc && !proc->process(accumulated_frame, accumulated_mask) ) {
            CF_ERROR("image processor '%s' fails", proc->cname());
            return false;
          }

          if( !save_incremental_frame(accumulated_frame, accumulated_mask,
              output_incremental_frame_writer,
              input_sequence->current_pos() - 1) ) {

            CF_ERROR("save_incremental_frame() fails");
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
    bool is_external_master_frame) const
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

    if ( input_options_.enable_bground_normalization ) {
      nomalize_image_histogramm(output_image, output_mask, output_image,
          input_options_.background_normalization_options,
          input_sequence->colorid());
    }
  }

  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if( input_options_.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }

    if ( input_options_.filter_bad_pixels ) {
      remove_bad_pixels(output_image, input_options_, false);
    }

    if( output_image.depth() != CV_32F ) {
      output_image.convertTo(output_image, CV_32F,
          1. / ((1 << input_sequence->bpp())));
    }

  }
  else {

    const DEBAYER_ALGORITHM algo =
        input_options_.debayer_method;

    if ( accumulation_options_.accumulation_method == frame_accumulation_bayer_average ) {

      raw_bayer_colorid_ =
          input_sequence->colorid();

      if( output_image.depth() == CV_32F ) {
        output_image.copyTo(raw_bayer_image_);
      }
      else {
        output_image.convertTo(raw_bayer_image_, CV_32F,
            1. / ((1 << input_sequence->bpp())));
      }

      if( input_options_.filter_bad_pixels && input_options_.bad_pixels_variation_threshold > 0 ) {
        if( !bayer_denoise(raw_bayer_image_, input_options_.bad_pixels_variation_threshold) ) {
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
        if( input_options_.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if ( input_options_.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options_, true);
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
        if( input_options_.detect_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if( input_options_.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options_, true);
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
            algo, toString(algo));
        return false;
    }
  }

  if( input_options_.enable_color_maxtrix && input_sequence->has_color_matrix() && output_image.channels() == 3 ) {
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

  if ( !output_mask.empty() && input_options_.inpaint_missing_pixels ) {
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

bool c_image_stacking_pipeline::weights_required() const
{
  const c_frame_accumulation_options & opts =
      accumulation_options();

  return opts.accumulation_method == frame_accumulation_weighted_average &&
      opts.lpg.dscale >= 0 && opts.lpg.k >= 0;
}

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  const c_frame_accumulation_options & acc_options =
      accumulation_options();

  c_lpg_sharpness_measure::create_map(src, dst, acc_options.lpg);
  if ( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }

//  CF_DEBUG("dst: %dx%d depth=%d channels=%d", dst.cols, dst.rows, dst.depth(), dst.channels());
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
  lock_guard lock(accumulator_lock_);

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


bool c_image_stacking_pipeline::save_preprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_output_frame_writer & output_writer,
    int seqindex) const
{

  if ( !output_options_.save_preprocessed_frames ) {
    return true;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_preprocessed_frames_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s-preproc.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return false;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_preprocessed_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  return output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);
}

bool c_image_stacking_pipeline::save_aligned_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_output_frame_writer & output_writer,
    int seqindex ) const
{
  if ( !output_options_.save_aligned_frames ) {
    return true;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_aligned_frames_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s-aligned.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",
          pathfilename.c_str(),
          strerror(errno));
      return false;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_aligned_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  return output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}

bool c_image_stacking_pipeline::save_ecc_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_output_frame_writer & output_writer,
    int seqindex) const
{

  if ( !output_options_.save_ecc_frames ) {
    return true;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_ecc_frames_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s-ecc.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return false;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_ecc_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  return output_writer.write(current_frame, cv::noArray()/*current_mask*/,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}


bool c_image_stacking_pipeline::save_postprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_output_frame_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_processed_aligned_frames ) {
    return true;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_postprocessed_frames_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s-postproc.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return false;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_processed_aligned_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  return output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);
}

bool c_image_stacking_pipeline::save_incremental_frame(const cv::Mat & accumulated_frame, const cv::Mat & accumulated_mask,
    c_output_frame_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_incremental_frames ) {
    return true;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_incremental_frames_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s.acc.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return false;
    }



    output_writer.open(pathfilename,
        accumulated_frame.size(),
        accumulated_frame.channels() > 1,
        output_options_.save_incremental_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  return output_writer.write(accumulated_frame, accumulated_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}

bool c_image_stacking_pipeline::save_accumulation_mask(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_output_frame_writer & output_writer,
    int seqindex) const
{
  if ( !output_options_.save_accumulation_masks ) {
    return false;
  }

  if ( !output_writer.is_open() ) {

    std::string pathfilename =
        output_options_.output_accumulation_masks_filename;

    if( pathfilename.empty() ) {
      pathfilename =
          ssprintf("%s-masks.avi",
              csequence_name());
    }

    if ( !is_absolute_path(pathfilename)  ) {
      pathfilename = ssprintf("%s/%s", output_path_.c_str(),
          pathfilename.c_str());
    }

    if ( !create_path(get_parent_directory(pathfilename)) ) {
      CF_ERROR("ERROR: create_path() fails for '%s' : %s",  pathfilename.c_str(), strerror(errno));
      return false;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        false,
        output_options_.save_accumulation_masks_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return false;
    }
  }

  if ( !current_mask.empty() ) {
    if ( !output_writer.write(current_mask, cv::noArray(), false, seqindex) ) {
      CF_ERROR("output_writer.write() fails");
      return false;
    }
  }
  else {
    if ( !output_writer.write(cv::Mat1b(current_frame.size(), 255), cv::noArray(), false, seqindex) ) {
      CF_ERROR("output_writer.write() fails");
      return false;
    }
  }
  return true;
}

int c_image_stacking_pipeline::select_master_frame(const c_input_sequence::sptr & input_sequence)
{
  INSTRUMENT_REGION("");

  const c_master_frame_options &master_options =
      master_frame_options();

  int selected_master_frame_index = 0;

  selected_master_frame_.release();
  selected_master_frame_mask_.release();

  switch (master_options.master_selection_method) {

    case master_frame_specific_index:
      selected_master_frame_index = image_registration_options_.master_frame_options.master_frame_index;
      break;

    case master_frame_middle_index:
      selected_master_frame_index = input_sequence->size() / 2;
      break;

    case master_frame_best_of_100_in_middle: {

      c_lpg_sharpness_measure measure;

      measure.set_k(6);
      measure.set_dscale(1);
      measure.set_uscale(6);
      measure.set_avgchannel(true);
      measure.set_squared(false);

      constexpr int max_frames_to_scan = 200;

      CF_DEBUG("Scan %d frames around of middle %d",
          max_frames_to_scan, input_sequence->size() / 2);

      int start_pos, backup_current_pos;

      if( input_sequence->size() <= max_frames_to_scan ) {
        start_pos = 0;
      }
      else {
        start_pos = input_sequence->size() / 2 - max_frames_to_scan / 2;
      }

      //input_sequence->set_auto_debayer(DEBAYER_DISABLE);
      input_sequence->set_auto_apply_color_matrix(false);

      backup_current_pos = input_sequence->current_pos();
      input_sequence->seek(start_pos);

      cv::Mat image, mask, dogs;
      int current_index, best_index = 0;
      double current_metric, best_metric = 0;

      total_frames_ = max_frames_to_scan;
      processed_frames_ = 0;
      accumulated_frames_ = 0;
      on_status_update();

      for( current_index = 0; processed_frames_ < total_frames_;
          processed_frames_ = ++current_index, on_frame_processed() ) {

        if ( !input_sequence->read(image, &mask) ) {
          CF_FATAL("input_sequence->read() fails\n");
          return false;
        }

        if( canceled() ) {
          break;
        }

        if( !image.empty() ) {

          if( is_bayer_pattern(input_sequence->colorid()) ) {
            if( !extract_bayer_planes(image, image, input_sequence->colorid()) ) {
              CF_ERROR("extract_bayer_planes() fails");
              return false;
            }
          }

          current_metric =
              measure.compute(image)[0];

          if( current_metric > best_metric ) {

            best_metric = current_metric;
            best_index = current_index;

            if( is_bayer_pattern(input_sequence->colorid()) ) {
              if( !nninterpolation(image, image, input_sequence->colorid()) ) {
                CF_ERROR("nninterpolation() fails");
                return false;
              }
            }

            accumulator_lock_.lock();
            image.copyTo(selected_master_frame_);
            mask.copyTo(selected_master_frame_mask_);
            accumulator_lock_.unlock();

            set_status_msg(ssprintf("SELECT REFERENCE FRAME...\n"
                "BEST: INDEX=%d METRIC: %g",
                best_index + start_pos,
                best_metric));

            // on_selected_master_frame_changed();
          }
        }
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
    serialize_base_input_options(section, save, input_options_);
    SERIALIZE_OPTION(section, save, input_options_, anscombe);
  }

  if( (section = get_group(settings, save, "roi")) ) {
    SERIALIZE_OPTION(section, save, roi_selection_options_, method);
    SERIALIZE_OPTION(section, save, roi_selection_options_, rectangle_roi_selection);
    SERIALIZE_OPTION(section, save, roi_selection_options_, planetary_disk_crop_size);
    SERIALIZE_OPTION(section, save, roi_selection_options_, planetary_disk_gbsigma);
    SERIALIZE_OPTION(section, save, roi_selection_options_, planetary_disk_stdev_factor);
  }

  // c_frame_upscale_options upscale_options_;
  if( (section = get_group(settings, save, "upscale")) ) {
    SERIALIZE_OPTION(section, save, upscale_options_, upscale_option);
    SERIALIZE_OPTION(section, save, upscale_options_, upscale_stage);
  }

  // c_frame_registration_options frame_registration_options_;
  if( (section = get_group(settings, save, "frame_registration")) ) {

    SERIALIZE_OPTION(section, save, image_registration_options_, accumulate_and_compensate_turbulent_flow);

    if( (subsection = get_group(section, save, "master_frame")) ) {
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, master_selection_method);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, master_fiename);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, master_frame_index);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, apply_input_frame_processors);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, generate_master_frame);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, max_frames_to_generate_master_frame);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, feature_scale);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, ecc_scale);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, eccflow_scale);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, master_sharpen_factor);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, accumulated_sharpen_factor);
      SERIALIZE_OPTION(subsection, save, image_registration_options_.master_frame_options, save_master_frame);
    }

    if( (subsection = get_group(section, save, "image_registration")) ) {

      SERIALIZE_OPTION(subsection, save, image_registration_options_, enable_frame_registration);
      SERIALIZE_OPTION(subsection, save, image_registration_options_, motion_type);
      SERIALIZE_OPTION(subsection, save, image_registration_options_, registration_channel);
      SERIALIZE_OPTION(subsection, save, image_registration_options_, interpolation);
      SERIALIZE_OPTION(subsection, save, image_registration_options_, border_mode);
      SERIALIZE_OPTION(subsection, save, image_registration_options_, border_value);

      if( (subsubsection = get_group(subsection, save, "image_registration")) ) {

        struct c_feature_based_registration_options & feature_registration =
            image_registration_options_.feature_registration;

        SERIALIZE_OPTION(subsubsection, save, feature_registration, enabled);
        SERIALIZE_OPTION(subsubsection, save, feature_registration, scale);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_extractor"),
            save, feature_registration, sparse_feature_extractor);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_matcher"),
            save, feature_registration, sparse_feature_matcher);
      }

      if( (subsubsection = get_group(subsection, save, "ecc")) ) {

        struct c_ecc_registration_options & ecc =
            image_registration_options_.ecc;

        SERIALIZE_OPTION(subsubsection, save, ecc, enabled);
        SERIALIZE_OPTION(subsubsection, save, ecc, scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, eps);
        SERIALIZE_OPTION(subsubsection, save, ecc, min_rho);
        SERIALIZE_OPTION(subsubsection, save, ecc, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, ecc, update_step_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_noise);
        SERIALIZE_OPTION(subsubsection, save, ecc, normalization_scale);
        SERIALIZE_OPTION(subsubsection, save, ecc, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_minimum_image_size);
        SERIALIZE_OPTION(subsubsection, save, ecc, enable_ecch);
        SERIALIZE_OPTION(subsubsection, save, ecc, ecch_estimate_translation_first);
        SERIALIZE_OPTION(subsubsection, save, ecc, replace_planetary_disk_with_mask);
        SERIALIZE_OPTION(subsubsection, save, ecc, planetary_disk_mask_stdev_factor);
      }

      if( (subsubsection = get_group(subsection, save, "eccflow")) ) {

        struct c_eccflow_registration_options &eccflow =
            image_registration_options_.eccflow;

        SERIALIZE_OPTION(subsubsection, save, eccflow, enabled);
        SERIALIZE_OPTION(subsubsection, save, eccflow, update_multiplier);
        SERIALIZE_OPTION(subsubsection, save, eccflow, input_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, reference_smooth_sigma);
        SERIALIZE_OPTION(subsubsection, save, eccflow, max_iterations);
        SERIALIZE_OPTION(subsubsection, save, eccflow, support_scale);
        SERIALIZE_OPTION(subsubsection, save, eccflow, normalization_scale);
      }


      if( (subsubsection = get_group(subsection, save, "jovian_derotation")) ) {

        struct c_jovian_derotation_options & jovian_derotation =
            image_registration_options_.jovian_derotation;

        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, enabled);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, min_rotation);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, max_rotation);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, max_pyramid_level);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, num_orientations);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, eccflow_support_scale);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, eccflow_normalization_scale);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, eccflow_max_pyramid_level);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, derotate_all_frames);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, derotate_all_frames_max_context_size);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation, rotate_jovian_disk_horizontally);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.ellipse, stdev_factor);
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.ellipse, pca_blur);
        //SERIALIZE_OPTION(subsubsection, save, jovian_derotation.ellipse, force_reference_ellipse);
      }
    }
  }

  // c_frame_accumulation_options accumulation_options_;
  if( (section = get_group(settings, save, "accumulation")) ) {

    SERIALIZE_OPTION(section, save, accumulation_options_, accumulation_method);

    if( (subsection = get_group(section, save, "c_lpg_sharpness_measure")) ) {

      c_lpg_options &m =
          accumulation_options_.lpg;

      SERIALIZE_OPTION(subsection, save, m, k);
      SERIALIZE_OPTION(subsection, save, m, dscale);
      SERIALIZE_OPTION(subsection, save, m, uscale);
      SERIALIZE_OPTION(subsection, save, m, squared);
      SERIALIZE_OPTION(subsection, save, m, avgchannel);
    }

    if( (subsection = get_group(section, save, "c_laplacian_pyramid_focus_stacking")) ) {

      c_laplacian_pyramid_focus_stacking::options &fs =
          accumulation_options_.fs;

      SERIALIZE_OPTION(subsection, save, fs, fusing_policy);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
      SERIALIZE_OPTION(subsection, save, fs, avgchannel);
      SERIALIZE_OPTION(subsection, save, fs, kradius);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
    }
  }

  // c_image_stacking_output_options output_options_;
  if( (section = get_group(settings, save, "output")) ) {

    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, output_preprocessed_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, output_aligned_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, output_ecc_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, output_postprocessed_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, output_accumulation_masks_filename);

    SERIALIZE_OPTION(section, save, output_options_, save_preprocessed_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_aligned_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_ecc_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_processed_aligned_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_incremental_frames);
    SERIALIZE_OPTION(section, save, output_options_, save_accumulation_masks);

    SERIALIZE_OPTION(section, save, output_options_, dump_reference_data_for_debug);
    SERIALIZE_OPTION(section, save, output_options_, write_image_mask_as_alpha_channel);
  }

  if( (section = get_group(settings, save, "image_processing")) ) {
    if( save ) {

      if( image_processing_options_.input_image_processor ) {
        save_settings(settings, "input_image_processor",
            image_processing_options_.input_image_processor->name());
      }
      if( image_processing_options_.ecc_image_processor ) {
        save_settings(settings, "ecc_image_processor",
            image_processing_options_.ecc_image_processor->name());
      }
      if( image_processing_options_.aligned_image_processor ) {
        save_settings(settings, "aligned_image_processor",
            image_processing_options_.aligned_image_processor->name());
      }
      if( image_processing_options_.incremental_frame_processor ) {
        save_settings(settings, "incremental_frame_processor",
            image_processing_options_.incremental_frame_processor->name());
      }
      if( image_processing_options_.accumulated_image_processor ) {
        save_settings(settings, "accumulated_image_processor",
            image_processing_options_.accumulated_image_processor->name());
      }
    }
    else {

      std::string s;

      if( load_settings(settings, "input_image_processor", &s) && !s.empty() ) {
        image_processing_options_.input_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "ecc_image_processor", &s) && !s.empty() ) {
        image_processing_options_.ecc_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "aligned_image_processor", &s) && !s.empty() ) {
        image_processing_options_.aligned_image_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "incremental_frame_processor", &s) && !s.empty() ) {
        image_processing_options_.incremental_frame_processor =
            c_image_processor_collection::default_instance()->get(s);
      }
      if( load_settings(settings, "accumulated_image_processor", &s) && !s.empty() ) {
        image_processing_options_.accumulated_image_processor =
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
      PIPELINE_CTL(ctrls, input_options_.anscombe, "", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "ROI options", "");
    PIPELINE_CTL(ctrls, roi_selection_options_.method, "ROI selection:", "");
    PIPELINE_CTLC(ctrls, roi_selection_options_.planetary_disk_crop_size, "Crop Size", "",
        (_this->roi_selection_options_.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, roi_selection_options_.planetary_disk_gbsigma, "gbsigma", "",
        (_this->roi_selection_options_.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, roi_selection_options_.planetary_disk_stdev_factor, "Stdev factor", "",
        (_this->roi_selection_options_.method == roi_selection_planetary_disk));
    PIPELINE_CTLC(ctrls, roi_selection_options_.rectangle_roi_selection, "Rectangle:", "",
        (_this->roi_selection_options_.method == roi_selection_rectange_crop));
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Upscale options", "");
    PIPELINE_CTL(ctrls, upscale_options_.upscale_option, "Upscale:", "");
    PIPELINE_CTLC(ctrls, upscale_options_.upscale_stage, "Stage:", "",
        (_this->upscale_options_.upscale_option != frame_upscale_none));
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////

    PIPELINE_CTL_GROUP(ctrls, "Frame registration", "");
      PIPELINE_CTL_IMAGE_REGISTRATION_OPTIONS(ctrls, image_registration_options_);
    PIPELINE_CTL_END_GROUP(ctrls);
    ////////

    PIPELINE_CTL_GROUP(ctrls, "Frame accumulation", "");
      PIPELINE_CTL(ctrls, accumulation_options_.accumulation_method, "Acc. Method", "");

      PIPELINE_CTL_GROUP(ctrls, "weighted_average", "");
      PIPELINE_CTLC(ctrls, accumulation_options_.lpg.k, "K", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, accumulation_options_.lpg.dscale, "dscale", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, accumulation_options_.lpg.uscale, "uscale", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, accumulation_options_.lpg.avgchannel, "avgchannel", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTLC(ctrls, accumulation_options_.lpg.squared, "squared", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_weighted_average));
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "focus_stack", "");
      PIPELINE_CTLC(ctrls, accumulation_options_.fs.fusing_policy, "fusing_policy", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, accumulation_options_.fs.inpaint_mask_holes, "inpaint_mask_holes", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, accumulation_options_.fs.kradius, "kradius", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, accumulation_options_.fs.ksigma, "ksigma", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTLC(ctrls, accumulation_options_.fs.avgchannel, "avgchannel", "", (_this->accumulation_options_.accumulation_method == frame_accumulation_focus_stack));
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.input_image_processor, "input_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.ecc_image_processor, "ecc_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.aligned_image_processor, "aligned_image_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.incremental_frame_processor, "incremental_frame_processor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.accumulated_image_processor, "accumulated_image_processor", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////

    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
    PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
    PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");

    PIPELINE_CTL(ctrls, output_options_.save_preprocessed_frames, "save_preprocessed_frames", "");
    PIPELINE_CTLC(ctrls, output_options_.save_preprocessed_frame_mapping, "save_preprocessed_frame_mapping", "",
        (_this->output_options_.save_preprocessed_frames));
    PIPELINE_CTLC(ctrls, output_options_.output_preprocessed_frames_filename, "preprocessed_frames_filename", "",
        (_this->output_options_.save_preprocessed_frames));

    PIPELINE_CTL(ctrls, output_options_.save_aligned_frames, "save_aligned_frames", "");
    PIPELINE_CTLC(ctrls, output_options_.save_aligned_frame_mapping, "save_aligned_frame_mapping", "",
        (_this->output_options_.save_aligned_frames));
    PIPELINE_CTLC(ctrls, output_options_.output_aligned_frames_filename, "aligned_frames_filename", "",
        (_this->output_options_.save_aligned_frames));

    PIPELINE_CTL(ctrls, output_options_.save_ecc_frames, "save_ecc_frames", "");
    PIPELINE_CTLC(ctrls, output_options_.save_ecc_frame_mapping, "save_ecc_frame_mapping", "",
        (_this->output_options_.save_ecc_frames));
    PIPELINE_CTLC(ctrls, output_options_.output_ecc_frames_filename, "ecc_frames_filename", "",
        (_this->output_options_.save_ecc_frames));

    PIPELINE_CTL(ctrls, output_options_.save_processed_aligned_frames, "save_processed_aligned_frames", "");
    PIPELINE_CTLC(ctrls, output_options_.save_processed_aligned_frame_mapping, "save_processed_aligned_frame_mapping", "",
        (_this->output_options_.save_processed_aligned_frames));
    PIPELINE_CTLC(ctrls, output_options_.output_postprocessed_frames_filename, "postprocessed_frames_filename", "",
        (_this->output_options_.save_processed_aligned_frames));

    PIPELINE_CTL(ctrls, output_options_.save_incremental_frames, "save_incremental_frames", "");
    PIPELINE_CTLC(ctrls, output_options_.save_incremental_frame_mapping, "save_incremental_frame_mapping", "",
        (_this->output_options_.save_incremental_frames));
    PIPELINE_CTLC(ctrls, output_options_.output_incremental_frames_filename, "incremental_frames_filename", "",
        (_this->output_options_.save_incremental_frames));

    PIPELINE_CTL(ctrls, output_options_.save_accumulation_masks, "save_accumulation_masks", "");
    PIPELINE_CTLC(ctrls, output_options_.save_accumulation_masks_frame_mapping, "save_accumulation_masks_frame_mapping", "",
        (_this->output_options_.save_accumulation_masks));
    PIPELINE_CTLC(ctrls, output_options_.output_accumulation_masks_filename, "accumulation_masks_filename", "",
        (_this->output_options_.save_accumulation_masks));

    PIPELINE_CTL(ctrls, output_options_.write_image_mask_as_alpha_channel, "write_image_mask_as_alpha_channel", "");

    PIPELINE_CTL(ctrls, output_options_.dump_reference_data_for_debug, "dump_reference_data_for_debug", "");
    PIPELINE_CTL(ctrls, output_options_.debug_frame_registration, "debug_frame_registration", "");
    PIPELINE_CTLC(ctrls, output_options_.debug_frame_registration_frame_indexes, "debug_frame_registration_frame_indexes", "",
        (_this->output_options_.debug_frame_registration));

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
  image_registration_options_.master_frame_options.master_fiename = master_source_path;
}

std::string c_image_stacking_pipeline::master_source() const
{
  return image_registration_options_.master_frame_options.master_fiename;
}

void c_image_stacking_pipeline::set_master_frame_index(int v)
{
  image_registration_options_.master_frame_options.master_frame_index = v;
}

int c_image_stacking_pipeline::master_frame_index() const
{
  return image_registration_options_.master_frame_options.master_frame_index;
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

  p->input_options_ = this->input_options_;
  p->roi_selection_options_ = this->roi_selection_options_;
  p->upscale_options_ = this->upscale_options_;
  p->accumulation_options_ = this->accumulation_options_;
  p->output_options_ = this->output_options_;
  p->image_processing_options_ = this->image_processing_options_;

  const std::string backup_master_source_fiename =
      p->image_registration_options_.master_frame_options.master_fiename;

  const int backup_master_frame_index =
      p->image_registration_options_.master_frame_options.master_frame_index;

  p->image_registration_options_ = this->image_registration_options_;

  p->image_registration_options_.master_frame_options.master_fiename =
      backup_master_source_fiename;

  p->image_registration_options_.master_frame_options.master_frame_index =
      backup_master_frame_index;

  return true;
}
