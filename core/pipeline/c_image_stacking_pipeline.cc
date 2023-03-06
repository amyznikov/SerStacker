/*
 * c_stacking_pipeline.cc
 *
 *  Created on: Jan 12, 2021
 *      Author: amyznikov
 */

#include "c_image_stacking_pipeline.h"
#include <core/settings/opencv_settings.h>
#include <core/feature2d/feature2d_settings.h>
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
const c_enum_member * members_of<frame_accumulation_method>()
{
  static constexpr c_enum_member members[] = {

      { frame_accumulation_weighted_average, "weighted_average",
          "Simple weighted average with weights proportional to the smoothed sum of squared laplacian and gradient" },

      { frame_accumulation_focus_stack, "focus_stack",
          "Focus stacking based on paper of Wang and Chang 2011"},

      { frame_accumulation_fft, "fft",
          "Stupid experiments with fft-based stacking "},

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
const c_enum_member * members_of<master_frame_selection_method>()
{
  static constexpr c_enum_member members[] = {
      {master_frame_specific_index, "specific_index", },
      {master_frame_middle_index, "middle_index", },
      {master_frame_best_of_100_in_middle, "best_of_100_in_middle", },
      {master_frame_specific_index, nullptr, },
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
}

c_image_stacking_pipeline::~c_image_stacking_pipeline()
{
  cancel(true);
}

void c_image_stacking_pipeline::update_output_path()
{
  if( output_directory_.empty() ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/stacks",
            parent_directory.c_str());

  }
  else if( !is_absolute_path(output_directory_) ) {

    std::string parent_directory =
        input_sequence_->sources().empty() ? "." :
            get_parent_directory(input_sequence_->source(0)->filename());

    if( parent_directory.empty() ) {
      parent_directory = ".";
    }

    output_path_ =
        ssprintf("%s/%s",
            parent_directory.c_str(),
            output_directory_.c_str());
  }
  else {
    output_path_ =
        output_directory_;
  }
}

std::string c_image_stacking_pipeline::output_file_name() const
{
  return output_file_name_;
}

const c_anscombe_transform & c_image_stacking_pipeline::anscombe() const
{
  return anscombe_;
}

c_input_options & c_image_stacking_pipeline::input_options()
{
  return input_options_;
}

const c_input_options & c_image_stacking_pipeline::input_options() const
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
  return frame_registration_options_.image_registration_options.feature_registration.sparse_feature_extractor;
}

const c_sparse_feature_extractor_options & c_image_stacking_pipeline::sparse_feature_extractor_options() const
{
  return frame_registration_options_.image_registration_options.feature_registration.sparse_feature_extractor;
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
  return frame_registration_options_.master_frame_options;
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


const c_master_frame_options & c_image_stacking_pipeline::master_frame_options() const
{
  return frame_registration_options_.master_frame_options;
}

c_frame_registration_options & c_image_stacking_pipeline::frame_registration_options()
{
  return frame_registration_options_;
}

const c_frame_registration_options & c_image_stacking_pipeline::frame_registration_options() const
{
  return frame_registration_options_;
}

c_frame_registration::sptr c_image_stacking_pipeline::create_frame_registration(const c_image_registration_options & options) const
{
  return c_frame_registration::sptr(new c_frame_registration(options));
}

c_frame_registration::sptr c_image_stacking_pipeline::create_frame_registration() const
{
  return create_frame_registration(frame_registration_options_.image_registration_options);
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
  switch ( accumulation_options_.accumulation_method ) {
  case frame_accumulation_weighted_average:
    return c_frame_accumulation::ptr(new c_frame_weigthed_average());
  case frame_accumulation_focus_stack:
    return c_frame_accumulation::ptr(new c_laplacian_pyramid_focus_stacking(accumulation_options_.fs_));
  case frame_accumulation_fft :
    return c_frame_accumulation::ptr(new c_frame_accumulation_with_fft());
  default :
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

  if (true ) {

    lock_guard lock(accumulator_lock_);

    missing_pixel_mask_.release();

    external_master_frame_ = false;
    master_frame_index_ = -1;
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
    if ( !load_image(input_options().darkbayer_filename, darkbayer_) ) {
      CF_ERROR("load_image('%s') fails.", input_options().darkbayer_filename.c_str());
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

  set_pipeline_stage(stacking_stage_idle);
}

bool c_image_stacking_pipeline::run_pipeline()
{
  bool fOk = false;

  const c_jovian_derotation_options &jovian_derotation =
      frame_registration_options().image_registration_options.jovian_derotation;

  const bool do_jovian_derotaton_for_each_frame =
      jovian_derotation.enabled &&
          jovian_derotation.derotate_all_frames &&
          jovian_derotation.derotate_all_frames_max_context_size > 0;

  if( !do_jovian_derotaton_for_each_frame ) {

    if( !(fOk = run_image_stacking()) ) {
      CF_ERROR("run_image_stacking() fails");
    }
  }
  else {

    // save options

    const std::string backup_master_source =
        master_source_;

    const int backup_master_frame_index =
        master_frame_index_;

    const c_master_frame_options backup_master_options =
        master_frame_options();

    const c_input_options backup_input_options =
        input_options();

    try {

      const int num_sources =
          input_sequence()->sources().size();

      const int max_context_size =
          std::min(num_sources, jovian_derotation.derotate_all_frames_max_context_size);

      c_master_frame_options &master_options =
          master_frame_options();

      master_options.master_selection_method = master_frame_specific_index;
      master_frame_index_ = 0;
      input_options_.max_input_frames = max_context_size;

      for( int master_source = 0; master_source < num_sources; ++master_source ) {

        CF_DEBUG("\n\n=============================================\n\n"
            "SELECT master_source %d / %d",
            master_source,
            num_sources);

        master_source_ =
            input_sequence()->source(master_source)->filename();

        if( max_context_size == num_sources ) {
          input_options_.start_frame_index = 0;
        }
        else if( (input_options_.start_frame_index = master_source - max_context_size / 2) < 0 ) {
          input_options_.start_frame_index = 0;
        }
        else if( input_options_.start_frame_index + max_context_size >= num_sources ) {
          input_options_.start_frame_index = num_sources - max_context_size - 1;
        }

        output_file_name_postfix_ =
            ssprintf(".F%03d",
                master_source);

        if( !(fOk = run_image_stacking()) ) {
          CF_ERROR("run_image_stacking() fails for master_source=%d", master_source);
        }
      }

    }
    catch( ... ) {
      CF_ERROR("Exception in jovian loop");

      // restore options

      input_options_ =
          backup_input_options;

      master_frame_options() =
          backup_master_options;

      master_source_ =
          backup_master_source;

      master_frame_index_ =
          backup_master_frame_index;

      throw;
    }

    // restore options

    input_options_ =
        backup_input_options;

    master_frame_options() =
        backup_master_options;

    master_source_ =
        backup_master_source;

    master_frame_index_ =
        backup_master_frame_index;

  }

  return fOk;
}


bool c_image_stacking_pipeline::run_image_stacking()
{
  std::string output_file_name;
  cv::Mat2f upscaled_remap;
  cv::Mat tmp;
  bool fOk;


  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());

  const c_frame_registration_options & registration_options =
      frame_registration_options_;

  const c_jovian_derotation_options & jovian_derotation =
      registration_options.image_registration_options.jovian_derotation;

  CF_ERROR("FIXME: Not saving pipeline config !");
  //  save(ssprintf("%s/%s.cfg",
  //      output_path_.c_str(),
  //      cname()));


  /////////////////////////////////////////////////////////////////////////////
  // SETUP FRAME REGISTRATION

  if ( registration_options.image_registration_options.enable_frame_registration ) {

    set_status_msg("CREATE REFERENCE FRAME ...");

    const c_master_frame_options & master_options =
        master_frame_options();

    c_input_sequence::sptr input_sequence;
    int master_source_index = -1;
    int master_frame_index = -1;
    int max_frames_to_stack = 0;

    cv::Mat reference_frame;
    cv::Mat reference_mask;

    std::string master_file_name =
        master_source_;

    CF_DEBUG("master_options.master_file_name=%s", master_file_name.c_str());

    if ( master_file_name.empty() ) {
      master_file_name =
          input_sequence_->
              source(master_source_index = 0)->filename();
    }
    else {

      std::vector<c_input_source::sptr>::const_iterator source_pos =
          std::find_if(this->input_sequence()->sources().begin(), this->input_sequence()->sources().end(),
              [this](const c_input_source::sptr & s ) -> bool {
                return s->filename() == master_source_;
              });

      if ( source_pos != this->input_sequence_->sources().end() ) {
        master_source_index = source_pos - this->input_sequence_->sources().begin();
      }
    }

    if ( master_source_index >= 0 ) {
      input_sequence = this->input_sequence();
      external_master_frame_ = false;
    }
    else if ( !(input_sequence = c_input_sequence::create(master_file_name)) ) {
      CF_ERROR("ERROR: c_input_sequence::create(master_file_name_=%s) fails", master_file_name.c_str());
      return false;
    }
    else {
      external_master_frame_ = true;
      master_source_index = 0;
    }

    if ( master_source_index >= (int) input_sequence->sources().size() ) {
      CF_FATAL("ERROR: master_source_index=%d exceeds input_sequence->sources().size()=%zu",
          master_source_index, input_sequence->sources().size());
      return false;
    }

    if ( !input_sequence->source(master_source_index)->enabled() ) {
      CF_FATAL("ERROR: master_source_index=%d is NOT enabled in input_sequence",
          master_source_index);
      return false;
    }

    if ( !input_sequence->open() ) {
      CF_FATAL("ERROR: Can not open master input source '%s'",
          master_file_name.c_str());
      return false;
    }

    set_pipeline_stage(stacking_stage_select_master_frame_index);

    master_frame_index =
        select_master_frame(input_sequence);

    if ( canceled() ) {
      return false;
    }

    set_pipeline_stage(stacking_stage_generate_reference_frame);

    set_status_msg("CREATE REFERENCE FRAME ...");

    if ( (master_frame_index /*= master_options.master_frame_index*/) < 0 ) {
      master_frame_index = 0;
    }
    else if ( master_frame_index >= input_sequence->source(master_source_index)->size() ) {
      CF_FATAL("ERROR: invalid master_frame_index_=%d specified for input source '%s'",
          master_frame_index, master_file_name.c_str());
      return false;
    }

    master_frame_index =
        input_sequence->global_pos(
            master_source_index,
            master_frame_index);

    if ( master_options.generate_master_frame ) {
      max_frames_to_stack =
          master_options.max_frames_to_generate_master_frame;
    }

    if ( !create_reference_frame(input_sequence, master_frame_index, max_frames_to_stack,
        reference_frame, reference_mask) ) {
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
            frame_registration_options().image_registration_options.registration_channel);

    if ( master_options.save_master_frame ) {
      write_image(ssprintf("%s/%s-master.tiff", output_path_.c_str(), csequence_name()),
          output_options_,
          reference_frame,
          reference_mask);
    }


    if ( !(frame_registration_ = create_frame_registration()) ) {
      CF_FATAL("create_frame_registration() fails");
      return false;
    }

    if( upscale_required(frame_upscale_before_align) ) {

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
      if( registration_options.image_registration_options.motion_type > IMAGE_MOTION_EUCLIDEAN ||
          registration_options.image_registration_options.eccflow.enabled ) {
        flow_accumulation_.reset(new c_frame_weigthed_average());
      }
    }
  }


  /////////////////////////////////////////////////////////////////////////////

  set_pipeline_stage(stacking_stage_in_progress);

  if ( accumulation_options_.accumulation_method != frame_accumulation_none ) {

    lock_guard lock(accumulator_lock_);

    if ( !(frame_accumulation_ = create_frame_accumulation()) ) {
      CF_ERROR("ERROR: create_frame_accumulation() fails");
      return false;
    }
  }

  /////////////////////////////////////////////////////////////////////////////

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

    if ( jovian_derotation.enabled && jovian_derotation.rotate_jovian_disk_horizontally ) {
//
//      const c_jovian_derotation & jovian_derotation =
//          frame_registration_->jovian_derotation();
//
//      const cv::RotatedRect &E =
//          jovian_derotation.reference_ellipse();
//
//      const cv::Rect &BB =
//          jovian_derotation.reference_bounding_box();
//
//      const cv::Mat T =
//          createEuclideanTransform(E.center.x + BB.x, E.center.y + BB.y,
//              //accumulated_image.cols / 2, accumulated_image.rows / 2,
//              E.center.x + BB.x, E.center.y + BB.y,
//              1.0,
//              -E.angle * CV_PI / 180,
//              CV_32F);
//
//      const cv::Mat M =
//          createRemap(ECC_MOTION_AFFINE,
//              T,
//              accumulated_image.size(),
//              CV_32F);
//
//        cv::remap(accumulated_image, accumulated_image,
//            M, cv::noArray(),
//            cv::INTER_LINEAR,
//            cv::BORDER_REPLICATE);
//
//        cv::remap(accumulated_mask, accumulated_mask,
//            M, cv::noArray(),
//            cv::INTER_LINEAR,
//            cv::BORDER_CONSTANT);
//
//      cv::compare(accumulated_mask, 255,
//          accumulated_mask,
//          cv::CMP_GE);
//
//      output_file_name =
//          ssprintf("%s/%s%s.32F.PPR.tiff",
//              output_path_.c_str(),
//              csequence_name(),
//              output_file_name_postfix_.c_str());
//
//      CF_DEBUG("Saving '%s'", output_file_name.c_str());
//      if( !write_image(output_file_name, output_options, accumulated_image, accumulated_mask) ) {
//        CF_ERROR("write_image('%s') fails", output_file_name.c_str());
//      }
    }
  }




  //////////////////

  set_status_msg("FINISHED");

  return true;
}

bool c_image_stacking_pipeline::create_reference_frame(const c_input_sequence::sptr & input_sequence,
    int master_frame_index, int max_frames_to_stack,
    cv::Mat & reference_frame,
    cv::Mat & reference_mask)
{
  INSTRUMENT_REGION("");

  const c_master_frame_options & master_options =
      master_frame_options();

  master_frame_generation_ = true;

  if ( !input_sequence->seek(master_frame_index) ) {
    CF_ERROR("ERROR: input_sequence->seek(master_frame_index=%d) fails", master_frame_index);
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if ( !read_input_frame(input_sequence, reference_frame, reference_mask) ) {
    CF_FATAL("read_input_frame(reference_frame) fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if ( reference_frame.empty() ) {
    return false;
  }

  if ( !select_image_roi(roi_selection_, reference_frame, reference_mask, reference_frame, reference_mask) ) {
    CF_FATAL("select_image_roi(reference_frame) fails");
    return false;
  }

  if ( canceled() ) {
    return false;
  }

  if( master_options.apply_input_frame_processors  ) {
    if ( image_processing_options_.input_image_processor ) {
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
        frame_registration_options().
            image_registration_options.registration_channel;

    if ( !extract_channel(reference_frame, reference_frame, cv::noArray(), cv::noArray(), master_channel) ) {
      CF_ERROR("extract_channel(master_channel=%d) fails",master_channel);
      return false;
    }

    if ( canceled() ) {
      return false;
    }
  }

  // setup master index indicator
 this->master_frame_index_ = master_frame_index;

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
        frame_registration_options().image_registration_options;

    if ( master_options.eccflow_scale > 1 ) {
      master_registration_options.eccflow.enabled = true;
      master_registration_options.eccflow.support_scale = master_options.eccflow_scale;
    }
    else {
      master_registration_options.eccflow.enabled = false;
      master_registration_options.eccflow.support_scale = 0;
    }

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

    const int startpos =
        std::max(0, std::max(input_options_.start_frame_index,
            master_frame_index - max_frames_to_stack / 2));

    const int endpos =
        std::min( startpos + max_frames_to_stack,
            input_sequence->size());

    if ( max_frames_to_stack > input_sequence->size() ) {
      max_frames_to_stack = input_sequence->size();
    }

    CF_DEBUG("input_options.start_frame_index=%d master_frame_index=%d max_frames_to_stack=%d input_sequence->size()=%d",
        input_options_.start_frame_index, master_frame_index,
        max_frames_to_stack,
        input_sequence->size());

    if ( !process_input_sequence(input_sequence, startpos, endpos) ) {
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

  master_frame_generation_ = false;
  return true;
}


bool c_image_stacking_pipeline::process_input_sequence(const c_input_sequence::sptr & input_sequence, int startpos, int endpos)
{
  INSTRUMENT_REGION("");

  cv::Mat current_frame, current_mask, current_weights;
  cv::Mat2f current_remap;

  c_video_writer output_preprocessed_frames_writer;
  c_video_writer output_aligned_frames_writer;
  c_video_writer output_ecc_writer;
  c_video_writer output_postprocessed_frames_writer;
  c_video_writer output_accumulation_masks_writer;
  c_video_writer output_incremental_frame_writer;

  const c_master_frame_options & master_options =
      master_frame_options();

  const c_frame_registration_options & registration_options =
      frame_registration_options_;

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

  for ( ; processed_frames_ < total_frames_; ++processed_frames_, on_status_changed() ) {

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

    if ( !read_input_frame(input_sequence, current_frame, current_mask) ) {
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
      if( !image_processing_options_.input_image_processor->process(current_frame, current_mask) ) {
        CF_ERROR("input_image_processor->process(current_frame) fails");
        continue;
      }
    }

    if ( canceled() ) {
      set_status_msg("canceled");
      break;
    }

    if ( !master_frame_generation_ ) {

      save_preprocessed_frame(current_frame, current_mask,
          output_preprocessed_frames_writer,
          input_sequence->current_pos() - 1);

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }


    if ( master_frame_generation_ && current_frame.channels() > 1 ) {

      bool fOk =
          extract_channel(current_frame, current_frame,
              cv::noArray(), cv::noArray(),
              frame_registration_options().image_registration_options.registration_channel);

      if ( !fOk ) {
        CF_ERROR("extract_channel(registration_channel=%d) fails",
            frame_registration_options().image_registration_options.registration_channel);
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

    if ( upscale_required(frame_upscale_before_align) ) {

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


      const bool is_master_frame =
          !external_master_frame_ &&
          input_sequence->current_pos() == master_frame_index_ + 1;

      const bool generated_master_frame =
          !master_frame_generation_ && master_options.generate_master_frame;

      if ( is_master_frame && !generated_master_frame ) {

        if ( upscale_required(frame_upscale_after_align) ) {

          upscale_image(upscale_options_.upscale_option,
              current_frame, current_mask,
              current_frame, current_mask);

          if( !current_weights.empty() ) {
            upscale_image(upscale_options_.upscale_option,
                current_weights, cv::noArray(),
                current_weights, cv::noArray());
          }

        }

      }
      else {

        if ( output_options_.debug_frame_registration ) {

          if ( output_options_.debug_frame_registration_frame_indexes.empty()  ) {
            frame_registration_->set_debug_path(ssprintf("%s/debug/registration-%d",
                output_path_.c_str(), input_sequence->current_pos() - 1));
          }
          else {

            const std::vector<int> :: const_iterator pos =
                std::find(output_options_.debug_frame_registration_frame_indexes.begin(),
                    output_options_.debug_frame_registration_frame_indexes.end(),
                    input_sequence->current_pos() - 1);

            if ( pos == output_options_.debug_frame_registration_frame_indexes.end() ) {
              frame_registration_->set_debug_path("");
            }
            else {
              frame_registration_->set_debug_path(ssprintf("%s/debug/registration-%d",
                  output_path_.c_str(), *pos));
            }
          }

          if( canceled() ) {
            set_status_msg("canceled");
            break;
          }
        }


        if ( !frame_registration_->register_frame(current_frame, current_mask) ) {
          CF_ERROR("[F %6d] reg->register_frame() fails\n", processed_frames_ + startpos);
          continue;
        }

        if( canceled() ) {
          set_status_msg("canceled");
          break;
        }

        if ( flow_accumulation_ ) {

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

            CF_DEBUG("flow_accumulation_->add(turbulence)");
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

        if ( !upscale_required(frame_upscale_after_align) ) {
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
            registration_options.image_registration_options.interpolation,
            master_frame_generation_ ?
                ECC_BORDER_REFLECT101 :
                registration_options.image_registration_options.border_mode,
            registration_options.image_registration_options.border_value);

        if( !current_weights.empty() ) {

          frame_registration_->custom_remap(current_remap,
              current_weights, current_weights,
              cv::noArray(), cv::noArray(),
              registration_options.image_registration_options.interpolation,
              ECC_BORDER_CONSTANT);
        }
      }

      if ( !master_frame_generation_ ) {

        save_aligned_frame(current_frame, current_mask,
            output_aligned_frames_writer,
            input_sequence->current_pos() - 1);

        if ( canceled() ) {
          set_status_msg("canceled");
          break;
        }

        if ( !frame_registration_->current_ecc_image().empty() ) {

          save_ecc_frame(frame_registration_->current_ecc_image(),
              frame_registration_->current_ecc_mask(),
              output_ecc_writer,
              input_sequence->current_pos() - 1);
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

          save_postprocessed_frame(current_frame, current_mask,
              output_postprocessed_frames_writer,
              input_sequence->current_pos() - 1);

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

      save_accumulation_mask(current_frame, current_mask,
          output_accumulation_masks_writer,
          input_sequence->current_pos() - 1);

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }
    }

    if ( frame_accumulation_ ) {

      if ( true ) {
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

      on_accumulator_changed();

      if ( canceled() ) {
        set_status_msg("canceled");
        break;
      }


      if( !master_frame_generation_ && output_options_.save_incremental_frames ) {

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
          }
          else {
            save_incremental_frame(accumulated_frame, accumulated_mask,
                output_incremental_frame_writer,
                input_sequence->current_pos() - 1);
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

    on_status_changed();
  }

  return true;
}


bool c_image_stacking_pipeline::read_input_frame(const c_input_sequence::sptr & input_sequence,
    cv::Mat & output_image, cv::Mat & output_mask) const
{
  INSTRUMENT_REGION("");

  input_sequence->set_auto_debayer(DEBAYER_DISABLE);
  input_sequence->set_auto_apply_color_matrix(false);

  if ( !input_sequence->read(output_image, &output_mask) ) {
    CF_FATAL("input_sequence->read() fails\n");
    return false;
  }

//  CF_DEBUG("input_sequence->read(): %dx%d channels=%d pixel_depth=%d",
//      output_image.cols, output_image.rows,
//      output_image.channels(),
//      input_sequence->pixel_depth());

  if( !darkbayer_.empty() ) {

    if( darkbayer_.size() != output_image.size() || darkbayer_.channels() != output_image.channels() ) {
      CF_FATAL("darkbayer (%dx%d*%d) and input frame (%dx%d*%d) not match",
          darkbayer_.cols, darkbayer_.rows, darkbayer_.channels(),
          output_image.cols, output_image.rows, output_image.channels());
      return false;
    }

    if( output_image.depth() != CV_32F ) {
      output_image.convertTo(output_image, CV_32F);
    }

    cv::subtract(output_image, darkbayer_,
        output_image);
  }


  if ( !is_bayer_pattern(input_sequence->colorid()) ) {

    if( input_options_.drop_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
      CF_ERROR("CORRUPTED ASI FRAME DETECTED");
      output_image.release();
      return true; // return true with empty output image
    }

    if ( input_options_.filter_bad_pixels ) {
      remove_bad_pixels(output_image, input_options_, false);
    }

    output_image.convertTo(output_image, CV_32F,
        1. / ((1 << input_sequence->bpp())));

  }
  else {

    DEBAYER_ALGORITHM algo =
        default_debayer_algorithm();

    switch (algo) {

      case DEBAYER_DISABLE:
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
        break;

      case DEBAYER_NN:
        case DEBAYER_VNG:
        case DEBAYER_EA:
        if( !debayer(output_image, output_image, input_sequence->colorid(), algo) ) {
          CF_ERROR("debayer() fails");
          return false;
        }
        if( input_options_.drop_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if ( input_options_.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options_, true);
        }
        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));
        break;

      case DEBAYER_NN2:
        case DEBAYER_NNR:
        if( !extract_bayer_planes(output_image, output_image, input_sequence->colorid()) ) {
          CF_ERROR("extract_bayer_planes() fails");
          return false;
        }
        if( input_options_.drop_bad_asi_frames && is_corrupted_asi_frame(output_image) ) {
          CF_ERROR("CORRUPTED ASI FRAME DETECTED");
          output_image.release();
          return true; // return true with empty output image
        }
        if( input_options_.filter_bad_pixels ) {
          remove_bad_pixels(output_image, input_options_, true);
        }

        output_image.convertTo(output_image, CV_32F,
            1. / ((1 << input_sequence->bpp())));

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
    const c_input_options & input_optons,
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

  medianImage.copyTo(image, variationImage > input_optons.hot_pixels_variation_threshold * meanVariationImage);
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
      opts.m_.dscale() >= 0 && opts.m_.k() >= 0;
}

void c_image_stacking_pipeline::compute_weights(const cv::Mat & src, const cv::Mat & srcmask, cv::Mat & dst) const
{
  INSTRUMENT_REGION("");

  const c_frame_accumulation_options & acc_options =
      accumulation_options();

  acc_options.m_.create_map(src, dst);
  if ( !srcmask.empty() ) {
    dst.setTo(0, ~srcmask);
  }

  CF_DEBUG("dst: %dx%d depth=%d channels=%d", dst.cols, dst.rows, dst.depth(), dst.channels());
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


bool c_image_stacking_pipeline::compute_accumulated_image(cv::OutputArray dst, cv::OutputArray dstmask) const
{
  lock_guard lock(accumulator_lock_);

  if ( frame_accumulation_ ) {

    if ( frame_accumulation_->accumulated_frames() < 1 ) {
      return false;
    }

    if ( frame_accumulation_ ) {

      if( !frame_accumulation_->compute(dst, dstmask) ) {
        CF_ERROR("frame_accumulation_->compute() fails");
        return false;
      }

      return true;
    }
  }

  return false;
}

bool c_image_stacking_pipeline::get_selected_master_frame(cv::OutputArray dst, cv::OutputArray dstmask) const
{
  lock_guard lock(accumulator_lock_);

  if( dst.needed() ) {
    selected_master_frame_.copyTo(dst);
  }

  if( dstmask.needed() ) {
    selected_master_frame_mask_.copyTo(dstmask);
  }

  return true;
}


bool c_image_stacking_pipeline::upscale_required(frame_upscale_stage current_stage) const
{
  if ( master_frame_generation_  ) {
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

  return save_image(image_to_write, output_file_name);
}


void c_image_stacking_pipeline::save_preprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_preprocessed_frames ) {
    return;
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
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_preprocessed_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);
}

void c_image_stacking_pipeline::save_aligned_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer,
    int seqindex ) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_aligned_frames ) {
    return;
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
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_aligned_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}

void c_image_stacking_pipeline::save_ecc_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_ecc_frames ) {
    return;
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
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_ecc_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}


void c_image_stacking_pipeline::save_postprocessed_frame(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_processed_aligned_frames ) {
    return;
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
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        current_frame.channels() > 1,
        output_options_.save_processed_aligned_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(current_frame, current_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);
}

void c_image_stacking_pipeline::save_incremental_frame(const cv::Mat & accumulated_frame, const cv::Mat & accumulated_mask,
    c_video_writer & output_writer,
    int seqindex) const
{
//  const c_image_stacking_output_options & output_options =
//      output_options();

  if ( !output_options_.save_incremental_frames ) {
    return;
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
      return;
    }



    output_writer.open(pathfilename,
        accumulated_frame.size(),
        accumulated_frame.channels() > 1,
        output_options_.save_incremental_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  output_writer.write(accumulated_frame, accumulated_mask,
      output_options_.write_image_mask_as_alpha_channel,
      seqindex);

}

void c_image_stacking_pipeline::save_accumulation_mask(const cv::Mat & current_frame, const cv::Mat & current_mask,
    c_video_writer & output_writer,
    int seqindex) const
{
  if ( !output_options_.save_accumulation_masks ) {
    return;
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
      return;
    }

    output_writer.open(pathfilename,
        current_frame.size(),
        false,
        output_options_.save_accumulation_masks_frame_mapping);

    if ( !output_writer.is_open() ) {

      CF_ERROR("Can not open output writer '%s'",
          pathfilename.c_str());

      return;
    }
  }

  if ( !current_mask.empty() ) {
    output_writer.write(current_mask, cv::noArray(), false, seqindex);
  }
  else {
    output_writer.write(cv::Mat1b(current_frame.size(), 255), cv::noArray(), false, seqindex);
  }

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
      selected_master_frame_index = master_frame_index_;
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

      input_sequence->set_auto_debayer(DEBAYER_DISABLE);
      input_sequence->set_auto_apply_color_matrix(false);

      backup_current_pos = input_sequence->current_pos();
      input_sequence->seek(start_pos);

      cv::Mat image, mask, dogs;
      int current_index, best_index = 0;
      double current_metric, best_metric = 0;

      total_frames_ = max_frames_to_scan;
      processed_frames_ = 0;
      accumulated_frames_ = 0;
      on_status_changed();

      for( current_index = 0; processed_frames_ < total_frames_;
          processed_frames_ = ++current_index, on_status_changed() ) {

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

            on_selected_master_frame_changed();
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

    SERIALIZE_OPTION(section, save, input_options_, darkbayer_filename);
    SERIALIZE_OPTION(section, save, input_options_, missing_pixel_mask_filename);
    SERIALIZE_OPTION(section, save, input_options_, missing_pixels_marked_black);
    SERIALIZE_OPTION(section, save, input_options_, inpaint_missing_pixels);
    SERIALIZE_OPTION(section, save, input_options_, filter_bad_pixels);
    SERIALIZE_OPTION(section, save, input_options_, drop_bad_asi_frames);
    SERIALIZE_OPTION(section, save, input_options_, hot_pixels_variation_threshold);
    SERIALIZE_OPTION(section, save, input_options_, enable_color_maxtrix);
    SERIALIZE_OPTION(section, save, input_options_, anscombe);
    SERIALIZE_OPTION(section, save, input_options_, start_frame_index);
    SERIALIZE_OPTION(section, save, input_options_, max_input_frames);
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

    SERIALIZE_OPTION(subsection, save, frame_registration_options_, accumulate_and_compensate_turbulent_flow);

    if( (subsection = get_group(section, save, "master_frame")) ) {
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, master_selection_method);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, apply_input_frame_processors);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, generate_master_frame);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, max_frames_to_generate_master_frame);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, eccflow_scale);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, master_sharpen_factor);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, accumulated_sharpen_factor);
      SERIALIZE_OPTION(subsection, save, frame_registration_options_.master_frame_options, save_master_frame);
    }

    if( (subsection = get_group(section, save, "image_registration")) ) {

      struct c_image_registration_options & image_registration_options =
          frame_registration_options_.image_registration_options;

      SERIALIZE_OPTION(subsection, save, image_registration_options, enable_frame_registration);
      SERIALIZE_OPTION(subsection, save, image_registration_options, motion_type);
      SERIALIZE_OPTION(subsection, save, image_registration_options, registration_channel);
      SERIALIZE_OPTION(subsection, save, image_registration_options, interpolation);
      SERIALIZE_OPTION(subsection, save, image_registration_options, border_mode);
      SERIALIZE_OPTION(subsection, save, image_registration_options, border_value);

      if( (subsubsection = get_group(subsection, save, "image_registration")) ) {

        struct c_feature_based_registration_options & feature_registration =
            frame_registration_options_.image_registration_options.feature_registration;

        SERIALIZE_OPTION(subsubsection, save, feature_registration, enabled);
        SERIALIZE_OPTION(subsubsection, save, feature_registration, scale);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_extractor"),
            save, feature_registration, sparse_feature_extractor);

        SERIALIZE_OPTION(get_group(subsubsection, save, "sparse_feature_matcher"),
            save, feature_registration, sparse_feature_matcher);
      }

      if( (subsubsection = get_group(subsection, save, "ecc")) ) {

        struct c_ecc_registration_options & ecc =
            frame_registration_options_.image_registration_options.ecc;

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
            frame_registration_options_.image_registration_options.eccflow;

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
            frame_registration_options_.image_registration_options.jovian_derotation;

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
        SERIALIZE_OPTION(subsubsection, save, jovian_derotation.ellipse, force_reference_ellipse);
      }
    }
  }

  // c_frame_accumulation_options accumulation_options_;
  if( (section = get_group(settings, save, "accumulation")) ) {

    SERIALIZE_OPTION(section, save, accumulation_options_, accumulation_method);

    if( (subsection = get_group(section, save, "c_lpg_sharpness_measure")) ) {

      c_lpg_sharpness_measure &m =
          accumulation_options_.m_;

      SERIALIZE_PROPERTY(subsection, save, m, k);
      SERIALIZE_PROPERTY(subsection, save, m, dscale);
      SERIALIZE_PROPERTY(subsection, save, m, uscale);
      SERIALIZE_PROPERTY(subsection, save, m, squared);
      SERIALIZE_PROPERTY(subsection, save, m, avgchannel);
    }

    if( (subsection = get_group(section, save, "c_laplacian_pyramid_focus_stacking")) ) {

      c_laplacian_pyramid_focus_stacking::options &fs =
          accumulation_options_.fs_;

      SERIALIZE_OPTION(subsection, save, fs, fusing_policy);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
      SERIALIZE_OPTION(subsection, save, fs, avgchannel);
      SERIALIZE_OPTION(subsection, save, fs, kradius);
      SERIALIZE_OPTION(subsection, save, fs, inpaint_mask_holes);
    }
  }

  // c_image_stacking_output_options output_options_;
  if( (section = get_group(settings, save, "output")) ) {

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

  // c_image_processing_options image_processing_options_;
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
