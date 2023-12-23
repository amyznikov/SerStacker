/*
 * c_epipolar_alignment_pipeline.cc
 *
 *  Created on: Dec 22, 2023
 *      Author: amyznikov
 */

#include "c_epipolar_alignment_pipeline.h"
#include <core/ssprintf.h>
#include <core/proc/morphology.h>
#include <core/proc/stereo/scale_sweep.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

namespace {

template<class T>
static double distance_between_points(const cv::Point_<T> & p1, const cv::Point_<T> & p2 )
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

template<class T>
static double distance_between_points(const cv::Vec<T, 2> & p1, const cv::Vec<T, 2> & p2 )
{
  return sqrt( (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
}

template<class T>
static inline bool IS_INSIDE_IMAGE(T x, T y, const cv::Size & image_size)
{
  return (x >= 0) && (y >= 0) && (x < image_size.width) && (y < image_size.height);
}

template<class T>
static inline bool IS_INSIDE_IMAGE(const cv::Point_<T> & point, const cv::Size & image_size)
{
  return IS_INSIDE_IMAGE(point.x, point.y, image_size);
}

template<class T>
static inline bool IS_INSIDE_IMAGE(const cv::Point_<T> & point, const cv::Mat & image)
{
  return IS_INSIDE_IMAGE(point.x, point.y, image.size());
}

}

c_epipolar_alignment_pipeline::c_epipolar_alignment_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}


bool c_epipolar_alignment_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section, subsection;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_options")) ) {
    SERIALIZE_OPTION(section, save, camera_options_, camera_intrinsics);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_pose")) ) {
    SERIALIZE_OPTION(section, save, camera_pose_options_, direction);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, robust_threshold);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsx);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsf);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {
    SERIALIZE_OPTION(section, save, feature2d_options_, detector);
    SERIALIZE_OPTION(section, save, feature2d_options_, descriptor);
    SERIALIZE_OPTION(section, save, feature2d_options_, matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {

    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_epipolar_stereo_frames);
    if( (subsection = SERIALIZE_GROUP(section, save, "epipolar_stereo_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, epipolar_stereo_output_options);
    }

    SERIALIZE_OPTION(section, save, output_options_, save_scale_compression_remaps);
    if( (subsection = SERIALIZE_GROUP(section, save, "scale_compression_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, scale_compression_output_options);
    }

  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl>& c_epipolar_alignment_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Camera parameters", "");
      PIPELINE_CTL_CAMERA_INTRINSICTS(ctrls, camera_options_.camera_intrinsics);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Feature2D options", "");
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Detector Options", "");
      PIPELINE_CTL_FEATURE2D_DETECTOR_OPTIONS(ctrls, feature2d_options_.detector);
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Descriptor Options", "");
      PIPELINE_CTL_FEATURE2D_DESCRIPTOR_OPTIONS(ctrls, feature2d_options_.descriptor);
      PIPELINE_CTL_END_GROUP(ctrls);
      PIPELINE_CTL_GROUP(ctrls, "Feature2D Matcher Options", "");
      PIPELINE_CTL_FEATURE2D_MATCHER_OPTIONS(ctrls, feature2d_options_.matcher);
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");
      PIPELINE_CTL(ctrls, camera_pose_options_.direction, "Motion direction", "Optimize assuming a priori known motion direction");
      PIPELINE_CTL(ctrls, camera_pose_options_.max_iterations, "max iterations", "Number of iterations for outliers removal");
      PIPELINE_CTL(ctrls, camera_pose_options_.robust_threshold, "robust threshold", "Parameter for robust function in pixels");
      PIPELINE_CTL(ctrls, camera_pose_options_.max_levmar_iterations, "max levmar iterations", "Number of levmar iterations");
      PIPELINE_CTL(ctrls, camera_pose_options_.epsx, "levmar epsx", "levmar epsx parameter");
      PIPELINE_CTL(ctrls, camera_pose_options_.epsf, "levmar epsf", "levmar epsf parameter");
    PIPELINE_CTL_END_GROUP(ctrls);


    ////////
    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");

      PIPELINE_CTL_GROUP(ctrls, "Save Epipolar Stereo Frames", "");
        PIPELINE_CTL(ctrls, output_options_.save_epipolar_stereo_frames, "save_epipolar_stereo_frames", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.epipolar_stereo_output_options,
            _this->output_options_.save_epipolar_stereo_frames);
      PIPELINE_CTL_END_GROUP(ctrls);


      PIPELINE_CTL_GROUP(ctrls, "Save scale compression remaps", "");
        PIPELINE_CTL(ctrls, output_options_.save_scale_compression_remaps, "save_scale_compression_remaps", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.scale_compression_output_options,
            _this->output_options_.save_scale_compression_remaps);
      PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_END_GROUP(ctrls);
    ////////
  }

  return ctrls;
}

bool c_epipolar_alignment_pipeline::copyParameters(const base::sptr & dst) const
{
  if ( !base::copyParameters(dst) ) {
    CF_ERROR("c_epipolar_alignment_pipeline::base::copyParameters() fails");
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
  p->feature2d_options_ = this->feature2d_options_;
  p->output_options_ = this->output_options_;

  return true;
}

c_sparse_feature_extractor_and_matcher::sptr c_epipolar_alignment_pipeline::create_keypoints_extractor() const
{
  return c_sparse_feature_extractor_and_matcher::create(feature2d_options_);
}


bool c_epipolar_alignment_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  const bool enable_morph_gradient = true;

  if( current_frame_.empty() || previous_frame_.empty() ) {
    return false;
  }

  const cv::Size size(std::max(current_frame_.cols, previous_frame_.cols),
      current_frame_.rows + previous_frame_.rows);

  const cv::Rect roi[2] = {
      cv::Rect(0, 0, current_frame_.cols, current_frame_.rows),
      cv::Rect(0, current_frame_.rows, previous_frame_.cols, previous_frame_.rows),
  };

  if( display_frame.needed() ) {

    display_frame.create(size,
        current_frame_.type());

    cv::Mat & display =
        display_frame.getMatRef();

    // current_frame_.copyTo(display(roi[0]));
    cv::warpPerspective(current_frame_, display(roi[0]),
        currentDerotationHomography_,
        current_frame_.size(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

    if ( !enable_morph_gradient ) {

      cv::warpPerspective(current_frame_, display(roi[0]),
          currentDerotationHomography_,
          current_frame_.size(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);

      previous_frame_.copyTo(display(roi[1]));
    }
    else {

      morphological_gradient(current_frame_, display(roi[0]));
      morphological_gradient(previous_frame_, display(roi[1]));

      cv::warpPerspective(display(roi[0]), display(roi[0]),
          currentDerotationHomography_,
          current_frame_.size(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }
  }

  if( display_mask.needed()  ) {

    display_mask.create(size, CV_8UC1);

    cv::Mat & display =
        display_mask.getMatRef();

    // current_frame_.copyTo(display(roi[0]));
    cv::warpPerspective(cv::Mat1b(current_frame_.size(), (uint8_t) 255),
        display(roi[0]),
        currentDerotationHomography_,
        current_frame_.size(),
        cv::INTER_LINEAR,
        cv::BORDER_REPLICATE);

    cv::compare(display(roi[0]), 254, display(roi[0]),
        cv::CMP_GE);

    display(roi[1]).setTo(255);
  }

  return true;
}

bool c_epipolar_alignment_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

  if ( !(keypoints_extractor_ = create_keypoints_extractor() ) ) {
    CF_ERROR("create_keypoints_extractor() fails");
    return false;
  }

  current_frame_.release();
  current_mask_.release();
  previous_frame_.release();
  previous_mask_.release();
  matched_current_positions_.clear();
  matched_previous_positions_.clear();

  return true;
}

void c_epipolar_alignment_pipeline::cleanup_pipeline()
{
  base::cleanup_pipeline();
}

bool c_epipolar_alignment_pipeline::run_pipeline()
{
  if( !input_sequence_ ) {
    CF_ERROR("No input_sequence provided, can not run");
    return false;
  }

  if ( !input_sequence_->open() ) {
    CF_ERROR("input_sequence_->open() fails");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if( is_live_sequence ) {
    total_frames_ = INT_MAX;
  }
  else {

    const int start_pos =
        std::max(input_options_.start_frame_index, 0);

    const int end_pos =
        input_options_.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                input_options_.start_frame_index + input_options_.max_input_frames);

    total_frames_ = end_pos - start_pos;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          total_frames_,
          start_pos,
          end_pos,
          input_sequence_->size(),
          input_options_.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  processed_frames_ = 0;
  accumulated_frames_ = 0;
  for( ; processed_frames_ < total_frames_;  ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !input_sequence_->read(current_frame_, &current_mask_) ) {
        CF_DEBUG("input_sequence_->read() fails");
        return false;
      }
    }

    if( canceled() ) {
      break;
    }

    if( input_options_.input_image_processor ) {

      lock_guard lock(mutex());

      if( !input_options_.input_image_processor->process(current_frame_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }

      if( canceled() ) {
        break;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( true ) {
      lock_guard lock(mutex());

      accumulated_frames_ = processed_frames_;

      std::swap(current_frame_, previous_frame_);
      std::swap(current_mask_, previous_mask_);
      std::swap(current_keypoints_, previous_keypoints_);
      std::swap(current_descriptors_, previous_descriptors_);
    }

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}


bool c_epipolar_alignment_pipeline::process_current_frame()
{
  if ( !extract_keypoints() ) {
    CF_ERROR("extract_keypoints() fails");
    return false;
  }

  if ( !match_keypoints() ) {
    CF_ERROR("match_keypoints() fails");
    return false;
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
    return false;
  }

  if ( !save_epipolar_stereo_frames() ) {
    CF_ERROR("save_epipolar_stereo_frames() fails");
    return false;
  }

  if ( !save_scale_compression_remaps() ) {
    CF_ERROR("save_scale_compression_remaps() fails");
    return false;
  }


  return true;
}


bool c_epipolar_alignment_pipeline::extract_keypoints()
{
  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  if( feature2d_options_.matcher.type == FEATURE2D_MATCHER_OptFlowPyrLK ) {
    keypoints_extractor_->detect(current_frame_, current_keypoints_, current_mask_);
  }
  else {
    keypoints_extractor_->detectAndCompute(current_frame_, current_mask_,
        current_keypoints_, current_descriptors_);
  }


//  CF_DEBUG("current_keypoints_: %zu current_descriptors_: %dx%d depth=%d",
//      current_keypoints_.size(),
//      current_descriptors_.rows, current_descriptors_.cols,
//      current_descriptors_.depth());

  return true;
}

bool c_epipolar_alignment_pipeline::match_keypoints()
{
  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  matched_current_positions_.clear();
  matched_previous_positions_.clear();

  if( !keypoints_extractor_->matcher() ) {

    if( !current_keypoints_.empty() && !previous_frame_.empty() ) {

      const bool fOK =
          match_optflowpyrlk(previous_frame_, current_frame_,
              previous_keypoints_,
              feature2d_options_.matcher.optflowpyrlk,
              matched_previous_positions_,
              matched_current_positions_);

      if( !fOK ) {
        CF_ERROR("match_optflowpyrlk() fails");
        return false;
      }
    }
  }
  else if( !current_keypoints_.empty() && !previous_keypoints_.empty() ) {

    std::vector<cv::Point2f> cps, pps;

    const size_t num_matches =
        ::match_keypoints(keypoints_extractor_->matcher(),
            current_keypoints_,
            current_descriptors_,
            previous_keypoints_,
            previous_descriptors_,
            nullptr,
            &cps,
            &pps);

    for( int i = 0; i < num_matches; ++i ) {

      const cv::Point2f &cp = cps[i];
      const cv::Point2f &pp = pps[i];
//      const float d2 = (cp.x - pp.x) * (cp.x - pp.x) + (cp.y - pp.y) * (cp.y - pp.y);
//      if( d2 > 0 ) {
        matched_current_positions_.emplace_back(cp);
        matched_previous_positions_.emplace_back(pp);
//      }
    }
  }

  CF_DEBUG("%zu / %zu matches", matched_current_positions_.size(), current_keypoints_.size());

  return true;
}

bool c_epipolar_alignment_pipeline::estmate_camera_pose()
{
  if( matched_current_positions_.size() < 6 || matched_previous_positions_.size() < 6 ) {
    return true; // ignore
  }

  lock_guard lock(mutex());

  INSTRUMENT_REGION("");

  current_inliers_.release();

  current_euler_anges_ = cv::Vec3d(0, 0, 0);
  current_translation_vector_ = cv::Vec3d(0, 0, 1);

  bool fOk =
      lm_camera_pose_and_derotation_homography(
          camera_options_.camera_intrinsics.camera_matrix,
          matched_current_positions_,
          matched_previous_positions_,
          current_euler_anges_,
          current_translation_vector_,
          &currentRotationMatrix_,
          &currentEssentialMatrix_,
          &currentFundamentalMatrix_,
          &currentDerotationHomography_,
          current_inliers_,
          &camera_pose_options_);

  if ( !fOk ) {
    CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
  }

//  CF_DEBUG("current_inliers_.size=%dx%d nz=%d",
//      current_inliers_.rows, current_inliers_.cols,
//      cv::countNonZero(current_inliers_));

  compute_epipoles(currentFundamentalMatrix_,
      currentEpipoles_);

  currentEpipole_ =
      0.5 * (currentEpipoles_[0] + currentEpipoles_[1]);

  //  CF_DEBUG("A: (%g %g %g)", current_euler_anges_(0) * 180 / CV_PI, current_euler_anges_(1) * 180 / CV_PI, current_euler_anges_(2) * 180 / CV_PI);
  //  CF_DEBUG("T: (%g %g %g)", current_translation_vector_(0), current_translation_vector_(1), current_translation_vector_(2));
  //  CF_DEBUG("E: (%g %g) EE: {%+g %+g} {%+g %+g}", currentEpipole_.x, currentEpipole_.y,
  //      currentEpipoles_[0].x, currentEpipoles_[0].y, currentEpipoles_[1].x, currentEpipoles_[1].y);

  if ( distance_between_points(currentEpipoles_[0], currentEpipoles_[1]) > 1 ) {
    CF_WARNING("\nWARNING!\n"
        "Something looks POOR: Computed epipoles differ after derotation:\n"
        "E0 = {%g %g}\n"
        "E1 = {%g %g}\n",
        currentEpipoles_[0].x, currentEpipoles_[0].y,
        currentEpipoles_[1].x, currentEpipoles_[1].y);
  }

  return true;
}

bool c_epipolar_alignment_pipeline::save_epipolar_stereo_frames()
{
  if( !output_options_.save_epipolar_stereo_frames ) {
    return true;
  }

  cv::Mat image, mask;

  if( get_display_image(image, mask) ) {

    bool fOk =
        add_output_writer(epipolar_stereo_writer_,
            output_options_.epipolar_stereo_output_options,
            "epipolar",
            ".avi");

    if( !fOk ) {
      CF_ERROR("add_output_writer('%s') fails",
          epipolar_stereo_writer_.filename().c_str());
      return false;
    }

    if( !epipolar_stereo_writer_.write(image) ) {
      CF_ERROR("epipolar_stereo_writer_.write('%s') fails.",
          epipolar_stereo_writer_.filename().c_str());
      return false;
    }
  }

  return true;
}

bool c_epipolar_alignment_pipeline::save_scale_compression_remaps()
{
  if( !output_options_.save_scale_compression_remaps ) {
    return true;
  }

  if( current_frame_.empty() || previous_frame_.empty() ) {
    return true;
  }

  const int min_iteration = 0;
  const int max_iteration = 150;
  const bool enable_morph_gradient = true;

  const cv::Size image_size =
      current_frame_.size();

  cv::Mat current_frame, current_mask, previous_frame, previous_mask;
  cv::Mat scale_compressed_frame, scale_compressed_mask;
  cv::Mat2f scale_compression_map;

  cv::Mat display;

  c_output_frame_writer writer;

  const cv::Matx33d H =
      currentDerotationHomography_.inv();

  const cv::Size display_size =
      cv::Size(std::max(previous_frame_.cols, current_frame_.cols),
          previous_frame_.rows + 2 * current_frame_.rows);

  const cv::Rect roi[3] = {
      cv::Rect (0, 0, previous_frame_.cols, previous_frame_.rows),
      cv::Rect (0, previous_frame_.rows, current_frame_.cols, current_frame_.rows),
      cv::Rect (0, previous_frame_.rows + current_frame_.rows, current_frame_.cols, current_frame_.rows),
  };

//  cv::warpPerspective(current_frame_,
//      current_frame,
//      currentDerotationHomography_,
//      current_frame_.size(),
//      cv::INTER_LINEAR,
//      cv::BORDER_REPLICATE);

  if ( !enable_morph_gradient ) {
    current_frame =  current_frame_;
    previous_frame =  previous_frame_;
  }
  else {
    morphological_gradient(current_frame_, current_frame);
    morphological_gradient(previous_frame_, previous_frame);
  }

  current_mask =
      current_mask_.empty() ? cv::Mat1b(current_frame_.size(), 255) :
          current_mask_;


//  cv::warpPerspective(current_mask_.empty() ? cv::Mat1b(current_frame_.size(), 255) : current_mask_,
//      current_mask,
//      currentDerotationHomography_,
//      current_frame_.size(),
//      cv::INTER_LINEAR,
//      cv::BORDER_REPLICATE);

  for ( int i = min_iteration; i < max_iteration; ++i ) {

    if ( canceled() ) {
      return false;
    }


    create_scale_compression_remap(i, image_size, currentEpipole_, H, scale_compression_map);

    cv::remap(current_frame, scale_compressed_frame,
        scale_compression_map, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

    cv::remap(current_mask, scale_compressed_mask,
        scale_compression_map, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

    cv::compare(scale_compressed_mask, 254, scale_compressed_mask,
        cv::CMP_GE);

    if( !writer.is_open() ) {

      const std::string filename =
          ssprintf("%s/scale_remaps/F%05d.avi", output_path_.c_str(),
              input_sequence_->current_pos() - 1);

      if( !writer.open(filename, output_options_.scale_compression_output_options.ffmpeg_opts) ) {
        CF_ERROR("writer.open('%s') fails", filename.c_str());
        return false;
      }
    }

    if( display.empty() ) {
      display.create(display_size, current_frame.type());
      previous_frame.copyTo(display(roi[0]));
    }

    scale_compressed_frame.copyTo(display(roi[2]));
    cv::absdiff(scale_compressed_frame, previous_frame, display(roi[1]));
    display(roi[1]).setTo(0, ~scale_compressed_mask);

    if( !writer.write(display) ) {
      CF_ERROR("writer.write('%s') fails", writer.filename().c_str());
      return false;
    }
  }

  return true;
}

