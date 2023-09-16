/*
 * c_virtual_stereo_pipeline.cc
 *
 *  Created on: Mar 9, 2023
 *      Author: amyznikov
 */

#include "c_virtual_stereo_pipeline.h"
#include <core/feature2d/feature2d_settings.h>
#include <core/proc/polar_trasform.h>
#include <core/proc/image_registration/ecc2.h>
#include <core/proc/colormap.h>
#include <core/readdir.h>
#include <chrono>
#include <thread>

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

c_virtual_stereo_pipeline::c_virtual_stereo_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

c_virtual_stereo_input_options & c_virtual_stereo_pipeline::input_options()
{
  return input_options_;
}

const c_virtual_stereo_input_options & c_virtual_stereo_pipeline::input_options() const
{
  return input_options_;
}

c_virtual_stereo_camera_options & c_virtual_stereo_pipeline::camera_options()
{
  return camera_options_;
}

const c_virtual_stereo_camera_options & c_virtual_stereo_pipeline::camera_options() const
{
  return camera_options_;
}

c_virtual_stereo_image_processing_options & c_virtual_stereo_pipeline::image_processing_options()
{
  return image_processing_options_;
}

const c_virtual_stereo_image_processing_options & c_virtual_stereo_pipeline::image_processing_options() const
{
  return image_processing_options_;
}

c_virtual_stereo_feature2d_options & c_virtual_stereo_pipeline::feature2d_options()
{
  return feature2d_options_;
}

const c_virtual_stereo_feature2d_options & c_virtual_stereo_pipeline::feature2d_options() const
{
  return feature2d_options_;
}

c_lm_camera_pose_options & c_virtual_stereo_pipeline::camera_pose_options()
{
  return camera_pose_options_;
}

const c_lm_camera_pose_options & c_virtual_stereo_pipeline::camera_pose_options() const
{
  return camera_pose_options_;
}

c_virtual_stereo_output_options & c_virtual_stereo_pipeline::output_options()
{
  return output_options_;
}

const c_virtual_stereo_output_options & c_virtual_stereo_pipeline::output_options() const
{
  return output_options_;
}

bool c_virtual_stereo_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, input_options_);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_options")) ) {
    SERIALIZE_OPTION(section, save, camera_options_, camera_intrinsics);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "feature2d")) ) {
    SERIALIZE_OPTION(section, save, feature2d_options_, detector);
    SERIALIZE_OPTION(section, save, feature2d_options_, descriptor);
    SERIALIZE_OPTION(section, save, feature2d_options_, matcher);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_pose")) ) {
    SERIALIZE_OPTION(section, save, camera_pose_options_, direction);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, robust_threshold);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsx);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsf);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "stereo_matcher")) ) {
    SERIALIZE_OPTION(section, save, stereo_matcher_options_, enable_stereo_matcher);
    stereo_matcher_.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "epipolar_matcher")) ) {
    epipolar_matcher_.serialize(section, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, input_processor);
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, feature2d_preprocessor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_polar_frames);
    SERIALIZE_OPTION(section, save, output_options_, polar_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_disparity_frames);
    SERIALIZE_OPTION(section, save, output_options_, disparity_frames_filename);
    SERIALIZE_OPTION(section, save, output_options_, save_homography_video);
    SERIALIZE_OPTION(section, save, output_options_, homography_video_filename);
//    SERIALIZE_OPTION(section, save, output_options_, save_median_hat_video);
//    SERIALIZE_OPTION(section, save, output_options_, median_hat_video_filename);
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_virtual_stereo_pipeline::get_controls()
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
    PIPELINE_CTL_GROUP(ctrls, "Stereo Matcher Options", "");
      PIPELINE_CTL_STEREO_MATCHER_OPTIONS(ctrls, stereo_matcher_);
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Epipolar Matcher", "");
      PIPELINE_CTL(ctrls, epipolar_matcher_.options().enabled, "enable epipolar matcher", "");
      PIPELINE_CTL_GROUP(ctrls, "Epipolar Matcher Options", "");
        PIPELINE_CTL(ctrls, epipolar_matcher_.options().max_disparity, "max_disparity", "");
        PIPELINE_CTL(ctrls, epipolar_matcher_.options().diff_threshold, "diff_threshold", "");
        PIPELINE_CTL(ctrls, epipolar_matcher_.options().avg_scale, "avg_scale", "");
        PIPELINE_CTL(ctrls, epipolar_matcher_.options().enable_debug, "enable_debug", "");
      PIPELINE_CTL_END_GROUP(ctrls);
    PIPELINE_CTL_END_GROUP(ctrls);
    ////////

    PIPELINE_CTL_GROUP(ctrls, "Image processing", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.input_processor, "Input image preprocessor", "");
      PIPELINE_CTL_PROCESSOR_SELECTION(ctrls, image_processing_options_.feature2d_preprocessor, "Feature2D image preprocessor", "");
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL(ctrls, output_options_.default_display_type, "display_type", "");
      PIPELINE_CTL(ctrls, output_options_.output_directory, "output_directory", "");
      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTL(ctrls, output_options_.progress_video_filename, "progress_video_filename", "");
      PIPELINE_CTL(ctrls, output_options_.save_polar_frames, "save_polar_frames", "");
      PIPELINE_CTL(ctrls, output_options_.polar_frames_filename, "polar_frames_filename", "");
      PIPELINE_CTL(ctrls, output_options_.save_disparity_frames, "save_disparity_frames", "");
      PIPELINE_CTL(ctrls, output_options_.disparity_frames_filename, "disparity_frames_filename", "");
      PIPELINE_CTL(ctrls, output_options_.save_homography_video, "save_homography_video", "");
      PIPELINE_CTL(ctrls, output_options_.homography_video_filename, "homography_video_filename", "");
//      PIPELINE_CTL(ctrls, output_options_.save_median_hat_video, "save_median_hat_video", "");
//      PIPELINE_CTL(ctrls, output_options_.median_hat_video_filename, "median_hat_video_filename", "");

    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
  }

  return ctrls;
}

bool c_virtual_stereo_pipeline::copyParameters(const base::sptr & dst) const
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
  p->camera_options_ = this->camera_options_;
  p->image_processing_options_ = this->image_processing_options_;
  p->feature2d_options_ = this->feature2d_options_;
  p->camera_pose_options_  = this->camera_pose_options_;
  p->stereo_matcher_options_ = this->stereo_matcher_options_;
  p->output_options_ = this->output_options_;

  return true;
}


bool c_virtual_stereo_pipeline::initialize_pipeline()
{
  CF_DEBUG("Enter");

  lock_guard lock(mutex());


  if ( !base::initialize_pipeline() ) {
    CF_ERROR("Base::initialize_pipeline() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  output_path_ =
      create_output_path(output_options().output_directory);


  /////////////////////////////////////////////////////////////////////////////

  if( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }


  /////////////////////////////////////////////////////////////////////////////

  if ( !(keypoints_extractor_ = create_keypoints_extractor() ) ) {
    CF_ERROR("create_keypoints_extractor() fails");
    return false;
  }

  if( !(keypoints_matcher_ = create_sparse_feature_matcher(feature2d_options_.matcher)) ) {
    CF_ERROR("create_sparse_feature_matcher() fails");
    return false;
  }

  /////////////////////////////////////////////////////////////////////////////

  current_disparity_.release();

  current_image_.release();
  previous_image_.release();

  current_mask_.release();
  previous_mask_.release();

  current_keypoints_.clear();
  previous_keypoints_.clear();

  current_descriptors_.release();
  previous_descriptors_.release();

  /////////////////////////////////////////////////////////////////////////////

  matched_current_positions_.clear();
  matched_previous_positions_.clear();

  /////////////////////////////////////////////////////////////////////////////

//  current_block_array_->release();
//  previous_block_array_->release();
//  current_median_hat_->release();
//  previous_median_hat_->release();
//  current_median_hat_mask_->release();
//  previous_median_hat_mask_->release();

  /////////////////////////////////////////////////////////////////////////////

  CF_DEBUG("Leave");

  return true;
}

void c_virtual_stereo_pipeline::cleanup_pipeline()
{
  lock_guard lock(mutex());

  close_input_sequence();

  if ( progress_video_writer_.is_open() ) {
    progress_video_writer_.close();
    CF_DEBUG("SAVED '%s'", progress_video_writer_.filename().c_str());
  }

  if ( polar_frames_writer_.is_open() ) {
    polar_frames_writer_.close();
    CF_DEBUG("SAVED '%s'", polar_frames_writer_.filename().c_str());
  }

  if ( disparity_frames_writer_.is_open() ) {
    disparity_frames_writer_.close();
    CF_DEBUG("SAVED '%s'", disparity_frames_writer_.filename().c_str());
  }

  if ( homography_video_writer_.is_open() ) {
    homography_video_writer_.close();
    CF_DEBUG("SAVED '%s'", homography_video_writer_.filename().c_str());
  }

//  if ( median_hat_video_writer_.is_open() ) {
//    median_hat_video_writer_.close();
//    CF_DEBUG("SAVED '%s'", median_hat_video_writer_.filename().c_str());
//  }

}

bool c_virtual_stereo_pipeline::open_input_sequence()
{
  if ( !input_sequence_ ) {
    CF_ERROR("ERROR: input_sequence_ not set");
    return false;
  }

  if (  !input_sequence_->open() ) {
    CF_ERROR("ERROR: input_sequence->open() fails");
    return false;
  }

  return true;
}

void c_virtual_stereo_pipeline::close_input_sequence()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }
}


bool c_virtual_stereo_pipeline::seek_input_sequence(int pos)
{
  if ( !input_sequence_->seek(pos) ) {
    CF_ERROR("ERROR: input_sequence->seek(start_pos=%d) fails", pos);
    return false;
  }
  return true;
}

bool c_virtual_stereo_pipeline::read_input_frame(cv::Mat & output_image, cv::Mat & output_mask)
{
  // lock_guard lock(mutex());

  if( !base::read_input_frame(input_sequence_, input_options_, output_image, output_mask, false, false) ) {
    CF_DEBUG("base::read_input_frame() fails");
    return false;
  }

  if ( output_image.depth() == CV_32F || output_image.depth() == CV_64F ) {
    output_image.convertTo(output_image, CV_8U, 255);
  }
  else if ( output_image.depth() == CV_16U ) {
    output_image.convertTo(output_image, CV_8U, 255./65535.);
  }

  return true;
}

c_sparse_feature_extractor::ptr c_virtual_stereo_pipeline::create_keypoints_extractor() const
{
  return create_sparse_feature_extractor(feature2d_options_.detector, feature2d_options_.descriptor);
}

bool c_virtual_stereo_pipeline::run_pipeline()
{
  CF_DEBUG("Starting '%s: %s' ...",
      csequence_name(), cname());


  ///////////////////////////////////

  if ( !open_input_sequence() ) {
    CF_ERROR("open_input_sequence() fails");
    return false;
  }

  const bool is_live_sequence =
      input_sequence_->is_live();

  if ( is_live_sequence ) {

    total_frames_ = INT_MAX;
    processed_frames_ = 0;
    accumulated_frames_ = 0;

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
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1",
          total_frames_);
      return false;
    }

    if( !seek_input_sequence(start_pos) ) {
      CF_ERROR("ERROR: seek_input_source(start_pos=%d) fails", start_pos);
      return false;
    }
  }


  set_status_msg("RUNNING ...");

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if ( true ) {

      lock_guard lock(mutex());

      if( !read_input_frame(current_image_, current_mask_) ) {
        CF_DEBUG("read_input_frame() fails");
        break;
      }

      if( image_processing_options_.input_processor ) {
        if( !image_processing_options_.input_processor->process(current_image_, current_mask_) ) {
          CF_ERROR("ERROR: input_processor->process() fails");
          return false;
        }
      }

      if( canceled() ) {
        break;
      }
    }

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( !write_progress_video() ) {
      CF_DEBUG("write_progress_video() fails");
      return false;
    }

    if ( !write_homography_video() ) {
      CF_DEBUG("write_homography_video() fails");
      return false;
    }

    // give chance to GUI thread to call get_display_image()
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    if ( true ) {
      lock_guard lock(mutex());

      accumulated_frames_ = processed_frames_;

      std::swap(current_image_, previous_image_);
      std::swap(current_mask_, previous_mask_);
      std::swap(current_keypoints_, previous_keypoints_);
      std::swap(current_descriptors_, previous_descriptors_);
//      std::swap(current_block_array_, previous_block_array_);
//      std::swap(current_median_hat_, previous_median_hat_);
//      std::swap(current_median_hat_mask_, previous_median_hat_mask_);

    }
  }


  return true;
}

bool c_virtual_stereo_pipeline::process_current_frame()
{
  INSTRUMENT_REGION("");

  if ( !extract_keypoints() ) {
    CF_ERROR("extract_keypoints() fails");
    return false;
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
  }

  if ( !run_polar_stereo() ) {
    CF_ERROR("run_stereo_matcher() fails");
    return false;
  }

  if ( !run_epipolar_stereo() ) {
    CF_ERROR("run_epipolar_matcher() fails");
    return false;
  }

  return true;
}

bool c_virtual_stereo_pipeline::extract_keypoints()
{
  lock_guard lock(mutex());

  bool fOK =
      keypoints_extractor_->detectAndCompute(current_image_, current_mask_,
          current_keypoints_, current_descriptors_);

  if ( !fOK ) {
    CF_ERROR("keypoints_extractor_->detectAndCompute() fails");
    return false;
  }

  if( !previous_keypoints_.empty() ) {

    //current_matches_.clear();
    matched_current_positions_.clear();
    matched_previous_positions_.clear();

    std::vector<cv::Point2f> cps, pps;

    const size_t num_matches =
        match_keypoints(keypoints_matcher_,
            current_keypoints_,
            current_descriptors_,
            previous_keypoints_,
            previous_descriptors_,
            nullptr,
            &cps,
            &pps);

    // CF_DEBUG("num_matches=%zu cps=%zu",
    //    num_matches, cps.size());

    if( !num_matches ) {
      CF_ERROR("match_keypoints() fails");
    }
    else {

      for( int i = 0; i < num_matches; ++i ) {

        const cv::Point2f &cp = cps[i];
        const cv::Point2f &pp = pps[i];
        const float d2 = (cp.x - pp.x) * (cp.x - pp.x) + (cp.y - pp.y) * (cp.y - pp.y);
        if( d2 > 0 ) {
          matched_current_positions_.emplace_back(cp);
          matched_previous_positions_.emplace_back(pp);
        }
      }
    }
  }

  return true;
}

bool c_virtual_stereo_pipeline::estmate_camera_pose()
{
  if( matched_current_positions_.size() < 6 || matched_previous_positions_.size() < 6 ) {
    return true; // ignore
  }

  lock_guard lock(mutex());

  currentInliers_.release();

  currentEulerAnges_ = cv::Vec3d(0, 0, 0);
  currentTranslationVector_ = cv::Vec3d(0, 0, 1);

  bool fOk =
      lm_camera_pose_and_derotation_homography(
          camera_options_.camera_intrinsics.camera_matrix,
          matched_current_positions_,
          matched_previous_positions_,
          currentEulerAnges_,
          currentTranslationVector_,
          &currentRotationMatrix_,
          &currentEssentialMatrix_,
          &currentFundamentalMatrix_,
          &currentDerotationHomography_,
          currentInliers_,
          &camera_pose_options_);

  if ( !fOk ) {
    CF_ERROR("estimate_camera_pose_and_derotation_homography() fails");
  }

//  CF_DEBUG("currentInliers_.size=%dx%d nz=%d",
//      currentInliers_.rows, currentInliers_.cols,
//      cv::countNonZero(currentInliers_));

  compute_epipoles(currentFundamentalMatrix_,
      currentEpipoles_);

  currentEpipole_ =
      0.5 * (currentEpipoles_[0] + currentEpipoles_[1]);

  //  CF_DEBUG("A: (%g %g %g)", currentEulerAnges_(0) * 180 / CV_PI, currentEulerAnges_(1) * 180 / CV_PI, currentEulerAnges_(2) * 180 / CV_PI);
  //  CF_DEBUG("T: (%g %g %g)", currentTranslationVector_(0), currentTranslationVector_(1), currentTranslationVector_(2));
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

bool c_virtual_stereo_pipeline::run_polar_stereo()
{
  if( !output_options_.save_polar_frames && !stereo_matcher_.enabled() ) {
    return true; // ignore
  }

  cv::Mat frames[2];
  cv::Mat masks[2];
  cv::Mat2f inverse_remap;

  if( !create_stereo_frames(frames, masks, &inverse_remap) ) {
    return true; // ignore
  }

  if( output_options_.save_polar_frames ) {

    cv::Mat display;

    const cv::Size dst_size(frames[0].cols + frames[1].cols,
        std::max(frames[0].rows, frames[1].rows));

    const cv::Rect dst_roi[2] = {
        cv::Rect(0, 0, frames[0].cols, frames[0].rows),
        cv::Rect(frames[0].cols, 0, frames[1].cols, frames[1].rows),
    };

    display.create(dst_size, frames[0].type());
    frames[0].copyTo(display(dst_roi[0]));
    frames[1].copyTo(display(dst_roi[1]));

    if( !polar_frames_writer_.is_open() ) {

      const std::string output_video_filename =
          generate_output_filename(output_options_.polar_frames_filename,
              "polar",
              ".png");

      bool fOK =
          polar_frames_writer_.open(output_video_filename,
              display.size(),
              display.channels() > 1,
              false);

      if( !fOK ) {
        CF_ERROR("polar_frames_writer_.open('%s') fails",
            output_video_filename.c_str());
        return false;
      }

      CF_DEBUG("Created '%s' display.size()=%dx%d",
          output_video_filename.c_str(),
          display.cols,
          display.rows);
    }

    if( !polar_frames_writer_.write(display, cv::noArray(), false, 0) ) {
      CF_ERROR("polar_frames_writer_.write() fails: %s",
          progress_video_writer_.filename().c_str());
      return false;
    }
  }

  if( stereo_matcher_.enabled() ) {

    if( !stereo_matcher_.compute(frames[0], frames[1], current_disparity_) ) {
      CF_ERROR("stereo_matcher_.compute() fails");
      return false;
    }

    cv::remap(current_disparity_, current_disparity_,
        inverse_remap, cv::noArray(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar::all(-1));

    if( output_options_.save_disparity_frames && !current_disparity_.empty() ) {

      if( !disparity_frames_writer_.is_open() ) {

        const std::string output_video_filename =
            generate_output_filename(output_options_.disparity_frames_filename,
                "disparity",
                ".tiff");

        bool fOK =
            disparity_frames_writer_.open(output_video_filename,
                current_disparity_.size(),
                current_disparity_.channels() > 1,
                false);

        if( !fOK ) {
          CF_ERROR("disparity_frames_writer_.open('%s') fails",
              output_video_filename.c_str());
          return false;
        }

        CF_DEBUG("Created '%s' display.size()=%dx%d",
            output_video_filename.c_str(),
            current_disparity_.cols,
            current_disparity_.rows);
      }

      if( !disparity_frames_writer_.write(current_disparity_, cv::noArray(), false, 0) ) {
        CF_ERROR("disparity_frames_writer_.write() fails: %s",
            disparity_frames_writer_.filename().c_str());
        return false;
      }
    }
  }

  return true;
}

bool c_virtual_stereo_pipeline::run_epipolar_stereo()
{
  if ( !epipolar_matcher_.enabled() ||  current_image_.empty() || previous_image_.empty() ) {
    return true; // ignore
  }

  if( !epipolar_matcher_.options().enable_debug ) {
    epipolar_matcher_.set_debug_path("");
  }
  else {
    epipolar_matcher_.set_debug_path(ssprintf("%s/epipolar_debug/frame%05d", output_path_.c_str(),
        input_sequence_->current_pos() - 1));
  }

  bool fOK =
      epipolar_matcher_.match(current_image_, current_mask_,
          previous_image_, previous_mask_,
          currentDerotationHomography_,
          currentEpipole_);

  if( !fOK ) {
    CF_ERROR("epipolar_matcher_.match() fails");
    return false;
  }

  return true;
}


bool c_virtual_stereo_pipeline::create_stereo_frames(cv::Mat frames[2], cv::Mat masks[2], cv::Mat2f * inverse_remap)
{
  if( current_image_.empty() || previous_image_.empty() ) {
    return false;
  }

  INSTRUMENT_REGION("");

  cv::Mat2f rmaps[2];

  const cv::Size src_size =
      current_image_.size();

  create_epipolar_remaps(src_size,
      currentEpipole_,
      currentDerotationHomography_.inv(),
      rmaps[0],
      rmaps[1],
      nullptr,
      inverse_remap);

  const cv::Size dst_size(rmaps[0].cols * 2,
      rmaps[0].rows);

  cv::remap(current_image_, frames[0],
      rmaps[0], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  cv::remap(previous_image_, frames[1],
      rmaps[1], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  const cv::Mat current_mask =
      current_mask_.empty() ?
          cv::Mat1b(current_image_.size(), 255) :
          current_mask_;

  const cv::Mat previous_mask =
      previous_mask_.empty() ?
          cv::Mat1b(previous_image_.size(), 255) :
          previous_mask_;

  cv::remap(current_mask, masks[0],
      rmaps[0], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  cv::remap(previous_mask, masks[1],
      rmaps[1], cv::noArray(),
      cv::INTER_LINEAR,
      cv::BORDER_CONSTANT);

  return true;
}


bool c_virtual_stereo_pipeline::create_homography_display(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( current_image_.empty() || previous_image_.empty() ) {
    return false;
  }

  c_homography_image_transform homography;
  c_homography_ecc_motion_model model(&homography);
  c_ecc_forward_additive ecc(&model);
  c_ecch ecch(&ecc);

  cv::Mat cimg, pimg;

  if ( current_image_.channels() == 1 ) {
    cimg = current_image_;
  }
  else {
    cv::cvtColor(current_image_, cimg, cv::COLOR_BGR2GRAY);
  }

  if ( previous_image_.channels() == 1 ) {
    pimg = previous_image_;
  }
  else {
    cv::cvtColor(previous_image_, pimg, cv::COLOR_BGR2GRAY);
  }

  ecch.set_minimum_image_size(64);

  if ( !estimate_image_transform(&homography, matched_current_positions_, matched_previous_positions_) ) {
    CF_ERROR("estimate_image_transform() fails");
  }

  if ( !ecch.set_reference_image(pimg, previous_mask_) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  if ( !ecch.align(cimg, current_mask_) ) {
    CF_ERROR("ecch.align() fails");
    return false;
  }

  display_frame.create(previous_image_.size(), previous_image_.type());

  cv::Mat & dst_image = display_frame.getMatRef();
  cv::remap(current_image_, dst_image, ecc.current_remap(), cv::noArray(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  cv::addWeighted(dst_image, 0.5, previous_image_, 0.5, 0, dst_image);

  return true;
}


void c_virtual_stereo_pipeline::draw_matched_positions(cv::Mat & image,
    const std::vector<cv::Point2f> & current_positions,
    const std::vector<cv::Point2f> & previous_positions,
    const cv::Mat1b & inliers)
{

  const int N = current_positions.size();
  const cv::Scalar inlierColor = CV_RGB(220, 220, 0);
  const cv::Scalar outlierColor = CV_RGB(220, 0, 0);

  for( int i = 0; i < N; ++i ) {

    const cv::Point2f &cp =
        current_positions[i];

    const cv::Point2f &pp =
        previous_positions[i];

    const cv::Scalar color =
        inliers.empty() || inliers[i][0] ?
            inlierColor :
            outlierColor;

    cv::line(image, pp, cp, color, 1, cv::LINE_4);
    cv::rectangle(image, cv::Point(cp.x - 1, cp.y - 1), cv::Point(cp.x + 1, cp.y + 1), color, 1, cv::LINE_4);
  }
}
;


bool c_virtual_stereo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  //////////////////

  lock_guard lock(mutex());

  if( current_image_.empty() || previous_image_.empty() ) {
    return false;
  }

  const cv::Size size = current_image_.size();
  const cv::Size dst_size(2 * size.width, 2 * size.height);

  const cv::Rect cRoi(0, 0, size.width, size.height);
  const cv::Rect wRoi(0, size.height, size.width, size.height);
  const cv::Rect pRoi(size.width, 0, size.width, size.height);
  const cv::Rect dRoi(size.width, size.height, size.width, size.height);

  cv::Mat3b cimg(dst_size);

  if ( current_image_.channels() == 3 ) {
    current_image_.copyTo(cimg(cRoi));
  }
  else {
    cv::cvtColor(current_image_, cimg(cRoi), cv::COLOR_GRAY2BGR);
  }

  if ( !matched_previous_positions_.empty() ) {

    cv::Mat img = cimg(cRoi);
    draw_matched_positions(img,
        matched_current_positions_,
        matched_previous_positions_,
        currentInliers_);
  }

  cv::Mat tmp1, tmp2;
  std::vector<cv::Point2f> warped_current_positions_;

  /////////////////
  if( !matched_current_positions_.empty() ) {
    cv::perspectiveTransform(matched_current_positions_, warped_current_positions_,
        currentDerotationHomography_);
  }

  cv::warpPerspective(current_image_, tmp1,
      currentDerotationHomography_,
      pRoi.size(),
      cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
      cv::BORDER_CONSTANT);

  if ( tmp1.channels() == 1 ) {
    cv::cvtColor(tmp1, tmp1, cv::COLOR_GRAY2BGR);
  }


  /////////////////

  if ( previous_image_.channels() == 3 ) {
    previous_image_.copyTo(tmp2);
  }
  else {
    cv::cvtColor(previous_image_, tmp2, cv::COLOR_GRAY2BGR);
  }

  cv::addWeighted(tmp1, 0.5, tmp2, 0.5, 0, tmp2);


  /////////////////


  if( !warped_current_positions_.empty() && !matched_previous_positions_.empty() ) {

    draw_matched_positions(tmp1,
        warped_current_positions_,
        matched_previous_positions_,
        currentInliers_);
  }

  if( IS_INSIDE_IMAGE(currentEpipole_, tmp1.size()) ) {

    const cv::Point2f E(currentEpipole_.x, currentEpipole_.y);

    cv::ellipse(tmp1, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
    cv::line(tmp1, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(tmp1, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }

  if( IS_INSIDE_IMAGE(currentEpipole_, tmp2.size()) ) {

    const cv::Point2f E(currentEpipole_.x, currentEpipole_.y);

    cv::ellipse(tmp2, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
    cv::line(tmp2, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
    cv::line(tmp2, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
  }

  /////////////////


  tmp1.copyTo(cimg(wRoi));
  tmp2.copyTo(cimg(pRoi));

  if ( current_disparity_.empty() ) {
    cimg(dRoi).setTo(0);
  }
  else {
    current_disparity_.convertTo(tmp1, CV_8U, 255. / stereo_matcher_.currentMaxDisparity());
    apply_colormap(tmp1, cimg(dRoi), COLORMAP_TURBO);
  }


  /////////////////

  display_frame.move(cimg);

  if ( display_mask.needed() ) {
    current_mask_.copyTo(display_mask);
  }

  return true;
}

bool c_virtual_stereo_pipeline::write_progress_video()
{
  if ( !output_options_.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !progress_video_writer_.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(output_options_.progress_video_filename,
            "progress",
            ".avi");

    bool fOK =
        progress_video_writer_.open(output_video_filename,
            display.size(),
            display.channels() > 1,
            false);

    if( !fOK ) {
      CF_ERROR("progress_video_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !progress_video_writer_.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("progress_video_writer_.write() fails: %s",
        progress_video_writer_.filename().c_str());
    return false;
  }

  return true;
}


bool c_virtual_stereo_pipeline::write_homography_video()
{
  if ( !output_options_.save_homography_video ) {
    return true;
  }

  cv::Mat display;

  if ( !create_homography_display(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !homography_video_writer_.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(output_options_.homography_video_filename,
            "homography",
            ".avi");

    bool fOK =
        homography_video_writer_.open(output_video_filename,
            display.size(),
            display.channels() > 1,
            false);

    if( !fOK ) {
      CF_ERROR("homography_video_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !homography_video_writer_.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("homography_video_writer_.write() fails: %s",
        homography_video_writer_.filename().c_str());
    return false;
  }

  return true;
}


