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


static void correlate(const cv::Mat & src1, const cv::Mat & src2, cv::Mat & dst, const cv::Mat1f & G, double eps = 255)
{
  cv::Mat s1, s2, cov, norm;

  cv::sepFilter2D(src1, s1, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(src1, s1, s1, cv::noArray(), CV_32F);

  cv::sepFilter2D(src2, s2, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::subtract(src2, s2, s2, cv::noArray(), CV_32F);


  cv::sepFilter2D(s1.mul(s2), cov, CV_32F, G, G, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);

  cv::sepFilter2D(s1.mul(s1), s1, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::sepFilter2D(s2.mul(s2), s2, CV_32F, G, G, cv::Point(-1, -1), eps, cv::BORDER_REPLICATE);
  cv::sqrt(s1.mul(s2), norm);

  cv::divide(cov, norm, dst);
}


static void extract_pixel_matches(const cv::Mat2f & rmap, const cv::Mat1b & mask,
    std::vector<cv::Point2f> & cpts, std::vector<cv::Point2f> & rpts)
{

  for ( int y = 0; y < mask.rows; ++y ) {
    for ( int x = 0; x < mask.cols; ++x ) {
      if ( mask[y][x] ) {
        rpts.emplace_back(x,y);
        cpts.emplace_back(rmap[y][x]);
      }
    }
  }

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

  if( (section = SERIALIZE_GROUP(settings, save, "eccflow")) ) {
    SERIALIZE_PROPERTY(section, save, eccflow_, support_scale);
    SERIALIZE_PROPERTY(section, save, eccflow_, support_scale);
    SERIALIZE_PROPERTY(section, save, eccflow_, max_iterations);
    SERIALIZE_PROPERTY(section, save, eccflow_, update_multiplier);
    SERIALIZE_PROPERTY(section, save, eccflow_, normalization_scale);
    SERIALIZE_PROPERTY(section, save, eccflow_, input_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, eccflow_, reference_smooth_sigma);
    SERIALIZE_PROPERTY(section, save, eccflow_, scale_factor);
    SERIALIZE_PROPERTY(section, save, eccflow_, min_image_size);
    SERIALIZE_PROPERTY(section, save, eccflow_, noise_level);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "camera_pose")) ) {

    SERIALIZE_OPTION(section, save, *this, correlation_threshold_);
    SERIALIZE_OPTION(section, save, *this, correlation_eps_);
    SERIALIZE_OPTION(section, save, *this, correlation_kernel_radius_);

    SERIALIZE_OPTION(section, save, camera_pose_options_, direction);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, robust_threshold);
    SERIALIZE_OPTION(section, save, camera_pose_options_, max_levmar_iterations);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsx);
    SERIALIZE_OPTION(section, save, camera_pose_options_, epsf);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {

    SERIALIZE_OPTION(section, save, output_options_, output_directory);

    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    if( (subsection = SERIALIZE_GROUP(section, save, "progress_video_output_options")) ) {
      SERIALIZE_OPTION(subsection, save, output_options_, progress_video_output_options);
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

    PIPELINE_CTL_GROUP(ctrls, "Ecc Options", "");
    //    int max_iterations() const;
    //    double max_eps() const;
    //    double min_rho() const;
    //    enum ECC_INTERPOLATION_METHOD interpolation() const;
    //    double input_smooth_sigma() const;
    //    double reference_smooth_sigma() const;
    //    double update_step_scale() const;
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "EccFlow Options", "");
      PIPELINE_CTLP2(ctrls, eccflow_, support_scale, "support_scale", "");
      PIPELINE_CTLP2(ctrls, eccflow_, max_iterations, "max_iterations", "");
      PIPELINE_CTLP2(ctrls, eccflow_, update_multiplier, "update_multiplier", "");
      PIPELINE_CTLP2(ctrls, eccflow_, normalization_scale, "normalization_scale", "");
      PIPELINE_CTLP2(ctrls, eccflow_, input_smooth_sigma, "input_smooth_sigma", "");
      PIPELINE_CTLP2(ctrls, eccflow_, reference_smooth_sigma, "reference_smooth_sigma", "");
      PIPELINE_CTLP2(ctrls, eccflow_, scale_factor, "scale_factor", "");
      PIPELINE_CTLP2(ctrls, eccflow_, min_image_size, "min_image_size", "");
      PIPELINE_CTLP2(ctrls, eccflow_, noise_level, "noise_level", "");
    PIPELINE_CTL_END_GROUP(ctrls);
    ////////
    PIPELINE_CTL_GROUP(ctrls, "Camera Pose Estimation", "Parameters for lm_refine_camera_pose2()");

      PIPELINE_CTL(ctrls, correlation_threshold_, "corr. threshold", "");
      PIPELINE_CTL(ctrls, correlation_eps_, "corr. eps", "");
      PIPELINE_CTL(ctrls, correlation_kernel_radius_, "corr. radius", "");

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

      PIPELINE_CTL_GROUP(ctrls, "Save Progress video", "");
        PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
        PIPELINE_CTL_OUTPUT_WRITER_OPTIONS(ctrls, output_options_.progress_video_output_options,
            _this->output_options_.save_progress_video);
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
  p->camera_options_ = this->camera_options_;
  p->camera_pose_options_ = this->camera_pose_options_;
  p->output_options_ = this->output_options_;
  p->correlation_threshold_  = this->correlation_threshold_;
  p->correlation_eps_ = this->correlation_eps_;
  p->correlation_kernel_radius_ = this->correlation_kernel_radius_;

  return true;
}

bool c_epipolar_alignment_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  //  if( current_frame_.empty() || previous_frame_.empty() || current_remap_.empty() ) {
  //    return false;
  //  }

  if( cimg.empty() || rimg.empty() || mimg.empty() || current_remap_.empty() ) {
    return false;
  }

  cv::Mat current_frame, previous_frame, remapped_frame;

  if ( cimg.channels() == 3 ) {
    current_frame = cimg;
  }
  else {
    cv::cvtColor(cimg, current_frame, cv::COLOR_GRAY2BGR);
  }

  if ( rimg.channels() == 3 ) {
    previous_frame = rimg;
  }
  else {
    cv::cvtColor(rimg, previous_frame, cv::COLOR_GRAY2BGR);
  }

  if ( mimg.channels() == 3 ) {
    remapped_frame = rimg;
  }
  else {
    cv::cvtColor(mimg, remapped_frame, cv::COLOR_GRAY2BGR);
  }


  const cv::Size size =
      current_frame.size();

  const cv::Size display_size(2 * size.width, 3 * size.height);

  const cv::Rect roi[6] = {
      cv::Rect(0, 0, size.width, size.height),
      cv::Rect(size.width, 0, size.width, size.height),
      cv::Rect(0, size.height, size.width, size.height),
      cv::Rect(size.width, size.height, size.width, size.height),
      cv::Rect(0, 2 * size.height, size.width, size.height),
      cv::Rect(size.width, 2 * size.height, size.width, size.height),
  };

  if ( display_mask.needed() ) {
    display_mask.release();
  }

  if( display_frame.needed() ) {

    display_frame.create(display_size,
        current_frame.type());

    cv::Mat & display =
        display_frame.getMatRef();

    previous_frame.copyTo(display(roi[0]));
    current_frame.copyTo(display(roi[1]));

    cv::warpPerspective(current_frame, display(roi[2]),
        currentDerotationHomography_,
        size,
        cv::INTER_LINEAR, // cv::INTER_LINEAR may introduce some blur on kitti images
        cv::BORDER_CONSTANT);

    cv::addWeighted(previous_frame, 0.7, display(roi[2]), 0.7, 0,
        display(roi[2]));

    cv::addWeighted(remapped_frame, 0.7, previous_frame, 0.7, 0,
        display(roi[3]));

    if( display.channels() == 1 ) {
      current_correlation_.convertTo(display(roi[4]), display.depth(), 100);
    }
    else {
      cv::Mat tmp;
      current_correlation_.convertTo(tmp, display.depth(), 100);
      cv::cvtColor(tmp, display(roi[4]),
          cv::COLOR_GRAY2BGR);
    }

    if ( current_inliers_.rows == matched_previous_positions_.size() ) {

      cv::Mat3b m = display(roi[5]);
      m.setTo(cv::Vec3b::all(0));

      for ( int i = 0, n = matched_previous_positions_.size(); i < n; ++i ) {

        const cv::Point2f & p =
            matched_previous_positions_[i];

        const int x = (int)(p.x);
        const int y = (int)(p.y);

        if ( current_inliers_[i][0] ) {
          m[y][x][0] = 255;
          m[y][x][1] = 255;
          m[y][x][2] = 255;
        }
        else {
          m[y][x][0] = 0;
          m[y][x][1] = 0;
          m[y][x][2] = 255;
        }
      }

    }


    for( int i = 0; i < 6; ++i ) {

      cv::Mat pane =
          display(roi[i]);

      cv::rectangle(pane, cv::Point(0, 0), cv::Point(pane.cols - 1, pane.rows - 1),
          CV_RGB(140, 128, 64), 1,
          cv::LINE_4);

      if( IS_INSIDE_IMAGE(currentEpipole_, size) ) {

        const cv::Point2f E(currentEpipole_.x, currentEpipole_.y);

        cv::ellipse(pane, E, cv::Size(11, 11), 0, 0, 360, CV_RGB(255, 60, 60), 1, cv::LINE_8);
        cv::line(pane, cv::Point2f(E.x - 9, E.y - 9), cv::Point2f(E.x + 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
        cv::line(pane, cv::Point2f(E.x + 9, E.y - 9), cv::Point2f(E.x - 9, E.y + 9), CV_RGB(0, 255, 0), 1, cv::LINE_8);
      }
    }
  }


  return true;
}

bool c_epipolar_alignment_pipeline::initialize()
{
  eccflow_.set_support_scale(3);
  eccflow_.set_noise_level(1e-3);
  eccflow_.set_min_image_size(4);
  eccflow_.set_scale_factor(0.75);
  eccflow_.set_max_iterations(3);

  return true;
}


bool c_epipolar_alignment_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  current_frame_.release();
  current_mask_.release();
  previous_frame_.release();
  previous_mask_.release();
  matched_current_positions_.clear();
  matched_previous_positions_.clear();
  current_remap_.release();
  current_inliers_.release();

  output_path_ =
      create_output_path(output_options_.output_directory);

  G =
      cv::getGaussianKernel(2 * std::max(correlation_kernel_radius_, 1) + 1,
          0, CV_32F);

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
  if ( !extract_and_match_keypoints() ) {
    CF_ERROR("extract_keypoints() fails");
    return false;
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
    return false;
  }

  if ( !save_progess_video() ) {
    CF_ERROR("save_epipolar_stereo_frames() fails");
    return false;
  }

  return true;
}


bool c_epipolar_alignment_pipeline::extract_and_match_keypoints()
{
  INSTRUMENT_REGION("");

  if ( previous_frame_.empty() ) {
    // silently ignore if still wait for next frame
    return true;
  }

  lock_guard lock(mutex());

  if( current_frame_.channels() == 1 ) {
    cv::morphologyEx(current_frame_, cimg, cv::MORPH_GRADIENT,
        cv::Mat1b(3, 3, (uint8_t) 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }
  else {
    cv::cvtColor(current_frame_, cimg,
        cv::COLOR_BGR2GRAY);
    cv::morphologyEx(cimg, cimg, cv::MORPH_GRADIENT,
        cv::Mat1b(3, 3, (uint8_t) 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }

  if( previous_frame_.channels() == 1 ) {
    cv::morphologyEx(previous_frame_, rimg, cv::MORPH_GRADIENT,
        cv::Mat1b(3, 3, (uint8_t) 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }
  else {
    cv::cvtColor(previous_frame_, rimg,
        cv::COLOR_BGR2GRAY);
    cv::morphologyEx(rimg, rimg, cv::MORPH_GRADIENT,
        cv::Mat1b(3, 3, (uint8_t) 255),
        cv::Point(-1, -1), 1,
        cv::BORDER_REPLICATE);
  }


  current_remap_.release();
  if( !eccflow_.compute(cimg, rimg, current_remap_, current_mask_, previous_mask_) ) {
    CF_ERROR("eccflow_.compute() fails");
    return false;
  }

  cv::remap(cimg, mimg, current_remap_, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT);

  correlate(rimg, mimg, current_correlation_, G,
      correlation_eps_);

  cv::compare(current_correlation_,
      cv::Scalar::all(correlation_threshold_),
      current_correlation_mask_,
      cv::CMP_GE);

  matched_current_positions_.clear();
  matched_previous_positions_.clear();

  extract_pixel_matches(current_remap_,
      current_correlation_mask_,
      matched_current_positions_,
      matched_previous_positions_);

  return true;
}


bool c_epipolar_alignment_pipeline::estmate_camera_pose()
{
  INSTRUMENT_REGION("");

  if( matched_current_positions_.size() < 6 || matched_previous_positions_.size() < 6 ) {
    return true; // ignore
  }

  lock_guard lock(mutex());

  current_inliers_.create(matched_current_positions_.size(), 1);
  current_inliers_.setTo(255);

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

  CF_DEBUG("current_inliers_.size=%dx%d nz=%d matched_previous_positions_.size=%zu",
      current_inliers_.rows, current_inliers_.cols,
      cv::countNonZero(current_inliers_),
      matched_previous_positions_.size());

  compute_epipoles(currentFundamentalMatrix_,
      currentEpipoles_);

  currentEpipole_ =
      0.5 * (currentEpipoles_[0] + currentEpipoles_[1]);

    CF_DEBUG("A: (%g %g %g)", current_euler_anges_(0) * 180 / CV_PI, current_euler_anges_(1) * 180 / CV_PI, current_euler_anges_(2) * 180 / CV_PI);
    CF_DEBUG("T: (%g %g %g)", current_translation_vector_(0), current_translation_vector_(1), current_translation_vector_(2));
    CF_DEBUG("E: (%g %g) EE: {%+g %+g} {%+g %+g}", currentEpipole_.x, currentEpipole_.y,
        currentEpipoles_[0].x, currentEpipoles_[0].y, currentEpipoles_[1].x, currentEpipoles_[1].y);

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

bool c_epipolar_alignment_pipeline::save_progess_video()
{
  if( !output_options_.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    CF_WARNING("get_display_image() fails");
    return true;
  }

  bool fOk =
      add_output_writer(progress_writer_,
            output_options_.progress_video_output_options,
            "progress",
            ".avi");

  if( !fOk ) {
    CF_ERROR("add_output_writer('%s') fails",
        progress_writer_.filename().c_str());
    return false;
  }

  if( !progress_writer_.write(display) ) {
    CF_ERROR("progress_writer_.write('%s') fails.",
        progress_writer_.filename().c_str());
    return false;
  }

  return true;
}


