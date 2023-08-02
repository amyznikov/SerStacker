/*
 * c_virtual_stereo_pipeline.cc
 *
 *  Created on: Mar 9, 2023
 *      Author: amyznikov
 */

#include "c_virtual_stereo_pipeline.h"
#include <core/feature2d/feature2d_settings.h>
#include <chrono>
#include <thread>


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

  if( (section = SERIALIZE_GROUP(settings, save, "image_processing")) ) {
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, input_processor);
    SERIALIZE_IMAGE_PROCESSOR(section, save, image_processing_options_, feature2d_preprocessor);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
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
//      PIPELINE_CTL(ctrls, camera_options_.camera_intrinsics.image_size, "image_size", "image_size");
//      PIPELINE_CTL_CV_MATX(ctrls, camera_options_.camera_intrinsics.camera_matrix, "camera_matrix", "camera_matrix");
//      PIPELINE_CTL(ctrls, camera_options_.camera_intrinsics.dist_coeffs, "dist_coeffs", "dist_coeffs");
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
    PIPELINE_CTL_END_GROUP(ctrls);

    ////////
  }

  return ctrls;
}

bool c_virtual_stereo_pipeline::initialize_pipeline()
{
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

  return true;
}

void c_virtual_stereo_pipeline::cleanup_pipeline()
{
  close_input_sequence();

  if ( progress_video_writer_.is_open() ) {
    progress_video_writer_.close();
    CF_DEBUG("SAVED '%s'", progress_video_writer_.filename().c_str());
  }
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
  lock_guard lock(mutex());

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

    if( !read_input_frame(current_image_, current_mask_) ) {
      CF_DEBUG("read_input_frame() fails");
      break;
    }

    if( canceled() ) {
      break;
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    if ( !write_progress_video() ) {
      CF_DEBUG("write_progress_video() fails");
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
    }
  }


  return true;
}

bool c_virtual_stereo_pipeline::process_current_frame()
{
  lock_guard lock(mutex());

  if( image_processing_options_.input_processor ) {
    if( !image_processing_options_.input_processor->process(current_image_, current_mask_) ) {
      CF_ERROR("ERROR: input_processor->process() fails");
      return false;
    }
  }

  //  if ( image_processing_options_.feature2d_preprocessor ) {
  //  }

  keypoints_extractor_->detectAndCompute(current_image_, current_mask_,
      current_keypoints_, current_descriptors_);

  if( !previous_keypoints_.empty() ) {

    //current_matches_.clear();
    matched_current_positions_.clear();
    matched_previous_positions_.clear();
    current_inliers_.release();

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
        if( d2 > 9 ) {
          matched_current_positions_.emplace_back(cp);
          matched_previous_positions_.emplace_back(pp);
        }
      }

    }
  }

  if( !estmate_camera_pose() ) {
    CF_ERROR("estmate_camera_pose() fails");
  }

  return true;
}

bool c_virtual_stereo_pipeline::estmate_camera_pose()
{
  if (matched_previous_positions_.size() < 4 ) {
    return true; // ignore
  }





  //matched_current_positions_.emplace_back(cp);
  //matched_previous_positions_.emplace_back(pp);

  return true;
}


bool c_virtual_stereo_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if( current_image_.empty() ) {
    return false;
  }

  cv::Mat cimg;

  if ( current_image_.channels() == 3 ) {
    cimg = current_image_;
  }
  else {
    cv::cvtColor(current_image_, cimg, cv::COLOR_GRAY2BGR);
  }


  if ( !matched_previous_positions_.empty() ) {

    const int N = matched_current_positions_.size();
    //CF_DEBUG("N=%d", N);

    for ( int i = 0; i < N; ++i ) {

      const cv::Point2f & cp =
          matched_current_positions_[i];

      const cv::Point2f & pp =
          matched_previous_positions_[i];

      cv::Scalar color(rand() % 255 + 32, rand() % 255 + 32, rand() % 255 + 32);

      cv::line(cimg, pp, cp, color, 1, cv::LINE_4);
      cv::rectangle(cimg, cv::Point(cp.x-1, cp.y-1), cv::Point(cp.x+1, cp.y+1), color, 1, cv::LINE_4);
    }

  }

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

    CF_DEBUG("Created '%s'", output_video_filename.c_str());
  }

  if( !progress_video_writer_.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("progress_video_writer_.write() fails: %s",
        progress_video_writer_.filename().c_str());
    return false;
  }

  return true;
}

