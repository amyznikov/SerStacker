/*
 * c_roi_tracker_pipeline.cc
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#include "c_roi_tracker_pipeline.h"
#include <core/ssprintf.h>
#include <type_traits>
#include <chrono>
#include <thread>
#include <core/debug.h>

c_roi_tracker_pipeline::c_roi_tracker_pipeline(const std::string & name,
    const c_input_sequence::sptr & input_sequence) :
    base(name, input_sequence)
{
}

const c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options() const
{
  return _input_options;
}

c_roi_tracker_input_options & c_roi_tracker_pipeline::input_options()
{
  return _input_options;
}

const c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options() const
{
  return tracker_options_;
}

c_roi_tracker_pipeline_options & c_roi_tracker_pipeline::tracker_options()
{
  return tracker_options_;
}

const c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options() const
{
  return output_options_;
}

c_roi_tracker_output_options & c_roi_tracker_pipeline::output_options()
{
  return output_options_;
}


bool c_roi_tracker_pipeline::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  if ( !base::serialize(settings, save)) {
    return false;
  }

  if( (section = SERIALIZE_GROUP(settings, save, "input_options")) ) {
    serialize_base_input_options(section, save, _input_options);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "tracker_options")) ) {
    SERIALIZE_OPTION(section, save, tracker_options_, roi);
    ::serialize(tracker_options_.tracker, settings, save);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "output_options")) ) {
    SERIALIZE_OPTION(section, save, output_options_, output_directory);
    SERIALIZE_OPTION(section, save, output_options_, save_progress_video);
    SERIALIZE_OPTION(section, save, output_options_, progress_video_filename);
    SERIALIZE_OPTION(section, save, output_options_, default_display_type);
  }

  return true;
}

const std::vector<c_image_processing_pipeline_ctrl> & c_roi_tracker_pipeline::get_controls()
{
  static std::vector<c_image_processing_pipeline_ctrl> ctrls;

  if( ctrls.empty() ) {

    PIPELINE_CTL_GROUP(ctrls, "Input options", "");
      POPULATE_PIPELINE_INPUT_OPTIONS(ctrls)
    PIPELINE_CTL_END_GROUP(ctrls);

    PIPELINE_CTL_GROUP(ctrls, "ROI tracking", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.enabled, "enable tracker", "");
      PIPELINE_CTL(ctrls, tracker_options_.roi, "ROI Rect X;Y;WxH;", "");

      PIPELINE_CTL(ctrls, tracker_options_.tracker.tracker_type,  "Tracker type", "Select tarcker type to use");

      PIPELINE_CTL_GROUP(ctrls, "BOOSTING", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.boosting.numClassifiers, "numClassifiers", "the number of classifiers to use in a OnlineBoosting algorithm");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.boosting.samplerOverlap, "samplerOverlap", "search region parameters to use in a OnlineBoosting algorithm");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.boosting.samplerSearchFactor, "samplerSearchFactor", "search region parameters to use in a OnlineBoosting algorithm");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.boosting.iterationInit, "iterationInit", "the initial iterations");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.boosting.featureSetNumFeatures, "featureSetNumFeatures", "# features");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "MIL", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerInitInRadius, "samplerInitInRadius", "radius for gathering positive instances during init");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerInitMaxNegNum, "samplerInitMaxNegNum", "# negative samples to use during init");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerSearchWinSize, "samplerSearchWinSize", "size of search window");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerTrackInRadius, "samplerTrackInRadius", "radius for gathering positive instances during tracking");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerTrackMaxPosNum, "samplerTrackMaxPosNum", "# positive samples to use during tracking");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.samplerTrackMaxNegNum, "samplerTrackMaxNegNum", "# negative samples to use during tracking");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.mil.featureSetNumFeatures, "featureSetNumFeatures", "# features");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "KCF", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.detect_thresh, "detect_thresh", "detection confidence threshold");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.sigma, "sigma", "gaussian kernel bandwidth");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.lambda, "lambda", "regularization");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.interp_factor, "interp_factor", "linear interpolation factor for adaptation");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.output_sigma_factor, "output_sigma_factor", "spatial bandwidth (proportional to target)");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.pca_learning_rate, "pca_learning_rate", "compression learning rate");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.resize, "resize", "activate the resize feature to improve the processing speed");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.split_coeff, "split_coeff", "split the training coefficients into two matrices");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.wrap_kernel, "wrap_kernel", "wrap around the kernel values");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.compress_feature, "compress_feature", "activate the pca method to compress the features");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.max_patch_size, "max_patch_size", "threshold for the ROI size");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.compressed_size, "compressed_size", "feature size after compression");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.desc_pca, "desc_pca", " compressed descriptors of TrackerKCF::MODE");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.kcf.desc_npca, "desc_npca", "non-compressed descriptors of TrackerKCF::MODE");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "TLD", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "MEDIANFLOW", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.pointsInGrid, "pointsInGrid", "square root of number of keypoints used; increase it to trade accurateness for speed");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.winSize, "winSize", "window size parameter for Lucas-Kanade optical flow");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.maxLevel, "maxLevel", "maximal pyramid level number for Lucas-Kanade optical flow");
      PIPELINE_CTL_BITFLAGS(ctrls, tracker_options_.tracker.medianflow.termCriteria.type, cv::TermCriteria::Type, "termCriteria.type", "termination criteria for Lucas-Kanade optical flow");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.termCriteria.maxCount, "termCriteria.COUNT", "termination criteria for Lucas-Kanade optical flow");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.termCriteria.epsilon, "termCriteria.epsilon", "termination criteria for Lucas-Kanade optical flow");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.winSizeNCC, "winSizeNCC", "window size around a point for normalized cross-correlation check");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.medianflow.maxMedianLengthOfDisplacementDifference, "maxMedianLengthOfDisplacementDifference", "criterion for loosing the tracked object");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "MOSSE", "");
      PIPELINE_CTL_END_GROUP(ctrls);

      PIPELINE_CTL_GROUP(ctrls, "CSRT", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_hog, "use_hog", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_color_names, "use_color_names", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_gray, "use_gray", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_rgb, "use_rgb", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_channel_weights, "use_channel_weights", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.use_segmentation, "use_segmentation", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.window_function, "window_function", "Window function: hann, cheb, kaiser");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.kaiser_alpha, "kaiser_alpha", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.cheb_attenuation, "cheb_attenuation", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.template_size, "template_size", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.gsl_sigma, "gsl_sigma", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.hog_orientations, "hog_orientations", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.hog_clip, "hog_clip", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.padding, "padding", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.filter_lr, "filter_lr", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.weights_lr, "weights_lr", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.num_hog_channels_used, "num_hog_channels_used", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.admm_iterations, "admm_iterations", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.histogram_bins, "histogram_bins", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.histogram_lr, "histogram_lr", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.background_ratio, "background_ratio", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.number_of_scales, "number_of_scales", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.scale_sigma_factor, "scale_sigma_factor", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.scale_model_max_area, "scale_model_max_area", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.scale_lr, "scale_lr", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.scale_step, "scale_step", "");
      PIPELINE_CTL(ctrls, tracker_options_.tracker.csrt.psr_threshold, "psr_threshold", "we lost the target, if the psr is lower than this");
      PIPELINE_CTL_END_GROUP(ctrls);

#if CV_VERSION_CURRRENT < CV_VERSION_INT(4, 5, 1)
      PIPELINE_CTL_GROUP(ctrls, "GOTURN", "");
      PIPELINE_CTL_END_GROUP(ctrls);
#endif

    PIPELINE_CTL_END_GROUP(ctrls);


    PIPELINE_CTL_GROUP(ctrls, "Output options", "");
      PIPELINE_CTL_BROWSE_FOR_DIRECTORY(ctrls, output_options_.output_directory, "output_directory", "");
      PIPELINE_CTL(ctrls, output_options_.save_progress_video, "save_progress_video", "");
      PIPELINE_CTLC(ctrls, output_options_.progress_video_filename, "progress_video_filename", "", _this->output_options_.save_progress_video);
    PIPELINE_CTL_END_GROUP(ctrls);
  }

  return ctrls;
}

bool c_roi_tracker_pipeline::initialize_pipeline()
{
  if ( !base::initialize_pipeline() ) {
   CF_ERROR("c_camera_calibration_pipeline: base::initialize() fails");
   return false;
 }

  output_path_ =
      create_output_path(output_options_.output_directory);

  if( tracker_options_.tracker.enabled ) {

    if( (objbox_ = tracker_options_.roi).empty() ) {
      CF_ERROR("INPUT ERROR: Tracking ROI is empty: x=%d y=%d w=%d h=%d ",
          objbox_.x, objbox_.y,
          objbox_.width, objbox_.height);
      return false;
    }

    if( !tracker_.initialize(tracker_options_.tracker) ) {
      CF_ERROR("tracker_.initialize() fails");
      return false;
    }
  }

  return true;
}

void c_roi_tracker_pipeline::cleanup_pipeline()
{
  if ( input_sequence_ ) {
    input_sequence_->close();
  }

  if ( progress_writer_.is_open() ) {
    CF_DEBUG("Closing '%s'", progress_writer_.filename().c_str());
    progress_writer_.close();
  }

  tracker_.release();
}

bool c_roi_tracker_pipeline::run_pipeline()
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
    processed_frames_ = 0;
    accumulated_frames_ = 0;
  }
  else {

    const int start_pos =
        std::max(_input_options.start_frame_index, 0);

    const int end_pos =
        _input_options.max_input_frames < 1 ?
            input_sequence_->size() :
            std::min(input_sequence_->size(),
                _input_options.start_frame_index + _input_options.max_input_frames);

    total_frames_ = end_pos - start_pos;
    processed_frames_ = 0;
    accumulated_frames_ = 0;

    if( total_frames_ < 1 ) {
      CF_ERROR("INPUT ERROR: Number of frames to process = %d is less than 1\n"
          "start_pos=%d end_pos=%d input_sequence_->size()=%d max_input_frames=%d is_live_sequence=%d",
          total_frames_,
          start_pos,
          end_pos,
          input_sequence_->size(),
          _input_options.max_input_frames,
          is_live_sequence);
      return false;
    }

    if( !input_sequence_->seek(start_pos) ) {
      CF_ERROR("ERROR: input_sequence_->seek(start_pos=%d) fails", start_pos);
      return false;
    }
  }

  set_status_msg("RUNNING ...");

  for( ; processed_frames_ < total_frames_; ++processed_frames_, on_frame_processed() ) {

    if( canceled() ) {
      break;
    }

    if( true ) {
      lock_guard lock(mutex());
      if( !input_sequence_->read(current_image_, &current_mask_) ) {
        CF_DEBUG("input_sequence_->read() fails");
        break;
      }
    }

    if( canceled() ) {
      break;
    }

    if( _input_options.input_image_processor ) {
      lock_guard lock(mutex());
      if( !_input_options.input_image_processor->process(current_image_, current_mask_) ) {
        CF_ERROR("input_image_processor->process() fails");
        return false;
      }
    }

    if( !process_current_frame() ) {
      CF_ERROR("process_current_frame() fails");
      return false;
    }

    ++accumulated_frames_;

    if( !is_live_sequence ) {
      // give chance to GUI thread to call get_display_image()
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

  }

  return true;
}




bool c_roi_tracker_pipeline::process_current_frame()
{
  if ( tracker_options_.tracker.enabled ) {

    bool hasUpdates = false;

    if ( !tracker_.track(current_image_, objbox_, &hasUpdates) ) {
      CF_ERROR("tracker_.track() fails");
      return false;
    }


    CF_DEBUG("hasUpdates: %d objbox_: %d;%d %dx%d", hasUpdates,
        objbox_.x, objbox_.y, objbox_.width, objbox_.height);

  }

  if ( !write_progress_video() ) {
    CF_ERROR("write_progress_video() fails");
    return false;
  }

  return true;
}

bool c_roi_tracker_pipeline::get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask)
{
  lock_guard lock(mutex());

  if ( current_image_.empty() ) {
    return false;
  }

  if ( current_image_.channels() == 1 ) {
    cv::cvtColor(current_image_, display_frame, cv::COLOR_GRAY2BGR);
  }
  else {
    current_image_.copyTo(display_frame);
  }

  if ( tracker_options_.tracker.enabled ) {
    cv::rectangle(display_frame.getMatRef(), objbox_,
        CV_RGB(32, 255, 64), 1, cv::LINE_8);
  }

  return true;
}

bool c_roi_tracker_pipeline::write_progress_video()
{
  if ( !output_options_.save_progress_video ) {
    return true;
  }

  cv::Mat display;

  if ( !get_display_image(display, cv::noArray()) ) {
    return true; // ignore
  }

  if ( !progress_writer_.is_open() ) {

    const std::string output_video_filename =
        generate_output_filename(output_options_.progress_video_filename,
            "progress",
            ".avi");

    bool fOK =
        progress_writer_.open(output_video_filename);

    if( !fOK ) {
      CF_ERROR("progress_writer_.open('%s') fails",
          output_video_filename.c_str());
      return false;
    }

    CF_DEBUG("Created '%s' display.size()=%dx%d",
        output_video_filename.c_str(),
        display.cols,
        display.rows);
  }

  if( !progress_writer_.write(display, cv::noArray(), false, 0) ) {
    CF_ERROR("homography_video_writer_.write() fails: %s",
        progress_writer_.filename().c_str());
    return false;
  }


  return true;
}

