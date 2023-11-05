/*
 * c_roi_tracker_pipeline.h
 *
 *  Created on: Sep 13, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_roi_tracker_pipeline_h__
#define __c_roi_tracker_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/tracking/c_roi_tracker.h>
#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>

struct c_roi_tracker_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_roi_tracker_pipeline_options
{
  cv::Rect roi;
  c_roi_tracker_options tracker;
};

struct c_roi_tracker_output_options:
    c_image_processing_pipeline_output_options
{
  std::string progress_video_filename;
  bool save_progress_video = true;
};


class c_roi_tracker_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_roi_tracker_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_roi_tracker_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "roi_tracker";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_roi_tracker_pipeline.</strong><br>"
        "This pipeline is for experiments with ROI trackers<br>";
    return tooltip_;
  }

  const c_roi_tracker_input_options & input_options() const;
  c_roi_tracker_input_options & input_options();

  const c_roi_tracker_pipeline_options & tracker_options() const;
  c_roi_tracker_pipeline_options & tracker_options();

  const c_roi_tracker_output_options & output_options() const;
  c_roi_tracker_output_options & output_options();

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();
  bool write_progress_video();

protected:
  c_roi_tracker_input_options input_options_;
  c_roi_tracker_pipeline_options tracker_options_;
  c_roi_tracker_output_options output_options_;

  cv::Mat current_image_;
  cv::Mat current_mask_;

  c_roi_tracker tracker_;
  cv::Rect objbox_;

  c_output_frame_writer progress_writer_;

};

#endif /* __c_roi_tracker_pipeline_h__ */
