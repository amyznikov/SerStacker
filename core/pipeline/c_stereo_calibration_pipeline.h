/*
 * c_stereo_calibration_pipeline.h
 *
 *  Created on: Mar 1, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_calibration_pipeline_h__
#define __c_stereo_calibration_pipeline_h__

#include "c_image_processing_pipeline.h"
#include "stereo_calibration/c_stereo_calibration.h"

struct c_stereo_calibration_input_options
{
  std::string left_stereo_source;
  std::string right_stereo_source;

  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};


class c_stereo_calibration_pipeline:
    public c_image_processing_pipeline,
    public c_stereo_calibration
{
public:
  typedef c_stereo_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_stereo_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_stereo_calibration_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "stereo_calibration";
    return classname_;
  }

  c_stereo_calibration_input_options & input_options();
  const c_stereo_calibration_input_options & input_options() const;

  void set_output_directory(const std::string & output_directory) override;
  const std::string & output_directory() const override;

  bool serialize(c_config_setting setting, bool save) override;

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask);

  c_notification<void()> on_current_frame_changed;
  c_notification<void()> on_accumulator_changed;
  c_notification<void(STEREO_CALIBRATION_STAGE oldstage, STEREO_CALIBRATION_STAGE newstage)> on_pipeline_stage_changed;

protected:
  bool canceled() override;
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool run_stereo_calibration();
  bool write_output_videos();
  void update_output_path() override;
  bool read_input_frame(const c_input_source::sptr & source, cv::Mat & output_image, cv::Mat & output_mask) const;
  void update_display_image() override;

protected:

  cv::Mat missing_pixel_mask_;
  c_stereo_calibration_input_options input_options_;
  c_input_source::sptr input_sources_[2];

  mutable std::mutex accumulator_lock_;
  mutable std::mutex display_lock_;
};

#endif /* __c_stereo_calibration_pipeline_h__ */
