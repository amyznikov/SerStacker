/*
 * c_camera_calibration_pipeline.h
 *
 *  Created on: Feb 22, 2023
 *      Author: amyznikov
 *
 * Based on /opencv/apps/interactive-calibration
 */

#pragma once
#ifndef __c_chessboard_camera_calibration_pipeline_h__
#define __c_chessboard_camera_calibration_pipeline_h__

#include "c_image_processing_pipeline.h"
#include "camera_calibration/c_camera_calibration.h"


struct c_camera_calibration_input_options
{
  int start_frame_index = 0;
  int max_input_frames = -1;

  bool inpaint_missing_pixels = true;
  bool enable_color_maxtrix = true;
};


class c_camera_calibration_pipeline :
    public c_image_processing_pipeline,
    public c_camera_calibration
{
public:
  typedef c_camera_calibration_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_camera_calibration_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_camera_calibration_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "camera_calibration";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_camera_calibration_pipeline.</strong><br>"
        "This pipeline uses cv::calibrateCamera() for camera calibration<br>";
    return tooltip_;
  }

  c_camera_calibration_input_options & input_options();
  const c_camera_calibration_input_options & input_options() const;

  bool serialize(c_config_setting setting, bool save) override;
  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;

protected:
  bool canceled() const override;
  bool initialize_pipeline()override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool run_chessboard_corners_collection();
  bool open_input_sequence();
  void close_input_sequence();
  bool seek_input_sequence(int pos);
  bool read_input_frame(const c_input_sequence::sptr & input_sequence, cv::Mat & output_image, cv::Mat & output_mask) const;
  bool write_output_videos();

protected:

  cv::Mat missing_pixel_mask_;
  c_camera_calibration_input_options input_options_;
  mutable std::mutex display_lock_;
};

#endif /* __c_chessboard_camera_calibration_pipeline_h__ */
