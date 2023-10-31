/*
 * c_vlo_pipeline.h
 *
 *  Created on: Oct 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_pipeline_h__
#define __c_vlo_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/proc/threshold.h>
#include <core/io/c_vlo_file.h>
#include <core/io/c_output_frame_writer.h>


struct c_vlo_pipeline_input_options :
    c_image_processing_pipeline_input_options
{
  bool sort_echos_by_distance = false;
};


struct c_vlo_pipeline_processing_options
{
   bool enable_reflectors_detection = false;
   bool enable_double_echo_detection = true;
   bool enable_auto_threshold = true;

   THRESHOLD_TYPE auto_threshold_type = THRESHOLD_TYPE_YEN;
   double auto_threshold_value = 0;
   double auto_clip_min = 0.1; // percentage of 'black' pixels
   double auto_clip_max = 99.9; // 100-percentage of 'white' pixels
};

struct c_vlo_pipeline_output_options :
    c_image_processing_pipeline_output_options
{
  bool save_progress_video = false;
  bool save_cloud3d_ply = false;

  std::string progress_video_filename;
  std::string cloud3d_filename;
};

class c_vlo_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_vlo_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "vlo";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_vlo_pipeline</strong><br>";
    return tooltip_;
  }

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

protected:
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool process_current_frame();
  bool run_reflectors_detection();
  bool save_progress_video();
  bool save_cloud3d_ply();

protected:
  c_vlo_pipeline_input_options input_options_;
  c_vlo_pipeline_processing_options processing_options_;
  c_vlo_pipeline_output_options output_options_;
  c_vlo_scan current_scan_;
  cv::Mat1b current_reflection_mask_;

  c_output_frame_writer progress_writer_;
  c_output_frame_writer reflectors_writer_;

};

#endif /* __c_vlo_pipeline_h__ */
