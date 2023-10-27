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
#include <core/io/c_vlo_file.h>
#include <core/io/c_output_frame_writer.h>


struct c_vlo_pipeline_input_options :
    c_image_processing_pipeline_input_options
{
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
  bool save_progress_video();
  bool save_cloud3d_ply();

protected:
  c_vlo_pipeline_input_options input_options_;
  c_vlo_pipeline_output_options output_options_;

  struct {
    VLO_VERSION version;
    c_vlo_scan1 scan1;
    c_vlo_scan3 scan3;
    c_vlo_scan5 scan5;
  } current_scan_;


  c_output_frame_writer progress_writer_;

};

#endif /* __c_vlo_pipeline_h__ */
