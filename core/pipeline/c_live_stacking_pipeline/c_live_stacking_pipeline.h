/*
 * c_live_stacking_pipeline.h
 *
 *  Created on: Jul 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_live_stacking_pipeline_h__
#define __c_live_stacking_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/improc/c_image_processor.h>
#include <core/average/c_frame_accumulation.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/settings/opencv_settings.h>

struct c_live_stacking_input_options:
    c_image_processing_pipeline_input_options
{
};


struct c_live_stacking_registration_options
{
  bool enabled = false;
  int minimum_image_size = 32;
  double min_rho = 0.7;
};



enum live_stacking_accumulation_type
{
  live_stacking_accumulation_disable,
  live_stacking_accumulation_average,
  // live_stacking_accumulation_bayer_average,
};

struct c_live_stacking_accumulation_options
{
  live_stacking_accumulation_type accumulation_type =
      live_stacking_accumulation_average;

  bool ignore_input_mask = true;
};



struct c_live_stacking_output_options:
    c_image_processing_pipeline_output_options
{
  double display_scale = -1;

  bool save_accumulated_file = true;
  std::string output_accumuated_file_name;

  bool save_accumulated_video = false;
  c_output_frame_writer_options output_accumulated_video_options;

};

class c_live_stacking_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_live_stacking_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_live_stacking_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string& get_class_name() const override
  {
    return class_name();
  }

  static const std::string& class_name()
  {
    static const std::string classname_ =
        "live_stacking";
    return classname_;
  }

  static const std::string& tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_live_stacking_pipeline.</strong><br>"
            "This pipeline uses specified c_image_processor for generic image processing<br>";
    return tooltip_;
  }

  const c_live_stacking_input_options & input_options() const;
  c_live_stacking_input_options & input_options();

  const c_live_stacking_registration_options & registration_options() const;
  c_live_stacking_registration_options & registration_options() ;

  const c_live_stacking_accumulation_options & accumulation_options() const;
  c_live_stacking_accumulation_options & accumulation_options();

  const c_live_stacking_output_options & output_options() const;
  c_live_stacking_output_options & output_options();

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame();

protected:
  static c_image_transform::sptr create_image_transfrom(const c_live_stacking_registration_options & opts);

  static c_frame_accumulation::ptr create_frame_accumulation(const cv::Size & image_size, int cn,
      live_stacking_accumulation_type type);

  bool accumulate_image(const cv::Mat & current_image, const cv::Mat & current_mask);

protected:
  c_live_stacking_input_options input_options_;
  c_live_stacking_registration_options registration_options_;
  c_live_stacking_accumulation_options accumulation_options_;
  c_live_stacking_output_options output_options_;

  c_ecch ecch_;
  //c_ecc_forward_additive ecc_;
  // c_eccflow eccflow_;
  //c_ecc_motion_model::sptr ecc_motion_model_;
  c_image_transform::sptr image_transform_;

  c_frame_accumulation::ptr frame_accumulation_;

  cv::Mat reference_image_;
  cv::Mat reference_mask_;
  cv::Mat current_image_;
  cv::Mat current_mask_;
  cv::Mat aligned_image_;
  cv::Mat aligned_mask_;
  double input_display_scale_ = -1;


  c_output_frame_writer accumulated_video_writer_;



};


#endif /* __c_live_stacking_pipeline_h__ */
