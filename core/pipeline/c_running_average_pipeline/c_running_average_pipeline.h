/*
 * c_running_average_pipeline.h
 *
 *  Created on: Feb 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_running_average_pipeline_h__
#define __c_running_average_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/improc/c_image_processor.h>
#include <core/average/c_frame_accumulation.h>
#include <core/proc/image_registration/c_frame_registration.h>
#include <core/settings/opencv_settings.h>

struct c_running_average_input_options:
    c_image_processing_pipeline_input_options
{
      c_image_processor::sptr ecc_image_processor;
};

struct c_running_average_registration_options
{
  bool double_align_moode = false;
  double reference_unsharp_sigma_ = 1;
  double reference_unsharp_alpha_ = 0.95;
  //ECC_ALIGN_METHOD ecc_method = ECC_ALIGN_LM;

  bool enable_ecc = false;
  bool enable_eccflow = false;

  IMAGE_MOTION_TYPE ecc_motion_type =
      IMAGE_MOTION_TRANSLATION;

  double min_rho = 0.7;
};

struct c_running_average_update_options
{
  double reference_weight = 15;
  double running_weight = 2;

  c_lpg_options lpg;

  c_running_average_update_options()
  {
    lpg.dscale = 2;
    lpg.uscale = 7;
    lpg.avgchannel = true;
  }

};

struct c_running_average_output_options:
    c_image_processing_pipeline_output_options
{
  double display_scale = -1;

  bool save_accumulated_video = false;
  bool save_reference_video = false;
  c_output_frame_writer_options output_accumulated_video_options;
  c_output_frame_writer_options output_reference_video_options;

};

class c_running_average_pipeline :
  public c_image_processing_pipeline
{
public:
  typedef c_running_average_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_running_average_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string& get_class_name() const override
  {
    return class_name();
  }

  static const std::string& class_name()
  {
    static const std::string classname_ =
        "running_average";
    return classname_;
  }

  static const std::string& tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_running_average_pipeline.</strong><br>"
            "test for running average registered frames<br>";
    return tooltip_;
  }

  ///

  void set_double_align_moode(bool v);
  bool double_align_moode() const;

  ///

  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();
  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  bool copyParameters(const base::sptr & dst) const override;

  //////
  bool ecc_ctls_enabled() const;
  bool eccflow_ctls_enabled() const;
  /////

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame1();
  bool process_current_frame2();
  void compute_weights(const cv::Mat & src, const cv::Mat & srcmask,  cv::Mat & dst) const;
  bool average_add(c_running_frame_average & average, const cv::Mat & src, const cv::Mat & srcmask,
      double avgw, const cv::Mat2f * rmap = nullptr);

protected:
  c_running_average_input_options input_options_;
  c_running_average_registration_options registration_options_;
  c_running_average_update_options average_options_;
  c_running_average_output_options output_options_;

  c_ecch ecch_;
  c_image_transform::sptr ecc_tramsform_;
  c_eccflow eccflow_;

  c_running_frame_average average1_;
  c_running_frame_average average2_;

  cv::Mat current_image_;
  cv::Mat current_mask_;

  c_output_frame_writer accumulated_video_writer_;
  c_output_frame_writer reference_video_writer_;

};

#endif /* __c_running_average_pipeline_h__ */
