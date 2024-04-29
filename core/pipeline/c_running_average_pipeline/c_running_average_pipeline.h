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
};

struct c_running_average_registration_options
{
  bool double_align_moode = false;
  double reference_unsharp_sigma_ = 1;
  double reference_unsharp_alpha_ = 0.95;

  bool enable_ecc = false;
  bool enable_eccflow = false;

  IMAGE_MOTION_TYPE ecc_motion_type =
      IMAGE_MOTION_TRANSLATION;

  double min_rho = 0.7;
};

struct c_running_average_update_options
{
  double reference_running_weight = 10;
  double running_weight = 15;
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

  void set_ecch_minimum_image_size(int v);
  int ecch_minimum_image_size() const;

  void set_ecch_minimum_pyramid_level(int v);
  int ecch_minimum_pyramid_level() const;

  ///

  void set_ecc_max_iterations(int v);
  int ecc_max_iterations() const;

  void set_ecc_max_eps(double v);
  double ecc_max_eps() const;

  void set_ecc_min_rho(double v);
  double ecc_min_rho() const;

  void set_ecc_interpolation(enum ECC_INTERPOLATION_METHOD v);
  enum ECC_INTERPOLATION_METHOD ecc_interpolation() const;

  void set_ecc_input_smooth_sigma(double v);
  double ecc_input_smooth_sigma() const;

  void set_ecc_reference_smooth_sigma(double v);
  double ecc_reference_smooth_sigma() const;

  void set_ecc_update_step_scale(double v);
  double ecc_update_step_scale() const;

  ///

  void set_eccflow_support_scale(int v);
  int eccflow_support_scale() const;

  void set_eccflow_min_image_size(int v);
  int eccflow_min_image_size() const;

  void set_eccflow_normalization_scale(int v);
  int eccflow_normalization_scale() const;

  void set_eccflow_noise_level(double v);
  double eccflow_noise_level() const;

  void set_eccflow_input_smooth_sigma(double v);
  double eccflow_input_smooth_sigma() const;

  void set_eccflow_reference_smooth_sigma(double v);
  double eccflow_reference_smooth_sigma() const;

  void set_eccflow_min_rho(double v);
  double eccflow_min_rho() const;

  void set_eccflow_update_multiplier(double v);
  double eccflow_update_multiplier() const;

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
  bool start_pipeline();
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frame1();
  bool process_current_frame2();

protected:
  c_running_average_input_options input_options_;
  c_running_average_registration_options registration_options_;
  c_running_average_update_options average_options_;
  c_running_average_output_options output_options_;

  c_ecch ecch_;
  c_ecc_forward_additive ecc_;
  c_ecc_motion_model::sptr ecc_model_;
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
