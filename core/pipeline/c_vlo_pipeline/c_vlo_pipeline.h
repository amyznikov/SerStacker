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


enum VLO_INTENSITY_CHANNEL
{
  VLO_INTENSITY_PEAK,
  VLO_INTENSITY_AREA,
};

struct c_vlo_lookup_table_item
{
  double distance = 0;
  double peak = 0;
  double area = 0;
  double peak2 = 0;
  double area2 = 0;
  double num_measurements = 0;
};

typedef std::vector<c_vlo_lookup_table_item>
  c_vlo_lookup_table_statistics;


struct c_vlo_pipeline_input_options :
    c_image_processing_pipeline_input_options
{
  bool sort_echos_by_distance = false;
};


struct c_vlo_pipeline_processing_options
{
   bool enable_reflectors_detection = false;
   bool enable_blom_detection = false;
   bool enable_double_echo_detection = true;
   bool enable_auto_threshold = true;


   THRESHOLD_TYPE auto_threshold_type = THRESHOLD_TYPE_YEN;
   double auto_threshold_value = 0;
   double auto_clip_min = 0.1; // percentage of 'black' pixels
   double auto_clip_max = 99.9; // 100-percentage of 'white' pixels

   bool enable_gather_lookup_table_statistics = false;
   std::string vlo_lookup_table_statistics_filename;
   VLO_INTENSITY_CHANNEL vlo_intensity_channel = VLO_INTENSITY_PEAK;


   double high_intensity_threshold = 118;
   double blom_slope_min = 0.25;
   double blom_slope_max = 1.50;
   double walk_error = 150;
   double double_echo_distance = 2500;

};

struct c_vlo_pipeline_output_options :
    c_image_processing_pipeline_output_options
{
  bool save_progress_video = false;

  std::string progress_video_filename;
  std::string cloud3d_filename;

  bool save_cloud3d_ply = false;
  c_vlo_reader::DATA_CHANNEL cloud3d_intensity_channel =
      c_vlo_reader::DATA_CHANNEL_ECHO_PEAK;
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
  bool run_blom_detection();
  bool update_vlo_lookup_table_statistics();
  bool save_progress_video();
  bool save_cloud3d_ply();


protected:
  c_vlo_pipeline_input_options input_options_;
  c_vlo_pipeline_processing_options processing_options_;
  c_vlo_pipeline_output_options output_options_;
  c_vlo_scan current_scan_;
  cv::Mat1b current_reflection_mask_;
  cv::Mat1b current_reflection2_mask_;

  c_output_frame_writer progress_writer_;
  c_output_frame_writer reflectors_writer_;
  c_output_frame_writer blom_reflectors_mask_writer_;
  c_output_frame_writer blom_reflectors_writer_;
  c_output_frame_writer blom_distances_writer_;
  c_output_frame_writer blom_slopes_writer_;
  c_output_frame_writer blom_intensity_writer_;
  c_output_frame_writer blom_display_writer_;
  c_output_frame_writer blom_mask_writer_;


  c_vlo_lookup_table_statistics vlo_lookup_table_statistics_;

};

#endif /* __c_vlo_pipeline_h__ */