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
#include <core/proc/vlo/vlo.h>
#include <core/proc/threshold.h>


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
   bool enable_gather_lookup_table_statistics = false;
   std::string vlo_lookup_table_statistics_filename;
   VLO_INTENSITY_CHANNEL vlo_intensity_channel = VLO_INTENSITY_PEAK;

   bool enable_blom_detection2 = false;
   c_vlo_depth_segmentation_options depth_segmentation_;

   bool enable_double_echo_statistics = false;
   bool enable_bloom_slopes_statistics = false;

   double saturation_level = 110;
};

struct c_vlo_pipeline_output_options :
    c_image_processing_pipeline_output_options
{
  std::string cloud3d_filename;

  bool save_cloud3d_ply = false;
  c_vlo_reader::DATA_CHANNEL cloud3d_intensity_channel =
      c_vlo_reader::DATA_CHANNEL_PEAK;

  bool save_progress_video = false;
  c_output_frame_writer_options progress_writer_options;

  bool save_bloom2_display = false;
  c_output_frame_writer_options display_writer_options;

  bool save_bloom2_segments = false;
  c_output_frame_writer_options segments_writer_options;

  bool save_bloom2_intensity_profiles = false;
  c_output_frame_writer_options intensity_writer_options;

  bool save_walls = false;
  c_output_frame_writer_options walls_writer_options;

  bool save_segment_statistics = false;

  bool save_blured_intensities = false;
  double blured_intensities_sigma = 3;
  int blured_intensities_kradius = 9;
  c_output_frame_writer_options blured_intensities_writer_options;
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
  bool copyParameters(const base::sptr & dst) const override;

protected:
  bool initialize_pipeline() override;
  void cleanup_pipeline() override;
  bool run_pipeline() override;
  bool process_current_frame();
  bool run_blom_detection2();
  bool update_double_echo_statistics();
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

  c_vlo_lookup_table_statistics vlo_lookup_table_statistics_;

  c_output_frame_writer progress_writer_;

  c_output_frame_writer blom2_display_writer_;
  c_output_frame_writer bloom2_segments_writer_;
  c_output_frame_writer bloom2_intensity_writer_;
  c_output_frame_writer bloom2_blured_intensities_writer_;
  c_output_frame_writer bloom2_walls_writer_;

  c_output_text_writer doubled_echo_stats_writer_;
  c_output_text_writer bloom_profile_slopes_writer_;

};

#endif /* __c_vlo_pipeline_h__ */
