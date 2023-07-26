/*
 * c_stereo_matcher_pipeline.h
 *
 *  Created on: Jun 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stereo_matcher_pipeline_h__
#define __c_stereo_matcher_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/stereo/c_stereo_input_options.h>
#include <core/proc/stereo/c_regular_stereo_matcher.h>
#include <core/proc/camera_calibration/stereo_calibrate.h>
//#include <core/pipeline/c_regular_stereo_pipeline/c_regular_stereo.h>
//#include <core/pipeline/stereo/c_stereo_rectification_options.h>
#include <core/io/c_stereo_input.h>

struct c_stereo_matcher_input_options:
    c_stereo_input_options
{
};

struct c_stereo_matcher_stereo_rectification_options
{
  std::string camera_intrinsics_yml;
  std::string camera_extrinsics_yml;
  bool enabled = false;
};

struct c_stereo_matcher_processing_options
{
  double camera_focus = 7.215377e+02; // from KITTI
  double stereo_baseline = 0.54;// [m], from KITTI
};

struct c_stereo_matcher_image_processing_options
{
  c_image_processor::sptr input_image_processor;
  c_image_processor::sptr remapped_image_processor;
  c_image_processor::sptr output_image_processor;
};

struct c_stereo_matcher_output_options :
    c_image_processing_pipeline_output_options
{
  std::string progress_video_filename;
  std::string depthmap_filename;
  std::string cloud3d_image_filename;
  std::string cloud3d_ply_filename;

  bool save_progress_video = false;
  bool save_depthmaps = true;
  bool save_cloud3d_image = true;
  bool save_cloud3d_ply = true;
};


class c_stereo_matcher_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_stereo_matcher_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  enum DISPLAY_TYPE {
    DISPLAY_DISPARITY,
    DISPLAY_QUAD,
  };


  c_stereo_matcher_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  ~c_stereo_matcher_pipeline();

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "stereo_matcher";

    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_stereo_matcher_pipeline.</strong><br>"
        "Test a variety of stereo matchers<br>";
    return tooltip_;
  }

  c_stereo_matcher_input_options & input_options();
  const c_stereo_matcher_input_options & input_options() const;

  c_stereo_matcher_stereo_rectification_options & stereo_rectification_options();
  const c_stereo_matcher_stereo_rectification_options & stereo_rectification_options() const;

  c_stereo_matcher_processing_options & processing_options();
  const c_stereo_matcher_processing_options & processing_options() const ;

  c_stereo_matcher_image_processing_options & image_processing_options();
  const c_stereo_matcher_image_processing_options & image_processing_options() const;

  c_regular_stereo_matcher& stereo_matcher();
  const c_regular_stereo_matcher& stereo_matcher() const;

  c_stereo_matcher_output_options & output_options();
  const c_stereo_matcher_output_options & output_options() const;

  const c_enum_member * get_display_types() const override;
  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override;
  bool serialize(c_config_setting settings, bool save) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();


protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool process_current_frames();

protected:
  bool open_input_source();
  void close_input_source();
  bool seek_input_source(int pos);
  bool read_input_source();
  bool update_stereo_rectification_remap();

protected:
  c_stereo_input_source input_;
  c_stereo_matcher_input_options input_options_;
  c_stereo_matcher_stereo_rectification_options stereo_rectification_options_;
  c_stereo_matcher_processing_options processing_options_;
  c_stereo_matcher_image_processing_options image_processing_options_;
  c_stereo_matcher_output_options output_options_;

  c_stereo_camera_intrinsics stereo_intrinsics_;
  c_stereo_camera_extrinsics stereo_extrinsics_;
  c_stereo_camera_intrinsics new_intrinsics_;
  c_stereo_camera_extrinsics new_extrinsics_;
  c_regular_stereo_matcher stereo_matcher_;

  cv::Mat2f rmaps_[2];
  cv::Mat current_frames_[2];
  cv::Mat current_masks_[2];
  cv::Mat current_disparity_;
  cv::Matx33d R_[2];
  cv::Matx34d P_[2];
  cv::Matx44d Q_;
  cv::Rect validRoi_[2];
};

#endif /* __c_stereo_matcher_pipeline_h__ */
