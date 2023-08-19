/*
 * c_virtual_stereo_pipeline.h
 *
 *  Created on: Mar 9, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_virtual_stereo_pipeline_h__
#define __c_virtual_stereo_pipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/improc/c_image_processor.h>
#include <core/settings/opencv_settings.h>
#include <core/settings/camera_settings.h>
#include <core/proc/camera_calibration/camera_pose.h>
#include <core/io/c_output_frame_writer.h>

struct c_virtual_stereo_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_virtual_stereo_camera_options
{
  c_camera_intrinsics camera_intrinsics = {
      // fallback to kitti
      .image_size = cv::Size(1242, 375),
      .camera_matrix =
          cv::Matx33d(
              7.215377e+02, 0.000000e+00, 6.095593e+02,
              0.000000e+00, 7.215377e+02, 1.728540e+02,
              0.000000e+00, 0.000000e+00, 1.000000e+00)
  };
};


struct c_virtual_stereo_feature2d_options
{
  c_sparse_feature_detector_options detector;
  c_sparse_feature_descriptor_options descriptor;
  c_feature2d_matcher_options matcher;
};


struct c_virtual_stereo_polar_warp_options
{
  double maxRadius = 100;
  int interpolation_flags = cv::INTER_LINEAR;
  int polar_flags =  cv::WARP_POLAR_LINEAR;
  bool enabled = false;
};

struct c_virtual_stereo_output_options :
    c_image_processing_pipeline_output_options
{
    std::string progress_video_filename;
//  std::string depthmap_filename;
//  std::string cloud3d_image_filename;
//  std::string cloud3d_ply_filename;
//
    bool save_progress_video = false;
//  bool save_depthmaps = true;
//  bool save_cloud3d_image = true;
//  bool save_cloud3d_ply = true;
};


struct c_virtual_stereo_image_processing_options
{
  c_image_processor::sptr input_processor;
  c_image_processor::sptr feature2d_preprocessor;
};


class c_virtual_stereo_pipeline :
    public c_image_processing_pipeline
{
public:
  typedef c_virtual_stereo_pipeline this_class;
  typedef c_image_processing_pipeline base;
  typedef std::shared_ptr<this_class> sptr;

  c_virtual_stereo_pipeline(const std::string & name,
      const c_input_sequence::sptr & input_sequence);

  const std::string & get_class_name() const override
  {
    return class_name();
  }

  static const std::string & class_name()
  {
    static const std::string classname_ =
        "virtual_stereo";
    return classname_;
  }

  static const std::string & tooltip()
  {
    static const std::string tooltip_ =
        "<strong>c_virtual_stereo_pipeline.</strong><br>"
        "The pipeline for virtual stereo experimentation<br>";
    return tooltip_;
  }


  c_virtual_stereo_input_options & input_options();
  const c_virtual_stereo_input_options & input_options() const;

  c_virtual_stereo_camera_options & camera_options();
  const c_virtual_stereo_camera_options & camera_options() const;

  c_virtual_stereo_image_processing_options & image_processing_options();
  const c_virtual_stereo_image_processing_options & image_processing_options() const ;

  c_virtual_stereo_feature2d_options & feature2d_options();
  const c_virtual_stereo_feature2d_options & feature2d_options() const;

  c_lm_camera_pose_options & camera_pose_options();
  const c_lm_camera_pose_options & camera_pose_options() const;

  c_virtual_stereo_polar_warp_options & polar_warp_options();
  const c_virtual_stereo_polar_warp_options & polar_warp_options() const;

  c_virtual_stereo_output_options & output_options();
  const c_virtual_stereo_output_options & output_options() const;

  bool copyParameters(const base::sptr & dst) const override;

  bool serialize(c_config_setting settings, bool save) override;
  bool get_display_image(cv::OutputArray display_frame, cv::OutputArray display_mask) override;
  static const std::vector<c_image_processing_pipeline_ctrl> & get_controls();

protected:
  bool initialize_pipeline() override;
  bool run_pipeline() override;
  void cleanup_pipeline() override;
  bool open_input_sequence();
  void close_input_sequence();
  bool seek_input_sequence(int pos);
  bool read_input_frame(cv::Mat & output_image, cv::Mat & output_mask);
  bool write_progress_video();
  c_sparse_feature_extractor::ptr create_keypoints_extractor() const;
  bool process_current_frame();
  bool estmate_camera_pose();

protected:
  c_virtual_stereo_input_options input_options_;
  c_virtual_stereo_camera_options camera_options_;
  c_virtual_stereo_image_processing_options image_processing_options_;
  c_virtual_stereo_feature2d_options feature2d_options_;
  c_lm_camera_pose_options camera_pose_options_;
  c_virtual_stereo_polar_warp_options polar_warp_options_;
  c_virtual_stereo_output_options output_options_;

  c_sparse_feature_extractor::ptr keypoints_extractor_;
  c_feature2d_matcher::ptr keypoints_matcher_;

  cv::Mat current_image_;
  cv::Mat previous_image_;

  cv::Mat current_mask_;
  cv::Mat previous_mask_;

  std::vector<cv::KeyPoint> current_keypoints_;
  std::vector<cv::KeyPoint> previous_keypoints_;

  cv::Mat current_descriptors_;
  cv::Mat previous_descriptors_;

  std::vector<cv::Point2f> matched_current_positions_;
  std::vector<cv::Point2f> matched_previous_positions_;

  //  cv::Mat1b current_inliers_;    // current inliers mask
  //  cv::Mat1b previous_inliers_;    // previous inliers mask

  cv::Vec3d currentEulerAnges_;
  cv::Vec3d currentTranslationVector_;
  cv::Matx33d currentRotationMatrix_;
  cv::Matx33d currentEssentialMatrix_;
  cv::Matx33d currentFundamentalMatrix_;
  cv::Matx33d currentDerotationHomography_;
  cv::Point2d currentEpipoles_[2];
  cv::Point2d currentEpipole_;
  cv::Mat1b currentInliers_;

  //cv::Matx33d camera_matrix_;

  c_output_frame_writer progress_video_writer_;

};


#endif /* __c_virtual_stereo_pipeline_h__ */
