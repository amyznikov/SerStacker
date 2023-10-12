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
#include <core/proc/stereo/c_epipolar_matcher.h>
#include <core/proc/image_registration/pyrflowlk2.h>
#include <core/proc/image_registration/morph_gradient_flow.h>

#include <core/io/c_output_frame_writer.h>

struct c_virtual_stereo_input_options :
    c_image_processing_pipeline_input_options
{
};

struct c_virtual_stereo_image_processing_options
{
  c_image_processor::sptr input_processor;
  c_image_processor::sptr feature2d_preprocessor;
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



struct c_virtual_stereo_feature2d_options :
    public c_sparse_feature_extractor_and_matcher_options
{
};

struct c_virtual_stereo_matcher_options
{
  bool enable_stereo_matcher = true;
};


struct c_virtual_stereo_epipolar_flow_options
{
  int support_scale = 3;
  int max_iterations = 1;
  int normalization_scale = -1;
  int max_pyramid_level = -1;
  double noise_level = -1;
  double update_multiplier = 1.5;
  double input_smooth_sigma = 0;
  double reference_smooth_sigma = 0;

  bool enabled = false;
  bool enable_debug = false;
};

struct c_virtual_stereo_pyrflowlk_options
{
  c_sparse_feature_detector_options detector;
  c_pyrflowlk2_options pyrflowlk;
  bool enabled = false;
  bool enable_debug = false;
  int max_pyramid_level = 3;
  int block_radius = 2;
  int search_radius = 7;
};

struct c_virtual_stereo_output_options :
    c_image_processing_pipeline_output_options
{
    std::string progress_video_filename;
    std::string polar_frames_filename;
    std::string disparity_frames_filename;
    std::string homography_video_filename;
    //std::string pyrflowlk_frames_filename;

    bool save_progress_video = false;
    bool save_polar_frames = false;
    bool save_disparity_frames = false;
    bool save_homography_video = false;
    //bool save_pyrflowlk_frames = false;

    bool save_epipolar_flow_debug_images = false;
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

  c_virtual_stereo_epipolar_flow_options & epipolar_flow_options();
  const c_virtual_stereo_epipolar_flow_options & epipolar_flow_options() const;

  c_virtual_stereo_pyrflowlk_options & pyrflowlk_options();
  const c_virtual_stereo_pyrflowlk_options & pyrflowlk_options() const;

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
  c_sparse_feature_extractor_and_matcher::sptr create_keypoints_extractor() const;
  bool process_current_frame();
  bool extract_keypoints();
  bool match_keypoints();
  bool estmate_camera_pose();
  bool create_stereo_frames(cv::Mat frames[2], cv::Mat masks[2], cv::Mat2f * inverse_remap);
  bool run_polar_stereo();
  bool run_epipolar_stereo();
  bool run_epipolar_flow();
  bool run_pyrflowlk();

  bool create_homography_display(cv::OutputArray display_frame, cv::OutputArray display_mask);
  bool write_homography_video();

  static void draw_matched_positions(cv::Mat & image,
      const std::vector<cv::Point2f> & current_positions,
      const std::vector<cv::Point2f> & previous_positions,
      const cv::Mat1b & inliers);


protected:
  c_virtual_stereo_input_options input_options_;
  c_virtual_stereo_camera_options camera_options_;
  c_virtual_stereo_image_processing_options image_processing_options_;
  c_virtual_stereo_feature2d_options feature2d_options_;
  c_lm_camera_pose_options camera_pose_options_;
  c_virtual_stereo_matcher_options stereo_matcher_options_;
  c_virtual_stereo_epipolar_flow_options epipolar_flow_options_;
  c_virtual_stereo_pyrflowlk_options pyrflowlk_options_;
  c_virtual_stereo_output_options output_options_;

  //c_sparse_feature_extractor::sptr keypoints_extractor_;
  //c_feature2d_matcher::sptr keypoints_matcher_;
  c_sparse_feature_extractor_and_matcher::sptr keypoints_extractor_;
  c_regular_stereo_matcher stereo_matcher_;
  c_epipolar_matcher epipolar_matcher_;
  c_epipolar_flow epipolar_flow_;
  c_feature2d::sptr pyrflowlk_keypoints_detector_;

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

//  c_epipolar_matcher::c_block_array current_block_array_;
//  c_epipolar_matcher::c_block_array previous_block_array_;

  cv::Vec3d currentEulerAnges_;
  cv::Vec3d currentTranslationVector_;
  cv::Matx33d currentRotationMatrix_;
  cv::Matx33d currentEssentialMatrix_;
  cv::Matx33d currentFundamentalMatrix_;
  cv::Matx33d currentDerotationHomography_;
  cv::Point2d currentEpipoles_[2];
  cv::Point2d currentEpipole_;
  cv::Mat1b currentInliers_;
  cv::Mat current_disparity_;

  //cv::Matx33d camera_matrix_;

  c_output_frame_writer progress_video_writer_;
  c_output_frame_writer polar_frames_writer_;
  c_output_frame_writer disparity_frames_writer_;
  c_output_frame_writer homography_video_writer_;
  //c_output_frame_writer median_hat_video_writer_;

};


#endif /* __c_virtual_stereo_pipeline_h__ */
