/*
 * c_frame_registration.h
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#ifndef __c_frame_registration_h__
#define __c_frame_registration_h__

#include <core/proc/extract_channel.h>
#include <core/feature2d/feature2d.h>
#include "c_jovian_derotation.h"
#include "image_transform.h"
#include "ecc_motion_model.h"


struct c_feature_based_registration_options {
  bool enabled = false;
  double scale = 0.5;
  c_sparse_feature_extractor_options sparse_feature_extractor;
  c_feature2d_matcher_options sparse_feature_matcher;
};

struct c_ecc_registration_options {
  bool enabled = true;
  double scale = 0.5;
  double eps = 0.2;
  double min_rho = 0.8;
  double input_smooth_sigma = 1.0;
  double reference_smooth_sigma = 1.0;
  double update_step_scale = 1.5;
  double normalization_noise = 0.01;
  int normalization_scale = 0;
  int max_iterations = 15;
  int ecch_minimum_image_size = 16;
  bool enable_ecch = true;
  bool ecch_estimate_translation_first = true;
  bool replace_planetary_disk_with_mask = false;
  double planetary_disk_mask_stdev_factor = 0.5;
};

struct c_eccflow_registration_options {
  bool enabled = false;
  double update_multiplier = 1.5;
  double input_smooth_sigma = 0;
  double reference_smooth_sigma = 0;
  int max_iterations = 1;
  int support_scale = 4;
  int normalization_scale = -1;
};

struct c_jovian_derotation_options {
  bool enabled = false;
  c_jovian_ellipse_detector_options ellipse;
  double min_rotation = -40 * CV_PI / 180;
  double max_rotation = +40 * CV_PI / 180;
  int max_pyramid_level = -1;
  int num_orientations = 1;
  int eccflow_support_scale = 4;
  int eccflow_normalization_scale = 0;
  int eccflow_max_pyramid_level = 0;
  //bool align_planetary_disk_masks = true;
  bool derotate_all_frames = false;
  int  derotate_all_frames_max_context_size = 3;
  bool rotate_jovian_disk_horizontally = false;
};

enum master_frame_selection_method {
  master_frame_specific_index,
  master_frame_middle_index,
  master_frame_best_of_100_in_middle,
};


struct c_master_frame_options
{
  master_frame_selection_method master_selection_method =
      master_frame_specific_index;

  std::string master_fiename;
  int master_frame_index = 0;

  bool apply_input_frame_processors = true;
  bool generate_master_frame = true;
  int max_frames_to_generate_master_frame = 3000;
  double master_sharpen_factor = 0.5;
  double accumulated_sharpen_factor = 1;
  bool save_master_frame = true;
  double feature_scale = 0.5;
  double ecc_scale = 0.5;
  int eccflow_scale = 0;
};

struct c_image_registration_options
{
  bool enable_frame_registration = true;

  IMAGE_MOTION_TYPE motion_type = IMAGE_MOTION_AFFINE;
  bool accumulate_and_compensate_turbulent_flow = false;

  color_channel_type registration_channel = color_channel_gray;
  enum ECC_INTERPOLATION_METHOD interpolation = ECC_INTER_LINEAR;
  enum ECC_BORDER_MODE border_mode = ECC_BORDER_REFLECT101;
  cv::Scalar border_value = cv::Scalar(0, 0, 0);

  c_master_frame_options master_frame_options;
  struct c_feature_based_registration_options feature_registration;
  struct c_ecc_registration_options ecc;
  struct c_eccflow_registration_options eccflow;
  struct c_jovian_derotation_options jovian_derotation;
};

struct c_image_registration_status
{
  struct {
    double extract_feature_image = 0;
    double estimate_feature_transform = 0;
    double extract_ecc_image = 0;
    double ecc_align = 0;
    double create_remap = 0;
    double extract_smflow_image = 0;
    double smflow_align = 0;
    double remap = 0;
  } timings;

  struct {
    double rho = 0;
    double min_rho = 0;
    double eps = 0;
    double current_eps = 0;
    int num_iterations = 0;
    int max_iterations = 0;
    bool enabled = false;
    bool ok = false;
  } ecc;
};

class c_frame_registration
{
public:
  typedef c_frame_registration this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::function<void(cv::InputOutputArray image, cv::InputOutputArray mask)> ecc_image_preprocessor_function;

  c_frame_registration();
  c_frame_registration(const c_image_registration_options & optsions);

  void set_debug_path(const std::string & v);
  const std::string & debug_path() const;

  c_image_registration_options & options();
  const c_image_registration_options & options() const;

  void set_image_transform(const c_image_transform::sptr & transform);
  const c_image_transform::sptr & image_transform() const;

  virtual c_sparse_feature_extractor::ptr create_keypoints_extractor() const;
  const c_sparse_feature_extractor::ptr & set_keypoints_extractor(const c_sparse_feature_extractor::ptr & extractor);
  const c_sparse_feature_extractor::ptr & keypoints_extractor() const;

  const c_ecc_forward_additive & ecc() const;
  const c_ecch & ecch() const;
  const c_eccflow & eccflow() const;
  const c_jovian_derotation & jovian_derotation() const;
  c_jovian_derotation & jovian_derotation();

  void set_ecc_image_preprocessor(const ecc_image_preprocessor_function & func);
  const ecc_image_preprocessor_function & ecc_image_preprocessor() const;

  const c_image_registration_status & status() const;

public: // ops
  virtual ~c_frame_registration() = default;

  virtual bool setup_reference_frame(cv::InputArray image,
      cv::InputArray msk = cv::noArray());

  virtual bool create_image_transfrom();

  virtual bool register_frame(cv::InputArray src, cv::InputArray srcmask,
      cv::OutputArray dst = cv::noArray(), cv::OutputArray dstmask = cv::noArray());

  virtual bool remap(cv::InputArray src, cv::OutputArray dst,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      enum ECC_INTERPOLATION_METHOD interpolation_method = ECC_INTER_UNKNOWN,
      enum ECC_BORDER_MODE border_mode = ECC_BORDER_UNKNOWN,
      const cv::Scalar & border_value = cv::Scalar()) const;

  virtual bool custom_remap(const cv::Mat2f & rmap,
      cv::InputArray src, cv::OutputArray dst,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      enum ECC_INTERPOLATION_METHOD interpolation_method = ECC_INTER_UNKNOWN,
      enum ECC_BORDER_MODE border_mode = ECC_BORDER_UNKNOWN,
      const cv::Scalar & border_value = cv::Scalar()) const;

public: // artifacts
  const cv::Mat & reference_feature_image() const;
  const cv::Mat & reference_feature_mask() const;
  const cv::Mat & reference_ecc_image() const;
  const cv::Mat & reference_ecc_mask() const;

  const cv::Mat & current_feature_image() const;
  const cv::Mat & current_feature_mask() const;
  const cv::Mat & current_ecc_image() const;
  const cv::Mat & current_ecc_mask() const;

  //const cv::Mat1f & current_transform() const;
  const cv::Mat2f & current_remap() const;

protected:
  virtual bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const;

  virtual bool create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const;

  virtual bool create_reference_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const;

  virtual bool create_current_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const;

  bool insert_planetary_disk_shape(const cv::Mat & src_ecc_image,
      const cv::Mat & src_mask,
      cv::Mat & dst_ecc_image,
      cv::Mat & dst_ecc_mask) const;

  virtual bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask);

  virtual bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      c_image_transform * current_transform);

  virtual bool detect_and_match_keypoints(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      std::vector<cv::Point2f> & output_matched_current_positions,
      std::vector<cv::Point2f> & output_matched_reference_positions,
      std::vector<cv::KeyPoint> * _current_keypoints,
      cv::Mat * _current_descriptors,
      std::vector<cv::DMatch> * _current_matches,
      std::vector<std::vector<cv::DMatch> > * _current_matches12) const;

  virtual bool base_remap(const cv::Mat2f & rmap,
      cv::InputArray src, cv::OutputArray dst,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      enum ECC_INTERPOLATION_METHOD interpolation_method = ECC_INTER_UNKNOWN,
      enum ECC_BORDER_MODE border_mode = ECC_BORDER_UNKNOWN,
      const cv::Scalar & border_value = cv::Scalar()) const;

protected:
  c_image_registration_options options_;

  cv::Size reference_frame_size_;
  cv::Size current_frame_size_;

  cv::Mat reference_feature_image_;
  cv::Mat current_feature_image_;

  cv::Mat reference_feature_mask_;
  cv::Mat current_feature_mask_;

  c_sparse_feature_extractor::ptr keypoints_extractor_;
  c_feature2d_matcher::ptr keypoints_matcher_;
  std::vector<cv::KeyPoint> reference_keypoints_;
  cv::Mat reference_descriptors_;
  std::vector<cv::KeyPoint> current_keypoints_;
  cv::Mat current_descriptors_;

  cv::Point2f planetary_disk_reference_centroid_;
  cv::Point2f planetary_disk_current_centroid_;
  cv::Rect planetary_disk_reference_component_rect_;
  cv::Rect planetary_disk_current_component_rect_;
  cv::Mat planetary_disk_reference_component_mask_;
  cv::Mat planetary_disk_current_component_mask_;

  std::vector<cv::DMatch> current_matches_;
  std::vector<std::vector<cv::DMatch> > current_matches12_;
  std::vector<cv::Point2f> matched_current_positions_;
  std::vector<cv::Point2f> matched_reference_positions_;

  c_image_transform::sptr image_transform_;
  cv::Mat1f image_transform_defaut_parameters_;

  c_ecch ecch_;
  c_ecc_forward_additive ecc_;
  c_eccflow eccflow_;
  c_ecc_motion_model::sptr ecc_motion_model_;

  c_jovian_derotation jovian_derotation_;
  ecc_image_preprocessor_function ecc_image_preprocessor_;

  cv::Mat2f current_remap_;

  c_image_registration_status current_status_;

  std::string debug_path_;
  //bool enable_debug_ = false;
};

#endif /* __c_frame_registration_h__ */
