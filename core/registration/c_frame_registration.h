/*
 * c_frame_registration.h
 *
 *  Created on: Sep 28, 2020
 *      Author: amyznikov
 */

#ifndef __c_frame_registration_h__
#define __c_frame_registration_h__

#include <core/proc/extract_channel.h>
#include <core/proc/eccalign.h>

struct c_ecc_options {
  double scale = 1.;
  double eps = 0.2;
  double min_rho = 0.8;
  double input_smooth_sigma = 1.0;
  double reference_smooth_sigma = 1.0;
  double update_step_scale = 1.5;
  double normalization_noise = 0.01;
  int normalization_scale = 0;
  int max_iterations = 30;
};

struct c_eccflow_options {
  double update_multiplier = 1.5;
  int max_iterations = 1;
  int support_scale = 5;
};

struct c_frame_registration_base_options {

  ECC_MOTION_TYPE motion_type = ECC_MOTION_EUCLIDEAN;
  color_channel_type registration_channel = color_channel_gray;
  int interpolation_flags = cv::INTER_AREA;
  int remap_border_mode = cv::BORDER_REFLECT101;
  cv::Scalar remap_border_value = cv::Scalar(0, 0, 0);

  c_ecc_options ecc;
  c_eccflow_options eccflow;

  double feature_scale = 1.;

  bool enable_ecc = true;
  bool enable_eccflow = false;
};


struct c_frame_registration_status {
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
public: // opts

  typedef c_frame_registration this_class;
  typedef std::shared_ptr<this_class> ptr;

  void set_motion_type(enum ECC_MOTION_TYPE v);
  enum ECC_MOTION_TYPE motion_type() const;

  void set_registration_channel(color_channel_type channel);
  int registration_channel() const;

  void set_interpolation_flags(int v);
  int interpolation_flags() const;

  void set_remap_border_mode(int v);
  int remap_border_mode() const;

  void set_remap_border_value(const cv::Scalar & v);
  const cv::Scalar & remap_border_value() const;

  void set_enable_ecc(bool v);
  bool enable_ecc() const;

  void set_enable_eccflow(bool v);
  bool enable_eccflow() const;

  void set_feature_scale(double v);
  double feature_scale() const;

  void set_ecc_scale(double v);
  double ecc_scale() const;

  void set_ecc_normalization_scale(int v);
  int ecc_normalization_scale() const;

  void set_ecc_normalization_noise(double v);
  double ecc_normalization_noise() const;

  c_ecc_forward_additive & ecc();
  const c_ecc_forward_additive & ecc() const;

  c_ecch_flow & eccflow();
  const c_ecch_flow & eccflow() const;

  c_frame_registration_base_options & base_options();
  const c_frame_registration_base_options & base_options() const;

  const c_frame_registration_status & status() const;

public: // ops
  c_frame_registration();
  c_frame_registration(const c_frame_registration_base_options & opts);

  virtual ~c_frame_registration() = default;

  virtual bool setup_referece_frame(cv::InputArray image,
      cv::InputArray msk = cv::noArray());

  virtual bool register_frame(cv::InputArray src, cv::OutputArray dst,
      cv::InputArray srcmsk = cv::noArray(),
      cv::OutputArray dstmsk = cv::noArray());

  virtual bool remap(cv::InputArray src, cv::OutputArray dst,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      int interpolation_flags = -1,
      int border_mode = -1,
      const cv::Scalar & border_value = cv::Scalar()) const;

  virtual bool custom_remap(cv::InputArray src, cv::OutputArray dst,
      //      const cv::Rect & srcROI,
      const cv::Mat2f & rmap,
      cv::InputArray src_mask = cv::noArray(),
      cv::OutputArray dst_mask = cv::noArray(),
      int interpolation_flags = -1,
      int border_mode = -1,
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

//  const cv::Rect & reference_ROI() const;
//  const cv::Rect & current_ROI() const;

  const cv::Mat1f & current_transform() const;
  const cv::Mat2f & current_remap() const;

protected: // specs
  virtual bool create_feature_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk) const = 0;

  virtual bool create_ecc_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const;

  virtual bool create_eccflow_image(cv::InputArray src, cv::InputArray srcmsk,
      cv::OutputArray dst, cv::OutputArray dstmsk,
      double scale) const;

  virtual bool extract_reference_features(cv::InputArray reference_feature_image,
      cv::InputArray reference_feature_mask) = 0;

  virtual bool estimate_feature_transform(cv::InputArray current_feature_image,
      cv::InputArray current_feature_mask,
      cv::Mat1f * current_transform) = 0;



protected:
  c_frame_registration_base_options base_options_;

  //cv::Size current_input_frame_size_;
  //cv::Rect reference_ROI_;
  //cv::Rect current_ROI_;

  cv::Mat reference_feature_image_;
  cv::Mat reference_feature_mask_;

  cv::Mat current_feature_image_;
  cv::Mat current_feature_mask_;

  //c_ecc_inverse_compositional ecc_;
  c_ecc_forward_additive ecc_;
  c_ecch_flow eccflow_;

  cv::Mat1f current_transform_;
  cv::Mat2f current_remap_;

  c_frame_registration_status current_status_;
};

#endif /* __c_frame_registration_h__ */
