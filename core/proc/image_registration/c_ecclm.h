/*
 * c_ecclm.h
 *
 *  Created on: Jun 20, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_ecclm_h__
#define __c_ecclm_h__

#include <opencv2/opencv.hpp>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif


#include "c_image_transform.h"

class c_ecclm
{
public:
  typedef c_ecclm this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclm();
  c_ecclm(c_image_transform * transform = nullptr);
  virtual ~c_ecclm() = default;

  void set_image_transform(c_image_transform * image_transform);
  c_image_transform * image_transform() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  void set_epsx(double v);
  double epsx() const;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

  void release_current_image();

  bool align();

  bool align(cv::InputArray current_image, cv::InputArray current_mask,
      cv::InputArray reference_image, cv::InputArray reference_mask);

  bool align(cv::InputArray current_image, cv::InputArray current_mask);

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat1f & current_image() const;
  const cv::Mat1b & current_mask() const;

protected:
  double compute_rhs(const cv::Mat1f & params);
  double compute_jac(const cv::Mat1f & params, cv::Mat1f & H, cv::Mat1f & v);

protected:
  c_image_transform * image_transform_ = nullptr;
  cv::Mat1f reference_image_;
  cv::Mat1b reference_mask_;
  cv::Mat1f current_image_;
  cv::Mat1b current_mask_;
  cv::Mat1f gx_, gy_;
  std::vector<cv::Mat1f> J;
  cv::Mat1f JJ;

  double update_step_scale_ = 2;
  double epsx_ = 1e-5;
  int max_iterations_ = 50;
};

class c_ecclmp
{
public:
  typedef c_ecclmp this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclmp();
  c_ecclmp(c_image_transform * image_transform = nullptr);
  virtual ~c_ecclmp() = default;

  void set_image_transform(c_image_transform * image_transform);
  c_image_transform * image_transform() const;

  void set_maxlevel(int v);
  int maxlevel() const;

  void set_epsx(double v);
  double epsx() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool set_current_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool align();

  bool align(cv::InputArray current_image, cv::InputArray current_mask);

  bool align(cv::InputArray current_image, cv::InputArray current_mask,
      cv::InputArray reference_image, cv::InputArray reference_mask);

protected:
  std::vector<c_ecclm::uptr> pyramid_;
  c_image_transform  * image_transform_ = nullptr;
  double epsx_ = 1e-5;
  int max_iterations_ = 50;
  int maxlevel_ = 7;
};

bool ecclm_convert_input_image(cv::InputArray src, cv::InputArray src_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask);

#endif /* __c_ecclm_h__ */
