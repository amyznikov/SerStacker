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



// Base interface
class c_ecclm_motion_model
{
public:
  typedef c_ecclm_motion_model this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  virtual ~c_ecclm_motion_model() = default;

  virtual cv::Mat1d parameters() const = 0;
  virtual bool set_parameters(const cv::Mat1d & p) = 0;


  virtual bool remap(const cv::Mat1d & params, const cv::Size & size,
      cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask);

  virtual bool create_remap(const cv::Mat1d & params, const cv::Size & size, cv::Mat2f & map) = 0;
  virtual bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[/*nb parameters*/]) = 0;

//  virtual double compute_rhs(const cv::Mat1d & params,
//      const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
//      const cv::Mat1f & current_image, const cv::Mat1b & current_mask) = 0;
//
//  virtual double compute_jac(const cv::Mat1d & params,
//      const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
//      const cv::Mat1f & current_image, const cv::Mat1b & current_mask,
//      cv::Mat1d & A, cv::Mat1d & v) = 0;

protected:
//  static double compute_rhs(const cv::Mat1f & reference_image,
//      const cv::Mat1f & remapped_current_image,
//      const cv::Mat1b & mask,
//      cv::Mat1f & rhs);

//  virtual cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const = 0;
//  virtual bool create_remap(cv::Mat2f & map, const cv::Size & size) const = 0;

//  virtual int  num_adustable_parameters() const = 0;
//  virtual bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const = 0;
//  virtual bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
//  virtual bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
};


class c_ecclm
{
public:
  typedef c_ecclm this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::shared_ptr<this_class> uptr;

  c_ecclm();
  c_ecclm( c_ecclm_motion_model * model = nullptr);
  virtual ~c_ecclm() = default;

  void set_model(c_ecclm_motion_model * model);
  c_ecclm_motion_model * model() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_epsx(double v);
  double epsx() const;

  void set_epsfn(double v);
  double epsfn() const;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray());

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

protected:
  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

  bool align();

  double compute_rhs(const cv::Mat1d & params);
  double compute_jac(const cv::Mat1d & params, cv::Mat1d & H, cv::Mat1d & v);

protected:
  c_ecclm_motion_model * model_ = nullptr;
  cv::Mat1f reference_image_;
  cv::Mat1b reference_mask_;
  cv::Mat1f current_image_;
  cv::Mat1b current_mask_;
//  cv::Mat1f remapped_image;
//  cv::Mat1b remapped_mask;
  cv::Mat1f gx_, gy_;
  std::vector<cv::Mat1f> J;


  int max_iterations_ = 50;
  double epsx_ = 1e-5;
  double epsfn_ = 1e-5;
};


class c_ecclm_translation :
    public c_ecclm_motion_model
{
public:
  typedef c_ecclm_translation this_class;
  typedef c_ecclm_motion_model base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;


  void set_translation(const cv::Vec2d & v);
  const cv::Vec2d & translation() const;

public: // c_ecclm_motion_model

  cv::Mat1d parameters() const override;
  bool set_parameters(const cv::Mat1d & p) override;

  bool create_remap(const cv::Vec2d & T, const cv::Size & size, cv::Mat2f & map);
  bool create_remap(const cv::Mat1d & params, const cv::Size & size, cv::Mat2f & map) override;
  bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[/*nb parameters*/]) override;

protected:
  cv::Vec2d translation_;
};


class c_ecclm_affine :
    public c_ecclm_motion_model
{
public:
  typedef c_ecclm_affine this_class;
  typedef c_ecclm_motion_model base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  void set_transform(const cv::Matx23d & v);
  const cv::Matx23d & transform() const;

public: // c_ecclm_motion_model

  cv::Mat1d parameters() const override;
  bool set_parameters(const cv::Mat1d & p) override;

  bool create_remap(const cv::Matx23d & a, const cv::Size & size, cv::Mat2f & rmap);
  bool create_remap(const cv::Mat1d & params, const cv::Size & size, cv::Mat2f & rmap) override;
  bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[/*nb parameters*/]) override;

//  double compute_rhs(const cv::Mat1d & params,
//      const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
//      const cv::Mat1f & current_image, const cv::Mat1b & current_mask) override;
//
//  double compute_jac(const cv::Mat1d & params,
//      const cv::Mat1f & reference_image, const cv::Mat1b & reference_mask,
//      const cv::Mat1f & current_image, const cv::Mat1b & current_mask,
//      cv::Mat1d & A, cv::Mat1d & v) override;
//
//  static bool create_remap(const cv::Size & size, const cv::Matx23d & m,
//      cv::Mat2f & map);
//
//  static bool remap_image(const cv::Size & size, const cv::Matx23d & m,
//      const cv::Mat1f & src, const cv::Mat1b & src_mask,
//      cv::Mat1f & dst, cv::Mat1b & dst_mask);

protected:
  cv::Matx23d a_ =
      cv::Matx23d::eye();
};


#endif /* __c_ecclm_h__ */
