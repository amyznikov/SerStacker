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

  virtual ~c_ecclm_motion_model()
  {
  }

  const cv::Mat1f & parameters() const;
  virtual bool set_parameters(const cv::Mat1f & p) = 0;
  virtual cv::Mat1f scale_parameters(const cv::Mat1f & p, double scale) const = 0;

  virtual bool remap(const cv::Mat1f & params, const cv::Size & size,
      cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask);

  bool remap(const cv::Size & size, cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask);

  virtual bool create_remap(const cv::Mat1f & params, const cv::Size & size, cv::Mat2f & map) = 0;
  bool create_remap(const cv::Size & size, cv::Mat2f & map);

  virtual bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy,
      cv::Mat1f J[/*nb parameters*/]) = 0;

protected:
  cv::Mat1f parameters_;
};


class c_ecclm
{
public:
  typedef c_ecclm this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclm();
  c_ecclm(c_ecclm_motion_model * model = nullptr);
  virtual ~c_ecclm() = default;

  void set_model(c_ecclm_motion_model * model);
  c_ecclm_motion_model * model() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

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

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat1f & current_image() const;
  const cv::Mat1b & current_mask() const;


protected:
  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

  bool align();

  double compute_rhs(const cv::Mat1f & params);
  double compute_jac(const cv::Mat1f & params, cv::Mat1f & H, cv::Mat1f & v);

protected:
  c_ecclm_motion_model * model_ = nullptr;
  cv::Mat1f reference_image_;
  cv::Mat1b reference_mask_;
  cv::Mat1f current_image_;
  cv::Mat1b current_mask_;
  cv::Mat1f gx_, gy_;
  std::vector<cv::Mat1f> J;

  int max_iterations_ = 50;
  double update_step_scale_ = 1.5;
  double epsx_ = 1e-5;
  double epsfn_ = 1e-5;


};

class c_ecclmp
{
public:
  typedef c_ecclmp this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclmp();
  c_ecclmp( c_ecclm_motion_model * model = nullptr);
  virtual ~c_ecclmp() = default;

  void set_model(c_ecclm_motion_model * model);
  c_ecclm_motion_model * model() const;

  void set_maxlevel(int v);
  int maxlevel() const;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool align(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

protected:

protected:
  std::vector<c_ecclm::uptr> pyramid_;
  c_ecclm_motion_model * model_ = nullptr;
  int maxlevel_ = 7;
};



class c_ecclm_translation :
    public c_ecclm_motion_model
{
public:
  typedef c_ecclm_translation this_class;
  typedef c_ecclm_motion_model base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;


  c_ecclm_translation();
  c_ecclm_translation(const cv::Vec2f & T);

  void set_translation(const cv::Vec2f & T);
  cv::Vec2f translation() const;

  bool set_parameters(const cv::Mat1f & p) final;
  cv::Mat1f scale_parameters(const cv::Mat1f & p, double scale) const final;
  bool create_remap(const cv::Vec2f & T, const cv::Size & size, cv::Mat2f & map);
  bool create_remap(const cv::Mat1f & params, const cv::Size & size, cv::Mat2f & map) final;
  bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[/*nb parameters*/]) final;
};


class c_ecclm_affine :
    public c_ecclm_motion_model
{
public:
  typedef c_ecclm_affine this_class;
  typedef c_ecclm_motion_model base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclm_affine();
  c_ecclm_affine(const cv::Matx23f & matrix);

  void set_matrix(const cv::Matx23f & v);
  cv::Matx23f matrix() const;

  bool set_parameters(const cv::Mat1f & p) final;
  cv::Mat1f scale_parameters(const cv::Mat1f & p, double scale) const final;
  bool create_remap(const cv::Matx23f & a, const cv::Size & size, cv::Mat2f & rmap);
  bool create_remap(const cv::Mat1f & params, const cv::Size & size, cv::Mat2f & rmap) final;
  bool create_steppest_descend_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[/*nb parameters*/]) final;
};


#endif /* __c_ecclm_h__ */
