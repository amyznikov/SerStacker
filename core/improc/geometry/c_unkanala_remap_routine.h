/*
 * c_unkanala_remap_routine.h
 *
 *  Created on: Aug 29, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_unkanala_remap_routine_h__
#define __c_unkanala_remap_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unkanala.h>


class c_unkanala_remap_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_unkanala_remap_routine,
      "unkanala_remap", "Apply unkanala_remap to image");

  const cv::Size & image_size() const
  {
    return intrinsics_.image_size;
  }

  void set_image_size(const cv::Size & v)
  {
    intrinsics_.image_size = v;
    remap_.release();
  }

  double focal_length_x() const
  {
    return intrinsics_.focal_length_x;
  }

  void set_focal_length_x(double v)
  {
    intrinsics_.focal_length_x = v;
    remap_.release();
  }

  double focal_length_y() const
  {
    return intrinsics_.focal_length_y;
  }

  void set_focal_length_y(double v)
  {
    intrinsics_.focal_length_y = v;
    remap_.release();
  }


  double principal_point_x() const
  {
    return intrinsics_.principal_point_x;
  }

  void set_principal_point_x(double v)
  {
    intrinsics_.principal_point_x = v;
    remap_.release();
  }

  double principal_point_y () const
  {
    return intrinsics_.principal_point_y;
  }

  void set_principal_point_y (double v)
  {
    intrinsics_.principal_point_y = v;
    remap_.release();
  }

  const std::vector<double> & distortion_coefficients() const
  {
    return intrinsics_.distortion_coefficients;
  }

  void set_distortion_coefficients(const std::vector<double> & v)
  {
    intrinsics_.distortion_coefficients = v;
    remap_.release();
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, image_size, "");
    BIND_PCTRL(ctls, focal_length_x, "");
    BIND_PCTRL(ctls, focal_length_y, "");
    BIND_PCTRL(ctls, principal_point_x, "");
    BIND_PCTRL(ctls, principal_point_y, "");
    BIND_PCTRL(ctls, distortion_coefficients, "");
  }

  bool serialize(c_config_setting settings, bool save)
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, image_size);
      SERIALIZE_PROPERTY(settings, save, *this, focal_length_x);
      SERIALIZE_PROPERTY(settings, save, *this, focal_length_y);
      SERIALIZE_PROPERTY(settings, save, *this, principal_point_x);
      SERIALIZE_PROPERTY(settings, save, *this, principal_point_y);
      SERIALIZE_PROPERTY(settings, save, *this, distortion_coefficients);
      return true;
    }
    return false;
  }


  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if ( remap_.empty() ) {

      if ( !create_unkanala_remap(intrinsics_, remap_) ) {
        CF_ERROR("create_unkanala_remap() fails");
        return  false;
      }
    }

    if( image.needed() ) {
      cv::remap(image.getMat(), image,
          remap_, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }

    if( mask.needed() ) {

      if ( !mask.empty() ) {
        cv::remap(mask.getMat(), mask,
            remap_, cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);
      }
      else {
        cv::remap(cv::Mat1b(image.size(), 255), mask,
            remap_, cv::noArray(),
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);
      }

      cv::compare(mask.getMat(), 250, mask, cv::CMP_GT);
    }


    return true;
  }

protected:
  c_kanala_intrinsics intrinsics_;
  cv::Mat2f remap_;
};

#endif /* __c_unkanala_remap_routine_h__ */
