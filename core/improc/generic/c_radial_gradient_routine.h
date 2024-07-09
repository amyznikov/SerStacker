/*
 * c_radial_gradient_routine.h
 *
 *  Created on: Sep 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_radial_gradient_routine_h__
#define __c_radial_gradient_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/gradient.h>

class c_radial_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_radial_gradient_routine,
      "radial_gradient", "compute radial gradient on image");

  enum OutputType {
    OutputRadialGradient,
    OutputTangentialGradient,
  };

  void set_output_type(OutputType v)
  {
    output_type_ = v;
  }

  OutputType output_type() const
  {
    return output_type_;
  }

  void set_reference_point(const cv::Point2f & v)
  {
    reference_point_ = v;
  }

  const cv::Point2f & reference_point() const
  {
    return reference_point_;
  }

  void set_kradius(int  v)
  {
    kradius_ = v;
  }

  int kradius() const
  {
    return kradius_;
  }

//  void set_ddepth(PIXEL_DEPTH v)
//  {
//    ddepth_ = v;
//  }
//
//  PIXEL_DEPTH ddepth() const
//  {
//    return ddepth_;
//  }

  void set_delta(double v)
  {
    delta_ = v;
  }

  double delta() const
  {
    return delta_;
  }

  void set_scale(double  v)
  {
    scale_ = v;
  }

  double scale() const
  {
    return scale_;
  }

  void set_magnitude(bool v)
  {
    magnitude_ = v;
  }

  bool magnitude() const
  {
    return magnitude_;
  }

  void set_squared(bool v)
  {
    squared_ = v;
  }

  bool squared() const
  {
    return squared_;
  }

  void set_erode_mask(bool v)
  {
    erode_mask_ = v;
  }

  bool erode_mask() const
  {
    return erode_mask_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;


protected:
  OutputType output_type_ = OutputRadialGradient;
  cv::Point2f reference_point_ = cv::Point2f(0,0);
  int kradius_ = 3;
  double scale_ = 1;
  double delta_ = 0;
  bool magnitude_ = false;
  bool squared_ = false;
  bool erode_mask_ = false;
};

#endif /* __c_radial_gradient_routine_h__ */
