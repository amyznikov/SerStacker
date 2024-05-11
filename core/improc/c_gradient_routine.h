/*
 * c_gradient_routine.h
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#ifndef __c_gradient_routine_h__
#define __c_gradient_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/pixtype.h>
#include <core/proc/gradient.h>

class c_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gradient_routine,
      "gradient", "compute image gradient");

  enum ComputeMethod
  {
    ComputeMethodFilter1D,
    ComputeMethodSobel,
    ComputeMethodDiagonalGradient,
  };

  enum OutputType {
    OutputGradient,
    OutputGradientX,
    OutputGradientY,
    OutputGradientMagnitude,
    OutputGradientPhase,
    OutputGradientPhaseW,
    OutputGradientPhase90,
    OutputGradientPhase90W,
    // OutputTextureFromGradients,
  };

  void set_compute_method(ComputeMethod v)
  {
    compute_method_ = v;
  }

  ComputeMethod compute_method() const
  {
    return compute_method_;
  }

  void set_border_type(cv::BorderTypes v)
  {
    border_type_ = v;
  }

  cv::BorderTypes border_type() const
  {
    return border_type_;
  }

  void set_output_type(OutputType v)
  {
    output_type_ = v;
  }

  OutputType output_type() const
  {
    return output_type_;
  }

//  void set_order_x(int  v)
//  {
//    order_x_ = v;
//  }
//
//  int order_x() const
//  {
//    return order_x_;
//  }
//
//  void set_order_y(int  v)
//  {
//    order_y_ = v;
//  }
//
//  int order_y() const
//  {
//    return order_y_;
//  }

//  void set_kradius(int  v)
//  {
//    kradius_ = v;
//  }
//
//  int kradius() const
//  {
//    return kradius_;
//  }

  void set_ddepth(PIXEL_DEPTH v)
  {
    ddepth_ = v;
  }

  PIXEL_DEPTH ddepth() const
  {
    return ddepth_;
  }

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


  void set_erode_mask(bool v)
  {
    erode_mask_ = v;
  }

  bool erode_mask() const
  {
    return erode_mask_;
  }

  void set_squared(bool v)
  {
    squared_ = v;
  }

  bool squared() const
  {
    return squared_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  ComputeMethod compute_method_ = ComputeMethodFilter1D;
  OutputType output_type_ = OutputGradientMagnitude;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_NO_CHANGE;
  cv::BorderTypes border_type_  = cv::BORDER_DEFAULT;
//  int order_x_ = 1;
//  int order_y_ = 0;
 // int kradius_ = 3;
  double scale_ = 1;
  double delta_ = 0;
  bool squared_ = false;
  bool erode_mask_ = false;
};

#endif /* __c_gradient_routine_h__ */
