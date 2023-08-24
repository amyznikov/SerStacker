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

  enum OutputType {
    OutputGradient,
    OutputGradientMagnitude,
    OutputGradientPhase,
    OutputGradientPhase90,
    OutputGradientPhase90W,
    OutputTextureFromGradients,
  };

  void set_output(OutputType v)
  {
    output_ = v;
  }

  OutputType output() const
  {
    return output_;
  }

  void set_order_x(int  v)
  {
    order_x_ = v;
  }

  int order_x() const
  {
    return order_x_;
  }

  void set_order_y(int  v)
  {
    order_y_ = v;
  }

  int order_y() const
  {
    return order_y_;
  }

  void set_kradius(int  v)
  {
    kradius_ = v;
  }

  int kradius() const
  {
    return kradius_;
  }

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

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output, "Output type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, order_x, "Order of x derivative");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, order_y, "Order of y derivative");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, kradius, "kernel radius in pixels");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ddepth, "Destination image depth");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, delta, "Optional value added to the filtered pixels before storing them in dst.");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "Optional multiplier to differentiate kernel.");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, squared, "Square output");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, erode_mask, "Update image mask if not empty");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, output);
      SERIALIZE_PROPERTY(settings, save, *this, order_x);
      SERIALIZE_PROPERTY(settings, save, *this, order_y);
      SERIALIZE_PROPERTY(settings, save, *this, kradius);
      SERIALIZE_PROPERTY(settings, save, *this, scale);
      SERIALIZE_PROPERTY(settings, save, *this, ddepth);
      SERIALIZE_PROPERTY(settings, save, *this, delta);
      SERIALIZE_PROPERTY(settings, save, *this, squared);
      SERIALIZE_PROPERTY(settings, save, *this, erode_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  OutputType output_ = OutputGradientMagnitude;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_NO_CHANGE;
  int order_x_ = 1;
  int order_y_ = 0;
  int kradius_ = 3;
  double scale_ = 1;
  double delta_ = 0;
  bool squared_ = false;
  bool erode_mask_ = false;
};

#endif /* __c_gradient_routine_h__ */
