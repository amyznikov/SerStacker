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

class c_gradient_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gradient_routine,
      "gradient", "compute image gradient");

  enum OutputType {
    OutputGradientMagnitude,
    OutputGradientX,
    OutputGradientY,
  };

  void set_output(OutputType v)
  {
    output_ = v;
  }

  OutputType output() const
  {
    return output_;
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
    ADD_IMAGE_PROCESSOR_CTRL(ctls, output, "Output image");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, squared, "Square output");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ddepth, "Destination image depth");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, delta, "Optional value added to the filtered pixels before storing them in dst.");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, erode_mask, "Update image mask if not empty");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, output);
      SERIALIZE_PROPERTY(settings, save, *this, squared);
      SERIALIZE_PROPERTY(settings, save, *this, ddepth);
      SERIALIZE_PROPERTY(settings, save, *this, delta);
      SERIALIZE_PROPERTY(settings, save, *this, erode_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  OutputType output_ = OutputGradientMagnitude;
  PIXEL_DEPTH ddepth_ = PIXEL_DEPTH_NO_CHANGE;
  double delta_ = 0;
  bool squared_ = false;
  bool erode_mask_ = false;
};

#endif /* __c_gradient_routine_h__ */
