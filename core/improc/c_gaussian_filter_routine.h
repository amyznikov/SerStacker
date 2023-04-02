/*
 * c_gaussian_filter_routine.h
 *
 *  Created on: Oct 9, 2021
 *      Author: amyznikov
 */

#ifndef __c_gaussian_filter_routine_h__
#define __c_gaussian_filter_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/c_gaussian_filter.h>

class c_gaussian_filter_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_filter_routine,
      "gaussian_filter", "gaussian_filter");

  void set_sigma(double v)
  {
    sigma_ = v;
  }

  double sigma() const
  {
    return sigma_;
  }

  void set_ignore_mask(bool v)
  {
    ignore_mask_ = v;
  }

  bool ignore_mask() const
  {
    return ignore_mask_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, sigma, "Gaussian kernel sigma");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, ignore_mask, "Ignore alpha mask");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, sigma);
      SERIALIZE_PROPERTY(settings, save, *this, ignore_mask);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if ( ignore_mask_ || mask.empty() || cv::countNonZero(mask) == mask.size().area() ) {
      c_gaussian_filter(sigma_, sigma_).apply(image.getMat(), cv::noArray(), image, cv::BORDER_REFLECT101);
    }
    else {
      cv::Mat tmp;
      image.getMat().copyTo(tmp);
      tmp.setTo(0, ~mask.getMat());
      c_gaussian_filter(sigma_, sigma_).apply(tmp, mask, image, cv::BORDER_REFLECT101);
    }

    return true;
  }

protected:
  double sigma_ = 1;
  bool ignore_mask_ = true;
};

#endif /* __c_gaussian_filter_routine_h__ */
