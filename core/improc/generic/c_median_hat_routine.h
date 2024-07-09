/*
 * c_median_hat_routine.h
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_hat_routine_h__
#define __c_median_hat_routine_h__

#include <core/improc/c_image_processor.h>

class c_median_hat_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_hat_routine,
      "median_hat",
      "Difference between input image and it's median blur");

  enum DisplayType {
    DisplayMedianBlur ,
    DisplayMedianHat ,
  };

  void set_radius(int v)
  {
    radius_ = v;
  }

  int radius() const
  {
    return radius_;
  }

  void set_iterations(int v)
  {
    iterations_ = v;
  }

  int iterations() const
  {
    return iterations_;
  }

  void set_display_type(DisplayType v)
  {
    display_type_ = v;
  }

  DisplayType display_type() const
  {
    return display_type_;
  }

  void set_absdiff(bool v)
  {
    absdiff_ = v;
  }

  bool absdiff() const
  {
    return absdiff_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, radius, "Filter radius [px]");
    BIND_PCTRL(ctls, iterations, "Number of medianBlur iterations");
    BIND_PCTRL(ctls, display_type, "Output image");
    BIND_PCTRL(ctls, absdiff, "Use cv::absdiff() instead of cv::subtract()");

  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, radius);
      SERIALIZE_PROPERTY(settings, save, *this, iterations);
      SERIALIZE_PROPERTY(settings, save, *this, display_type);
      SERIALIZE_PROPERTY(settings, save, *this, absdiff);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    const int r = image.depth() < CV_32F ? radius_ : 2;
    const int ksize = 2 * radius_ + 1;

    cv::Mat m;

    for( int i = 0; i < iterations_; ++i ) {

      if( m.empty() ) {
        cv::medianBlur(image, m, ksize);
      }
      else {
        cv::medianBlur(m, m, ksize);
      }
    }

    if ( display_type_ == DisplayMedianBlur ) {
      image.move(m);
    }
    else if ( absdiff_ ) {
      cv::absdiff(image.getMat(), m, image);
    }
    else {

      int ddepth = image.depth();

      switch (ddepth) {
        case CV_8U:
          ddepth = CV_8S;
          break;
        case CV_16U:
          ddepth = CV_16S;
          break;
      }

      cv::subtract(image.getMat(), m, image,
          cv::noArray(),
          ddepth);
    }

    return true;
  }

protected:
  int radius_ = 2;
  int iterations_ = 1;
  DisplayType display_type_ = DisplayMedianHat;
  bool absdiff_ = false;
};

#endif /* __c_median_hat_routine_h__ */
