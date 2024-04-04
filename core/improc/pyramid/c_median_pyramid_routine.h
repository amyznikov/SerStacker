/*
 * c_median_pyramid_routine.h
 *
 *  Created on: Sep 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_median_pyramid_routine_h__
#define __c_median_pyramid_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/median_pyramid.h>

class c_median_pyramid_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_median_pyramid_routine,
      "median_pyramid", "Display Median Pyramid layers");

  enum DisplayType {
    DisplayMedianBlur,
    DisplayMedianHat,
  };

  void set_display_type(DisplayType v)
  {
    display_type_ = v;
  }

  DisplayType display_type() const
  {
    return display_type_;
  }

  void set_display_level(int v)
  {
    display_level_ = v;
  }

  int display_level() const
  {
    return display_level_;
  }

  void set_downstrike_mode(DOWNSTRIKE_MODE v)
  {
    downstrike_mode_ = v;
  }

  DOWNSTRIKE_MODE downstrike_mode() const
  {
    return downstrike_mode_;
  }

  void set_ksize(int v)
  {
    ksize_  = v;
  }

  int ksize()
  {
    return ksize_;
  }

  void set_median_iterations(int v)
  {
    median_iterations_ = v;
  }

  int median_iterations() const
  {
    return median_iterations_;
  }


  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, display_type, "Specify display type");
    BIND_PCTRL(ctls, display_level, "Specify display pyramid layer");
    BIND_PCTRL(ctls, ksize, "Specify Median blur kernel size");
    BIND_PCTRL(ctls, median_iterations, "Specify Median iterations");
    BIND_PCTRL(ctls, downstrike_mode, "Specify downstrike mode");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, display_type);
      SERIALIZE_PROPERTY(settings, save, *this, display_level);
      SERIALIZE_PROPERTY(settings, save, *this, ksize);
      SERIALIZE_PROPERTY(settings, save, *this, median_iterations);
      SERIALIZE_PROPERTY(settings, save, *this, downstrike_mode);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    build_median_pyramid(image, ksize_, median_iterations_, downstrike_mode_, median_blurs_, median_hats_);

    if( median_blurs_.empty() || median_hats_.size() != median_blurs_.size() ) {
      CF_ERROR("build_median_pyramid() fails");
      return false;
    }

    const int display_level =
        std::max(0, std::min(display_level_,
            (int) median_blurs_.size() - 1));

    switch (display_type_) {
      case DisplayMedianBlur:
        median_blurs_[display_level].copyTo(image);
        break;
      case DisplayMedianHat:
        median_hats_[display_level].copyTo(image);
        break;
    }

    if ( mask.needed() && !mask.empty() ) {
      mask.release();
    }

    return true;
  }

protected:
  DisplayType display_type_ = DisplayMedianBlur;
  DOWNSTRIKE_MODE downstrike_mode_ = DOWNSTRIKE_UNEVEN;
  int ksize_ = 3;
  int median_iterations_ = 1;
  int display_level_ = 0;
  std::vector<cv::Mat> median_blurs_;
  std::vector<cv::Mat> median_hats_;
};

#endif /* __c_median_pyramid_routine_h__ */
