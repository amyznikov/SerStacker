/*
 * c_star_extractor_routine.h
 *
 *  Created on: May 16, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_star_extractor_routine_h__
#define __c_star_extractor_routine_h__

#include <core/improc/c_image_processor.h>

class c_star_extractor_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_star_extractor_routine,
      "star_extractor", "Extract stars on sky image");

  enum DisplayType {
    DisplayRichKeypoints,
    DisplaySourceImage,
    DisplayFilteredImage,
  };

  ////////////

  void set_display_type(DisplayType v)
  {
    _display_type = v;
  }

  DisplayType display_type() const
  {
    return _display_type;
  }

  void set_median_filter_size(int v)
  {
    _median_filter_size = v;
  }

  int median_filter_size() const
  {
    return _median_filter_size;
  }

  void set_sigma1(double v)
  {
    _sigma1 = v;
  }

  double sigma1() const
  {
    return _sigma1;
  }

  void set_sigma2(double v)
  {
    _sigma2 = v;
  }

  double sigma2() const
  {
    return _sigma2;
  }

  void set_noise_sigma(double v)
  {
    _noise_sigma = v;
  }

  double noise_sigma() const
  {
    return _noise_sigma;
  }

  void set_noise_scale(double v)
  {
    _noise_scale = v;
  }

  double noise_scale() const
  {
    return _noise_scale;
  }

  ////////////

  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  DisplayType _display_type = DisplayRichKeypoints;
  int _median_filter_size = 3;
  double _sigma1 = 10;
  double _sigma2 = 2;
  double _noise_sigma = 100;
  double _noise_scale = 10;
};

#endif /* __c_star_extractor_routine_h__ */
