/*
 * c_neighbourhood_average_routine.h
 *
 *  Created on: Nov 23, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_neighbourhood_average_routine_h__
#define __c_neighbourhood_average_routine_h__

#include <core/improc/c_image_processor.h>

class c_neighbourhood_average_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_neighbourhood_average_routine,
      "c_neighbourhood_average_routine", "cv::filter2d with hole in center of SE");


  void set_se_shape(cv::MorphShapes v)
  {
    se_shape_ = v;
  }

  cv::MorphShapes se_shape() const
  {
    return se_shape_;
  }

  void set_se_size(const cv::Size & v)
  {
    se_size_ = v;
  }

  const cv::Size& se_size() const
  {
    return se_size_;
  }

  void set_anchor(const cv::Point & v)
  {
    anchor_ = v;
  }

  const cv::Point & anchor() const
  {
    return anchor_;
  }

  void set_border_type(cv::BorderTypes v)
  {
    border_type_ = v;
  }

  cv::BorderTypes border_type() const
  {
    return border_type_;
  }

//  void set_border_value(const cv::Scalar & v)
//  {
//    border_value_ = v;
//  }
//
//  const cv::Scalar & border_value() const
//  {
//    return border_value_;
//  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  cv::MorphShapes se_shape_ = cv::MORPH_RECT;
  cv::BorderTypes border_type_ = cv::BORDER_REPLICATE;
  //  cv::Scalar border_value_;

  cv::Size se_size_ = cv::Size(3, 3);
  cv::Point anchor_ = cv::Point(-1, -1);
};

#endif /* __c_neighbourhood_average_routine_h__ */
