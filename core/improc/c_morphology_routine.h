/*
 * c_morphology_routine.h
 *
 *  Created on: Jun 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_
#define CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_

#include <core/improc/c_image_processor.h>
#include <core/proc/morphology.h>

class c_morphology_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_morphology_routine,
      "morphology",
      "Apply morphological operation on image");

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

  void set_operation(cv::MorphTypes v)
  {
    operation_ = v;
  }

  cv::MorphTypes operation() const
  {
    return operation_;
  }

  void set_iterations(int v)
  {
    iterations_ = v;
  }

  int iterations() const
  {
    return iterations_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }

  void set_borderValue(const cv::Scalar& v)
  {
    borderValue_ = v;
  }

  const cv::Scalar& borderValue() const
  {
    return borderValue_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, operation, "Type of a morphological operation");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, se_shape, "Shape of structuring element");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, se_size, "Size of structuring element");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, anchor, "Anchor position with the kernel. Negative values mean that the anchor is at the kernel center");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, iterations, "Number of times erosion and dilation are applied");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderType, "Pixel extrapolation method, see #BorderTypes. BORDER_WRAP is not supported");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, borderValue, "Border value in case of a constant border");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, operation);
      SERIALIZE_PROPERTY(settings, save, *this, se_shape);
      SERIALIZE_PROPERTY(settings, save, *this, se_size);
      SERIALIZE_PROPERTY(settings, save, *this, anchor);
      SERIALIZE_PROPERTY(settings, save, *this, iterations);
      SERIALIZE_PROPERTY(settings, save, *this, borderType);
      SERIALIZE_PROPERTY(settings, save, *this, borderValue);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override
  {
    cv::morphologyEx(image.getMat(), image,
        operation_,
        cv::getStructuringElement(se_shape_, se_size_, anchor_),
        anchor_,
        iterations_,
        borderType_,
        borderValue_);

    return true;
  }

protected:
  cv::MorphShapes se_shape_ = cv::MORPH_RECT;
  cv::Size se_size_ = cv::Size(3, 3);
  cv::Point anchor_ = cv::Point(-1,-1);
  cv::MorphTypes operation_ = cv::MORPH_ERODE;
  int iterations_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_CONSTANT;
  cv::Scalar borderValue_ = cv::Scalar::all(0);
};

#endif /* CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_ */