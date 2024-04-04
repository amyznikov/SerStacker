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
#include <core/proc/geo-reconstruction.h>


class c_morphology_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_morphology_routine,
      "morphology",
      "Apply morphological operation on image");

  enum OPERATION {
      MORPH_ERODE    = cv::MORPH_ERODE, //!< see #erode
      MORPH_DILATE   = cv::MORPH_DILATE, //!< see #dilate
      MORPH_OPEN     = cv::MORPH_OPEN, //!< an opening operation
      MORPH_CLOSE    = cv::MORPH_CLOSE, //!< a closing operation
      MORPH_GRADIENT = cv::MORPH_GRADIENT, //!< a morphological gradient
      MORPH_TOPHAT   = cv::MORPH_TOPHAT, //!< "top hat"
      MORPH_BLACKHAT = cv::MORPH_BLACKHAT, //!< "black hat"
      MORPH_HITMISS  = cv::MORPH_HITMISS,  //!< "hit or miss"
      MORPH_SMOOTH_OPEN, //!< morphological_smooth_open
      MORPH_SMOOTH_CLOSE, //!< morphological_smooth_close
      MORPH_INTERNAL_GRADIENT, //!< morphological_internal_gradient
      MORPH_EXTERNAL_GRADIENT, //!< morphological_external_gradient
      MORPH_LAPLACIAN, //!< morphological_laplacian
      MORPH_RAMPLEE, //!< rampLee
      MORPH_TEXLEE, //!< texLee
      MORPH_GEO_FILL_HOLES4, //!< geo_fill_holes
      MORPH_GEO_FILL_HOLES8, //!< geo_fill_holes
      MORPH_LAPLACIAN_ABS, //!< morphological_laplacian
  };


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

  void set_operation(OPERATION v)
  {
    operation_ = v;
  }

  OPERATION operation() const
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

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, operation, "Type of a morphological operation");
    BIND_PCTRL(ctls, se_shape, "Shape of structuring element");
    BIND_PCTRL(ctls, se_size, "Size of structuring element");
    BIND_PCTRL(ctls, anchor, "Anchor position with the kernel. Negative values mean that the anchor is at the kernel center");
    BIND_PCTRL(ctls, iterations, "Number of times erosion and dilation are applied");
    BIND_PCTRL(ctls, borderType, "Pixel extrapolation method, see #BorderTypes. BORDER_WRAP is not supported");
    BIND_PCTRL(ctls, borderValue, "Border value in case of a constant border");
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
    switch (operation_) {

      case MORPH_SMOOTH_OPEN:
        morphological_smooth_open(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_SMOOTH_CLOSE:
        morphological_smooth_close(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_INTERNAL_GRADIENT:
        morphological_internal_gradient(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_EXTERNAL_GRADIENT:
        morphological_external_gradient(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_LAPLACIAN:
        morphological_laplacian(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_LAPLACIAN_ABS:
        morphological_laplacian_abs(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_RAMPLEE:
        rampLee(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_TEXLEE:
        texLee(image.getMat(), image,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            borderType_,
            borderValue_);
        break;

      case MORPH_GEO_FILL_HOLES4:
        geo_fill_holes(image.getMat(), image, 4);
        break;

      case MORPH_GEO_FILL_HOLES8:
        geo_fill_holes(image.getMat(), image, 8);
        break;

      default:
        cv::morphologyEx(image.getMat(), image,
            operation_,
            cv::getStructuringElement(se_shape_, se_size_, anchor_),
            anchor_,
            iterations_,
            borderType_,
            borderValue_);
        break;
    }

    return true;
  }

protected:
  cv::MorphShapes se_shape_ = cv::MORPH_RECT;
  cv::Size se_size_ = cv::Size(3, 3);
  cv::Point anchor_ = cv::Point(-1,-1);
  OPERATION operation_ = MORPH_ERODE;
  int iterations_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_CONSTANT;
  cv::Scalar borderValue_ = cv::Scalar::all(0);
};

#endif /* CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_ */
