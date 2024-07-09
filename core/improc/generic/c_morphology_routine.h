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
    SE.release();
  }

  cv::MorphShapes se_shape() const
  {
    return se_shape_;
  }

  void set_se_size(const cv::Size & v)
  {
    se_size_ = v;
    SE.release();
  }

  const cv::Size& se_size() const
  {
    return se_size_;
  }

  void set_anchor(const cv::Point & v)
  {
    anchor_ = v;
    SE.release();
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

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask) override;


protected:
  cv::MorphShapes se_shape_ = cv::MORPH_RECT;
  cv::Size se_size_ = cv::Size(3, 3);
  cv::Point anchor_ = cv::Point(-1,-1);
  OPERATION operation_ = MORPH_ERODE;
  int iterations_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_CONSTANT;
  cv::Scalar borderValue_ = cv::Scalar::all(0);
  cv::Mat SE;
};

#endif /* CORE_IMPROC_C_MORPHOLOGY_ROUTINE_H_ */
