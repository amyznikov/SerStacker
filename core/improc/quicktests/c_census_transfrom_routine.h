/*
 * c_census_transfrom_routine.h
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_census_transfrom_routine_h__
#define __c_census_transfrom_routine_h__

#include <core/improc/c_image_processor.h>
#include <opencv2/stereo/descriptor.hpp>

class c_census_transfrom_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_census_transfrom_routine,
      "census", "c_census_transfrom_routine");

  enum CensusType {
    CV_DENSE_CENSUS = cv::stereo::CV_DENSE_CENSUS,
    CV_SPARSE_CENSUS = cv::stereo::CV_SPARSE_CENSUS,
    CV_CS_CENSUS = cv::stereo::CV_CS_CENSUS,
    CV_MODIFIED_CS_CENSUS = cv::stereo::CV_MODIFIED_CS_CENSUS,
    CV_MODIFIED_CENSUS_TRANSFORM = cv::stereo::CV_MODIFIED_CENSUS_TRANSFORM,
    CV_MEAN_VARIATION = cv::stereo::CV_MEAN_VARIATION,
    CV_STAR_KERNEL = cv::stereo::CV_STAR_KERNEL
  };

  void set_type(CensusType v)
  {
    census_type_ = v;
  }

  CensusType type() const
  {
    return census_type_;
  }

  void set_kernel_size(int v)
  {
    kernel_size_ = v;
  }

  int kernel_size() const
  {
    return kernel_size_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, type, "census type");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, kernel_size, "kernel size");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, type);
      SERIALIZE_PROPERTY(settings, save, *this, kernel_size);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    cv::Mat gray;

    if( image.channels() == 1 ) {
      gray = image.getMat();
    }
    else {
      cv::cvtColor(image, gray,
          cv::COLOR_BGR2GRAY);
    }

    cv::Mat dist1(gray.size(), CV_32SC1);
    cv::stereo::censusTransform(gray, kernel_size_, dist1, census_type_);
    dist1.copyTo(image);

    return true;
  }

protected:
  int kernel_size_ = 5;
  CensusType census_type_ = CV_SPARSE_CENSUS;

};

#endif /* __c_census_transfrom_routine_h__ */
