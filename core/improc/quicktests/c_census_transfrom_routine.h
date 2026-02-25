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

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  int _kernel_size = 5;
  CensusType _census_type = CV_SPARSE_CENSUS;

};

#endif /* __c_census_transfrom_routine_h__ */
