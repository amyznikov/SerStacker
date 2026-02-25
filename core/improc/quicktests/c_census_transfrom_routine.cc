/*
 * c_census_transfrom_routine.cc
 *
 *  Created on: Apr 3, 2023
 *      Author: amyznikov
 */

#include "c_census_transfrom_routine.h"

template<>
const c_enum_member * members_of<c_census_transfrom_routine::CensusType>()
{
  static const c_enum_member members[] = {
    {c_census_transfrom_routine::CV_DENSE_CENSUS, "DENSE_CENSUS", "cv::stereo::CV_DENSE_CENSUS"},
    {c_census_transfrom_routine::CV_SPARSE_CENSUS, "SPARSE_CENSUS", "cv::stereo::CV_SPARSE_CENSUS"},
    {c_census_transfrom_routine::CV_CS_CENSUS, "CS_CENSUS", "cv::stereo::CV_CS_CENSUS"},
    {c_census_transfrom_routine::CV_MODIFIED_CS_CENSUS, "MODIFIED_CS_CENSUS", "cv::stereo::CV_MODIFIED_CS_CENSUS"},
    {c_census_transfrom_routine::CV_MODIFIED_CENSUS_TRANSFORM, "MODIFIED_CENSUS_TRANSFORM", "cv::stereo::CV_MODIFIED_CENSUS_TRANSFORM"},
    {c_census_transfrom_routine::CV_MEAN_VARIATION, "MEAN_VARIATION", "cv::stereo::CV_MEAN_VARIATION"},
    {c_census_transfrom_routine::CV_STAR_KERNEL, "STAR_KERNEL", "cv::stereo::CV_STAR_KERNEL"},
    {c_census_transfrom_routine::CV_SPARSE_CENSUS}
  };

  return members;
}

void c_census_transfrom_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "kernel_size", ctx(&this_class::_kernel_size), "");
   ctlbind(ctls, "census_type", ctx(&this_class::_census_type), "");
}

bool c_census_transfrom_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _census_type);
    SERIALIZE_OPTION(settings, save, *this, _kernel_size);
    return true;
  }
  return false;
}

bool c_census_transfrom_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
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
  cv::stereo::censusTransform(gray, _kernel_size, dist1, _census_type);
  dist1.copyTo(image);

  return true;
}

