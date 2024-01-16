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
