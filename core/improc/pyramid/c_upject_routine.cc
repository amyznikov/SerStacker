/*
 * c_upject_routine.cc
 *
 *  Created on: Jun 14, 2023
 *      Author: amyznikov
 */

#include "c_upject_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_upject_routine::UpjectMode>()
{
  static const c_enum_member members[] = {
      {c_upject_routine::UpjectUneven, "Uneven"},
      {c_upject_routine::UpjectEven, "Even"},
      {c_upject_routine::UpjectUneven},
  };

  return members;
}

template<>
const c_enum_member * members_of<c_upject_routine::FillMode>()
{
  static const c_enum_member members[] = {
      {c_upject_routine::FillZero, "Zeros"},
      {c_upject_routine::FillAvg, "Avg"},
      {c_upject_routine::FillZero},
  };

  return members;
}

void c_upject_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, mode, "Which rows and columns to reject");
  BIND_PCTRL(ctls, fill_mode, "How to fill empty pixels");
}

bool c_upject_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, mode);
    SERIALIZE_PROPERTY(settings, save, *this, fill_mode);
    return true;
  }
  return false;
}

bool c_upject_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat img;
  cv::Mat zmask;

  switch (mode_) {
    case UpjectUneven:
      upject_uneven(image.getMat(), img, dstSize_, &zmask, CV_8U);
      image.move(img);
      break;
    case UpjectEven:
      upject_even(image.getMat(), img, dstSize_, &zmask, CV_8U);
      image.move(img);
      break;
    default:
      CF_ERROR("Invalid downstrike mode specified");
      return false;
  }

  switch (fill_mode_) {
    case FillZero:
      break;

    case FillAvg: {

      static const float k[3] = {
          0.5, 0, 0.5,
      };

      static const cv::Matx13f Kx =
          cv::Matx13f(k);

      static const cv::Matx31f Ky =
          cv::Matx31f(k);

      cv::Mat tmp1, tmp2;

      cv::filter2D(image.getMat(), tmp1, image.depth(), Kx);
      image.getMat().copyTo(tmp1, zmask);

      cv::filter2D(tmp1, tmp2, image.depth(), Ky);
      tmp1.copyTo(tmp2, tmp1 != 0);

      tmp2.copyTo(image);

      break;
    }
  }

  if( mask.needed() && !mask.empty() ) {
    mask.release();
  }

  return true;
}
