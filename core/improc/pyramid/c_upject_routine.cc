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

void c_upject_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "mode", ctx(&this_class::_mode), "");
   ctlbind(ctls, "fill_mode", ctx(&this_class::_fill_mode), "");
}

bool c_upject_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _mode);
    SERIALIZE_OPTION(settings, save, *this, _fill_mode);
    return true;
  }
  return false;
}

bool c_upject_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat img;
  cv::Mat zmask;

  switch (_mode) {
    case UpjectUneven:
      upject_uneven(image.getMat(), img, _dstSize, &zmask, CV_8U);
      image.move(img);
      break;
    case UpjectEven:
      upject_even(image.getMat(), img, _dstSize, &zmask, CV_8U);
      image.move(img);
      break;
    default:
      CF_ERROR("Invalid downstrike mode specified");
      return false;
  }

  switch (_fill_mode) {
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
