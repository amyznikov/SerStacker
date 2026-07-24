/*
 * c_scale_channels_routine.cc
 *
 *  Created on: Aug 10, 2021
 *      Author: amyznikov
 */

#include "c_scale_channels_routine.h"

static bool applyChannelTransform(cv::InputArray src, cv::OutputArray dst,
    const cv::Scalar& stretch, const cv::Scalar& shift)
{
  const int channels = src.channels();

  switch (channels) {
    case 1: {
      const cv::Matx12f M(
          (float)stretch[0], (float)shift[0]
          );
      cv::transform(src, dst, M);
      break;
    }
    case 2: {
      const cv::Matx23f M(
          float(stretch[0]), 0.0f, float(shift[0]),
          0.0f, float(stretch[1]), float(shift[1])
          );
      cv::transform(src, dst, M);
      break;
    }
    case 3: {
      const cv::Matx34f M(
          float(stretch[0]), 0.0f, 0.0f, float(shift[0]),
          0.0f, float(stretch[1]), 0.0f, float(shift[1]),
          0.0f, 0.0f, float(stretch[2]), float(shift[2])
          );
      cv::transform(src, dst, M);
      break;
    }
    case 4: {
      using Matx45f = cv::Matx<float, 4, 5>;
      const Matx45f M( {
          float(stretch[0]),   0.0f,       0.0f,       0.0f,       float(shift[0]),
          0.0f,         float(stretch[1]), 0.0f,       0.0f,       float(shift[1]),
          0.0f,         0.0f,       float(stretch[2]), 0.0f,       float(shift[2]),
          0.0f,         0.0f,       0.0f,       float(stretch[3]), float(shift[3])
        });
      cv::transform(src, dst, M);
      break;
    }
    default: {
      CF_ERROR("NOT supported number of channels: %d" ,channels);
      return false;
    }
  }

  return true;
}

void c_scale_channels_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "stretch (B;G;R;A)", ctx(&this_class::_stretch), "");
  ctlbind(ctls, "shifts  (B;G;R;A)", ctx(&this_class::_shift), "");
}

bool c_scale_channels_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _stretch);
    SERIALIZE_OPTION(settings, save, *this, _shift);
    return true;
  }
  return false;
}

bool c_scale_channels_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  return applyChannelTransform(image, image, _stretch, _shift);
}
