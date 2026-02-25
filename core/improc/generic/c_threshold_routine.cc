/*
 * c_threshold_routine.cc
 *
 *  Created on: Sep 18, 2022
 *      Author: amyznikov
 */

#include "c_threshold_routine.h"
#include <core/proc/morphology.h>
#include <core/proc/geo-reconstruction.h>
#include <core/proc/planetary-disk-detection.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>





template<>
const c_enum_member* members_of<c_threshold_routine::THRESHOLD_TYPE>()
{
  static const c_enum_member members[] = {
      { c_threshold_routine::THRESHOLD_VALUE, "VALUE", "Use user-specified value for compare operation"},
      { c_threshold_routine::THRESHOLD_OTSU, "OTSU", "Use Otsu algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_TRIANGLE, "TRIANGLE", "Use Triangle algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_MOMENTS, "MOMENTS", "Use MOMENTS algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_ISODATA, "ISODATA", "Use ISODATA algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_HUANG, "HUANG", "Use HUANG algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_YEN, "YEN", "Use YEN algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_MEAN, "MEAN", "Select pixels with values above mean value" },
      { c_threshold_routine::THRESHOLD_MINIMUM, "MINIMUM", "Use MINIMUM algorithm to choose the optimal threshold value" },
      { c_threshold_routine::THRESHOLD_PLANETARY_DISK, "PLANETARY_DISK", "" },
      { c_threshold_routine::THRESHOLD_NOISE, "NOISE", "" },
      { c_threshold_routine::THRESHOLD_CLEAR_MASK, "CLEAR_MASK", "" },

      { c_threshold_routine::THRESHOLD_OTSU }
  };

  return members;
}

void c_threshold_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "compare", ctx(&this_class::_compare), "");
   ctlbind(ctls, "threshold_type", ctx(&this_class::_threshold_type), "");
   ctlbind(ctls, "threshold_value", ctx(&this_class::_threshold_value), "");
   ctlbind(ctls, "threshold_scale", ctx(&this_class::_threshold_scale), "");
   ctlbind(ctls, "fill_holes", ctx(&this_class::_fill_holes), "");
   ctlbind(ctls, "invert", ctx(&this_class::_invert), "");
}

bool c_threshold_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _compare);
    SERIALIZE_OPTION(settings, save, *this, _threshold_type);
    SERIALIZE_OPTION(settings, save, *this, _threshold_value);
    SERIALIZE_OPTION(settings, save, *this, _threshold_scale);
    SERIALIZE_OPTION(settings, save, *this, _fill_holes);
    SERIALIZE_OPTION(settings, save, *this, _invert);
    SERIALIZE_OPTION(settings, save, *this, _input_channel);
    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    return true;
  }
  return false;
}


bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat src, dst;

  switch (_input_channel) {
    case DATA_CHANNEL::IMAGE:
      src = image.getMat();
      break;
    case DATA_CHANNEL::MASK:
      src = mask.getMat();
      break;
  }


  if( _threshold_type == THRESHOLD_CLEAR_MASK ) {
    switch (_output_channel) {
      case DATA_CHANNEL::IMAGE:
        image.setTo(cv::Scalar::all(255));
        break;
      case DATA_CHANNEL::MASK:
        mask.release();
        break;
    }
    return true;
  }

  const int cn =
      image.channels();

  if ( _threshold_type == THRESHOLD_PLANETARY_DISK ) {

    bool fOK =
        simple_planetary_disk_detector(src, mask,
            1,
            0.25 * _threshold_scale,
            2,
            nullptr,
            nullptr,
            &dst,
            nullptr,
            nullptr);

    if ( !fOK ) {
      CF_ERROR("simple_planetary_disk_detector() fails");
      return false;
    }

    if ( _fill_holes ) {
      geo_fill_holes(dst, dst, 8);
    }

    switch (_compare) {
      case cv::CMP_GT:
        break;
      case cv::CMP_LT:
        cv::bitwise_not(dst, dst);
        break;
      case cv::CMP_EQ:
        morphological_gradient(dst, dst);
        break;
      case cv::CMP_NE:
        morphological_gradient(dst, dst);
        cv::bitwise_not(dst, dst);
        break;
      case cv::CMP_GE:
        morphological_internal_gradient(dst, dst);
        break;
      case cv::CMP_LE:
        morphological_external_gradient(dst, dst);
        break;
    }
  }
  else {

    std::vector<cv::Mat> channels;

    if( cn == 1 ) {
      channels.emplace_back(src.clone());
    }
    else {
      cv::split(src, channels);
    }

    for ( int i = 0; i < cn; ++i ) {

      const double threshold_value =
          get_threshold_value(channels[i], mask,
              (::THRESHOLD_TYPE)_threshold_type,
              _threshold_value);

      cv::compare(channels[i], threshold_value * _threshold_scale, channels[i],
          _compare);

      if ( _fill_holes ) {
        geo_fill_holes(channels[i], channels[i], 8);
      }

    }

    if( cn == 1 ) {
      dst = channels[0];
    }
    else {
      cv::merge(channels, dst);
    }

    if ( _fill_holes ) {
      geo_fill_holes(dst, dst, 8);
    }

  }


  if ( _invert ) {
    cv::bitwise_not(dst, dst);
  }

  switch (_output_channel) {

    case DATA_CHANNEL::IMAGE:
      image.move(dst);
      break;

    case DATA_CHANNEL::MASK:
      if( _input_channel == DATA_CHANNEL::MASK ) {
        mask.create(dst.size(), CV_8UC1);
        mask.setTo(cv::Scalar::all(0));
        mask.setTo(255, dst != 0);
      }
      else {

        if( dst.channels() != 1 ) {
          reduce_color_channels(dst, dst, cv::REDUCE_MIN);
        }

        if( dst.depth() != CV_8U ) {
          cv::compare(dst, 0, dst, cv::CMP_NE);
        }

        if( mask.empty() ) {
          mask.move(dst);
        }
        else {
          cv::bitwise_and(dst, mask, mask);
        }

      }
      break;
  }


  return true;
}

