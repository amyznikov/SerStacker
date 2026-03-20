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

template<>
const c_enum_member* members_of<c_threshold_routine::MASK_MODE>()
{
  static const c_enum_member members[] = {
      { c_threshold_routine::MASK_MODE_REPLACE, "REPLACE", ""},
      { c_threshold_routine::MASK_MODE_AND, "AND", ""},
      { c_threshold_routine::MASK_MODE_OR, "OR", ""},
      { c_threshold_routine::MASK_MODE_XOR, "XOR", ""},
      { c_threshold_routine::MASK_MODE_NAND, "NAND", ""},
      { c_threshold_routine::MASK_MODE_NOR, "NOR", ""},
      { c_threshold_routine::MASK_MODE_NXOR, "NXOR", ""},

      { c_threshold_routine::MASK_MODE_REPLACE }
  };

  return members;
}

template<>
const c_enum_member* members_of<c_threshold_routine::MULTI_CHANEL_REDUCTION>()
{
  static const c_enum_member members[] = {
      { c_threshold_routine::MULTI_CHANEL_REDUCTION_NONE, "NONE", ""},
      { c_threshold_routine::MULTI_CHANEL_REDUCTION_MAX, "MAX", ""},
      { c_threshold_routine::MULTI_CHANEL_REDUCTION_MIN, "MIN", ""},
      { c_threshold_routine::MULTI_CHANEL_REDUCTION_MIN }
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
   ctlbind(ctls, "channel reduction", ctx(&this_class::_reduction_mode), "");
   ctlbind(ctls, "mask_mode", ctx(&this_class::_mask_mode), "");
   ctlbind(ctls, "input", ctx(&this_class::_input_channel), "");
   ctlbind(ctls, "output", ctx(&this_class::_output_channel), "");
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
    SERIALIZE_OPTION(settings, save, *this, _reduction_mode);
    SERIALIZE_OPTION(settings, save, *this, _mask_mode);
    SERIALIZE_OPTION(settings, save, *this, _input_channel);
    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    return true;
  }
  return false;
}


bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
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

  cv::Mat src, srcm, dstm;

  switch (_input_channel) {
    case DATA_CHANNEL::IMAGE:
      src = image.getMat();
      srcm = mask.getMat();
      break;
    case DATA_CHANNEL::MASK:
      src = mask.getMat();
      break;
  }


  if ( _threshold_type != THRESHOLD_PLANETARY_DISK ) {

    std::vector<cv::Mat> channels;

    cv::split(src, channels);

    for( int i = 0, cn = src.channels(); i < cn; ++i ) {

      const double threshold_value =
          get_threshold_value(channels[i], mask,
              (::THRESHOLD_TYPE) (_threshold_type),
              _threshold_value);

      cv::compare(channels[i], threshold_value * _threshold_scale,
          channels[i],
          _compare);

      if( _fill_holes ) {
        geo_fill_holes(channels[i], channels[i], 8);
      }
    }

    cv::merge(channels, dstm);

  }
  else {

    if( srcm.channels() > 1 ) {
      reduce_color_channels(srcm, cv::REDUCE_MAX);
    }

    const bool fOK =
        simple_planetary_disk_detector(src, srcm,
            1,
            0.25 * _threshold_scale,
            2,
            nullptr,
            nullptr,
            &dstm,
            nullptr,
            nullptr);

    if( !fOK ) {
      CF_ERROR("simple_planetary_disk_detector() fails");
      return false;
    }

    if( _fill_holes ) {
      geo_fill_holes(dstm, dstm, 8);
    }

    switch (_compare) {
      case cv::CMP_GT:
        break;
      case cv::CMP_LT:
        cv::bitwise_not(dstm, dstm);
        break;
      case cv::CMP_EQ:
        morphological_gradient(dstm, dstm);
        break;
      case cv::CMP_NE:
        morphological_gradient(dstm, dstm);
        cv::bitwise_not(dstm, dstm);
        break;
      case cv::CMP_GE:
        morphological_internal_gradient(dstm, dstm);
        break;
      case cv::CMP_LE:
        morphological_external_gradient(dstm, dstm);
        break;
    }
  }


  if( dstm.channels() > 1 ) {
    switch (_reduction_mode) {
      case MULTI_CHANEL_REDUCTION_MAX:
        reduce_color_channels(dstm, cv::REDUCE_MAX);
        break;
      case MULTI_CHANEL_REDUCTION_MIN:
        reduce_color_channels(dstm, cv::REDUCE_MIN);
        break;
      default:
        break;
    }
  }

  if ( _invert ) {
    cv::bitwise_not(dstm, dstm);
  }

  if( !srcm.empty() ) {

    if( srcm.channels() != dstm.channels() ) {
      if( dstm.channels() == 1 ) {
        cv::merge(std::vector<cv::Mat>(srcm.channels(), dstm), dstm);
      }
      else if( srcm.channels() == 1 ) {
        cv::merge(std::vector<cv::Mat>(srcm.channels(), srcm), srcm);
      }
      else {
        CF_ERROR("Not supported combination of srcm.channels=%d and dstm,channels=%d",
            srcm.channels(), dstm.channels());
        return false;
      }
    }

    switch (_mask_mode) {
      case MASK_MODE_REPLACE:
        break;
      case MASK_MODE_NAND:
        cv::bitwise_and(~srcm, dstm, dstm);
        break;
      case MASK_MODE_AND:
        cv::bitwise_and(srcm, dstm, dstm);
        break;
      case MASK_MODE_NOR:
        cv::bitwise_or(~srcm, dstm, dstm);
        break;
      case MASK_MODE_OR:
        cv::bitwise_or(srcm, dstm, dstm);
        break;
      case MASK_MODE_NXOR:
        cv::bitwise_xor(~srcm, dstm, dstm);
        break;
      case MASK_MODE_XOR:
        cv::bitwise_xor(srcm, dstm, dstm);
        break;
      default:
        break;
    }
  }

  switch (_output_channel) {
    case DATA_CHANNEL::IMAGE:
      image.move(dstm);
      break;
    case DATA_CHANNEL::MASK:
      mask.move(dstm);
      break;
  }

  return true;
}

//
//bool c_threshold_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
//{
//  if( _threshold_type == THRESHOLD_CLEAR_MASK ) {
//    switch (_output_channel) {
//      case DATA_CHANNEL::IMAGE:
//        image.setTo(cv::Scalar::all(255));
//        break;
//      case DATA_CHANNEL::MASK:
//        mask.release();
//        break;
//    }
//    return true;
//  }
//
//  cv::Mat src, srcm, dst, dstm;
//
//  if ( _threshold_type == THRESHOLD_PLANETARY_DISK ) {
//
//    src = image.getMat();
//    srcm = mask.getMat();
//
//    bool fOK =
//        simple_planetary_disk_detector(src, mask,
//            1,
//            0.25 * _threshold_scale,
//            2,
//            nullptr,
//            nullptr,
//            &dst,
//            nullptr,
//            nullptr);
//
//    if ( !fOK ) {
//      CF_ERROR("simple_planetary_disk_detector() fails");
//      return false;
//    }
//
//    if ( _fill_holes ) {
//      geo_fill_holes(dst, dst, 8);
//    }
//
//    switch (_compare) {
//      case cv::CMP_GT:
//        break;
//      case cv::CMP_LT:
//        cv::bitwise_not(dst, dst);
//        break;
//      case cv::CMP_EQ:
//        morphological_gradient(dst, dst);
//        break;
//      case cv::CMP_NE:
//        morphological_gradient(dst, dst);
//        cv::bitwise_not(dst, dst);
//        break;
//      case cv::CMP_GE:
//        morphological_internal_gradient(dst, dst);
//        break;
//      case cv::CMP_LE:
//        morphological_external_gradient(dst, dst);
//        break;
//    }
//
//  }
//
//  else {
//
//    std::vector<cv::Mat> channels;
//
//    switch (_input_channel) {
//      case DATA_CHANNEL::IMAGE:
//        src = image.getMat();
//        srcm = mask.getMat();
//        break;
//      case DATA_CHANNEL::MASK:
//        src = mask.getMat();
//        break;
//    }
//
//    const int cn = src.channels();
//    const int mcn = srcm.channels();
//
//
//
//    if( cn == 1 ) {
//      channels.emplace_back(src.clone());
//    }
//    else {
//      cv::split(src, channels);
//    }
//
//    for ( int i = 0; i < cn; ++i ) {
//
//      const double threshold_value =
//          get_threshold_value(channels[i], mask,
//              (::THRESHOLD_TYPE)_threshold_type,
//              _threshold_value);
//
//      cv::compare(channels[i], threshold_value * _threshold_scale, channels[i],
//          _compare);
//
//      if ( _fill_holes ) {
//        geo_fill_holes(channels[i], channels[i], 8);
//      }
//
//    }
//
//    if( cn == 1 ) {
//      dst = channels[0];
//    }
//    else {
//      cv::merge(channels, dst);
//    }
//
//    if ( _fill_holes ) {
//      geo_fill_holes(dst, dst, 8);
//    }
//
//  }
//
//
//  if ( _invert ) {
//    cv::bitwise_not(dst, dst);
//  }
//
//  switch (_output_channel) {
//
//    case DATA_CHANNEL::IMAGE:
//      image.move(dst);
//      break;
//
//    case DATA_CHANNEL::MASK:
//      if( _input_channel == DATA_CHANNEL::MASK ) {
//        mask.create(dst.size(), CV_8UC1);
//        mask.setTo(cv::Scalar::all(0));
//        mask.setTo(255, dst != 0);
//      }
//      else {
//
//        if( dst.channels() != 1 ) {
//          reduce_color_channels(dst, dst, cv::REDUCE_MIN);
//        }
//
//        if( dst.depth() != CV_8U ) {
//          cv::compare(dst, 0, dst, cv::CMP_NE);
//        }
//
//        if( mask.empty() ) {
//          mask.move(dst);
//        }
//        else {
//          cv::bitwise_and(dst, mask, mask);
//        }
//
//      }
//      break;
//  }
//
//
//  return true;
//}

