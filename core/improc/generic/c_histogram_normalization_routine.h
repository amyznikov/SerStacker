/*
 * c_histogram_normalization_routine.h
 *
 *  Created on: Jun 19, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_histogram_normalization_routine_h__
#define __c_histogram_normalization_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/white_balance/histogram_normalization.h>
//#include <core/io/debayer.h>

class c_histogram_normalization_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_histogram_normalization_routine,
      "histogram_normalization", "histogram normalization");

  void set_normalization_type(histogram_normalization_type v)
  {
    normalization_type_ = v;
  }

  histogram_normalization_type normalization_type() const
  {
    return normalization_type_;
  }

  void set_colorid(COLORID v)
  {
    colorid_ = v;
  }

  COLORID colorid() const
  {
    return colorid_;
  }

  void set_stretch(const cv::Scalar & v)
  {
    stretch_ = v;
  }

  const cv::Scalar & stretch() const
  {
    return stretch_;
  }

  void set_offset(const cv::Scalar & v)
  {
    offset_ = v;
  }

  const cv::Scalar & offset() const
  {
    return offset_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override
  {
    BIND_PCTRL(ctls, normalization_type, "normalization_type");
    BIND_PCTRL(ctls, colorid, "colorid");
    BIND_PCTRL(ctls, stretch, "stretch");
    BIND_PCTRL(ctls, offset, "offset");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, normalization_type);
      SERIALIZE_PROPERTY(settings, save, *this, colorid);
      SERIALIZE_PROPERTY(settings, save, *this, stretch);
      SERIALIZE_PROPERTY(settings, save, *this, offset);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  COLORID colorid_ = COLORID_UNKNOWN;
  histogram_normalization_type normalization_type_ = histogram_normalize_mean;
  cv::Scalar offset_ = cv::Scalar::all(0);
  cv::Scalar stretch_ = cv::Scalar::all(1);
};

#endif /* __c_histogram_normalization_routine_h__ */
