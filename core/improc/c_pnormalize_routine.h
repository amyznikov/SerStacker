/*
 * c_pnormalize_routine.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 *
 *  Subtract local mean and divide by local stdev
 */

#pragma once
#ifndef __c_pnormalize_routine_h__
#define __c_pnormalize_routine_h__

#include "c_image_processor.h"
#include <core/proc/pyrscale.h>

class c_pnormalize_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_pnormalize_routine, "pnormalize",
      "Subtract local mean and divide by stdev");

  void set_scale(int v)
  {
    scale_ = v;
  }

  int scale() const
  {
    return scale_;
  }


  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale, "Gaussian pyramid scale (max level)");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, scale);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    if( scale_ > 0 ) {

      pnormalize(image.getMat(), image.getMatRef(), scale_);
//
//      if( image.depth() != CV_32F ) {
//        image.getMat().convertTo(image, CV_32F);
//      }
//
//
//      pyramid_downscale(image, m, scale_, cv::BORDER_REPLICATE);
//      pyramid_upscale(m, image.size());
//      cv::subtract(image, m, m);
//
//      cv::Scalar mean, stdev;
//      double f = 0;
//
//      cv::meanStdDev(m, mean, stdev, mask);
//
//      for( int i = 0, cn = image.channels(); i < cn; ++i ) {
//        f += stdev[i];
//      }
//
//      cv::multiply(m, cv::Scalar::all(1. / f), image);
    }

    return true;
  }

  static void pnormalize(const cv::Mat & src, cv::Mat & dst, int pscale);

protected:
  // cv::Mat m;
  int scale_ = 3;
};

#endif /* __c_pnormalize_routine_h__ */
