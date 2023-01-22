/*
 * c_smap_routine.h
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_smap_routine_h__
#define __c_smap_routine_h__

#include "c_image_processor.h"
#include <core/proc/smap.h>

class c_smap_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_smap_routine,
      "smap", "smap");

  void set_lksize(int v)
  {
    lksize_ = v;
  }

  int lksize() const
  {
    return lksize_;
  }

  void set_scale_size(int v)
  {
    scale_size_ = v;
  }

  int scale_size() const
  {
    return scale_size_;
  }

  void set_minv(double v)
  {
    minv_ = v;
  }

  double minv() const
  {
    return minv_;
  }

  void get_parameters(std::vector<struct c_image_processor_routine_ctrl> * ctls) override
  {
    ADD_IMAGE_PROCESSOR_CTRL(ctls, lksize, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, scale_size, "");
    ADD_IMAGE_PROCESSOR_CTRL(ctls, minv, "");
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    if( base::serialize(settings, save) ) {
      SERIALIZE_PROPERTY(settings, save, *this, lksize);
      SERIALIZE_PROPERTY(settings, save, *this, scale_size);
      SERIALIZE_PROPERTY(settings, save, *this, minv);
      return true;
    }
    return false;
  }

  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override
  {
    cv::Mat1f smap;
    compute_smap(image, smap, lksize_, scale_size_, minv_);
    image.move(smap);
    return true;
  }

protected:
  int lksize_ = 7;
  int scale_size_ = 6;
  double minv_ = 1;
};



#endif /* __c_smap_routine_h__ */
