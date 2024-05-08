/*
 * c_gaussian_pyramid_routine.h
 *
 *  Created on: Jul 11, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_pyrdown_routine_h__
#define __c_pyrdown_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/unsharp_mask.h>

class c_gaussian_pyramid_routine:
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_gaussian_pyramid_routine, "gaussian_pyramid",
      "Calls <strong>cv::pyrDown()</strong> or <strong>cv::pyrUp()</strong> on image");

  enum UnsharpMaskOrder {
    UnsharpMaskNone,
    UnsharpMaskBefore,
    UnsharpMaskAfter,
    UnsharpMaskEachIteration,
  };

  void set_count(int v)
  {
    count_ = v;
  }

  int count() const
  {
    return count_;
  }

  void set_borderType(cv::BorderTypes v)
  {
    borderType_ = v;
  }

  cv::BorderTypes borderType() const
  {
    return borderType_;
  }

  void set_unsharp_sigma(double v)
  {
    unsharp_sigma_ = v;
  }

  double unsharp_sigma() const
  {
    return unsharp_sigma_;
  }

  void set_unsharp_alpha(double v)
  {
    unsharp_alpha_ = v;
  }

  double unsharp_alpha() const
  {
    return unsharp_alpha_;
  }

  void set_unsharp_outmin(double v)
  {
    unsharp_outmin_ = v;
  }

  double unsharp_outmin() const
  {
    return unsharp_outmin_;
  }

  void set_unsharp_outmax(double v)
  {
    unsharp_outmax_ = v;
  }

  double unsharp_outmax() const
  {
    return unsharp_outmax_;
  }

  void set_unsharp_mask_order(UnsharpMaskOrder v)
  {
    unsharp_mask_order_ = v;
  }

  UnsharpMaskOrder unsharp_mask_order() const
  {
    return unsharp_mask_order_;
  }

  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

protected:
  int count_ = 1;
  cv::BorderTypes borderType_ = cv::BORDER_DEFAULT;
  UnsharpMaskOrder unsharp_mask_order_ = UnsharpMaskNone;
  double unsharp_sigma_ = 1.5;
  double unsharp_alpha_ = 0.0;
  double unsharp_outmin_ = -1;
  double unsharp_outmax_ = -1;
};

#endif /* __c_pyrdown_routine_h__ */
