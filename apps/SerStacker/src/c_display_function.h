/*
 * c_display_function.h
 *
 *  Created on: Dec 14, 2020
 *      Author: amyznikov
 */

#ifndef __c_display_function_h__
#define __c_display_function_h__

#include <core/histogram/c_pixinsight_midtones_transfer_function.h>

class c_display_function
{
public:
  c_display_function();

  const c_pixinsight_midtones_transfer_function::ptr & mtf() const;

  void operator ()(const cv::Mat & src,
      cv::Mat & dst,
      int ddepth) const;

protected:
  c_pixinsight_midtones_transfer_function::ptr mtf_;
};

#endif /* __c_display_function_h__ */
