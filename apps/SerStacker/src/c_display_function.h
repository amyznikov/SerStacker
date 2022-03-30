/*
 * c_display_function.h
 *
 *  Created on: Dec 14, 2020
 *      Author: amyznikov
 */

#ifndef __c_display_function_h__
#define __c_display_function_h__

#include <core/mtf/c_pixinsight_mtf.h>

class c_image_display_function
{
public:
  c_image_display_function();

  const c_pixinsight_mtf::sptr & mtf() const;

  void operator ()(const cv::Mat & src,
      cv::Mat & dst,
      int ddepth) const;

protected:
  c_pixinsight_mtf::sptr mtf_;
};

#endif /* __c_display_function_h__ */
