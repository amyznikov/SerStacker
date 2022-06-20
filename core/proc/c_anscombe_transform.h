/*
 * c_anscombe_transform.h
 *
 *  Created on: Oct 25, 2020
 *      Author: amyznikov
 */

#ifndef __c_anscombe_transform_h__
#define __c_anscombe_transform_h__

#include <opencv2/opencv.hpp>

enum anscombe_method {
  anscombe_none = 0,
  anscombe_native = 1, // formulas from <https://en.wikipedia.org/wiki/Anscombe_transform>
  anscombe_sqrt = 2, // stupid sqrt
};


// https://en.wikipedia.org/wiki/Anscombe_transform
class c_anscombe_transform
{
public:
  void set_method(enum anscombe_method v);
  enum anscombe_method method() const;

  void apply(const cv::Mat & src, cv::Mat & dst) const;
  void inverse(const cv::Mat & src, cv::Mat & dst)  const;

protected:
  enum anscombe_method method_ =
      anscombe_sqrt;
};

#endif /* __c_anscombe_transform_h__ */
