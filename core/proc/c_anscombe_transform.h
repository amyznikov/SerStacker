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

const extern struct anscombe_method_desc {
  const char * name;
  enum anscombe_method value;
} anscombe_methods[];

std::string toStdString(enum anscombe_method m);
enum anscombe_method fromStdString(const std::string & s,
    enum anscombe_method defval);


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
