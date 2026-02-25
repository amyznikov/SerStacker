/*
 * c_anscombe_transform.h
 *
 *  Created on: Oct 25, 2020
 *      Author: amyznikov
 */

#ifndef __c_anscombe_transform_h__
#define __c_anscombe_transform_h__

#include <opencv2/opencv.hpp>
#include <core/ctrlbind/ctrlbind.h>

enum anscombe_method
{
  anscombe_none = 0,
  anscombe_native = 1, // formulas from <https://en.wikipedia.org/wiki/Anscombe_transform>
  anscombe_sqrt = 2, // stupid sqrt
};

struct c_anscombe_transform_options
{
  enum anscombe_method method = anscombe_sqrt;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_anscombe_transform_options> & ctx)
{
  using S = c_anscombe_transform_options;
  ctlbind(ctls, "method", ctx(&S::method), "");
}


// https://en.wikipedia.org/wiki/Anscombe_transform
class c_anscombe_transform
{
public:
  typedef c_anscombe_transform this_class;

  void set_method(enum anscombe_method v);
  enum anscombe_method method() const;

  void apply(const cv::Mat & src, cv::Mat & dst) const;
  void inverse(const cv::Mat & src, cv::Mat & dst)  const;

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_anscombe_transform_options _opts;
};


#endif /* __c_anscombe_transform_h__ */
