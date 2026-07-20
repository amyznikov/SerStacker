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
  anscombe_generalized = 2, // 2 * sqrt(g * v + c)
};

struct c_anscombe_transform_options
{
  enum anscombe_method method = anscombe_none;
  struct c_generalized_anscombe_transform_params {
    double g = 1; // stupid default stub
    double c = 3.0 / 8.0; // stupid default stub
    bool auto_estimate = true;
    bool dump_estimated_params = false;
  } generalized;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls,
    const c_ctlbind_context<RootObjectType, c_anscombe_transform_options> & ctx)
{
  using S = c_anscombe_transform_options;
  using OPTS = c_anscombe_transform_options::c_generalized_anscombe_transform_params;

  ctlbind(ctls, "anscombe method", ctx(&S::method), "");
  ctlbind_expandable_group(ctls, "anscombe params", ctx,
      std::function([](const S * s) { return s->method == anscombe_generalized; }),
      [&, ctx = CTL_CONTEXT(ctx, generalized)]() {
        ctlbind(ctls, "auto_estimate", CTL_CONTEXT(ctx, auto_estimate), "");
        ctlbind(ctls, "g", CTL_CONTEXT(ctx, g), "2 * sqrt(g * value + c)");
        ctlbind(ctls, "c", CTL_CONTEXT(ctx, c), "2 * sqrt(g * value + c)");
        ctlbind(ctls, "dump_estimated_params", CTL_CONTEXT(ctx, dump_estimated_params), "");
      });
}


// https://en.wikipedia.org/wiki/Anscombe_transform
class c_anscombe_transform
{
public:
  typedef c_anscombe_transform this_class;

  const c_anscombe_transform_options & opts() const
  {
    return _opts;
  }

  c_anscombe_transform_options & opts()
  {
    return _opts;
  }

  void set_method(enum anscombe_method v)
  {
    _opts.method = v;
  }

  enum anscombe_method method() const
  {
    return _opts.method;
  }

  bool apply(cv::InputArray src, cv::OutputArray dst) const;
  bool inverse(cv::InputArray src, cv::OutputArray dst) const;

  template<class RootObjectType>
  static inline void getcontrols(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, this_class> & ctx)
  {
    ctlbind(ctls, ctx(&this_class::_opts));
  }

protected:
  c_anscombe_transform_options _opts;
  mutable double auto_estimated_g = 1;
  mutable double auto_estimated_c = 3.0 / 8.0;
};


// Generalized anscombe formula:
//  v' = 2 * sqrt(g * v + c)
bool estimateGeneralizedAnscombeParams(const cv::Mat1f & src,
    const cv::Rect & brightRoi, const cv::Rect & darkRoi,
    double & output_g, double & output_c);

bool proposeAnscombeRois(cv::InputArray image,
    cv::Rect & outputBrightRoi,
    cv::Rect & outputDarkRoi,
    int scaleFactor = 16,
    cv::Size roiSizeInSrc = cv::Size(128, 128));

#endif /* __c_anscombe_transform_h__ */
