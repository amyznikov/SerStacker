/*
 * c_pixinsight_mtf.h
 *
 *  Created on: Dec 16, 2020
 *      Author: amyznikov
 */

#ifndef __c_pixinsight_midtones_transfer_function_h__
#define __c_pixinsight_midtones_transfer_function_h__

#include "c_midtones_transfer_function.h"

/** @brief Pixinsight midtones transfer functions.

See section 'Midtones balance' at
  <http://pixinsight.com/doc/tools/HistogramTransformation/HistogramTransformation.html>

@code

  c_pixinsight_midtones_transfer_function mtf;
  cv::Mat src, dst;

  mtf.set_shadows(0.1);
  mtf.set_highligths(0.9);
  mtf.set_midtones(0.5);
  mtf.apply(src, dst);

@endcode
 */
class c_pixinsight_mtf
    : public c_midtones_transfer_function
{
public:
  typedef c_pixinsight_mtf this_class;
  typedef c_midtones_transfer_function base;
  typedef std::shared_ptr<this_class> sptr;

  static sptr create();

  void set_midtones(double v);
  double midtones() const;

  bool apply(cv::InputArray src_image,
      cv::OutputArray dst_image,
      int ddepth = -1) const override;

  double apply(double pix) const override;

  bool find_midtones_balance(cv::InputArray input_image_histogram);

  static bool find_midtones_balance(cv::InputArray input_image_histogram,
      double * output_shadows,
      double * output_highlights,
      double * output_midtones);

protected:
  double midtones_ = 0.5;
};

#endif /* __c_pixinsight_midtones_transfer_function_h__ */
