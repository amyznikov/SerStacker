/*
 * c_midtones_transfer_function.h
 *
 *  Created on: Dec 16, 2020
 *      Author: amyznikov
 */

#ifndef __c_midtones_transfer_function_h__
#define __c_midtones_transfer_function_h__

#include <opencv2/opencv.hpp>

/** @brief Base class for midtones transfer functions.

The input / output ranges are in absolute units (pixel values).

The shadows / highligths are in relative units ( fractions of input range).

@code

  c_midtones_transfer_function mtf;
  cv::Mat src, dst;

  mtf.set_shadows(0.1);
  mtf.set_highligths(0.9);
  mtf.apply(src, dst);

@endcode
 */
class c_midtones_transfer_function
{
public:
  typedef c_midtones_transfer_function this_class;
  typedef std::shared_ptr<this_class> ptr;

  virtual ~c_midtones_transfer_function() = default;

  static ptr create();

  void set_input_range(double minval, double maxval);
  void get_input_range(double * minval, double * maxval) const;

  void set_output_range(double minval, double maxval);
  void get_output_range(double * minval, double * maxval) const;

  void set_shadows(double v);
  double shadows() const;

  void set_highlights(double v);
  double highlights() const;

  virtual bool apply(cv::InputArray src_image,
      cv::OutputArray dst_image,
      int ddepth = -1) const;

  virtual double apply(double pix) const;


public: // Utility subroutines

  static bool suggest_levels_range(int depth,
      double * minval,
      double * maxval);

protected:
  double input_range_[2] = {-1, -1};
  double output_range_[2] = {-1, -1};
  double shadows_ = 0;
  double highlights_ = 1;

};

#endif /* __c_midtones_transfer_function_h__ */
