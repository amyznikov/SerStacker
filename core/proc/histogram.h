/*
 * histogram.h
 *
 *  Created on: Aug 14, 2021
 *      Author: amyznikov
 */

#ifndef ___histogram_h___
#define ___histogram_h___

#include <opencv2/opencv.hpp>

class c_histogram_builder
{
public:
  void set_channels(int cn);
  int channels() const;

  void set_input_range(double minval, double maxval);
  void get_input_range(double *minval, double *maxval) const;

  void set_bins(int nbins);
  int bins() const;

  void set_cumulative(bool v);
  bool cumulative() const;

  void set_scaled(bool v);
  bool scaled() const;

  void set_logscale(bool v);
  bool logscale() const;

  void reset();
  void add_pixel(const cv::Scalar & pix);
  void compute(cv::OutputArray H);

protected:
  void initialize();

protected:
  cv::Mat1f H_;
  double scale_ = 0;
  double minval_ = -1;
  double maxval_ = - 1;
  int bins_ = 256;
  int channels_ = 1;
  bool cumulative_ = false;
  bool scaled_ = false;
  bool logscale_ = false;
};


// @brief build histogram for given multi-channel image.
// Output is single-channel CV_32FC1 matrix of size 'nbins rows' x 'image channels columns'.
// if input mask is not empty then it must be sigle-channel CV_8U matrix of the same size as input image.
bool create_histogram(cv::InputArray image,
    cv::InputArray mask,
    cv::OutputArray dst,
    /*[in, out]*/ double * minval,
    /*[in, out]*/ double * maxval,
    int nbins = -1,
    bool cumulative = false,
    bool scaled = false);

/// @brief  convert conventional image histogram H into cumulative
///         by accumulating the bin values along rows
bool accumulate_histogram(cv::InputArray H,
    cv::OutputArray CH);



#endif /* ___histogram_h___ */
