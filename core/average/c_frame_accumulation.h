/*
 * c_frame_accumulation.h
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_frame_stacking_h__
#define __c_frame_stacking_h__

#include <opencv2/opencv.hpp>

class c_frame_accumulation
{
public:
  typedef c_frame_accumulation this_class;
  typedef std::shared_ptr<this_class> ptr;

  virtual ~c_frame_accumulation() = default;

  virtual bool initialze(const cv::Size & image_size, int acctype, int weightstype) = 0;
  virtual bool add(cv::InputArray src, cv::InputArray mask = cv::noArray()) = 0;
  virtual bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const = 0;
  virtual void release() = 0;
  virtual cv::Size accumulator_size() const = 0;

  int accumulated_frames() const {
    return accumulated_frames_;
  }

protected:
  int accumulated_frames_ = 0;
};

class c_frame_weigthed_average :
    public c_frame_accumulation
{
public:
  typedef c_frame_weigthed_average this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  static constexpr int accdepth = CV_32F;

  c_frame_weigthed_average();
  c_frame_weigthed_average(const cv::Size & image_size, int acctype, int weightstype);

  bool initialze(const cv::Size & image_size, int acctype, int weightstype) override;
  bool add(cv::InputArray src, cv::InputArray mask = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void release() override;
  cv::Size accumulator_size() const override;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;

protected:
  cv::Mat accumulator_, counter_;
};


class c_frame_accumulation_with_fft :
    public c_frame_accumulation
{
public:
  typedef c_frame_accumulation_with_fft this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  bool initialze(const cv::Size & image_size, int acctype, int weightstype) override;
  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void release() override;
  cv::Size accumulator_size() const override;

  const std::vector<cv::Mat> & accumulators() const;
  const std::vector<cv::Mat> & weights() const;

protected:
  static int countNaNs(const cv::Mat & image);
  static bool fftPower(const cv::Mat & src, cv::Mat & dst, bool mc);
  static double power(double x);
  static double square(double x);

protected:
  std::vector<cv::Mat> accumulators_;
  std::vector<cv::Mat> weights_;
  cv::Rect rc_;
  cv::Size fftSize_;
  int border_top_ = 0;
  int border_bottom_ = 0;
  int border_left_ = 0;
  int border_right_ = 0;
};

#endif /* __c_frame_stacking_h__ */
