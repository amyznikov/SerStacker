/*
 * c_frame_accumulation.h
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#ifndef __c_frame_stacking_h__
#define __c_frame_stacking_h__

#include <opencv2/opencv.hpp>
#include <core/io/debayer.h>

class c_frame_accumulation
{
public:
  typedef c_frame_accumulation this_class;
  typedef std::shared_ptr<this_class> ptr;

  virtual ~c_frame_accumulation() = default;

  virtual bool add(cv::InputArray src, cv::InputArray mask = cv::noArray()) = 0;
  virtual bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const = 0;
  virtual bool get_acc_counters(cv::Mat & accw) const = 0;
  virtual bool reinitialize(cv::InputArray src, cv::InputArray accw) = 0;
  virtual void clear() = 0;

  virtual cv::Size accumulator_size() const = 0;

  int accumulated_frames() const
  {
    return _accumulated_frames;
  }

protected:
  int _accumulated_frames = 0;
};

class c_frame_weigthed_average :
    public c_frame_accumulation
{
public:
  typedef c_frame_weigthed_average this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  c_frame_weigthed_average();
  c_frame_weigthed_average(double max_weights_ratio);


  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool get_acc_counters(cv::Mat & accw) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;
  cv::Size accumulator_size() const final;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;
  const cv::Mat & max_weights() const;

  void set_max_weights_ratio(double v);
  double max_weights_ratio() const;

protected:
  cv::Mat _accumulator, _counter, _max_weights;
  double _max_weights_ratio = 0;
};

class c_laplacian_pyramid_focus_stacking :
    public c_frame_accumulation
{
public:
  typedef c_laplacian_pyramid_focus_stacking this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  enum fusing_policy {
    select_max_energy,
    weighted_average
  };

  struct options {
    enum fusing_policy fusing_policy = select_max_energy;
    bool inpaint_mask_holes = true;
    bool avgchannel = true;
    int kradius = 0;
    double ksigma = 0;
  };

  c_laplacian_pyramid_focus_stacking(const options & opts);

  bool add(cv::InputArray src, cv::InputArray mask = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool get_acc_counters(cv::Mat & accw) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;
  cv::Size accumulator_size() const final;

protected:
  static cv::Mat duplicate_channels(const cv::Mat & src, int cn);

protected:
  options _opts;
  std::vector<cv::Mat> acc;
  std::vector<cv::Mat> wwp;
  cv::Size _image_size;
  int _acctype = CV_32F;
  int _weightstype = CV_8U;
  cv::Mat1f G;
};


class c_frame_accumulation_with_fft :
    public c_frame_accumulation
{
public:
  typedef c_frame_accumulation_with_fft this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool get_acc_counters(cv::Mat & accw) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;
  cv::Size accumulator_size() const final;

  const std::vector<cv::Mat> & accumulators() const;
  const std::vector<cv::Mat> & weights() const;

protected:
  static int countNaNs(const cv::Mat & image);
  static bool fftPower(const cv::Mat & src, cv::Mat & dst, bool mc);
  static double power(double x);
  static double square(double x);

protected:
  std::vector<cv::Mat> _accumulators;
  std::vector<cv::Mat> _weights;
  cv::Rect _rc;
  cv::Size _fftSize;
  int _border_top = 0;
  int _border_bottom = 0;
  int _border_left = 0;
  int _border_right = 0;
};


class c_bayer_average :
    public c_frame_accumulation
{
public:
  typedef c_bayer_average this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  enum BAYER_COLOR_ID {
    BAYER_B = 0,
    BAYER_G = 1,
    BAYER_R = 2,
  };


  void set_bayer_pattern(COLORID colorid);
  COLORID bayer_pattern() const;

  void set_remap(const cv::Mat2f & rmap);
  const cv::Mat2f & remap() const;

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool get_acc_counters(cv::Mat & accw) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;
  cv::Size accumulator_size() const final;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;

protected:
  void generate_bayer_pattern_mask();

protected:
  cv::Mat1b _bayer_pattern;
  cv::Mat3f _accumulator;
  cv::Mat3f _counter;
  cv::Mat2f _rmap;
  COLORID _colorid = COLORID_UNKNOWN;
};


class c_running_frame_average
{
public:
  typedef c_running_frame_average this_class;
  typedef std::shared_ptr<this_class> ptr;

  int accumulated_frames() const
  {
    return _accumulated_frames;
  }

  cv::Size accumulator_size() const
  {
    return _accumulator.size();
  }

  const cv::Mat & accumulator() const
  {
    return _accumulator;
  }

  const cv::Mat1f & counter() const
  {
    return _counter;
  }

  bool remap(const cv::Mat2f & rmap);
  bool add(cv::InputArray current_image, cv::InputArray current_mask,  double w, const cv::Mat2f * rmap = nullptr);
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const;
  void clear();

protected:
  cv::Mat _accumulator;
  cv::Mat1f _counter;
  int _accumulated_frames = 0;
};

#endif /* __c_frame_stacking_h__ */
