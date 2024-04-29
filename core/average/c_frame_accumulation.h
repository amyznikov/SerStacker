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
  virtual void clear() = 0;

  virtual cv::Size accumulator_size() const = 0;

  int accumulated_frames() const
  {
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

  c_frame_weigthed_average();

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void clear() override;
  cv::Size accumulator_size() const override;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;

protected:
  cv::Mat accumulator_, counter_;
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

  bool add(cv::InputArray src, cv::InputArray mask = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void clear() override;
  cv::Size accumulator_size() const override;

protected:
  static cv::Mat duplicate_channels(const cv::Mat & src, int cn);

protected:
  options opts_;
  std::vector<cv::Mat> acc;
  std::vector<cv::Mat> wwp;
  cv::Size image_size_;
  int acctype_ = CV_32F;
  int weightstype_ = CV_8U;
  cv::Mat1f G;
};


class c_frame_accumulation_with_fft :
    public c_frame_accumulation
{
public:
  typedef c_frame_accumulation_with_fft this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void clear() override;
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

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) override;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const override;
  void clear() override;
  cv::Size accumulator_size() const override;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;

protected:
  void generate_bayer_pattern_mask();

protected:
  cv::Mat1b bayer_pattern_;
  cv::Mat3f accumulator_;
  cv::Mat3f counter_;
  cv::Mat2f rmap_;
  COLORID colorid_ = COLORID_UNKNOWN;
};


class c_running_frame_average
{
public:
  typedef c_running_frame_average this_class;
  typedef std::shared_ptr<this_class> ptr;

  int accumulated_frames() const
  {
    return accumulated_frames_;
  }

  cv::Size accumulator_size() const
  {
    return accumulator_.size();
  }

  const cv::Mat & accumulator() const
  {
    return accumulator_;
  }

  const cv::Mat1f & counter() const
  {
    return counter_;
  }

  bool remap(const cv::Mat2f & rmap);
  bool add(cv::InputArray current_image, cv::InputArray current_mask,  double w, const cv::Mat2f * rmap = nullptr);
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const;
  void clear();


protected:
  cv::Mat accumulator_;
  cv::Mat1f counter_;
  int accumulated_frames_ = 0;
};


#endif /* __c_frame_stacking_h__ */
