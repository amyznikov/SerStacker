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

class c_weigthed_average :
    public c_frame_accumulation
{
public:
  typedef c_weigthed_average this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  c_weigthed_average();

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool get_acc_counters(cv::Mat & accw) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;

  cv::Size accumulator_size() const final;

  const cv::Mat & accumulator() const;
  const cv::Mat & counter() const;

protected:
  cv::Mat _accumulator;
  cv::Mat1f _weights;
};

class c_canvas_average
{
public:
  typedef c_canvas_average this_class;
  typedef std::shared_ptr<this_class> ptr;

  static const int FRAME_MARGIN = 32;

  void setCanvasSize(const cv::Size & v)
  {
    clear();
    _canvasSize = v;
  }

  const cv::Size & canvasSize() const
  {
    return _canvasSize;
  }

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
    return _weights;
  }

  const cv::Rect & last_bbox() const
  {
    return _last_bbox;
  }

  /*
   * Add input frame to canvas. The rmap.size() must be equal to new_canvas_bbox.size().
   * */
//  bool add(cv::InputArray current_image, cv::InputArray current_weights_or_mask,
//      const cv::Mat2f & rmap, const cv::Point & boxpos = cv::Point());

  bool add(cv::InputArray remapped_current_image, cv::InputArray remapped_current_weights_or_mask,
      const cv::Point & boxpos = cv::Point(0,0));

  /*
   * Return fragment of canvas limited by requested rbbox or full canvas if rbbox is empty
   * */
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1,
      const cv::Rect & rbbox = cv::Rect()) const;

  /* Reset _accumulated_frames and maps to zero, not releasing memory */
  void reset();

  /* Release memory */
  void clear();

  static cv::Size computeCanvasSize(const cv::Size & inputFrameSize);

  template<typename Fn>
  inline auto synchronized(Fn && fn) const
  {
    std::scoped_lock lock(_mtx);
    if constexpr ( std::is_void_v<std::invoke_result_t<Fn>> ) {
      std::forward<Fn>(fn)();
    }
    else {
      return std::forward<Fn>(fn)();
    }
  }

protected:
  void maintainCanvasBoundaries(cv::Rect & bbox);

protected:
  mutable std::mutex _mtx;
  cv::Mat _accumulator;
  cv::Mat1f _weights;
  cv::Rect _last_bbox;
  cv::Size _canvasSize;
  int _accumulated_frames = 0;
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


#endif /* __c_frame_stacking_h__ */
