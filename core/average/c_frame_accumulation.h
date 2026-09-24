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
  virtual bool reinitialize(cv::InputArray src, cv::InputArray accw) = 0;
  virtual void clear() = 0;

  virtual cv::Size accumulator_size() const = 0;
  virtual cv::Mat get_accumulator() const = 0;
  virtual cv::Mat get_counter() const = 0;

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
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;

  cv::Size accumulator_size() const final {
    return _accumulator.size();
  }
  cv::Mat get_accumulator() const final {
    return _accumulator;
  }
  cv::Mat get_counter() const final {
    return _weights;
  }

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

  int accumulated_frames() const {
    return _accumulated_frames;
  }
  cv::Size accumulator_size() const {
    return _accumulator.size();
  }
  const cv::Mat & accumulator() const {
    return _accumulator;
  }
  const cv::Mat1f & counter() const {
    return _weights;
  }
  const cv::Rect & last_bbox() const {
    return _last_bbox;
  }

  /*
   * Add input frame to canvas. The rmap.size() must be equal to new_canvas_bbox.size().
   * */
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

  const cv::Mat & get_accumulator() const {
    return _accumulator;
  }

  const cv::Mat & get_counter() const {
    return _weights;
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
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;

  cv::Size accumulator_size() const final {
    return _image_size;
  }
  cv::Mat get_accumulator() const final {
    return acc[0];
  }
  cv::Mat get_counter() const final {
    return wwp[0];
  }

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

class c_bayer_drizzle :
    public c_frame_accumulation
{
public:
  typedef c_bayer_drizzle this_class;
  typedef c_frame_accumulation base;
  typedef std::shared_ptr<this_class> ptr;

  enum BAYER_COLOR_ID {
    BAYER_B = 0,
    BAYER_G = 1,
    BAYER_R = 2,
  };

  void set_bayer_pattern(COLORID colorid);
  COLORID bayer_pattern() const;

  void set_pixfrac(double v);
  double pixfrac() const;

  void set_remap(const cv::Mat2f & rmap);
  const cv::Mat2f & remap() const;

  bool add(cv::InputArray src, cv::InputArray weights = cv::noArray()) final;
  bool compute(cv::OutputArray avg, cv::OutputArray mask = cv::noArray(), double dscale = 1.0, int ddepth = -1) const final;
  bool reinitialize(cv::InputArray src, cv::InputArray accw) final;
  void clear() final;

  cv::Size accumulator_size() const final {
    return _accumulator.size();
  }
  cv::Mat get_accumulator() const final {
    return _accumulator;
  }
  cv::Mat get_counter() const final {
    return _counter;
  }

protected:
  void generate_bayer_lookup_mask();

protected:
  int bayer_lookup[2][2];
  cv::Mat3f _accumulator;
  cv::Mat3f _counter;
  cv::Mat2f _rmap;
  double _pixfrac = 1;
  COLORID _colorid = COLORID_UNKNOWN;
};

#endif /* __c_frame_stacking_h__ */
