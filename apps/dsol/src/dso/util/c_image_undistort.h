/**
 * This file is derived from DSO source codes
 *
 */

#pragma once
#ifndef __c_image_undistort_h__
#define __c_image_undistort_h__

#include "../c_image_and_exposure.h"
#include "util/NumType.h"

namespace dso
{

class c_photometric_undistort
{
public:
  typedef c_photometric_undistort this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class * load(const std::string & calibfile,
      const std::string & noiseImage,
      const std::string & vignetteImage,
      int w_, int h_);

  template<typename T>
  void processFrame(const T * image_in, float exposure_time,
      float factor = 1);

  void unMapFloatImage(float * image);

  const float* photometricGamma() const
  {
    return G_.empty() ? nullptr : G_.data();
  }

  c_image_and_exposure * output() const
  {
    return output_.get();
  }

protected:
  c_image_and_exposure::uptr output_;
  std::vector<float> G_;
  cv::Mat1f vignetteMapInv_;
  int w = 0;
  int h = 0;
};

/**
 * FIXME: replace everything below with cv::remap()
 */
class c_image_undistort
{
public:
  typedef c_image_undistort this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  virtual ~c_image_undistort()
  {
  }

  inline const Mat33 getK() const
  {
    return K;
  }

  inline const Eigen::Vector2i getSize() const
  {
    return Eigen::Vector2i(w, h);
  }

  inline const VecX getOriginalParameter() const
  {
    return parsOrg;
  }

  inline const Eigen::Vector2i getOriginalSize()
  {
    return Eigen::Vector2i(wOrg, hOrg);
  }

  const float * photometricGamma() const
  {
    return photometricUndist_ ? photometricUndist_->photometricGamma() : nullptr;
  }

  static this_class* load(const std::string & configFilename, const std::string & gammaFilename,
      const std::string & vignetteFilename);

  bool undistort(const cv::Mat & image_raw, c_image_and_exposure * output_image,
      float exposure = 0, double timestamp = 0, float factor = 1) const;

protected:

  virtual void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const = 0;

  bool load_photometric_calibration(const std::string & file, const std::string & noiseImage,
      const std::string &vignetteImage);

  bool load_parameters(const std::string & configFileName, int nPars,
      const std::string & prefix);

  void undistort_impl(const float in_data[],
      float out_data[]) const;


protected:
  c_photometric_undistort::uptr photometricUndist_;
  int w, h, wOrg, hOrg, wUp, hUp;
  int upsampleUndistFactor;
  Mat33 K;
  VecX parsOrg;
  bool passthrough = false;

  std::vector<float> remapX;
  std::vector<float> remapY;

  void applyBlurNoise(float * img) const;

  bool makeOptimalK_crop();
  bool makeOptimalK_full();

};

class c_image_undistort_fov:
    public c_image_undistort
{
public:
  typedef c_image_undistort_fov this_class;
  typedef c_image_undistort base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class * load(const std::string & configFileName,
      bool noprefix);

protected:
  void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const override;
};

class c_image_undistort_radtan:
    public c_image_undistort
{
public:
  typedef c_image_undistort_radtan this_class;
  typedef c_image_undistort base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class * load(const char * configFileName,
      bool noprefix);

protected:
  void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const override;

};

class c_image_undistort_equidistant:
    public c_image_undistort
{
public:
  typedef c_image_undistort_equidistant this_class;
  typedef c_image_undistort base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class * load(const char * configFileName,
      bool noprefix);

protected:
  void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const override;
};

class c_image_undistort_pinhole:
    public c_image_undistort
{
public:
  typedef c_image_undistort_pinhole this_class;
  typedef c_image_undistort base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class* load(const char * configFileName,
      bool noprefix);

protected:
  void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const override;
};

class c_image_undistort_kb:
    public c_image_undistort
{
public:
  typedef c_image_undistort_kb this_class;
  typedef c_image_undistort base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  static this_class * load(const char * configFileName, bool noprefix);

protected:
  void distort(float * in_x, float * in_y,
      float * out_x, float * out_y,
      int n) const override;

};

}

#endif // __c_image_undistort_h__
