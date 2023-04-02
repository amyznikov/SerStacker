/*
 * c_regular_stereo_matcher.h
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 *
 * https://learnopencv.com/depth-perception-using-stereo-camera-python-c
 * https://amroamroamro.github.io/mexopencv/opencv/stereo_match_demo.html
 * https://docs.opencv.org/3.4/dd/d53/tutorial_py_depthmap.html
 */

#pragma once
#ifndef __c_regular_stereo_matcher_h__
#define __c_regular_stereo_matcher_h__

// Must come from CMakeLists.txt
// #define HAVE_OpenCV_stereo 1

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#if HAVE_OpenCV_stereo
# include <opencv2/stereo.hpp>
#endif

#include "c_scale_sweep_stereo_matcher.h"
#include <core/settings/opencv_settings.h>

enum stereo_matcher_type  {
  stereo_matcher_cvStereoBM,
  stereo_matcher_cvStereoSGBM,
  stereo_matcher_ScaleSweep,
#if HAVE_OpenCV_stereo
  stereo_matcher_QuasiDenseStereo,
#endif
};

enum StereoBM_PreFilterType
{
  StereoBM_PREFILTER_NORMALIZED_RESPONSE = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE,
  StereoBM_PREFILTER_XSOBEL = cv::StereoBM::PREFILTER_XSOBEL
};

struct c_cvStereoBM_options
{
  int minDisparity = 0;
  int numDisparities = 128;
  int blockSize = 11;

  int speckleWindowSize = 0;
  int speckleRange = 0;
  int disp12MaxDiff = 0;

  StereoBM_PreFilterType preFilterType = StereoBM_PREFILTER_NORMALIZED_RESPONSE;
  int preFilterSize = 5;
  int preFilterCap = 1;
  int textureThreshold = 0;
  int uniquenessRatio = 0;
  int smallerBlockSize = 0;

  cv::Rect roi1;
  cv::Rect roi2;
};

enum StereoSGBM_Mode
{
  StereoSGBM_SGBM = cv::StereoSGBM::MODE_SGBM,
  StereoSGBM_HH = cv::StereoSGBM::MODE_HH,
  StereoSGBM_SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY,
  StereoSGBM_HH4 = cv::StereoSGBM::MODE_HH4
};


struct c_cvStereoSGBM_options
{
  int minDisparity = 0;
  int numDisparities = 80;
  int blockSize = 11;

  int speckleWindowSize = 1;
  int speckleRange = 1;
  int disp12MaxDiff = 0;

  int P1 = 0;
  int P2 = 256;
  int preFilterCap = 1;
  int uniquenessRatio = 0;
  StereoSGBM_Mode mode = StereoSGBM_SGBM;
};

struct c_ScaleSweep_options
{
  int max_disparity = 128;
  int max_scale = 2;
  double kernel_sigma = 1;
  int kernel_radius = 3;
  int normalization_scale = 0;

  std::string debug_directory;
  std::vector<cv::Point> debug_points;
};

class c_regular_stereo_matcher
{
public:
  c_regular_stereo_matcher();

  void set_matcher_type(stereo_matcher_type v);
  stereo_matcher_type matcher_type() const;

  const c_cvStereoBM_options & StereoBMOptions() const ;
  c_cvStereoBM_options & StereoBMOptions();
  void updateStereoBMOptions();

  const c_cvStereoSGBM_options & StereoSGBMOptions() const;
  c_cvStereoSGBM_options & StereoSGBMOptions();
  void updateStereoSGBMOptions();

#if HAVE_OpenCV_stereo
  const cv::stereo::PropagationParameters & quasiDenseStereoOptions() const;
  cv::stereo::PropagationParameters & quasiDenseStereoOptions();
  void updateQuasiDenseStereoOptions();
#endif

  const c_ScaleSweep_options & cScaleSweepOptions() const;
  c_ScaleSweep_options & ScaleSweepOptions();
  void updateScaleSweepOptions();

  double currentMaxDisparity() const;

  int currentReferenceImageIndex() const;

  bool compute( cv::InputArray left, cv::InputArray right,
      cv::OutputArray disparity);

  bool serialize(c_config_setting settings, bool save);


protected:
  bool create_stereo_matcher(const cv::Size & image_size);

protected:
  stereo_matcher_type matcher_type_ = stereo_matcher_cvStereoBM;
  cv::Ptr<cv::StereoMatcher> stereoMatcher_;

  c_cvStereoBM_options cvStereoBM_options_;
  c_cvStereoSGBM_options cvStereoSGBM_options_;
  c_ScaleSweep_options cScaleSweep_options_;

#if HAVE_OpenCV_stereo
  cv::Ptr<cv::stereo::QuasiDenseStereo> quasiDenseStereo_;
  cv::stereo::PropagationParameters quasiDenseStereo_options_;
#endif

};

#endif /* __c_regular_stereo_matcher_h__ */
