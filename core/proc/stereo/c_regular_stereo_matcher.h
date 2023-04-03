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
#define HAVE_OpenCV_stereo 1

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
  stereo_matcher_StereoBinaryBM,
  stereo_matcher_StereoBinarySGBM,
#endif
};

enum StereoBM_PreFilterType
{
  StereoBM_PREFILTER_NORMALIZED_RESPONSE = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE,
  StereoBM_PREFILTER_XSOBEL = cv::StereoBM::PREFILTER_XSOBEL
};

struct c_cvStereoBMOptions
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


struct c_cvStereoSGBMOptions
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


#if HAVE_OpenCV_stereo

enum StereoBinaryKernelType
{
  CV_DENSE_CENSUS = cv::stereo::CV_DENSE_CENSUS,
  CV_SPARSE_CENSUS = cv::stereo::CV_SPARSE_CENSUS,
  CV_CS_CENSUS = cv::stereo::CV_CS_CENSUS,
  CV_MODIFIED_CS_CENSUS = cv::stereo::CV_MODIFIED_CS_CENSUS,
  CV_MODIFIED_CENSUS_TRANSFORM = cv::stereo::CV_MODIFIED_CENSUS_TRANSFORM,
  CV_MEAN_VARIATION = cv::stereo::CV_MEAN_VARIATION,
  CV_STAR_KERNEL = cv::stereo::CV_STAR_KERNEL
};

enum StereoBinarySpeckleRemovalTechnique
{
  CV_SPECKLE_REMOVAL = cv::stereo::CV_SPECKLE_REMOVAL_ALGORITHM,
  CV_SPECKLE_REMOVAL_AVG = cv::stereo::CV_SPECKLE_REMOVAL_AVG_ALGORITHM,
};

enum StereoBinarySubpixelInterpolationMethod
{
  CV_QUADRATIC_INTERPOLATION = cv::stereo::CV_QUADRATIC_INTERPOLATION,
  CV_SIMETRICV_INTERPOLATION = cv::stereo::CV_SIMETRICV_INTERPOLATION
};

struct c_cvStereoBinaryOptions
{
  int minDisparity = 0;
  int numDisparities = 80;
  int blockSize = 9;
  int speckleWindowSize = 0;
  int speckleRange = 0;
  int disp12MaxDiff = 1;
};

enum StereoBinaryBMPrefilterType
{
  StereoBinaryBM_PREFILTER_NORMALIZED_RESPONSE = cv::stereo::StereoBinaryBM::PREFILTER_NORMALIZED_RESPONSE,
  StereoBinaryBM_PREFILTER_XSOBEL = cv::stereo::StereoBinaryBM::PREFILTER_XSOBEL
};

struct c_cvStereoBinaryBMOptions: c_cvStereoBinaryOptions
{
  StereoBinaryBMPrefilterType preFilterType = StereoBinaryBM_PREFILTER_NORMALIZED_RESPONSE;
  int preFilterSize = 5;
  int preFilterCap = 5;
  int textureThreshold = 100;
  int uniquenessRatio = 0;
  int smallerBlockSize = 3;
  int scalleFactor = 0;
  StereoBinarySpeckleRemovalTechnique spekleRemovalTechnique = CV_SPECKLE_REMOVAL;
  bool usePrefilter = false;
  StereoBinaryKernelType kernelType = CV_MODIFIED_CENSUS_TRANSFORM;
  int agregationWindowSize = 5;
};

enum StereoBinarySGBMMode {
  StereoBinarySGBM_SGBM = cv::stereo::StereoBinarySGBM::MODE_SGBM,
  StereoBinarySGBM_HH = cv::stereo::StereoBinarySGBM::MODE_HH
};

struct c_cvStereoBinarySGBMOptions: c_cvStereoBinaryOptions
{
  int preFilterCap = 0;
  int uniquenessRatio = 0;
  int P1 = 0;
  int P2 = 0;
  StereoBinarySGBMMode mode = StereoBinarySGBM_SGBM;
  StereoBinarySpeckleRemovalTechnique spekleRemovalTechnique = CV_SPECKLE_REMOVAL;
  StereoBinaryKernelType kernelType = CV_MODIFIED_CENSUS_TRANSFORM;
  StereoBinarySubpixelInterpolationMethod subPixelInterpolationMethod = CV_QUADRATIC_INTERPOLATION;
};

#endif // HAVE_OpenCV_stereo


class c_regular_stereo_matcher
{
public:
  c_regular_stereo_matcher();

  void set_matcher_type(stereo_matcher_type v);
  stereo_matcher_type matcher_type() const;

  const c_cvStereoBMOptions & StereoBMOptions() const ;
  c_cvStereoBMOptions & StereoBMOptions();
  void updateStereoBMOptions();

  const c_cvStereoSGBMOptions & StereoSGBMOptions() const;
  c_cvStereoSGBMOptions & StereoSGBMOptions();
  void updateStereoSGBMOptions();

#if HAVE_OpenCV_stereo
  const cv::stereo::PropagationParameters & quasiDenseStereoOptions() const;
  cv::stereo::PropagationParameters & quasiDenseStereoOptions();
  void updateQuasiDenseStereoOptions();

  const c_cvStereoBinaryBMOptions & StereoBinaryBMOptions() const;
  c_cvStereoBinaryBMOptions & StereoBinaryBMOptions();
  void updateStereoBinaryBMOptions();

  const c_cvStereoBinarySGBMOptions & StereoBinarySGBMOptions() const;
  c_cvStereoBinarySGBMOptions & StereoBinarySGBMOptions();
  void updateStereoBinarySGBMOptions();
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
  void reset_all_matchers();

protected:
  stereo_matcher_type matcher_type_ = stereo_matcher_cvStereoBM;

  cv::Ptr<cv::StereoBM> stereoBM_;
  c_cvStereoBMOptions stereoBM_options_;

  cv::Ptr<cv::StereoSGBM> stereoSGBM_;
  c_cvStereoSGBMOptions stereoSGBM_options_;

  cv::Ptr<cScaleSweepStereoMatcher> scaleSweep_;
  c_ScaleSweep_options cScaleSweep_options_;

#if HAVE_OpenCV_stereo
  cv::Ptr<cv::stereo::QuasiDenseStereo> quasiDenseStereo_;
  cv::stereo::PropagationParameters quasiDenseStereo_options_;
  cv::Size quasiDenseStereo_image_size_;

  cv::Ptr<cv::stereo::StereoBinaryBM> stereoBinaryBM_;
  c_cvStereoBinaryBMOptions stereoBinaryBM_options_;

  cv::Ptr<cv::stereo::StereoBinarySGBM> stereoBinarySGBM_;
  c_cvStereoBinarySGBMOptions stereoBinarySGBM_options_;

#endif



};

#endif /* __c_regular_stereo_matcher_h__ */
