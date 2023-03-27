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

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "c_scale_sweep_stereo_matcher.h"

enum stereo_matcher_type  {
  stereo_matcher_cvStereoBM,
  stereo_matcher_cvStereoSGBM,
  // stereo_matcher_ScaleSweep,
};

struct c_cvStereoBM_options
{
  int minDisparity = 0;
  int numDisparities = 0;
  int blockSize = 21;

  int speckleWindowSize = 0;
  int speckleRange = 0;
  int disp12MaxDiff = 0;

  int preFilterType = 0;
  int preFilterSize = 0;
  int preFilterCap = 0;
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
  int numDisparities = 16;
  int blockSize = 3;

  int speckleWindowSize = 0;
  int speckleRange = 0;
  int disp12MaxDiff = 0;

  int P1 = 0;
  int P2 = 0;
  int preFilterCap = 0;
  int uniquenessRatio = 0;
  StereoSGBM_Mode mode = StereoSGBM_SGBM;
};

class c_regular_stereo_matcher
{
public:
  c_regular_stereo_matcher();

  void set_matcher_type(stereo_matcher_type v);
  stereo_matcher_type matcher_type() const;

  const c_cvStereoBM_options & cvStereoBM_options() const ;
  c_cvStereoBM_options & cvStereoBM_options();

  const c_cvStereoSGBM_options & cvStereoSGBM_options() const;
  c_cvStereoSGBM_options cvStereoSGBM_options();

protected:
  bool create_stereo_matcher();

protected:
  stereo_matcher_type matcher_type_ = stereo_matcher_cvStereoBM;
  cv::Ptr<cv::StereoMatcher> matcher_;

  c_cvStereoBM_options cvStereoBM_options_;
  c_cvStereoSGBM_options cvStereoSGBM_options_;
};

#endif /* __c_regular_stereo_matcher_h__ */
