/*
 * c_sweepscan_stereo_matcher.h
 *
 *  Created on: Mar 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sweepscan_stereo_matcher_h__
#define __c_sweepscan_stereo_matcher_h__

#include <core/proc/sharpness_measure/c_lpg_sharpness_measure.h>
#include <core/proc/stereo/ssdesc.h>

class c_sweepscan_stereo_matcher
{
public:
  typedef c_sweepscan_stereo_matcher this_clss;

  enum OutputType {
    OutputTextureMap,
    OutputTextureMask,
    OutputDisparityMap,
    OutputErrorMap,
  };


  c_sweepscan_stereo_matcher();

  bool match(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::InputArray referenceImage, cv::InputArray referenceMask,
      cv::Mat & outputMatches, cv::Mat1b * outputMask);

  void set_output_type(OutputType v);
  OutputType output_type() const;

  void set_max_disparity(int v);
  int max_disparity() const;

  void set_ssflags(int v);
  int ssflags() const;

  void set_ss_sigma(double v);
  double ss_sigma() const;

  void set_ss_radius(int v);
  int ss_radius() const;

  void set_max_scale(int v);
  int max_scale() const;

  void set_kernel_sigma(double v);
  double kernel_sigma() const;

  void set_kernel_radius(int v);
  int kernel_radius() const;

  void set_normalization_scale(int v) ;
  int normalization_scale() const;

  void set_debug_directory(const std::string & v);
  const std::string & debug_directory() const;

  const std::vector<cv::Point>& debug_points() const;
  void set_debug_points(const std::vector<cv::Point> & v);

protected:
//  template<class MT>
//  bool match_impl(cv::InputArray currentImage, cv::InputArray currentMask,
//      cv::InputArray referenceImage, cv::InputArray referenceMask,
//      cv::Mat & outputImage, cv::Mat1b * outputMask);

protected:
  OutputType output_type_ = OutputTextureMask;

  double ss_sigma_ = 2;
  int ss_radius_ = 0;
  int ss_maxlvl_ = 3;
  int ss_flags_ = sscmp_all;

  int max_disparity_ = 128;
  double kernel_sigma_ = 1;
  int kernel_radius_ = 3;
  int pscale_ = 0;

  std::string debug_directory_;
  std::vector<cv::Point> debug_points_;
};


class cSweepScanStereoMatcher :
    public cv::StereoMatcher,
    public c_sweepscan_stereo_matcher
{
public:
  typedef cSweepScanStereoMatcher this_class;
  typedef c_sweepscan_stereo_matcher base;
  typedef cv::Ptr<this_class> Ptr;

  cSweepScanStereoMatcher() = default;
  ~cSweepScanStereoMatcher() = default;

  static cv::Ptr<this_class> create();

  void compute( cv::InputArray left, cv::InputArray right,
      cv::OutputArray disparity ) override;

  int getMinDisparity() const override;
  void setMinDisparity(int minDisparity) override;

  int getNumDisparities() const override;
  void setNumDisparities(int numDisparities) override;

  int getBlockSize() const override;
  void setBlockSize(int blockSize) override;

  int getSpeckleWindowSize() const override;
  void setSpeckleWindowSize(int speckleWindowSize) override;

  int getSpeckleRange() const override;
  void setSpeckleRange(int speckleRange) override;

  int getDisp12MaxDiff() const override;
  void setDisp12MaxDiff(int disp12MaxDiff) override;
};


#endif /* __c_sweepscan_stereo_matcher_h__ */
