/*
 * c_regular_stereo_matcher.cc
 *
 *  Created on: Mar 27, 2023
 *      Author: amyznikov
 */

#include "c_regular_stereo_matcher.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<stereo_matcher_type>()
{
  static constexpr c_enum_member members[] = {
      { stereo_matcher_cvStereoBM, "StereoBM", "cv::StereoSGBM" },
      { stereo_matcher_cvStereoSGBM, "StereoSGBM", "cv::StereoSGBM" },
      { stereo_matcher_cvStereoBM },
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoSGBM_Mode>()
{
  static constexpr c_enum_member members[] = {
      { StereoSGBM_SGBM, "SGBM", "cv::StereoSGBM::MODE_SGBM" },
      { StereoSGBM_HH, "HH", "cv::StereoSGBM::MODE_HH" },
      { StereoSGBM_SGBM_3WAY, "SGBM_3WAY", "cv::StereoSGBM::MODE_SGBM_3WAY" },
      { StereoSGBM_HH4, "HH4", "cv::StereoSGBM::MODE_HH4" },
      { StereoSGBM_SGBM }
  };

  return members;
}


c_regular_stereo_matcher::c_regular_stereo_matcher()
{
}

void c_regular_stereo_matcher::set_matcher_type(stereo_matcher_type v)
{
  matcher_type_ = v;
  matcher_.reset();
}

stereo_matcher_type c_regular_stereo_matcher::matcher_type() const
{
  return matcher_type_;
}

const c_cvStereoBM_options & c_regular_stereo_matcher::cvStereoBM_options() const
{
  return cvStereoBM_options_;
}

c_cvStereoBM_options & c_regular_stereo_matcher::cvStereoBM_options()
{
  return cvStereoBM_options_;
}

const c_cvStereoSGBM_options & c_regular_stereo_matcher::cvStereoSGBM_options() const
{
  return cvStereoSGBM_options_;
}

c_cvStereoSGBM_options c_regular_stereo_matcher::cvStereoSGBM_options()
{
  return cvStereoSGBM_options_;
}

bool c_regular_stereo_matcher::create_stereo_matcher()
{
  switch (matcher_type_) {
    case stereo_matcher_cvStereoBM: {

      const c_cvStereoBM_options &opts =
          cvStereoBM_options_;

      cv::Ptr<cv::StereoBM> m =
          cv::StereoBM::create(opts.numDisparities,
              opts.blockSize);

      m->setMinDisparity(opts.minDisparity);
      m->setSpeckleWindowSize(opts.speckleWindowSize);
      m->setSpeckleRange(opts.speckleRange);
      m->setDisp12MaxDiff(opts.disp12MaxDiff);
      m->setPreFilterType(opts.preFilterType);
      m->setPreFilterSize(opts.preFilterSize);
      m->setPreFilterCap(opts.preFilterCap);
      m->setTextureThreshold(opts.textureThreshold);
      m->setUniquenessRatio(opts.uniquenessRatio);
      m->setSmallerBlockSize(opts.blockSize);
      // m->setROI1(opts.roi1);
      // m->setROI2(opts.roi2);

      matcher_ = m;
      break;
    }

    case stereo_matcher_cvStereoSGBM: {

      const c_cvStereoSGBM_options &opts =
          cvStereoSGBM_options_;

      cv::Ptr<cv::StereoSGBM> m =
          cv::StereoSGBM::create(opts.minDisparity,
              opts.numDisparities,
              opts.blockSize,
              opts.P1,
              opts.P2,
              opts.disp12MaxDiff,
              opts.preFilterCap,
              opts.uniquenessRatio,
              opts.speckleWindowSize,
              opts.speckleRange,
              opts.mode);

      // m->setMinDisparity(opts.minDisparity);
      // m->setNumDisparities(opts.numDisparities);
      // m->setBlockSize(opts.blockSize);
      // m->setSpeckleWindowSize(opts.speckleWindowSize);
      // m->setSpeckleRange(opts.speckleRange);
      // m->setDisp12MaxDiff(opts.disp12MaxDiff);
      // m->setPreFilterCap(opts.preFilterCap);
      // m->setUniquenessRatio(opts.uniquenessRatio);
      // m->setP1(opts.P1);
      // m->setP2(opts.P2);
      // m->setMode(opts.mode);

      matcher_ = m;
      break;
    }

    default:
      CF_ERROR("Unsupported matcher_type=%d specified", matcher_type_);
      return false;
  }

  return true;
}
