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
      { stereo_matcher_ScaleSweep, "ScaleSweep", "cScaleSweepStereoMatcher" },
      { stereo_matcher_cvStereoSGBM },
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBM_PreFilterType>()
{
  static constexpr c_enum_member members[] = {
      { StereoBM_PREFILTER_NORMALIZED_RESPONSE, "NORMALIZED_RESPONSE", "cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE" },
      { StereoBM_PREFILTER_XSOBEL, "XSOBEL", "cv::StereoBM::PREFILTER_XSOBEL" },
      { StereoBM_PREFILTER_NORMALIZED_RESPONSE }
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

const c_cvStereoBM_options& c_regular_stereo_matcher::StereoBMOptions() const
{
  return cvStereoBM_options_;
}

c_cvStereoBM_options& c_regular_stereo_matcher::StereoBMOptions()
{
  return cvStereoBM_options_;
}

void c_regular_stereo_matcher::updateStereoBMOptions()
{
  try {
    if( matcher_ ) {

      cv::StereoBM *m =
          dynamic_cast<cv::StereoBM*>(matcher_.get());

      if( m ) {

        const c_cvStereoBM_options &opts =
            cvStereoBM_options_;

        m->setNumDisparities(opts.numDisparities);
        m->setMinDisparity(opts.minDisparity);
        m->setBlockSize(opts.blockSize);
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
      }
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoBM(): %s", e.what());
  }
}

const c_cvStereoSGBM_options& c_regular_stereo_matcher::StereoSGBMOptions() const
{
  return cvStereoSGBM_options_;
}

c_cvStereoSGBM_options & c_regular_stereo_matcher::StereoSGBMOptions()
{
  return cvStereoSGBM_options_;
}

void c_regular_stereo_matcher::updateStereoSGBMOptions()
{
  try {
    if( matcher_ ) {

      cv::StereoSGBM *m =
          dynamic_cast<cv::StereoSGBM*>(matcher_.get());

      if( m ) {

        const c_cvStereoSGBM_options &opts =
            cvStereoSGBM_options_;

        m->setMode(opts.mode);
        m->setMinDisparity(opts.minDisparity);
        m->setNumDisparities(opts.numDisparities);
        m->setBlockSize(opts.blockSize);
        m->setSpeckleWindowSize(opts.speckleWindowSize);
        m->setSpeckleRange(opts.speckleRange);
        m->setDisp12MaxDiff(opts.disp12MaxDiff);
        m->setPreFilterCap(opts.preFilterCap);
        m->setUniquenessRatio(opts.uniquenessRatio);
        m->setP1(opts.P1);
        m->setP2(opts.P2);
      }
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoSGBM(): %s",
        e.what());
  }
}


const c_ScaleSweep_options & c_regular_stereo_matcher::cScaleSweepOptions() const
{
  return cScaleSweep_options_;
}

c_ScaleSweep_options & c_regular_stereo_matcher::ScaleSweepOptions()
{
  return cScaleSweep_options_;
}

void c_regular_stereo_matcher::updateScaleSweepOptions()
{
  try {
    if( matcher_ ) {

      cScaleSweepStereoMatcher *m =
          dynamic_cast<cScaleSweepStereoMatcher*>(matcher_.get());

      if( m ) {

        const c_ScaleSweep_options & opts =
            cScaleSweep_options_;

        m->set_max_disparity(opts.max_disparity);
        m->set_max_scale(opts.max_scale);
        m->set_kernel_sigma(opts.kernel_sigma);
        m->set_kernel_radius(opts.kernel_radius);
        m->set_debug_directory(opts.debug_directory);
        m->set_debug_points(opts.debug_points);
      }
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::updateScaleSweepOptions(): %s",
        e.what());
  }

}


double c_regular_stereo_matcher::currentMaxDisparity() const
{
  switch (matcher_type_) {
    case stereo_matcher_cvStereoBM:
      return cvStereoBM_options_.numDisparities;
    case stereo_matcher_cvStereoSGBM:
      return cvStereoSGBM_options_.numDisparities;
    case stereo_matcher_ScaleSweep:
      return cScaleSweep_options_.max_disparity;
  }
  return 16;
}

int c_regular_stereo_matcher::currentReferenceImageIndex() const
{
  switch (matcher_type_) {
    case stereo_matcher_cvStereoBM:
      return 0;
    case stereo_matcher_cvStereoSGBM:
      return 0;
    case stereo_matcher_ScaleSweep:
      return 1;
  }
  return 0;
}


bool c_regular_stereo_matcher::create_stereo_matcher()
{
  try {

    switch (matcher_type_) {
      case stereo_matcher_cvStereoBM: {

        const c_cvStereoBM_options & opts =
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

        const c_cvStereoSGBM_options & opts =
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

        matcher_ = m;
        break;
      }


      case stereo_matcher_ScaleSweep: {

        cv::Ptr<cScaleSweepStereoMatcher> m =
            cScaleSweepStereoMatcher::create();

        const c_ScaleSweep_options & opts =
            cScaleSweep_options_;

        m->set_max_disparity(opts.max_disparity);
        m->set_max_scale(opts.max_scale);
        m->set_kernel_sigma(opts.kernel_sigma);
        m->set_kernel_radius(opts.kernel_radius);
        m->set_debug_directory(opts.debug_directory);
        m->set_debug_points(opts.debug_points);

        matcher_ = m;
        break;
      }

      default:
        CF_ERROR("Unsupported matcher_type=%d specified",
            matcher_type_);
        return false;
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::create_stereo_matcher(): %s", e.what());
    return false;
  }

  return true;
}

bool c_regular_stereo_matcher::compute( cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  if( !matcher_ && !create_stereo_matcher() ) {
    CF_ERROR("create_stereo_matcher() fails");
    return false;
  }

  try {

    cv::Mat images[2];

    if( left.channels() == 1 ) {
      images[0] = left.getMat();
    }
    else {
      cv::cvtColor(left, images[0], cv::COLOR_BGR2GRAY);
    }

    if( right.channels() == 1 ) {
      images[1] = right.getMat();
    }
    else {
      cv::cvtColor(right, images[1], cv::COLOR_BGR2GRAY);
    }

    matcher_->compute(images[0], images[1], disparity);

    if( disparity.empty() ) {
      CF_ERROR("matcher_->compute() fails");
    }
    else {

      // These constands are copied from /opencv/modules/calib3d/src/stereobm.cpp
      static constexpr int DISPARITY_SHIFT_16S = 4;
      static constexpr int DISPARITY_SHIFT_32S = 8;

      switch (disparity.type()) {
        case CV_16SC1:
          disparity.getMat().convertTo(disparity, CV_32F, 1. / (1 << DISPARITY_SHIFT_16S), 0);
          break;
        case CV_32SC1:
          disparity.getMat().convertTo(disparity, CV_32F, 1. / (1 << DISPARITY_SHIFT_32S), 0);
          break;
        case CV_32FC1:
          //disps = disparity_map.getMat();
          // disparity_map.getMat().copyTo(disps);
          break;
        default:
          CF_ERROR("Invalid arg: the disparity map type=%d not supported. "
              "Must be one of CV_32FC1, CV_16SC1, CV_32SC1");
          return false;
      }
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::compute(): %s",
        e.what());
    return false;
  }

  return true;
}

bool c_regular_stereo_matcher::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  SERIALIZE_PROPERTY(settings, save, *this, matcher_type);

  if( (section = SERIALIZE_GROUP(settings, save, "StereoBM")) ) {

    c_cvStereoBM_options & opts =
        cvStereoBM_options_;

    SERIALIZE_OPTION(section, save, opts, minDisparity);
    SERIALIZE_OPTION(section, save, opts, numDisparities);
    SERIALIZE_OPTION(section, save, opts, blockSize);
    SERIALIZE_OPTION(section, save, opts, speckleWindowSize);
    SERIALIZE_OPTION(section, save, opts, speckleRange);
    SERIALIZE_OPTION(section, save, opts, disp12MaxDiff);
    SERIALIZE_OPTION(section, save, opts, preFilterType);
    SERIALIZE_OPTION(section, save, opts, preFilterSize);
    SERIALIZE_OPTION(section, save, opts, preFilterCap);
    SERIALIZE_OPTION(section, save, opts, textureThreshold);
    SERIALIZE_OPTION(section, save, opts, uniquenessRatio);
    SERIALIZE_OPTION(section, save, opts, smallerBlockSize);
    SERIALIZE_OPTION(section, save, opts, roi1);
    SERIALIZE_OPTION(section, save, opts, roi2);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "StereoSGBM")) ) {

    c_cvStereoSGBM_options & opts =
        cvStereoSGBM_options_;

    SERIALIZE_OPTION(section, save, opts, minDisparity);
    SERIALIZE_OPTION(section, save, opts, numDisparities);
    SERIALIZE_OPTION(section, save, opts, blockSize);
    SERIALIZE_OPTION(section, save, opts, speckleWindowSize);
    SERIALIZE_OPTION(section, save, opts, speckleRange);
    SERIALIZE_OPTION(section, save, opts, disp12MaxDiff);
    SERIALIZE_OPTION(section, save, opts, P1);
    SERIALIZE_OPTION(section, save, opts, P2);
    SERIALIZE_OPTION(section, save, opts, preFilterCap);
    SERIALIZE_OPTION(section, save, opts, uniquenessRatio);
    SERIALIZE_OPTION(section, save, opts, mode);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "ScaleSweep")) ) {

    c_ScaleSweep_options & opts  =
        cScaleSweep_options_;

    SERIALIZE_OPTION(section, save, opts, max_disparity);
    SERIALIZE_OPTION(section, save, opts, max_scale);
    SERIALIZE_OPTION(section, save, opts, kernel_sigma);
    SERIALIZE_OPTION(section, save, opts, kernel_radius);
    SERIALIZE_OPTION(section, save, opts, debug_directory);
    SERIALIZE_OPTION(section, save, opts, debug_points);
  }

  return true;
}
