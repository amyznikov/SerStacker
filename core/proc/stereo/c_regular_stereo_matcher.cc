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
  static const c_enum_member members[] = {
      { stereo_matcher_cvStereoBM, "StereoBM", "cv::StereoSGBM" },
      { stereo_matcher_cvStereoSGBM, "StereoSGBM", "cv::StereoSGBM" },
#if HAVE_OpenCV_stereo
      { stereo_matcher_QuasiDenseStereo, "QuasiDenseStereo", "cv::stereo::QuasiDenseStereo" },
      { stereo_matcher_StereoBinaryBM, "StereoBinaryBM", "cv::stereo::StereoBinaryBM" },
      { stereo_matcher_StereoBinarySGBM, "StereoBinarySGBM", "cv::stereo::StereoBinarySGBM" },
#endif
      { stereo_matcher_ScaleSweep, "ScaleSweep", "cScaleSweepStereoMatcher" },
      { stereo_matcher_cvStereoSGBM },
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBM_PreFilterType>()
{
  static const c_enum_member members[] = {
      { StereoBM_PREFILTER_NORMALIZED_RESPONSE, "NORMALIZED_RESPONSE", "cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE" },
      { StereoBM_PREFILTER_XSOBEL, "XSOBEL", "cv::StereoBM::PREFILTER_XSOBEL" },
      { StereoBM_PREFILTER_NORMALIZED_RESPONSE }
  };

  return members;
}


template<>
const c_enum_member* members_of<StereoSGBM_Mode>()
{
  static const c_enum_member members[] = {
      { StereoSGBM_SGBM, "SGBM", "cv::StereoSGBM::MODE_SGBM" },
      { StereoSGBM_HH, "HH", "cv::StereoSGBM::MODE_HH" },
      { StereoSGBM_SGBM_3WAY, "SGBM_3WAY", "cv::StereoSGBM::MODE_SGBM_3WAY" },
      { StereoSGBM_HH4, "HH4", "cv::StereoSGBM::MODE_HH4" },
      { StereoSGBM_SGBM }
  };

  return members;
}

#if HAVE_OpenCV_stereo

template<>
const c_enum_member* members_of<StereoBinaryKernelType>()
{
  static const c_enum_member members[] = {
      { CV_DENSE_CENSUS, "CV_DENSE_CENSUS", "cv::stereo::CV_DENSE_CENSUS" },
      { CV_SPARSE_CENSUS, "CV_SPARSE_CENSUS", "cv::stereo::CV_SPARSE_CENSUS" },
      { CV_CS_CENSUS, "CV_CS_CENSUS", "cv::stereo::CV_CS_CENSUS" },
      { CV_MODIFIED_CS_CENSUS, "CV_MODIFIED_CS_CENSUS", "cv::stereo::CV_MODIFIED_CS_CENSUS" },
      { CV_MODIFIED_CENSUS_TRANSFORM, "CV_MODIFIED_CENSUS_TRANSFORM", "cv::stereo::CV_MODIFIED_CENSUS_TRANSFORM" },
      { CV_MEAN_VARIATION, "CV_MEAN_VARIATION", "cv::stereo::CV_MEAN_VARIATION" },
      { CV_STAR_KERNEL, "STAR_KERNEL", "cv::stereo::CV_STAR_KERNEL" },
      { CV_MODIFIED_CENSUS_TRANSFORM }
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBinarySpeckleRemovalTechnique>()
{
  static const c_enum_member members[] = {
      { CV_SPECKLE_REMOVAL, "CV_SPECKLE_REMOVAL_ALGORITHM", "cv::stereo::CV_SPECKLE_REMOVAL_ALGORITHM" },
      { CV_SPECKLE_REMOVAL_AVG, "CV_SPECKLE_REMOVAL_AVG_ALGORITHM", "cv::stereo::CV_SPECKLE_REMOVAL_AVG_ALGORITHM" },
      {CV_SPECKLE_REMOVAL}
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBinarySubpixelInterpolationMethod>()
{
  static const c_enum_member members[] = {
      { CV_QUADRATIC_INTERPOLATION, "QUADRATIC", "cv::stereo::CV_QUADRATIC_INTERPOLATION" },
      { CV_SIMETRICV_INTERPOLATION, "SIMETRICV", "cv::stereo::CV_SIMETRICV_INTERPOLATION" },
      { CV_QUADRATIC_INTERPOLATION}
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBinaryBMPrefilterType>()
{
  static const c_enum_member members[] = {
      { StereoBinaryBM_PREFILTER_NORMALIZED_RESPONSE, "NORMALIZED_RESPONSE", "cv::stereo::StereoBinaryBM::PREFILTER_NORMALIZED_RESPONSE" },
      { StereoBinaryBM_PREFILTER_XSOBEL, "XSOBEL", "cv::stereo::StereoBinaryBM::PREFILTER_XSOBEL" },
      { StereoBinaryBM_PREFILTER_NORMALIZED_RESPONSE}
  };

  return members;
}

template<>
const c_enum_member* members_of<StereoBinarySGBMMode>()
{
  static const c_enum_member members[] = {
      { StereoBinarySGBM_SGBM, "SGBM", "cv::stereo::StereoBinarySGBM::MODE_SGBM" },
      { StereoBinarySGBM_HH, "HH", "cv::stereo::StereoBinarySGBM::MODE_HH" },
      { StereoBinarySGBM_SGBM}
  };

  return members;
}

#endif // HAVE_OpenCV_stereo


c_regular_stereo_matcher::c_regular_stereo_matcher()
{
#if HAVE_OpenCV_stereo
  // copied from opencv_contrib/modules/stereo/src/quasi_dense_stereo.cpp
  quasiDenseStereo_options_.borderX = 15;
  quasiDenseStereo_options_.borderY = 15;
  quasiDenseStereo_options_.corrWinSizeX = 5;
  quasiDenseStereo_options_.corrWinSizeY = 5;
  quasiDenseStereo_options_.correlationThreshold = (float)0.5;
  quasiDenseStereo_options_.textrureThreshold = 200;
  quasiDenseStereo_options_.neighborhoodSize = 5;
  quasiDenseStereo_options_.disparityGradient = 1;
  quasiDenseStereo_options_.lkTemplateSize = 3;
  quasiDenseStereo_options_.lkPyrLvl = 3;
  quasiDenseStereo_options_.lkTermParam1 = 3;
  quasiDenseStereo_options_.lkTermParam2 = (float)0.003;
  quasiDenseStereo_options_.gftQualityThres = (float)0.01;
  quasiDenseStereo_options_.gftMinSeperationDist = 10;
  quasiDenseStereo_options_.gftMaxNumFeatures = 500;
#endif
}

void c_regular_stereo_matcher::set_enabled(bool v)
{
  enabled_ = v;
}

bool c_regular_stereo_matcher::enabled() const
{
  return enabled_;
}

void c_regular_stereo_matcher::set_matcher_type(stereo_matcher_type v)
{
  matcher_type_ = v;
}

stereo_matcher_type c_regular_stereo_matcher::matcher_type() const
{
  return matcher_type_;
}

const c_cvStereoBMOptions& c_regular_stereo_matcher::StereoBMOptions() const
{
  return stereoBM_options_;
}

c_cvStereoBMOptions& c_regular_stereo_matcher::StereoBMOptions()
{
  return stereoBM_options_;
}

void c_regular_stereo_matcher::updateStereoBMOptions()
{
  try {
    if( stereoBM_ ) {

      const c_cvStereoBMOptions &opts =
          stereoBM_options_;

      stereoBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
      stereoBM_->setNumDisparities(opts.numDisparities);
      stereoBM_->setMinDisparity(opts.minDisparity);
      stereoBM_->setBlockSize(opts.blockSize);
      stereoBM_->setSpeckleWindowSize(opts.speckleWindowSize);
      stereoBM_->setSpeckleRange(opts.speckleRange);
      stereoBM_->setPreFilterType(opts.preFilterType);
      stereoBM_->setPreFilterSize(opts.preFilterSize);
      stereoBM_->setPreFilterCap(opts.preFilterCap);
      stereoBM_->setTextureThreshold(opts.textureThreshold);
      stereoBM_->setUniquenessRatio(opts.uniquenessRatio);
      stereoBM_->setSmallerBlockSize(opts.blockSize);
      // stereoBM_->setROI1(opts.roi1);
      // stereoBM_->setROI2(opts.roi2);
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoBM():\n"
        "%s",
        e.what());
  }
}

const c_cvStereoSGBMOptions& c_regular_stereo_matcher::StereoSGBMOptions() const
{
  return stereoSGBM_options_;
}

c_cvStereoSGBMOptions & c_regular_stereo_matcher::StereoSGBMOptions()
{
  return stereoSGBM_options_;
}

void c_regular_stereo_matcher::updateStereoSGBMOptions()
{
  try {
    if( stereoSGBM_ ) {

      const c_cvStereoSGBMOptions &opts =
          stereoSGBM_options_;

      stereoSGBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
      stereoSGBM_->setMode(opts.mode);
      stereoSGBM_->setMinDisparity(opts.minDisparity);
      stereoSGBM_->setNumDisparities(opts.numDisparities);
      stereoSGBM_->setBlockSize(opts.blockSize);
      stereoSGBM_->setSpeckleWindowSize(opts.speckleWindowSize);
      stereoSGBM_->setSpeckleRange(opts.speckleRange);
      stereoSGBM_->setPreFilterCap(opts.preFilterCap);
      stereoSGBM_->setUniquenessRatio(opts.uniquenessRatio);
      stereoSGBM_->setP1(opts.P1);
      stereoSGBM_->setP2(opts.P2);
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception:\n"
        "%s",
        e.what());
  }
}

#if HAVE_OpenCV_stereo

const cv::stereo::PropagationParameters & c_regular_stereo_matcher::quasiDenseStereoOptions() const
{
  return quasiDenseStereo_options_;
}

cv::stereo::PropagationParameters & c_regular_stereo_matcher::quasiDenseStereoOptions()
{
  return quasiDenseStereo_options_;
}

void c_regular_stereo_matcher::updateQuasiDenseStereoOptions()
{
  try {
    if( quasiDenseStereo_ ) {
      quasiDenseStereo_->Param = quasiDenseStereo_options_;
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception:\n"
        "%s",
        e.what());
  }
}

const c_cvStereoBinaryBMOptions & c_regular_stereo_matcher::StereoBinaryBMOptions() const
{
  return stereoBinaryBM_options_;
}

c_cvStereoBinaryBMOptions & c_regular_stereo_matcher::StereoBinaryBMOptions()
{
  return stereoBinaryBM_options_;
}

void c_regular_stereo_matcher::updateStereoBinaryBMOptions()
{
  try {
    if( stereoBinaryBM_ ) {

      const c_cvStereoBinaryBMOptions &opts =
          stereoBinaryBM_options_;

      stereoBinaryBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
      stereoBinaryBM_->setMinDisparity(opts.minDisparity);
      stereoBinaryBM_->setNumDisparities(opts.numDisparities);
      stereoBinaryBM_->setBlockSize(opts.blockSize);
      stereoBinaryBM_->setSpeckleWindowSize(opts.speckleWindowSize);
      stereoBinaryBM_->setSpeckleRange(opts.speckleRange);
      stereoBinaryBM_->setPreFilterType(opts.preFilterType);
      stereoBinaryBM_->setPreFilterSize(opts.preFilterSize);
      stereoBinaryBM_->setPreFilterCap(opts.preFilterCap);
      stereoBinaryBM_->setTextureThreshold(opts.textureThreshold);
      stereoBinaryBM_->setUniquenessRatio(opts.uniquenessRatio);
      stereoBinaryBM_->setSmallerBlockSize(opts.blockSize);
      stereoBinaryBM_->setScalleFactor(opts.scalleFactor);
      stereoBinaryBM_->setSpekleRemovalTechnique(opts.spekleRemovalTechnique);
      stereoBinaryBM_->setUsePrefilter(opts.usePrefilter);
      stereoBinaryBM_->setBinaryKernelType(opts.kernelType);
      stereoBinaryBM_->setAgregationWindowSize(opts.agregationWindowSize);
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception:\n"
        "%s",
        e.what());
  }
}


const c_cvStereoBinarySGBMOptions & c_regular_stereo_matcher::StereoBinarySGBMOptions() const
{
  return stereoBinarySGBM_options_;
}

c_cvStereoBinarySGBMOptions & c_regular_stereo_matcher::StereoBinarySGBMOptions()
{
  return stereoBinarySGBM_options_;
}

void c_regular_stereo_matcher::updateStereoBinarySGBMOptions()
{
  try {
    if( stereoBinarySGBM_ ) {

      const c_cvStereoBinarySGBMOptions &opts =
          stereoBinarySGBM_options_;

      stereoBinarySGBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
      stereoBinarySGBM_->setMinDisparity(opts.minDisparity);
      stereoBinarySGBM_->setNumDisparities(opts.numDisparities);
      stereoBinarySGBM_->setBlockSize(opts.blockSize);
      stereoBinarySGBM_->setSpeckleWindowSize(opts.speckleWindowSize);
      stereoBinarySGBM_->setSpeckleRange(opts.speckleRange);
      stereoBinarySGBM_->setPreFilterCap(opts.preFilterCap);
      stereoBinarySGBM_->setUniquenessRatio(opts.uniquenessRatio);
      stereoBinarySGBM_->setP1(opts.P1);
      stereoBinarySGBM_->setP2(opts.P2);
      stereoBinarySGBM_->setMode(opts.mode);
      stereoBinarySGBM_->setSpekleRemovalTechnique(opts.spekleRemovalTechnique);
      stereoBinarySGBM_->setBinaryKernelType(opts.kernelType);
      stereoBinarySGBM_->setSubPixelInterpolationMethod(opts.subPixelInterpolationMethod);
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception:\n"
        "%s",
        e.what());
  }
}


#endif // HAVE_OpenCV_stereo


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
    if( scaleSweep_ ) {

      const c_ScaleSweep_options &opts =
          cScaleSweep_options_;

      scaleSweep_->set_max_disparity(opts.max_disparity);
      scaleSweep_->set_max_scale(opts.max_scale);
      scaleSweep_->set_texture_threshold(opts.texture_threshold);
      scaleSweep_->set_disp12maxDiff(opts.disp12maxDiff);
      scaleSweep_->set_debug_directory(opts.debug_directory);
      scaleSweep_->set_debug_points(opts.debug_points);
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::updateScaleSweepOptions():\n"
        "%s",
        e.what());
  }
}


double c_regular_stereo_matcher::currentMaxDisparity() const
{
  switch (matcher_type_) {
    case stereo_matcher_cvStereoBM:
      return stereoBM_options_.numDisparities;
    case stereo_matcher_cvStereoSGBM:
      return stereoSGBM_options_.numDisparities;
#if HAVE_OpenCV_stereo
    case stereo_matcher_QuasiDenseStereo:
      return 128;
    case stereo_matcher_StereoBinaryBM:
      return stereoBinaryBM_options_.numDisparities;
    case stereo_matcher_StereoBinarySGBM:
      return stereoBinarySGBM_options_.numDisparities;
#endif // HAVE_OpenCV_stereo
    case stereo_matcher_ScaleSweep:
      return cScaleSweep_options_.max_disparity;
  }
  return 64;
}

int c_regular_stereo_matcher::currentReferenceImageIndex() const
{
  switch (matcher_type_) {
    case stereo_matcher_cvStereoBM:
      return 0;
    case stereo_matcher_cvStereoSGBM:
      return 0;
#if HAVE_OpenCV_stereo
    case stereo_matcher_QuasiDenseStereo:
      return 0;
    case stereo_matcher_StereoBinaryBM:
      return 0;
    case stereo_matcher_StereoBinarySGBM:
      return 0;
#endif // HAVE_OpenCV_stereo
    case stereo_matcher_ScaleSweep:
      return 1;
  }
  return 0;
}


void c_regular_stereo_matcher::reset_all_matchers()
{
  stereoBM_.reset();
  stereoSGBM_.reset();
  scaleSweep_.reset();
#if HAVE_OpenCV_stereo
  quasiDenseStereo_.reset();
  stereoBinaryBM_.reset();
  stereoBinarySGBM_.reset();
#endif
}

bool c_regular_stereo_matcher::create_stereo_matcher(const cv::Size & image_size)
{
  try {

    switch (matcher_type_) {
      case stereo_matcher_cvStereoBM:
        if( !stereoBM_ ) {

          reset_all_matchers();

          const c_cvStereoBMOptions &opts =
              stereoBM_options_;

          stereoBM_ =
              cv::StereoBM::create(opts.numDisparities,
                  opts.blockSize);

          stereoBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
          stereoBM_->setMinDisparity(opts.minDisparity);
          stereoBM_->setSpeckleWindowSize(opts.speckleWindowSize);
          stereoBM_->setSpeckleRange(opts.speckleRange);
          stereoBM_->setPreFilterType(opts.preFilterType);
          stereoBM_->setPreFilterSize(opts.preFilterSize);
          stereoBM_->setPreFilterCap(opts.preFilterCap);
          stereoBM_->setTextureThreshold(opts.textureThreshold);
          stereoBM_->setUniquenessRatio(opts.uniquenessRatio);
          stereoBM_->setSmallerBlockSize(opts.blockSize);
          // stereoBM_->setROI1(opts.roi1);
          // stereoBM_->setROI2(opts.roi2);

        }
        break;

      case stereo_matcher_cvStereoSGBM:
        if( !stereoSGBM_ ) {

          reset_all_matchers();

          const c_cvStereoSGBMOptions &opts =
              stereoSGBM_options_;

          stereoSGBM_ =
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
        }
        break;

      case stereo_matcher_ScaleSweep:
        if( !scaleSweep_ ) {

          reset_all_matchers();

          scaleSweep_ =
              cSweepScanStereoMatcher::create();

          const c_ScaleSweep_options &opts =
              cScaleSweep_options_;

          scaleSweep_->set_max_disparity(opts.max_disparity);
          scaleSweep_->set_max_scale(opts.max_scale);
          scaleSweep_->set_texture_threshold(opts.texture_threshold);
          scaleSweep_->set_disp12maxDiff(opts.disp12maxDiff);
          scaleSweep_->set_debug_directory(opts.debug_directory);
          scaleSweep_->set_debug_points(opts.debug_points);
        }
        break;

#if HAVE_OpenCV_stereo
      case stereo_matcher_QuasiDenseStereo:
        if( !quasiDenseStereo_ || image_size != quasiDenseStereo_image_size_ ) {

          reset_all_matchers();

          quasiDenseStereo_ =
              cv::stereo::QuasiDenseStereo::create(
                  quasiDenseStereo_image_size_ = image_size);

          quasiDenseStereo_->Param =
              quasiDenseStereo_options_;

        }
        break;

      case stereo_matcher_StereoBinaryBM:
        if ( !stereoBinaryBM_ ) {

          reset_all_matchers();

          const c_cvStereoBinaryBMOptions &opts =
              stereoBinaryBM_options_;

          stereoBinaryBM_ =
              cv::stereo::StereoBinaryBM::create(opts.numDisparities,
                  opts.blockSize);

          stereoBinaryBM_->setDisp12MaxDiff(opts.disp12MaxDiff);
          stereoBinaryBM_->setMinDisparity(opts.minDisparity);
          stereoBinaryBM_->setNumDisparities(opts.numDisparities);
          stereoBinaryBM_->setBlockSize(opts.blockSize);
          stereoBinaryBM_->setSpeckleWindowSize(opts.speckleWindowSize);
          stereoBinaryBM_->setSpeckleRange(opts.speckleRange);
          stereoBinaryBM_->setPreFilterType(opts.preFilterType);
          stereoBinaryBM_->setPreFilterSize(opts.preFilterSize);
          stereoBinaryBM_->setPreFilterCap(opts.preFilterCap);
          stereoBinaryBM_->setTextureThreshold(opts.textureThreshold);
          stereoBinaryBM_->setUniquenessRatio(opts.uniquenessRatio);
          stereoBinaryBM_->setSmallerBlockSize(opts.blockSize);
          stereoBinaryBM_->setScalleFactor(opts.scalleFactor);
          stereoBinaryBM_->setSpekleRemovalTechnique(opts.spekleRemovalTechnique);
          stereoBinaryBM_->setUsePrefilter(opts.usePrefilter);
          stereoBinaryBM_->setBinaryKernelType(opts.kernelType);
          stereoBinaryBM_->setAgregationWindowSize(opts.agregationWindowSize);
        }
        break;

      case stereo_matcher_StereoBinarySGBM:
        if ( !stereoBinarySGBM_ ) {

          reset_all_matchers();

          const c_cvStereoBinarySGBMOptions &opts =
              stereoBinarySGBM_options_;

          stereoBinarySGBM_ =
              cv::stereo::StereoBinarySGBM::create(
                  opts.minDisparity,
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

          stereoBinarySGBM_->setMinDisparity(opts.minDisparity);
          stereoBinarySGBM_->setNumDisparities(opts.numDisparities);
          stereoBinarySGBM_->setBlockSize(opts.blockSize);
          stereoBinarySGBM_->setSpeckleWindowSize(opts.speckleWindowSize);
          stereoBinarySGBM_->setSpeckleRange(opts.speckleRange);
          stereoBinarySGBM_->setPreFilterCap(opts.preFilterCap);
          stereoBinarySGBM_->setUniquenessRatio(opts.uniquenessRatio);
          stereoBinarySGBM_->setP1(opts.P1);
          stereoBinarySGBM_->setP2(opts.P2);
          stereoBinarySGBM_->setMode(opts.mode);
          stereoBinarySGBM_->setSpekleRemovalTechnique(opts.spekleRemovalTechnique);
          stereoBinarySGBM_->setBinaryKernelType(opts.kernelType);
          stereoBinarySGBM_->setSubPixelInterpolationMethod(opts.subPixelInterpolationMethod);
        }
        break;

#endif // HAVE_OpenCV_stereo


      default:
        CF_ERROR("Unsupported matcher_type=%d specified",
            matcher_type_);
        return false;
    }
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::create_stereo_matcher():\n"
        "%s", e.what());
    return false;
  }

  return true;
}

bool c_regular_stereo_matcher::compute( cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
{
  static const auto convertDisparityType =
      [](cv::InputArray src, cv::OutputArray dst) -> bool {

        // These constands are copied from /opencv/modules/calib3d/src/stereobm.cpp
          static constexpr int DISPARITY_SHIFT_16S = 4;
          static constexpr int DISPARITY_SHIFT_32S = 8;

          switch (src.type()) {
            case CV_16SC1:
            src.getMat().convertTo(dst, CV_32F, 1. / (1 << DISPARITY_SHIFT_16S), 0);
            break;
            case CV_32SC1:
            src.getMat().convertTo(dst, CV_32F, 1. / (1 << DISPARITY_SHIFT_32S), 0);
            break;
            case CV_32FC1:
            src.copyTo(dst);
            break;
            default:
            CF_ERROR("Invalid arg: the disparity map type=%d not supported. "
                "Must be one of CV_32FC1, CV_16SC1, CV_32SC1",
                src.type());
            return false;
          }

          return true;
        };

  try {

    cv::Mat disp;

    if( !create_stereo_matcher(left.size()) ) {
      CF_ERROR("create_stereo_matcher() fails");
      return false;
    }


    if( stereoBM_ ) {

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

      stereoBM_->compute(images[0], images[1], disp);

      return convertDisparityType(disp, disparity);
    }

    if( stereoSGBM_ ) {
      stereoSGBM_->compute(left, right, disp);
      return convertDisparityType(disp, disparity);
    }

    if( scaleSweep_ ) {
      scaleSweep_->compute(left, right, disp);
      return convertDisparityType(disp, disparity);
    }

#if HAVE_OpenCV_stereo
    if( quasiDenseStereo_ ) {

      cv::Mat L = left.getMat();
      cv::Mat R = right.getMat();

      quasiDenseStereo_->process(L, R);
      disp = quasiDenseStereo_->getDisparity();

      //CF_DEBUG("disp: %dx%d depth=%d channels=%d", disp.cols, disp.rows, disp.depth(), disp.channels());

      disparity.move(disp);

      return true;
    }

    if( stereoBinaryBM_ ) {

      cv::Mat images[2];

      if( left.channels() == 1 ) {
        images[0] = left.getMat();
      }
      else {
        cv::cvtColor(left, images[0],
            cv::COLOR_BGR2GRAY);
      }

      if( right.channels() == 1 ) {
        images[1] = right.getMat();
      }
      else {
        cv::cvtColor(right, images[1],
            cv::COLOR_BGR2GRAY);
      }

      stereoBinaryBM_->compute(images[0], images[1], disp);

      return convertDisparityType(disp, disparity);
    }

    if( stereoBinarySGBM_ ) {


      cv::Mat images[2];

      if( left.channels() == 1 ) {
        images[0] = left.getMat();
      }
      else {
        cv::cvtColor(left, images[0],
            cv::COLOR_BGR2GRAY);
      }

      if( right.channels() == 1 ) {
        images[1] = right.getMat();
      }
      else {
        cv::cvtColor(right, images[1],
            cv::COLOR_BGR2GRAY);
      }

      INSTRUMENT_REGION("BinarySGBM");
      stereoBinarySGBM_->compute(images[0], images[1], disp);

      return convertDisparityType(disp, disparity);
    }

#endif // HAVE_OpenCV_stereo

    return false;
  }
  catch( const std::exception &e ) {
    CF_ERROR("Exception in c_regular_stereo_matcher::compute():\n"
        "%s",
        e.what());
  }
  catch( ... ) {
    CF_ERROR("Unknown Exception in c_regular_stereo_matcher::compute()");
  }

  return false;
}

bool c_regular_stereo_matcher::serialize(c_config_setting settings, bool save)
{
  c_config_setting section;

  SERIALIZE_PROPERTY(settings, save, *this, enabled);
  SERIALIZE_PROPERTY(settings, save, *this, matcher_type);

  if( (section = SERIALIZE_GROUP(settings, save, "StereoBM")) ) {

    c_cvStereoBMOptions & opts =
        stereoBM_options_;

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

    c_cvStereoSGBMOptions & opts =
        stereoSGBM_options_;

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

#if HAVE_OpenCV_stereo
  if( (section = SERIALIZE_GROUP(settings, save, "QuasiDenseStereo")) ) {

    cv::stereo::PropagationParameters &opts =
        quasiDenseStereo_options_;

    SERIALIZE_OPTION(section, save, opts, corrWinSizeX);
    SERIALIZE_OPTION(section, save, opts, corrWinSizeY);
    SERIALIZE_OPTION(section, save, opts, borderX);
    SERIALIZE_OPTION(section, save, opts, borderY);
    SERIALIZE_OPTION(section, save, opts, correlationThreshold);
    SERIALIZE_OPTION(section, save, opts, textrureThreshold);
    SERIALIZE_OPTION(section, save, opts, neighborhoodSize);
    SERIALIZE_OPTION(section, save, opts, disparityGradient);
    SERIALIZE_OPTION(section, save, opts, lkTemplateSize);
    SERIALIZE_OPTION(section, save, opts, lkPyrLvl);
    SERIALIZE_OPTION(section, save, opts, lkTermParam1);
    SERIALIZE_OPTION(section, save, opts, lkTermParam2);
    SERIALIZE_OPTION(section, save, opts, gftQualityThres);
    SERIALIZE_OPTION(section, save, opts, gftMinSeperationDist);
    SERIALIZE_OPTION(section, save, opts, gftMaxNumFeatures);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "StereoBinaryBM")) ) {

    c_cvStereoBinaryBMOptions &opts =
        stereoBinaryBM_options_;

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
    SERIALIZE_OPTION(section, save, opts, scalleFactor);
    SERIALIZE_OPTION(section, save, opts, spekleRemovalTechnique);
    SERIALIZE_OPTION(section, save, opts, usePrefilter);
    SERIALIZE_OPTION(section, save, opts, kernelType);
    SERIALIZE_OPTION(section, save, opts, agregationWindowSize);
  }

  if( (section = SERIALIZE_GROUP(settings, save, "StereoBinarySGBM")) ) {

    c_cvStereoBinarySGBMOptions &opts =
        stereoBinarySGBM_options_;

    SERIALIZE_OPTION(section, save, opts, minDisparity);
    SERIALIZE_OPTION(section, save, opts, numDisparities);
    SERIALIZE_OPTION(section, save, opts, blockSize);
    SERIALIZE_OPTION(section, save, opts, speckleWindowSize);
    SERIALIZE_OPTION(section, save, opts, speckleRange);
    SERIALIZE_OPTION(section, save, opts, disp12MaxDiff);

    SERIALIZE_OPTION(section, save, opts, preFilterCap);
    SERIALIZE_OPTION(section, save, opts, uniquenessRatio);
    SERIALIZE_OPTION(section, save, opts, P1);
    SERIALIZE_OPTION(section, save, opts, P2);
    SERIALIZE_OPTION(section, save, opts, mode);
    SERIALIZE_OPTION(section, save, opts, spekleRemovalTechnique);
    SERIALIZE_OPTION(section, save, opts, kernelType);
    SERIALIZE_OPTION(section, save, opts, subPixelInterpolationMethod);
  }

#endif // HAVE_OpenCV_stereo

  if( (section = SERIALIZE_GROUP(settings, save, "ScaleSweep")) ) {

    c_ScaleSweep_options & opts  =
        cScaleSweep_options_;

    SERIALIZE_OPTION(section, save, opts, max_disparity);
    SERIALIZE_OPTION(section, save, opts, max_scale);
    SERIALIZE_OPTION(section, save, opts, texture_threshold);
    SERIALIZE_OPTION(section, save, opts, disp12maxDiff);
    SERIALIZE_OPTION(section, save, opts, debug_directory);
    SERIALIZE_OPTION(section, save, opts, debug_points);
  }


  return true;
}
