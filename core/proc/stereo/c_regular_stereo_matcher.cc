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
#if HAVE_OpenCV_stereo
      { stereo_matcher_QuasiDenseStereo, "QuasiDenseStereo", "cv::stereo::QuasiDenseStereo" },
#endif
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

void c_regular_stereo_matcher::set_matcher_type(stereo_matcher_type v)
{
  matcher_type_ = v;
  stereoMatcher_.reset();
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
    if( stereoMatcher_ ) {

      cv::StereoBM *m =
          dynamic_cast<cv::StereoBM*>(stereoMatcher_.get());

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
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoBM():\n"
        "%s",
        e.what());
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
    if( stereoMatcher_ ) {

      cv::StereoSGBM *m =
          dynamic_cast<cv::StereoSGBM*>(stereoMatcher_.get());

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
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoSGBM():\n"
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
    CF_ERROR("Exception in c_regular_stereo_matcher::updateStereoSGBM():\n"
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
    if( stereoMatcher_ ) {

      cScaleSweepStereoMatcher *m =
          dynamic_cast<cScaleSweepStereoMatcher*>(stereoMatcher_.get());

      if( m ) {

        const c_ScaleSweep_options & opts =
            cScaleSweep_options_;

        m->set_max_disparity(opts.max_disparity);
        m->set_max_scale(opts.max_scale);
        m->set_kernel_sigma(opts.kernel_sigma);
        m->set_kernel_radius(opts.kernel_radius);
        m->set_normalization_scale(opts.normalization_scale);
        m->set_debug_directory(opts.debug_directory);
        m->set_debug_points(opts.debug_points);
      }
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
      return cvStereoBM_options_.numDisparities;
    case stereo_matcher_cvStereoSGBM:
      return cvStereoSGBM_options_.numDisparities;
#if HAVE_OpenCV_stereo
    case stereo_matcher_QuasiDenseStereo:
      return 128;
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
#endif // HAVE_OpenCV_stereo
    case stereo_matcher_ScaleSweep:
      return 1;
  }
  return 0;
}


bool c_regular_stereo_matcher::create_stereo_matcher(const cv::Size & image_size)
{
  try {

#if HAVE_OpenCV_stereo
    quasiDenseStereo_.reset();
#endif

    stereoMatcher_.reset();

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

        stereoMatcher_ = m;
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

        stereoMatcher_ = m;
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
        m->set_normalization_scale(opts.normalization_scale);
        m->set_debug_directory(opts.debug_directory);
        m->set_debug_points(opts.debug_points);

        stereoMatcher_ = m;
        break;
      }

#if HAVE_OpenCV_stereo
      case stereo_matcher_QuasiDenseStereo: {

        quasiDenseStereo_ =
            cv::stereo::QuasiDenseStereo::create(image_size);

        quasiDenseStereo_->Param =
            quasiDenseStereo_options_;

        break;
      }
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
  try {

    if( !stereoMatcher_ && !create_stereo_matcher(left.size()) ) {
      CF_ERROR("create_stereo_matcher() fails");
      return false;
    }

    cv::Mat images[2];

#if HAVE_OpenCV_stereo
    if ( quasiDenseStereo_ ) {

      images[0] = left.getMat();
      images[1] = right.getMat();

      quasiDenseStereo_->process(images[0], images[1]);

      cv::Mat disp =
          quasiDenseStereo_->getDisparity();

      CF_DEBUG("disp: %dx%d depth=%d channels=%d", disp.cols, disp.rows,
          disp.depth(), disp.channels());

      disparity.move(disp);

      return true;
    }
#endif // HAVE_OpenCV_stereo

    if ( stereoMatcher_ ) {

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

      stereoMatcher_->compute(images[0], images[1], disparity);

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

      return true;
    }
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
#endif // HAVE_OpenCV_stereo

  if( (section = SERIALIZE_GROUP(settings, save, "ScaleSweep")) ) {

    c_ScaleSweep_options & opts  =
        cScaleSweep_options_;

    SERIALIZE_OPTION(section, save, opts, max_disparity);
    SERIALIZE_OPTION(section, save, opts, max_scale);
    SERIALIZE_OPTION(section, save, opts, kernel_sigma);
    SERIALIZE_OPTION(section, save, opts, kernel_radius);
    SERIALIZE_OPTION(section, save, opts, normalization_scale);
    SERIALIZE_OPTION(section, save, opts, debug_directory);
    SERIALIZE_OPTION(section, save, opts, debug_points);
  }


  return true;
}
