/*
 * c_bm3d_denoising_routine.cc
 *
 *  Created on: Sep 23, 2026
 *      Author: amyznikov
 */

#include "c_bm3d_denoising_routine.h"
#if HAVE_OpenCV_xphoto
# include <opencv2/xphoto/bm3d_image_denoising.hpp>
#endif
#include <core/ssprintf.h>


static inline int signedTypeFor(int dtype)
{
  if ( dtype == CV_8U ) {
    return CV_8S;
  }
  if ( dtype == CV_16U ) {
    return CV_16S;
  }

  return dtype;
}

void c_bm3d_denoising_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "luminanceOnly:", ctx(&this_class::luminanceOnly),
      "Set checked to denoise only luminance channel keeping colors noisy (for experimentation only).\n");

  ctlbind(ctls, "h", ctx(&this_class::h),
      "Parameter regulating filter strength.\n"
          "Big h value perfectly removes noise but also removes image details,\n"
          "smaller h value preserves details but also preserves some noise.");

  ctlbind(ctls, "templateWindowSize", ctx(&this_class::templateWindowSize),
      "Size in pixels of the template patch that is used for block-matching.\n"
          "Should be power of 2.");

  ctlbind(ctls, "searchWindowSize", ctx(&this_class::searchWindowSize),
      "Size in pixels of the window that is used to perform block-matching.\n"
          "Affect performance linearly: greater searchWindowsSize - greater denoising time.\n"
          "Must be larger than templateWindowSize");

  ctlbind(ctls, "blockMatchingStep1", ctx(&this_class::blockMatchingStep1),
      "Block matching threshold for the first step of BM3D (hard thresholding),\n"
          "i.e. maximum distance for which two blocks are considered similar.\n"
          "Value expressed in euclidean distance.");

  ctlbind(ctls, "blockMatchingStep2", ctx(&this_class::blockMatchingStep2),
      "Block matching threshold for the second step of BM3D (Wiener filtering),\n"
          "i.e. maximum distance for which two blocks are considered similar.\n"
          "Value expressed in euclidean distance.");

  ctlbind(ctls, "groupSize", ctx(&this_class::groupSize),
      "Maximum size of the 3D group for collaborative filtering.");

  ctlbind(ctls, "slidingStep", ctx(&this_class::slidingStep),
      "Sliding step to process every next reference block.");

  ctlbind(ctls, "beta", ctx(&this_class::beta),
      "Kaiser window parameter that affects the sidelobe attenuation of the transform of the window.\n"
          "Kaiser window is used in order to reduce border effects. To prevent usage of the window,\n"
          "set beta to zero.");

  ctlbind(ctls, "normType", ctx(&this_class::normType),
      " Norm used to calculate distance between blocks. L2 is slower than L1\n"
          "but yields more accurate results.");

  //  ctlbind(ctls, "step", ctx(&this_class::step),
  //      " Step of BM3D to be executed. Allowed are only BM3D_STEP1 and BM3D_STEPALL.\n"
  //          "BM3D_STEP2 is not allowed as it requires basic estimate to be present.");

  //  ctlbind(ctls, "transformType", ctx(&this_class::transformType),
  //      " Type of the orthogonal transform used in collaborative filtering step.\n"
  //          "Currently only Haar transform is supported.");
}

bool c_bm3d_denoising_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, luminanceOnly);
    SERIALIZE_OPTION(settings, save, *this, h);
    SERIALIZE_OPTION(settings, save, *this, templateWindowSize);
    SERIALIZE_OPTION(settings, save, *this, searchWindowSize);
    SERIALIZE_OPTION(settings, save, *this, blockMatchingStep1);
    SERIALIZE_OPTION(settings, save, *this, blockMatchingStep2);
    SERIALIZE_OPTION(settings, save, *this, groupSize);
    SERIALIZE_OPTION(settings, save, *this, slidingStep);
    SERIALIZE_OPTION(settings, save, *this, beta);
    SERIALIZE_OPTION(settings, save, *this, normType);
    //  SERIALIZE_OPTION(settings, save, *this, step);
    //  SERIALIZE_OPTION(settings, save, *this, transformType);
    return true;
  }
  return false;
}

bool c_bm3d_denoising_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
#if HAVE_OpenCV_xphoto

//  if ( image.depth() != CV_8U && image.depth() != CV_16U  ) {
//    CF_ERROR("bm3dDenoising() works only with CV_8U and CV_16U images");
//    return false;
//  }

  if( image.channels() == 1 ) {
    cv::xphoto::bm3dDenoising(image, image,
        h,
        templateWindowSize,
        searchWindowSize,
        blockMatchingStep1,
        blockMatchingStep2,
        groupSize,
        slidingStep,
        beta,
        normType,
        cv::xphoto::BM3D_STEPALL,
        cv::xphoto::HAAR);
  }
  else if ( luminanceOnly ) {
    std::vector<cv::Mat> channels;
    cv::Mat grayscale;

    cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);

    cv::split(image, channels);

    for( auto & channel : channels ) {
      cv::subtract(channel, grayscale, channel, cv::noArray(),
          signedTypeFor(channel.depth()));
    }

    cv::xphoto::bm3dDenoising(grayscale, grayscale,
        h,
        templateWindowSize,
        searchWindowSize,
        blockMatchingStep1,
        blockMatchingStep2,
        groupSize,
        slidingStep,
        beta,
        normType,
        cv::xphoto::BM3D_STEPALL,
        cv::xphoto::HAAR);

    for( auto & channel : channels ) {
      cv::add(channel, grayscale, channel, cv::noArray(),
          grayscale.depth());
    }

    cv::merge(channels, image);
  }

  else {
    std::vector<cv::Mat> channels;

    cv::split(image, channels);

    for( auto & channel : channels ) {
      cv::xphoto::bm3dDenoising(channel, channel,
          h,
          templateWindowSize,
          searchWindowSize,
          blockMatchingStep1,
          blockMatchingStep2,
          groupSize,
          slidingStep,
          beta,
          normType,
          cv::xphoto::BM3D_STEPALL,
          cv::xphoto::HAAR);
    }

    cv::merge(channels, image);
  }

  return true;
#else
  CF_ERROR("OpenCV module xphoto is not available. Can not call cv::xphoto::bm3dDenoising()");
  (void)(image);
  (void)(mask);
  return false;
#endif
}
