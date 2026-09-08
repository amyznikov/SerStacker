/*
 * c_alpha_test_routine.cc
 *
 *  Created on: Jun 26, 2026
 *      Author: amyznikov
 */

#include "c_alpha_test_routine.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
#include <core/proc/estimate_noise.h>
#include <core/proc/morphology.h>
#include <core/proc/gradient.h>
#include <core/proc/fft.h>
#include <core/proc/fast_gaussian_blur.h>
#include <core/proc/histogram-tools.h>
#include <core/proc/downstrike.h>
#include <core/ssprintf.h>
#include <core/proc/inpaint/average_pyramid_inpaint.h>
#include <core/io/c_stdio_file.h>
#include <core/proc/c_linear_regression.h>
#include <core/proc/c_line_estimate.h>
#include <core/proc/run-loop.h>
#include <core/readdir.h>
#include <random>
#include <core/io/c_stdio_file.h>



template<>
const c_enum_member * members_of<c_alpha_test_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_SCALED_IMAGE,"CURRENT_SCALED_IMAGE"},
      { c_alpha_test_routine::DISPLAY_REFERENCE_SCALED_IMAGE,"REFERENCE_SCALED_IMAGE"},
      { c_alpha_test_routine::DISPLAY_IFFT, "IFFT", "" },
      { c_alpha_test_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {
} // namespace

bool c_alpha_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, opts, apodization_size);
    SERIALIZE_OPTION(settings, save, opts, gsigma);
    SERIALIZE_OPTION(settings, save, opts, downscale_factor);

    return true;
  }
  return false;
}

void c_alpha_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, "ApodizationRadius", CTL_CONTEXT(ctx, opts.apodization_size), "");
  ctlbind(ctls, "gsigma", CTL_CONTEXT(ctx, opts.gsigma), "");
  ctlbind(ctls, "downscale factor", CTL_CONTEXT(ctx, opts.downscale_factor), "");
  ctlbind(ctls, "updateReference", CTL_CONTEXT(ctx, _updateReferenceImage), "Set checked to set current image as reference");
}

bool c_alpha_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat currentImage, currentMask;

  currentImage = image.getMat();
  currentMask = mask.getMat();

  if ( !referenceImage.empty() ) {
    cv::Vec2f Translation;
    double score;

    CF_DEBUG("Call compute()");
    pc.setCurrentImage(currentImage, currentMask);
    score = pc.compute(Translation);
    CF_DEBUG("compute: score: %g T: x=%g y=%g", score, Translation[0], Translation[1]);

    switch(_display)
    {
      case DISPLAY_CURRENT_IMAGE:
        break;
      case DISPLAY_REFERENCE_IMAGE:
        referenceImage.copyTo(image);
        referenceMask.copyTo(mask);
        break;
      case DISPLAY_CURRENT_SCALED_IMAGE:
        pc._scaledCurrentImage.copyTo(image);
        pc._scaledCurrentMask.copyTo(mask);
        break;
      case DISPLAY_REFERENCE_SCALED_IMAGE:
        pc._scaledReferenceImage.copyTo(image);
        pc._scaledReferenceMask.copyTo(mask);
        break;
      case DISPLAY_IFFT:
        pc._correlationMap.copyTo(image);
        mask.release();
        break;
    }
  }

  if ( referenceImage.empty() ||  _updateReferenceImage ) {
    currentImage.copyTo(referenceImage);
    currentMask.copyTo(referenceMask);
    pc.setup(referenceImage.size(), opts);
    pc.setReferenceImage(referenceImage, referenceMask);
  }

  return true;
}


