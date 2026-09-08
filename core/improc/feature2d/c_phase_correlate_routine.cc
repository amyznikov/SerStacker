/*
 * c_phase_correlate_routine.cc
 *
 *  Created on: Sep 8, 2026
 *      Author: amyznikov
 */

#include "c_phase_correlate_routine.h"
#include <core/proc/reduce_channels.h>
#include <core/proc/geo-reconstruction.h>

template<>
const c_enum_member * members_of<c_phase_correlate_routine::DISPLAY>()
{
  static const c_enum_member members[] = {
      { c_phase_correlate_routine::DISPLAY_CURRENT_IMAGE, "CURRENT_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_REFERENCE_IMAGE, "REFERENCE_IMAGE", "" },
      { c_phase_correlate_routine::DISPLAY_CURRENT_SCALED_IMAGE,"CURRENT_SCALED_IMAGE"},
      { c_phase_correlate_routine::DISPLAY_REFERENCE_SCALED_IMAGE,"REFERENCE_SCALED_IMAGE"},
      { c_phase_correlate_routine::DISPLAY_IFFT, "IFFT", "" },
      { c_phase_correlate_routine::DISPLAY_CURRENT_IMAGE}
  };
  return members;
}

/////////////////////////////////////
namespace {
} // namespace

bool c_phase_correlate_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _display);
    SERIALIZE_OPTION(settings, save, *this, _fillMaskHoles);
    serialize_phase_correlate_options(settings, save, opts);
    return true;
  }
  return false;
}

void c_phase_correlate_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Display", CTL_CONTEXT(ctx, _display), "Select image to display");
  ctlbind(ctls, CTL_CONTEXT(ctx, opts));
  ctlbind(ctls, "fillMaskHoles", CTL_CONTEXT(ctx, _fillMaskHoles), "Set checked to call geo_fill_holes(currentMask)");
  ctlbind(ctls, "updateReference", CTL_CONTEXT(ctx, _updateReferenceImage), "Set checked to set current image as reference");
}

bool c_phase_correlate_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat currentImage, currentMask;

  if ( image.channels() == 1 ) {
    currentImage = image.getMat();
  }
  else {
    cv::cvtColor(image, currentImage, cv::COLOR_BGR2GRAY);
  }

  if ( !mask.empty() ) {
    if ( mask.channels() != 1 ) {
      reduce_color_channels(mask, currentMask, cv::REDUCE_MIN);
      if ( currentMask.depth() != CV_8U ) {
        cv::compare(currentMask, 0, currentMask, cv::CMP_GT);
      }
    }
    else if ( currentMask.depth() != CV_8U ) {
      cv::compare(currentMask, 0, currentMask, cv::CMP_GT);
    }
    else {
      currentMask = mask.getMat();
    }

    if ( _fillMaskHoles ) {
      geo_fill_holes(currentMask, currentMask, 8);
    }
  }

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
        pc.scaledCurrentImage().copyTo(image);
        pc.scaledCurrentMask().copyTo(mask);
        break;
      case DISPLAY_REFERENCE_SCALED_IMAGE:
        pc.scaledReferenceImage().copyTo(image);
        pc.scaledReferenceMask().copyTo(mask);
        break;
      case DISPLAY_IFFT:
        pc.correlationMap().copyTo(image);
        mask.release();
        break;
    }
  }

  if ( _updateReferenceImage || referenceImage.empty() ) {
    currentImage.copyTo(referenceImage);
    currentMask.copyTo(referenceMask);
    pc.setup(referenceImage.size(), opts);
    pc.setReferenceImage(referenceImage, referenceMask);
  }

  return true;
}
