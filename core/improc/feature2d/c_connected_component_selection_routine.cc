/*
 * c_connected_component_selection_routine.cc
 *
 *  Created on: Jul 13, 2026
 *      Author: amyznikov
 */
#include "c_connected_component_selection_routine.h"
#include <core/proc/histogram-tools.h>
#include <core/proc/fft.h>

/**
 * Given binary mask src this routine extracts the list of connected components,
 *  selects maximal one by area and returns its bounding rectangle and mask
 *  */
static bool get_maximal_connected_component(const cv::Mat1b & src, cv::Rect * bounding_rect,
    cv::Mat * component_mask = nullptr)
{
  cv::Mat1i labels, stats;
  cv::Mat1d centroids;
  int N;

  if( (N = cv::connectedComponentsWithStats(src, labels, stats, centroids, 8, labels.type())) < 2 ) {
    return false;
  }

  struct ss
  {
    int label, area;
  };

  std::vector<ss> cstats;

  for( int i = 1; i < N; ++i ) {
    cstats.emplace_back();
    cstats.back().label = i;
    cstats.back().area = stats[i][cv::CC_STAT_AREA];
  }

  if( cstats.size() > 1 ) {
    std::sort(cstats.begin(), cstats.end(),
        [](const ss & p, const ss & n) {
          return p.area > n.area;
        });
  }

  if( cstats[0].area < 4 ) {
    CF_DEBUG("Small area: %d", cstats[0].area);
    return false;
  }

  if( component_mask ) {
    cv::compare(labels, cstats[0].label,
        *component_mask,
        cv::CMP_EQ);
  }

  if( bounding_rect ) {
    const cv::Rect rc(stats[cstats[0].label][cv::CC_STAT_LEFT], stats[cstats[0].label][cv::CC_STAT_TOP],
        stats[cstats[0].label][cv::CC_STAT_WIDTH], stats[cstats[0].label][cv::CC_STAT_HEIGHT]);
    if( bounding_rect ) {
      *bounding_rect = rc;
    }
  }

  return true;
}


void c_connected_component_selection_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  //ctlbind(ctls, "update_mask", CTL_CONTEXT(ctx, _update_mask), "");
  ctlbind(ctls, "mask_mode", ctx(&this_class::_mask_mode), "");
  ctlbind(ctls, "update_roi", CTL_CONTEXT(ctx, _update_roi), "");
  ctlbind(ctls, "fftROI", CTL_CONTEXT(ctx, _extend_roi_to_optimal_fft_size), "Extend ROI to optimal FFT size");
  ctlbind(ctls, "input", ctx(&this_class::_input_channel), "");
  ctlbind(ctls, "output", ctx(&this_class::_output_channel), "");
}

bool c_connected_component_selection_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    //SERIALIZE_OPTION(settings, save, *this, _update_mask);
    SERIALIZE_OPTION(settings, save, *this, _update_roi);
    SERIALIZE_OPTION(settings, save, *this, _extend_roi_to_optimal_fft_size);
    SERIALIZE_OPTION(settings, save, *this, _input_channel);
    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    SERIALIZE_OPTION(settings, save, *this, _mask_mode);
    return true;
  }
  return false;
}

bool c_connected_component_selection_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  cv::Mat1b src;
  cv::Mat1b comp;
  cv::Rect rc;

  switch (_input_channel) {
    case IMAGE: {
      if( image.channels() != 1 ) {
        CF_ERROR("Invalid arg: Single-channel input image expected");
        return false;
      }
      autoClip(image, mask, src, 0.01, 0.9999, 0, 255, CV_8U);
      break;
    }
    case MASK: {
      if( mask.type() != CV_8UC1 ) {
        CF_ERROR("Invalid arg: Single-channel CV_8UC1 input mask expected");
        return false;
      }
      src = mask.getMat();
      break;
    }
  }

  const bool update_mask =
      _mask_mode != COMBINE_MASK_MODE_KEEP;

  if( !get_maximal_connected_component(src, &rc, update_mask ? &comp : nullptr) ) {
    CF_ERROR("get_maximal_connected_component() fails");
    return false;
  }

  if ( _update_roi  ) {
    if ( _extend_roi_to_optimal_fft_size ) {
      rc = fftGetOptimalSquaredROI(image.size(), rc);
    }
    ctlbind_update_roi(rc);
  }


  if( update_mask ) {
    combine_masks(_mask_mode, comp, mask, comp);
    switch (_output_channel) {
      case IMAGE:
        image.move(comp);
        break;
      case MASK:
        mask.move(comp);
        break;
    }
  }

  return true;
}

