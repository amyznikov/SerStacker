/*
 * QFrameQualityEstimation.h
 *
 *  Created on: Aug 7, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFrameQualityEstimation_h__
#define __QFrameQualityEstimation_h__

#include "QCameraFrame.h"
#include <core/proc/sharpness_measure/c_local_variance_sharpness_measure.h>
#include <thread>

namespace serimager {

class QFrameQualityEstimation
{
public:
  typedef QFrameQualityEstimation ThisClass;

  void setEnabled(bool v)
  {
    _enabled = v;
  }

  bool isEnabled()
  {
    return _enabled;
  }

  /**
   * Estimate quality of frames and return the local index of the best quality frame.
   * Returns -1 if the input vector is empty.
   */
  int estimateFrameQuality(const std::vector<QCameraFrame::sptr> & frames)
  {
    if (frames.empty()) {
      return -1;
    }

    cv::Mat tmp;
    int best_frame_index = 0; // By default take at least the very first one
    float best_quality = -1.0f;

    for (int i = 0, n = (int)(frames.size()); i < n; ++i) {
      const auto & frame = frames[i];

      float quality = frame->quality();

      // WAITING LOOP: If a frame is currently being calculated on ANOTHER thread,
      // don't duplicate the calculations, but simply wait for the calculation to complete.
      // Yield the processor so that the computing thread finishes faster
      while (quality == QCameraFrame::QUALITY_ESTIMATING) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        quality = frame->quality();
      }

      if (quality >= 0.0f) {
        if (quality > best_quality) {
          best_quality = quality;
          best_frame_index = i;
        }
        continue;
      }

      if( quality == QCameraFrame::QUALITY_UNKNOWN ) {

        if( frame->try_start_quality_etimate() ) {
          float computed_quality = 0;

          // std::lock_guard<std::mutex> lock(_mtx);
          if ( !is_bayer_pattern(frame->colorid()) ) {
            computed_quality = compute_local_variance_map(frame->image(), opts);
          }
          else {
            average_bayer_planes(frame->image(), tmp);
            computed_quality = compute_local_variance_map(tmp, opts);
          }

          frame->set_quality(computed_quality);
          quality = computed_quality;
        }
        else {
          // If try_start_quality_etimate() returned false, then
          // another thread ALREADY intercepted this frame and moved it to QUALITY_ESTIMATING.
          // Step back by index to enter `while(QUALITY_ESTIMATING)` on the next iteration
          --i;
          continue;
        }
      }

      if (quality > best_quality) {
        best_quality = quality;
        best_frame_index = i;
      }
    }

    return best_frame_index;
  }

  using c_control_list = c_ctlist<ThisClass> ;
  using ctlbind_context = c_ctlbind_context<ThisClass>;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
  {
    ctlbind_local_variance_estimate_options(ctls, CTL_CONTEXT(ctx, opts));
  }

protected:
  c_local_variance_map_options opts;
  //std::mutex _mtx;
  bool _enabled = false;
};

} /* namespace serimager */

#endif /* __QFrameQualityEstimation_h__ */
