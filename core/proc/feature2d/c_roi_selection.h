/*
 * c_roi_selection.h
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_roi_selection_h__
#define __c_roi_selection_h__

#include <opencv2/opencv.hpp>
#include <core/proc/feature2d/planetary-disk-detection.h>

enum ROI_SELECTION_MODE {
  ROI_SELECTION_NONE,
  ROI_SELECTION_RECT,
  ROI_SELECTION_PLANETARY_DISK,
  ROI_SELECTION_GUI,
  // ROI_SELECTION_MAX_CONNECTED_COMPONENT,
};

struct c_roi_selection_options
{
  ROI_SELECTION_MODE mode = ROI_SELECTION_RECT;
  cv::BorderTypes borderType = cv::BORDER_DEFAULT;
  cv::Scalar borderValue;
  cv::Size outputSize;
  bool fixOutputSize = false;

  struct {
    cv::Rect rc;
  } rectSelection;

  struct {
    c_simple_planetary_disk_detector_options opts;
    bool putToCenter = false;
  } planetaryDiskSelection;
};

bool serialize_base_roi_selection_options(c_config_setting section, bool save,
    c_roi_selection_options & opts);

inline bool save_settings(c_config_setting section, const c_roi_selection_options & opts)
{
  return serialize_base_roi_selection_options(section, true,
      const_cast<c_roi_selection_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_roi_selection_options * opts)
{
  return serialize_base_roi_selection_options(section, false, *opts);
}

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_selection_options> & ctx)
{
  using S = c_roi_selection_options;

  ctlbind(ctls, "ROI Selection", CTL_CONTEXT(ctx, mode), "What to select into ROI");
  ctlbind(ctls, "force output size:", CTL_CONTEXT(ctx, fixOutputSize), "");
  ctlbind(ctls, "forced output size:", CTL_CONTEXT(ctx, outputSize), "");
  ctlbind(ctls, "borderType:", CTL_CONTEXT(ctx, borderType), "");
  ctlbind(ctls, "borderValue:", CTL_CONTEXT(ctx, borderValue), "");

  ctlbind_expandable_group(ctls, "RECTANGLE", [&, ctx = CTL_CONTEXT(ctx, rectSelection)]() {
    ctlbind(ctls, "ROI:", CTL_CONTEXT(ctx, rc), "");
  });

  ctlbind_expandable_group(ctls, "PLANETARY DISK", [&, ctx = CTL_CONTEXT(ctx, planetaryDiskSelection)]() {
    ctlbind(ctls, "putToCenter", CTL_CONTEXT(ctx, putToCenter));
    ctlbind(ctls, CTL_CONTEXT(ctx, opts));
  });
}

bool select_image_roi(const c_roi_selection_options & opts,
    cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask);

#endif /* __c_roi_selection_h__ */
