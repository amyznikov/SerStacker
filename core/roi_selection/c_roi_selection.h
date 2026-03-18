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
#include <core/ctrlbind/ctrlbind.h>

enum roi_selection_method
{
  roi_selection_none = 0,
  roi_selection_rectange_crop = 1,
  roi_selection_planetary_disk = 2,
};

struct c_roi_selection_options
{
  enum roi_selection_method method = roi_selection_none;
  cv::Rect rectangle_roi_selection;
  cv::Size planetary_disk_crop_size;
  double planetary_disk_gbsigma = 1;
  double planetary_disk_stdev_factor = 0.25;
  int se_close_size = 2;
};

template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_roi_selection_options> & ctx)
{
  using S = c_roi_selection_options;
  ctlbind(ctls, "ROI selection:", ctx(&S::method), "");

  ctlbind_expandable_group(ctls, "Rectangle Crop...");
    ctlbind(ctls, "Rectangle:", ctx(&S::rectangle_roi_selection), "");
  ctlbind_end_group(ctls);

  ctlbind_expandable_group(ctls, "Planetary Disk Crop...");
    ctlbind(ctls, "Crop Size:", ctx(&S::planetary_disk_crop_size), "");
    ctlbind(ctls, "gbsigma", ctx(&S::planetary_disk_gbsigma), "");
    ctlbind(ctls, "Stdev factor", ctx(&S::planetary_disk_stdev_factor), "");
    ctlbind(ctls, "se_close_size", ctx(&S::se_close_size), "");
  ctlbind_end_group(ctls);
}

class c_roi_selection
{
public:
  typedef c_roi_selection this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_roi_selection() = default;
  virtual ~c_roi_selection() = default;

  static sptr create(const c_roi_selection_options & opts);

  virtual bool select(cv::InputArray image, cv::InputArray image_mask,
      cv::Rect & outputROIRectangle ) = 0;
};


#endif /* __c_roi_selection_h__ */
