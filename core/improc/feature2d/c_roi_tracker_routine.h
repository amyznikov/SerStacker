/*
 * c_roi_tracker_routine.h
 *
 *  Created on: May 10, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_roi_tracker_routine_h__
#define __c_roi_tracker_routine_h__

#include <core/improc/c_image_processor.h>
#include <core/proc/tracking/c_roi_tracker.h>

class c_roi_tracker_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_roi_tracker_routine,
      "roi_tracker", "c_roi_tracker_routine");

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  c_roi_tracker _tracker;
  c_roi_tracker_options _tracker_opts;
  cv::Rect _roi;
  bool _initialized = false;
};

#endif /* __c_roi_tracker_routine_h__ */
