/*
 * c_roi_tracker_routine.cc
 *
 *  Created on: May 10, 2026
 *      Author: amyznikov
 */

#include "c_roi_tracker_routine.h"

//static const bool


void c_roi_tracker_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_command_button(ctls, "Grab ROI", ctx, [](this_class * _this) {
    _this->_tracker.release();
    if (  ctlbind_get_roi(&_this->_roi) && !_this->_roi.empty() ) {
      _this->_initialized = true;
      CF_DEBUG("ROI: {%d, %d %dx%d}", _this->_roi.x, _this->_roi.y, _this->_roi.width, _this->_roi.height);
    }
    else {
      _this->_initialized = false;
      CF_DEBUG("Invalid or not visible ROI");
    }
    return false;
  });

  ctlbind_expandable_group(ctls, "tracker",
      [&, ctx = CTL_CONTEXT(ctx,_tracker_opts)]() {
        ctlbind(ctls, ctx);
      });
}

bool c_roi_tracker_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    if ( c_config_setting tracker = SERIALIZE_GROUP(settings, save, "tracker") ) {
      ::serialize(_tracker_opts, tracker, save);
    }

    return true;
  }
  return false;
}

bool c_roi_tracker_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !_initialized ) {
    CF_ERROR("ROI NOT INITIALIZED");
    return false;
  }

  if( !_tracker.initialized() && !_tracker.initialize(_tracker_opts) ) {
    CF_ERROR("_tracker.initialize() fails");
    return false;
  }

  bool updated = false;
  if ( !_tracker.track(image, _roi, &updated) ) {
    CF_ERROR("_tracker.track() fails");
  }

  CF_DEBUG("ROI: {%d, %d %dx%d} updated=%d", _roi.x, _roi.y, _roi.width, _roi.height, updated);
  if ( updated ) {
    ctlbind_update_roi(_roi);
  }

  return true;
}

