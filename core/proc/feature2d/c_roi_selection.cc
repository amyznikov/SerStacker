/*
 * c_roi_selection.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_roi_selection.h"
#include "c_roi_rectangle_selection.h"
#include "c_planetary_disk_selection.h"
#include <core/settings/opencv_settings.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<roi_selection_method>()
{
  static const c_enum_member members[] = {
      { roi_selection_none, "none", },
      { roi_selection_rectange_crop, "rectangle", },
      { roi_selection_planetary_disk, "planetary_disk", },
      { roi_selection_none, },
  };
  return members;
}

c_roi_selection::sptr c_roi_selection::create(const c_roi_selection_options & opts)
{
  switch(opts.method)
  {
    case roi_selection_rectange_crop:
      return sptr(new c_roi_rectangle_selection(
          opts.rectangle_roi_selection));

    case roi_selection_planetary_disk:
      return sptr(new c_planetary_disk_selection(
          opts.planetary_disk_crop_size,
          opts.planetary_disk_options.gbsigma,
          opts.planetary_disk_options.stdev_factor,
          opts.planetary_disk_options.se_close_radius));

    default:
      break;
  }

  return nullptr;
}


bool select_image_roi(const c_roi_selection::sptr & roi_selection,
    const cv::Mat & src, const cv::Mat & srcmask,
    cv::Mat & dst, cv::Mat & dstmask)
{
  if ( !roi_selection ) {
    dst = src;
    dstmask = srcmask;
    return true;
  }

  cv::Rect ROI;
  if ( !roi_selection->select(src, srcmask, ROI) || ROI.empty() ) {
    CF_ERROR("roi_selection->select_roi() fails");
    return false;
  }

  dst = src(ROI);

  if ( !srcmask.empty() ) {
    dstmask = srcmask(ROI);
  }
  else {
    dstmask.release();
  }

  return true;
}

bool serialize_base_roi_selection_options(c_config_setting section, bool save,
    c_roi_selection_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, method);
  SERIALIZE_OPTION(section, save, opts, rectangle_roi_selection);
  SERIALIZE_OPTION(section, save, opts, planetary_disk_crop_size);
  if( auto planetary_disk_opts = SERIALIZE_GROUP(section, save, "planetary_disk") ) {
    serialize_base_planetary_disk_detector_options(planetary_disk_opts, save, opts.planetary_disk_options);
  }
  return true;
}
