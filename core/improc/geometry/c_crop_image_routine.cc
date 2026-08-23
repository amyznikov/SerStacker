/*
 * c_crop_image_routine.cc
 *
 *  Created on: Aug 31, 2023
 *      Author: amyznikov
 */

#include "c_crop_image_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_crop_image_routine::SELECTION_MODE>()
{
  static const c_enum_member members[] = {
      { c_crop_image_routine::ROI_SELECTION_NONE,"NONE", ""},
      { c_crop_image_routine::ROI_SELECTION_FIXED_RECT,"RECT", ""},
      { c_crop_image_routine::ROI_SELECTION_GUI, "GUI", "" },
      { c_crop_image_routine::ROI_SELECTION_PLANETARY_DISK, "PLANETARY_DISK", "" },
//      { c_crop_image_routine::ROI_SELECTION_MAXCC, "MAXCC", "" },
      { c_crop_image_routine::ROI_SELECTION_GUI}
  };
  return members;
}


void c_crop_image_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind(ctls, "Selection", CTL_CONTEXT(ctx, mode), "What to select into ROI");

  ctlbind(ctls, "fixed output size:", CTL_CONTEXT(ctx, fixOutputSize), "");
  ctlbind(ctls, "output size:", CTL_CONTEXT(ctx, outputSize), "");

  ctlbind_expandable_group(ctls, "RECT", [&, ctx = CTL_CONTEXT(ctx, fixed_rect_options)]() {
    ctlbind(ctls, "ROI:", CTL_CONTEXT(ctx, rc), "");
  });

  ctlbind_expandable_group(ctls, "PLANETARY DISK", [&, ctx = CTL_CONTEXT(ctx, planetary_disk_options)]() {
    ctlbind(ctls, CTL_CONTEXT(ctx, opts));
  });

}

bool c_crop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, mode);
    SERIALIZE_OPTION(settings, save, *this, fixOutputSize);
    SERIALIZE_OPTION(settings, save, *this, outputSize);

    if ( auto group = SERIALIZE_GROUP(settings, save, "fixed_rect_options") ) {
      SERIALIZE_OPTION(group, save, fixed_rect_options, rc );
    }

    if( auto group = SERIALIZE_GROUP(settings, save, "planetary_disk_options") ) {
      serialize_base_planetary_disk_detector_options(group, save, planetary_disk_options.opts);
    }

    return true;
  }
  return false;
}

bool c_crop_image_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  const cv::Size size = image.size();

  cv::Rect roi;

  bool fOk = false;

  switch (mode) {
    case ROI_SELECTION_NONE: {
      return true;
    }
    case ROI_SELECTION_FIXED_RECT: {
      fOk = !(roi = fixed_rect_options.rc).empty();
      break;
    }
    case ROI_SELECTION_GUI: {
      fOk = ctlbind_get_roi(&roi);
      break;
    }
    case ROI_SELECTION_PLANETARY_DISK: {
      fOk = simple_planetary_disk_detector(image, mask,
          planetary_disk_options.opts,
          nullptr,
          &roi,
          nullptr,
          nullptr,
          cv::noArray());

      if ( fOk && fixOutputSize && !outputSize.empty()) {
        // Expand roi to outputSize still centered
        cv::Point center(roi.x + roi.width / 2, roi.y + roi.height / 2);
        roi.x = center.x - outputSize.width / 2;
        roi.y = center.y - outputSize.height / 2;
        roi.width = outputSize.width;
        roi.height = outputSize.height;
        roi &= cv::Rect(0, 0, size.width, size.height);
      }

      break;
    }

    default:
      CF_ERROR("Bad selection method = %d", mode);
      return false;
  }

  if( !fOk || (roi &= cv::Rect(0, 0, size.width, size.height)).empty() ) {
    CF_ERROR("The selection ROI is empty or target object not detected");
    return false;
  }

  if ( !fixOutputSize || outputSize.empty() ) {
    image.getMat()(roi).copyTo(image);
    if ( !mask.empty() )  {
      mask.getMat()(roi).copyTo(mask);
    }
  }
  else {
    // Copy image.getMat()(roi) to outimage(outroi) centered
    cv::Mat outimage(outputSize, image.type(), cv::Scalar::all(0));
    cv::Mat outmask;

    const int dx = (outputSize.width - roi.width + 1) / 2;
    const int dy = (outputSize.height - roi.height + 1) / 2;

    const cv::Rect dst_rect(dx, dy, roi.width, roi.height);
    const cv::Rect dst_clipped = dst_rect & cv::Rect(0, 0, outputSize.width, outputSize.height);
    if (dst_clipped.empty()) {
      CF_ERROR("dst_clipped is empty");
      return false;
    }

    roi.x += (dst_clipped.x - dst_rect.x);
    roi.y += (dst_clipped.y - dst_rect.y);
    roi.width = dst_clipped.width;
    roi.height = dst_clipped.height;

    image.getMat()(roi).copyTo(outimage(dst_clipped));

    if (!mask.empty()) {
      outmask = cv::Mat::zeros(outputSize, mask.type());
      mask.getMat()(roi).copyTo(outmask(dst_clipped));
    }
    else if (dst_clipped.size() != outputSize) {
      outmask = cv::Mat::zeros(outputSize, CV_8UC1);
      outmask(dst_clipped).setTo(cv::Scalar::all(255));
    }

    image.move(outimage);
    if (!outmask.empty()) {
      mask.move(outmask);
    }
  }

  return true;
}
