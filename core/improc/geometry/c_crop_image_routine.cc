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

  ctlbind(ctls, "force output size:", CTL_CONTEXT(ctx, fixOutputSize), "");
  ctlbind(ctls, "forced output size:", CTL_CONTEXT(ctx, outputSize), "");
  ctlbind(ctls, "borderType:", CTL_CONTEXT(ctx, borderType), "");
  ctlbind(ctls, "borderValue:", CTL_CONTEXT(ctx, borderValue), "");

  ctlbind_expandable_group(ctls, "RECT", [&, ctx = CTL_CONTEXT(ctx, manualRectOptions)]() {
    ctlbind(ctls, "ROI:", CTL_CONTEXT(ctx, rc), "");
  });

  ctlbind_expandable_group(ctls, "PLANETARY DISK", [&, ctx = CTL_CONTEXT(ctx, planetaryDiskOptions)]() {
    ctlbind(ctls, "putToCenter", CTL_CONTEXT(ctx, putToCenter));
    ctlbind(ctls, CTL_CONTEXT(ctx, opts));
  });

}

bool c_crop_image_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, mode);
    SERIALIZE_OPTION(settings, save, *this, fixOutputSize);
    SERIALIZE_OPTION(settings, save, *this, outputSize);
    SERIALIZE_OPTION(settings, save, *this, borderType);
    SERIALIZE_OPTION(settings, save, *this, borderValue);

    if ( auto group = SERIALIZE_GROUP(settings, save, "manualRectOptions") ) {
      SERIALIZE_OPTION(group, save, manualRectOptions, rc );
    }

    if( auto group = SERIALIZE_GROUP(settings, save, "planetary_disk_options") ) {
      SERIALIZE_OPTION(group, save, planetaryDiskOptions, putToCenter);
      serialize_base_planetary_disk_detector_options(group, save, planetaryDiskOptions.opts);
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

  cv::Rect target_rect;

  switch (mode) {
    case ROI_SELECTION_NONE: {
      return true;
    }
    case ROI_SELECTION_FIXED_RECT: {
      fOk = !(roi = manualRectOptions.rc).empty();
      break;
    }
    case ROI_SELECTION_GUI: {
      fOk = ctlbind_get_roi(&roi);
      break;
    }
    case ROI_SELECTION_PLANETARY_DISK: {
      cv::Point2f planetary_disk_centroid;

      fOk = simple_planetary_disk_detector(image, mask,
          planetaryDiskOptions.opts,
          &planetary_disk_centroid,
          &roi,
          nullptr,
          nullptr,
          cv::noArray());

      if( fOk && fixOutputSize && !outputSize.empty() ) {
        int cx, cy;

        if( planetaryDiskOptions.putToCenter ) {
          cx = cvRound(planetary_disk_centroid.x);
          cy = cvRound(planetary_disk_centroid.y);
        }
        else {
          cx = roi.x + roi.width / 2;
          cy = roi.y + roi.height / 2;
        }

        target_rect.x = cx - outputSize.width / 2;
        target_rect.y = cy - outputSize.height / 2;
        target_rect.width = outputSize.width;
        target_rect.height = outputSize.height;
      }
      break;
    }

    default:
      CF_ERROR("Bad selection method = %d", mode);
      return false;

  }

  if( !fOk || roi.empty() ) {
    CF_ERROR("The selection ROI is empty or target object not detected");
    return false;
  }

  if( !fixOutputSize || outputSize.empty() ) {
    if( (roi &= cv::Rect(0, 0, size.width, size.height)).empty() ) {
      CF_ERROR("Selection ROI is completely outside the image");
      return false;
    }

    image.getMat()(roi).copyTo(image);
    if( !mask.empty() ) {
      mask.getMat()(roi).copyTo(mask);
    }
  }
  else {
    if( target_rect.empty() ) {
      target_rect = cv::Rect(roi.x, roi.y, outputSize.width, outputSize.height);
    }

    const cv::Rect src_roi = target_rect & cv::Rect(0, 0, size.width, size.height);
    if( src_roi.empty() ) {
      CF_ERROR("Target rect is completely outside the image boundaries");
      return false;
    }

    const int top = src_roi.y - target_rect.y;
    const int bottom = target_rect.y + target_rect.height - (src_roi.y + src_roi.height);
    const int left = src_roi.x - target_rect.x;
    const int right = target_rect.x + target_rect.width - (src_roi.x + src_roi.width);
    const bool has_padding = (top > 0 || bottom > 0 || left > 0 || right > 0);
    const bool need_mask = !mask.empty() || has_padding;

    cv::copyMakeBorder(image.getMat()(src_roi), image, top, bottom, left, right, borderType, borderValue);

    if( !need_mask ) {
      mask.release();
    }
    else {
      cv::Mat outmask = cv::Mat::zeros(outputSize, !mask.empty() ? mask.type() : CV_8UC1);
      const cv::Rect dst_roi(left, top, src_roi.width, src_roi.height);

      if( !mask.empty() ) {
        mask.getMat()(src_roi).copyTo(outmask(dst_roi));
      }
      else {
        outmask(dst_roi).setTo(cv::Scalar(255));
      }
      mask.move(outmask);
    }
  }

  return true;
}
