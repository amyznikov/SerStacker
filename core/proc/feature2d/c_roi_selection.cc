/*
 * c_roi_selection.cc
 *
 *  Created on: Sep 28, 2021
 *      Author: amyznikov
 */

#include "c_roi_selection.h"
#include <core/settings/opencv_settings.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<ROI_SELECTION_MODE>()
{
  static const c_enum_member members[] = {
      { ROI_SELECTION_NONE, "NONE", },
      { ROI_SELECTION_RECT, "RECTANGLE", },
      { ROI_SELECTION_PLANETARY_DISK, "PLANETARY_DISK", },
      { ROI_SELECTION_GUI, "GUI", },
      { ROI_SELECTION_NONE, },
  };
  return members;
}

bool select_image_roi(const c_roi_selection_options & opts,
    cv::InputArray src, cv::InputArray src_mask,
    cv::OutputArray dst, cv::OutputArray dst_mask)
{
  const cv::Size size = src.size();
  cv::Rect roi;
  bool fOk = false;

  cv::Rect target_rect;

  switch (opts.mode) {
    case ROI_SELECTION_NONE: {
      return true;
    }
    case ROI_SELECTION_RECT: {
      fOk = !(roi = opts.rectSelection.rc).empty();
      break;
    }
    case ROI_SELECTION_GUI: {
      fOk = ctlbind_get_roi(&roi);
      break;
    }
    case ROI_SELECTION_PLANETARY_DISK: {
      cv::Point2f planetary_disk_centroid;

      fOk = simple_planetary_disk_detector(src, src_mask,
          opts.planetaryDiskSelection.opts,
          &planetary_disk_centroid,
          &roi,
          nullptr,
          nullptr,
          cv::noArray());

      if( fOk && opts.fixOutputSize && !opts.outputSize.empty() ) {
        int cx, cy;

        if( opts.planetaryDiskSelection.putToCenter ) {
          cx = cvRound(planetary_disk_centroid.x);
          cy = cvRound(planetary_disk_centroid.y);
        }
        else {
          cx = roi.x + roi.width / 2;
          cy = roi.y + roi.height / 2;
        }

        target_rect.x = cx - opts.outputSize.width / 2;
        target_rect.y = cy - opts.outputSize.height / 2;
        target_rect.width = opts.outputSize.width;
        target_rect.height = opts.outputSize.height;
      }
      break;
    }

    default:
      CF_ERROR("Bad selection method = %d", opts.mode);
      return false;

  }

  if( !fOk || roi.empty() ) {
    CF_ERROR("The selection ROI is empty or target object not detected");
    return false;
  }

  if( !opts.fixOutputSize || opts.outputSize.empty() ) {
    if( (roi &= cv::Rect(0, 0, size.width, size.height)).empty() ) {
      CF_ERROR("Selection ROI is completely outside the image");
      return false;
    }

    src.getMat()(roi).copyTo(dst);
    if( !src_mask.empty() ) {
      src_mask.getMat()(roi).copyTo(dst_mask);
    }
  }
  else {
    if( target_rect.empty() ) {
      target_rect = cv::Rect(roi.x, roi.y, opts.outputSize.width, opts.outputSize.height);
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
    const bool need_mask = !src_mask.empty() || has_padding;

    cv::copyMakeBorder(src.getMat()(src_roi), dst, top, bottom, left, right,
        opts.borderType, opts.borderValue);

    if( !need_mask ) {
      dst_mask.release();
    }
    else {
      cv::Mat outmask = cv::Mat::zeros(opts.outputSize, !src_mask.empty() ? src_mask.type() : CV_8UC1);
      const cv::Rect dst_roi(left, top, src_roi.width, src_roi.height);

      if( !src_mask.empty() ) {
        src_mask.getMat()(src_roi).copyTo(outmask(dst_roi));
      }
      else {
        outmask(dst_roi).setTo(cv::Scalar(255));
      }
      dst_mask.move(outmask);
    }
  }

  return true;
}

bool serialize_base_roi_selection_options(c_config_setting section, bool save,
    c_roi_selection_options & opts)
{
  SERIALIZE_OPTION(section, save, opts, mode);
  SERIALIZE_OPTION(section, save, opts, fixOutputSize);
  SERIALIZE_OPTION(section, save, opts, outputSize);
  SERIALIZE_OPTION(section, save, opts, borderType);
  SERIALIZE_OPTION(section, save, opts, borderValue);

  if ( auto group = SERIALIZE_GROUP(section, save, "rectSelection") ) {
    SERIALIZE_OPTION(group, save, opts.rectSelection, rc );
  }

  if( auto group = SERIALIZE_GROUP(section, save, "planetaryDiskSelection") ) {
    SERIALIZE_OPTION(group, save, opts.planetaryDiskSelection, putToCenter);
    serialize_base_planetary_disk_detector_options(group, save, opts.planetaryDiskSelection.opts);
  }

  return true;
}
