/*
 * c_planetary_disk_selection.cc
 *
 *  Created on: Jul 16, 2021
 *      Author: amyznikov
 */

#include "c_planetary_disk_selection.h"
#include <core/proc/planetary-disk-detector.h>
#include <core/debug.h>


static bool select_crop_rectangle(const cv::Size & image_size, const cv::Size & crop_size,
    const cv::Point2f & crop_center, cv::Rect * ROI)
{
  if ( !crop_size.empty() ) {

    if ( image_size.width < crop_size.width || image_size.height < crop_size.height ) {
      CF_ERROR("ERROR: crop size = %dx%d larger than frame size = %dx%d in planetary_disk_registration",
          crop_size.width, crop_size.height, image_size.width, image_size.height);
      return false;
    }

    int l = cv::max(0, cvRound(crop_center.x - crop_size.width / 2));
    int t = cv::max(0, cvRound(crop_center.y - crop_size.height / 2));
    if ( l + crop_size.width > image_size.width ) {
      l = image_size.width - crop_size.width;
    }
    if ( t + crop_size.height > image_size.height ) {
      t = image_size.height - crop_size.height;
    }

    ROI->x = l;
    ROI->y = t;
    ROI->width = crop_size.width;
    ROI->height = crop_size.height;
  }

  return true;
}


c_planetary_disk_selection::ptr c_planetary_disk_selection::create()
{
  return ptr(new this_class());
}

c_planetary_disk_selection::ptr c_planetary_disk_selection::create(const cv::Size & crop_size)
{
  ptr obj(new this_class());
  obj->set_crop_size(crop_size);
  return obj;
}

bool c_planetary_disk_selection::detect_object_roi(cv::InputArray image, cv::InputArray mask,
    cv::Point2f & outputObjectLocation,
    cv::Rect & outputCropRect )
{
  if ( !simple_planetary_disk_detector(image, mask, &outputObjectLocation, 0, &outputCropRect) ) {
    CF_FATAL("simple_small_planetary_disk_detector() fails");
    return false;
  }

  if ( crop_size_.empty() ) {
    crop_size_ = outputCropRect.size() * 2;
  }

  if ( crop_size_ == image.size() ) {
    outputCropRect = cv::Rect(0,0, crop_size_.width, crop_size_.height);
  }
  else if ( !select_crop_rectangle(image.size(), crop_size_, outputObjectLocation, &outputCropRect) ) {
    CF_FATAL("select_crop_rectangle() fails");
    return false;
  }

  return true;
}

