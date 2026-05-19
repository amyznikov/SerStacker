/*
 * c_planetary_disk_selection.cc
 *
 *  Created on: Jul 16, 2021
 *      Author: amyznikov
 */

#include "c_planetary_disk_selection.h"
#include <core/proc/feature2d/planetary-disk-detection.h>
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


c_planetary_disk_selection::c_planetary_disk_selection()
{
}

c_planetary_disk_selection::c_planetary_disk_selection(const cv::Size & crop_size, double gbsigma, double stdev_factor, int se_close_size) :
    _crop_size(crop_size),
    _gbsigma(gbsigma),
    _stdev_factor(stdev_factor),
    _se_close_size(se_close_size)
{
}

const cv::Size & c_planetary_disk_selection::crop_size() const
{
  return _crop_size;
}

void c_planetary_disk_selection::set_crop_size(const cv::Size & size)
{
  _crop_size = size;
}

void c_planetary_disk_selection::set_gbsigma(double v)
{
  _gbsigma = v;
}

double c_planetary_disk_selection::gbsigma() const
{
  return _gbsigma;
}

void c_planetary_disk_selection::set_stdev_factor(double v)
{
  _stdev_factor = v;
}

double c_planetary_disk_selection::stdev_factor() const
{
  return _stdev_factor;
}

void c_planetary_disk_selection::set_se_close_size(int v)
{
  _se_close_size = v;
}

int c_planetary_disk_selection::se_close_size() const
{
  return _se_close_size;
}

c_planetary_disk_selection::ptr c_planetary_disk_selection::create()
{
  return ptr(new this_class());
}

c_planetary_disk_selection::ptr c_planetary_disk_selection::create(const cv::Size & crop_size, double gbsigma, double stdev_factor, int se_close_size)
{
  return ptr(new this_class(crop_size, gbsigma, stdev_factor, se_close_size));
}

bool c_planetary_disk_selection::select(cv::InputArray image, cv::InputArray mask,
    cv::Rect & outputROIRectangle )
{
  if ( !simple_planetary_disk_detector(image, mask, _gbsigma, _stdev_factor, _se_close_size, &_objpos, &_objrect) ) {
    CF_FATAL("simple_small_planetary_disk_detector() fails");
    return false;
  }

  if ( _crop_size.empty() ) {
    _crop_size = _objrect.size() * 2;
  }

  if ( _crop_size == image.size() ) {
    _objrect = cv::Rect(0,0, _crop_size.width, _crop_size.height);
  }
  else if ( !select_crop_rectangle(image.size(), _crop_size, _objpos, &_objrect) ) {
    CF_FATAL("select_crop_rectangle() fails");
    return false;
  }

  outputROIRectangle = _objrect;

  return true;
}

const cv::Point2f & c_planetary_disk_selection::detected_object_position() const
{
  return _objpos;
}

const cv::Rect & c_planetary_disk_selection::detected_object_roi() const
{
  return _objrect;
}

