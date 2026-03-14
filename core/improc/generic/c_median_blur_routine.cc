/*
 * c_image_processor_routine.cc
 *
 *  Created on: Jul 15, 2022
 *      Author: amyznikov
 */

#include "c_median_blur_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_median_blur_routine::Direction>()
{
  static const c_enum_member members[] = {
    {c_median_blur_routine::DirectionBoth, "Both", ""},
    {c_median_blur_routine::DirectionVert, "Vert", ""},
    {c_median_blur_routine::DirectionHorz, "Horz", ""},
    {c_median_blur_routine::DirectionBoth}
  };

  return members;
}

template<>
const c_enum_member * members_of<c_median_blur_routine::Output>()
{
  static const c_enum_member members[] = {
    {c_median_blur_routine::OutputMedian, "Median", ""},
    {c_median_blur_routine::OutputAbsDiff, "AbsDiff", "absdiff(src, med, dst)"},
    {c_median_blur_routine::OutputDiff, "Diff", "subtract(src, med, dst)"},
    {c_median_blur_routine::OutputNDiff, "NDiff", "subtract(med, src, dst)"},
    {c_median_blur_routine::OutputFillMaskHoles, "FillMaskHoles", "Fill masked pixels with median"},

    {c_median_blur_routine::OutputMedian}
  };

  return members;
}

void c_median_blur_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_spinbox(ctls, "radius", ctx(&this_class::_radius),1, 50, 1, "Kernel radius");
  ctlbind_spinbox(ctls, "iterations", ctx(&this_class::_iterations), 0, 20, 1, "Number of iterations");
  ctlbind(ctls, "direction", ctx(&this_class::_direction), "The filter orientation - vertical, horizontal or both");
  ctlbind(ctls, "output", ctx(&this_class::_output), "AbsDiff: absdiff(src, med) Diff: subtract(src, med) NDiff: subtract(med, src)");
}

bool c_median_blur_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_OPTION(settings, save, *this, _radius);
    SERIALIZE_OPTION(settings, save, *this, _iterations);
    SERIALIZE_OPTION(settings, save, *this, _direction);
    SERIALIZE_OPTION(settings, save, *this, _output);
    return true;
  }
  return false;
}

bool c_median_blur_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( _output == OutputFillMaskHoles && mask.empty() ) {
    return true;
  }

  const int ksize = 2 * _radius + 1;

  // Backup source image for the case if special output is requested
  const cv::Mat Src = image.getMat();
  cv::Mat Med(Src.size(), Src.type());

  for( int i = 0; i < _iterations; ++i ) {

    const cv::Mat src = i == 0 ? Src : Med;

    switch(_direction ) {
      case DirectionBoth: {
        cv::medianBlur(src, Med, ksize);
        break;
      }
      case DirectionVert:
        for (  int i = 0, n = src.cols; i < n; ++i ) {
          cv::medianBlur(src.col(i), Med.col(i), ksize);
        }
        break;
      case DirectionHorz:
        for (  int i = 0, n = src.rows; i < n; ++i ) {
          cv::medianBlur(src.row(i), Med.row(i), ksize);
        }
        break;
    }
  }

  switch( _output ) {
    case OutputAbsDiff:
      cv::absdiff(Src, Med, image);
      break;
    case OutputDiff:
      cv::subtract(Src, Med, image);
      break;
    case OutputNDiff:
      cv::subtract(Med, Src, image);
      break;
    case OutputFillMaskHoles:
      Med.copyTo(image, ~mask.getMat());
      break;
    case OutputMedian:
      image.move(Med);
      break;
  }

  return true;
}
