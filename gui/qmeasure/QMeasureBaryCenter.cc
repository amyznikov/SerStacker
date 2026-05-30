/*
 * QMeasureBaryCenter.cc
 *
 *  Created on: May 30, 2026
 *      Author: amyznikov
 */

#include "QMeasureBaryCenter.h"
#include <core/proc/reduce_channels.h>

QMeasureBaryCenter::QMeasureBaryCenter(const QString & name, const QString & tooltip) :
  Base(name, tooltip)
{
}

QMeasureSettingsWidget* QMeasureBaryCenter::createSettingsWidget(QWidget * parent) const
{
  return new QMeasureBaryCenterSettingsWidget(parent);
}

int QMeasureBaryCenter::compute(const cv::Mat & image, const cv::Mat & mask, cv::Point2d pos[4]) const
{
  cv::Mat imask;
  if( !mask.empty() ) {
    if( mask.channels() == 1 || mask.channels() == image.channels() ) {
      cv::bitwise_not(mask, imask);
    }
    else {
      reduce_color_channels(mask, imask, cv::REDUCE_MIN);
      cv::bitwise_not(imask, imask);
    }
    if( cv::countNonZero(imask) == 0 ) {
      imask.release();
    }
  }

  const int cn = image.channels();

  if ( imask.empty() ) {
    if ( cn == 1 ) {
      const cv::Moments m = cv::moments(image, false);
      pos[0].x = m.m10 / m.m00;
      pos[0].y = m.m01 / m.m00;
    }
    else {
      cv::Mat img;
      for ( int i = 0; i < cn; ++i ) {
        cv::extractChannel(image, img, i);
        const cv::Moments m = cv::moments(image, false);
        pos[i].x = m.m10 / m.m00;
        pos[i].y = m.m01 / m.m00;
      }
    }
  }
  else {
    cv::Mat img, msk;

    for ( int i = 0; i < cn; ++i ) {
      cv::extractChannel(image, img, i);
      if ( imask.channels() == 1 ) {
        msk = imask;
      }
      else {
        cv::extractChannel(imask, msk, i);
      }

      img.setTo(0, msk);

      const cv::Moments m = cv::moments(img, false);
      pos[i].x = m.m10 / m.m00;
      pos[i].y = m.m01 / m.m00;
    }
  }

  return image.channels();
}

QMeasureBaryCenterX::QMeasureBaryCenterX() :
  Base("BaryCenterX", "Compute BaryCenter X value over image ROI")
{
}

int QMeasureBaryCenterX::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  cv::Point2d p[4];
  int cn = Base::compute(image, mask, p);
  for ( int i = 0; i < cn; ++i ) {
    output_value->val[i] = p[i].x;
  }
  return cn;
}

QMeasureBaryCenterY::QMeasureBaryCenterY() :
  Base("BaryCenterY", "Compute BaryCenter Y value over image ROI")
{
}

int QMeasureBaryCenterY::compute(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const
{
  cv::Point2d p[4];
  int cn = Base::compute(image, mask, p);
  for ( int i = 0; i < cn; ++i ) {
    output_value->val[i] = p[i].y;
  }
  return cn;
}
