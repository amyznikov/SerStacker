/*
 * QLiveStereoCalibrationPipeline.cc
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#include "QLiveStereoCalibrationPipeline.h"

namespace serimager {

QLiveStereoCalibrationPipeline::QLiveStereoCalibrationPipeline(const QString & name, QObject * parent) :
    Base(name, parent)
{

}

bool QLiveStereoCalibrationPipeline::processFrame(const cv::Mat & image, COLORID colorid, int bpp)
{
  cv::Mat currentImage;

  displayColorid_ =
      colorid == COLORID_MONO ? COLORID_MONO :
          COLORID_BGR;

  if( !Base::convertImage(image, colorid, bpp, &currentImage, displayColorid_, CV_8U) ) {
    CF_ERROR("convertInputImage() fails");
    return false;
  }

  displayImage_ =
      currentImage;

  return true;
}

bool QLiveStereoCalibrationPipeline::getDisplayImage(cv::Mat * displayImage, COLORID * colorid, int * bpp)
{
  *displayImage = displayImage_;
  *colorid = displayColorid_;
  *bpp = 8;
  return true;
}

} /* namespace serimager */