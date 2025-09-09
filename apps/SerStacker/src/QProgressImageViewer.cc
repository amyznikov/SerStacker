/*
 * QProgressImageViewer.cc
 *
 *  Created on: Dec 20, 2023
 *      Author: amyznikov
 */

#include "QProgressImageViewer.h"
#include <core/proc/minmax.h>
#include <core/proc/histogram.h>
#include <core/debug.h>





namespace serstacker {

QProgressImageViewer::QProgressImageViewer(QWidget * parent) :
    Base(parent),
    IMtfDisplay("QProgressImageViewer")
{
  scene()->setBackgroundBrush(Qt::darkGray);

  addDisplay(_displayChannel = "0", -1, -1);

  setDisplayFunction(this);

  connect(this, &QImageViewer::displayImageChanged,
      this, &ThisClass::displayImageChanged);

  connect(this, &ThisClass::parameterChanged,
      this, &ThisClass::updateDisplay);

}

IMtfDisplay * QProgressImageViewer::mtfDisplay()
{
  return this;
}

const IMtfDisplay * QProgressImageViewer::mtfDisplay() const
{
  return this;
}

void QProgressImageViewer::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::Mat & mtfImage, cv::Mat & displayImage, int ddepth)
{
  if ( !currentImage.empty() ) {

    const DisplayParams & opts =
        displayParams();

    const bool needColormap =
        opts.colormap != COLORMAP_NONE &&
            currentImage.channels() == 1 &&
            ddepth == CV_8U;

    applyMtf(currentImage, needColormap ? cv::noArray() : currentMask,
        mtfImage, ddepth);

    if ( needColormap && !mtfImage.empty() ) {
      applyColorMap(mtfImage, currentMask, displayImage);
    }
    else {
      displayImage = mtfImage;
    }
  }
}


bool QProgressImageViewer::applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  if( currentImage.empty() ) {
    return false;
  }

  DisplayParams &opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  c_mtf_adjustment a;

  adjustMtfRange(mtf, currentImage, currentMask, &a);

  mtf->apply(currentImage, displayImage, ddepth);

  restoreMtfRange(mtf, a);

  if ( currentMask.size() == currentImage.size() ) {
    displayImage.setTo(0, ~currentMask.getMat());
  }

  return true;
}


bool QProgressImageViewer::applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
    cv::OutputArray colormapImage)
{
  if( displayImage.empty() || displayImage.type() != CV_8UC1 ) {
    return false;
  }

  DisplayParams &opts =
      displayParams();

  if( opts.colormap == COLORMAP_NONE || opts.lut.empty() ) {
    return false;
  }

  cv::applyColorMap(displayImage,
      colormapImage,
      opts.lut);

  if( displayMask.size() == colormapImage.size() ) {
    colormapImage.setTo(0, ~displayMask.getMat());
  }

  return true;
}


//const QStringList QProgressImageViewer::displayChannels() const
//{
//  static const c_enum_member members[] = {
//      {0, "VALUE", "PIXEL VALUE"},
//      {0}
//  };
//
//  return members;
//}

void QProgressImageViewer::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;
  getminmax(currentImage(), minval, maxval, currentMask());
}


void QProgressImageViewer::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(currentImage(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}


void QProgressImageViewer::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  create_histogram(mtfImage(),
      currentMask(),
      H,
      hmin, hmax,
      256,
      false,
      false);
}


} /* namespace serstacker */
