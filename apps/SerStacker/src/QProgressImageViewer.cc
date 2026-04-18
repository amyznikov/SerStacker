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
    IMtfDisplay(this, "QProgressImageViewer")
{
  scene()->setBackgroundBrush(Qt::darkGray);

  addDisplay(_displayChannel = "0", -1, -1);

  setDisplayFunction(this);

  connect(this, &QImageViewer::displayImageChanged,
      mtfDisplayEvents(), &QMtfDisplayEvents::displayImageChanged);

  connect(mtfDisplayEvents(), &QMtfDisplayEvents::parameterChanged,
      this, &ThisClass::updateDisplay);

}

//IMtfDisplay * QProgressImageViewer::mtfDisplay()
//{
//  return this;
//}
//
//const IMtfDisplay * QProgressImageViewer::mtfDisplay() const
//{
//  return this;
//}

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

  c_mtf *mtf =
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

void QProgressImageViewer::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;
  getMinMax(currentImage(), minval, maxval, currentMask());
}


void QProgressImageViewer::getMtfCurve(std::vector<float> & cy, size_t n)
{
  displayParams().mtf.get_mtf_curve(cy, n);
}

void QProgressImageViewer::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax, bool cumulative, bool normalized)
{
  createHistogram(currentImage(), currentMask(), hmin, hmax, 0, H, cumulative, normalized);
}

void QProgressImageViewer::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  createHistogram(mtfImage(), currentMask(), hmin, hmax, 0, H);
}


} /* namespace serstacker */
