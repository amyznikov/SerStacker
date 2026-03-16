/*
 * QImageViewMtfDisplayFunction.cc
 *
 *  Created on: Jan 6, 2023
 *      Author: amyznikov
 */

#include "QImageViewMtfDisplayFunction.h"
#include <core/proc/histogram.h>
#include <core/proc/pixtype.h>
#include <core/proc/minmax.h>
#include <core/ssprintf.h>
#include <core/debug.h>

QImageViewMtfDisplayFunction::QImageViewMtfDisplayFunction(QImageViewer * imageViewer, const QString & prefix) :
  QMtfDisplay(prefix, imageViewer),
  _imageViewer(imageViewer)
{
  QMtfDisplay::_displayChannel = "PIXEL_VALUE";
  QMtfDisplay::addDisplay(QMtfDisplay::_displayChannel, -1, -1);
}

QImageViewer * QImageViewMtfDisplayFunction::imageViewer() const
{
  return _imageViewer;
}

void QImageViewMtfDisplayFunction::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;

  if ( _imageViewer ) {

    const cv::Mat & image =
        _imageViewer->currentImage();

    const cv::Mat & mask =
        _imageViewer->currentMask();

    if ( !image.empty() ) {
      getminmax(image, minval, maxval, mask);
    }
  }
}
//
//void QImageViewMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
//{
//  if( !imageViewer_ ) {
//    H.release();
//  }
//  else {
//
//    const cv::Mat &image =
//        imageViewer_->currentImage();
//
//    const cv::Mat &mask =
//        imageViewer_->currentMask();
//
//    if( image.empty() ) {
//      H.release();
//    }
//    else {
//
//      create_histogram(image, mask,
//          H,
//          hmin, hmax,
//          256,
//          false,
//          false);
//    }
//  }
//}

void QImageViewMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax, bool cumulative, bool normalized)
{
  if ( !_imageViewer ) {
    H.release();
  }
  else {
    createHistogram(_imageViewer->currentImage(),
        _imageViewer->currentMask(),
        hmin, hmax,
        0,
        H,
        cumulative,
        normalized);
  }

}


void QImageViewMtfDisplayFunction::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  if ( _imageViewer ) {

    cv::Mat image, mask;

    double scale = 1.0;
    double offset = 0.0;

    const cv::Mat & currentImage =
        _imageViewer->mtfImage();

    _imageViewer->currentMask().copyTo(mask);

    if ( currentImage.depth() == CV_8U ) {
      currentImage.copyTo(image);
    }
    else if ( currentImage.depth() == CV_32F || currentImage.depth() == CV_64F ) {

      const double dstmin = 0, dstmax = 255;
      double srcmin = 0, srcmax = 1;

      cv::minMaxLoc(currentImage, &srcmin, &srcmax, nullptr, nullptr, mask);

      scale = (dstmax - dstmin) / (srcmax - srcmin);
      offset = dstmin - scale * srcmin;

      currentImage.convertTo(image, CV_8U, scale, offset);
    }
    else {
      getScaleOffset(currentImage.depth(), CV_8U, &scale, &offset);
      currentImage.convertTo(image, CV_8U, scale, offset);
    }

    createHistogram(image, mask, hmin, hmax, 0, H);

//    create_histogram(image, mask,
//        H,
//        hmin, hmax,
//        256,
//        false,
//        false);

    (*hmin -= offset) /= scale;
    (*hmax -= offset) /= scale;
  }

}

void QImageViewMtfDisplayFunction::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
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


void QImageViewMtfDisplayFunction::getMtfCurve(std::vector<float> & cy, size_t n)
{
  DisplayParams &opts = displayParams();
  c_mtf *mtf = &opts.mtf;
  mtf->get_mtf_curve(cy, n);

}

bool QImageViewMtfDisplayFunction::applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
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

bool QImageViewMtfDisplayFunction::applyColorMap(cv::InputArray displayImage, cv::InputArray displayMask,
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

