/*
 * QImageViewMtfDisplayFunction.cc
 *
 *  Created on: Jan 6, 2023
 *      Author: amyznikov
 */

#include "QImageViewMtfDisplayFunction.h"
#include <core/mtf/mtf-histogram.h>
#include <core/proc/histogram.h>
#include <core/proc/minmax.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace {
  enum DISPLAY_TYPE {
    DISPLAY_PIXEL_VALUE,
  };
}

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "PIXEL_VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}


QImageViewMtfDisplayFunction::QImageViewMtfDisplayFunction(QImageViewer * imageViewer, const QString & prefix) :
  QMtfDisplay(prefix, imageViewer),
  imageViewer_(imageViewer)
{
  QMtfDisplay::displayType_ =
      DISPLAY_PIXEL_VALUE;

  addDisplay(DISPLAY_PIXEL_VALUE, -1, -1);
}

QImageViewer * QImageViewMtfDisplayFunction::imageViewer() const
{
  return imageViewer_;
}

const c_enum_member * QImageViewMtfDisplayFunction::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QImageViewMtfDisplayFunction::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;

  if ( imageViewer_ ) {

    const cv::Mat & image =
        imageViewer_->currentImage();

    const cv::Mat & mask =
        imageViewer_->currentMask();

    if ( !image.empty() ) {
      getminmax(image, minval, maxval, mask);
    }
  }
}

void QImageViewMtfDisplayFunction::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  if( !imageViewer_ ) {
    H.release();
  }
  else {

    const cv::Mat &image =
        imageViewer_->currentImage();

    const cv::Mat &mask =
        imageViewer_->currentMask();

    if( image.empty() ) {
      H.release();
    }
    else {

      create_histogram(image, mask,
          H,
          hmin, hmax,
          256,
          false,
          false);
    }
  }
}

void QImageViewMtfDisplayFunction::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  if( !imageViewer_ ) {
    H.release();
  }
  else {

    const cv::Mat &image =
        imageViewer_->currentImage();

    const cv::Mat &mask =
        imageViewer_->currentMask();

    if( image.empty() ) {
      H.release();
    }
    else {

      DisplayParams & opts =
          displayParams();

      create_output_histogram(&opts.mtf,
          image, mask,
          H,
          hmin, hmax,
          256,
          false,
          false);
    }
  }
}

void QImageViewMtfDisplayFunction::createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  INSTRUMENT_REGION("");

  if ( !currentImage.empty() ) {

    DisplayParams & opts =
        displayParams();

    const bool needColormap =
        opts.colormap != COLORMAP_NONE &&
            currentImage.channels() == 1 &&
            ddepth == CV_8U;

    applyMtf(currentImage, needColormap ? cv::noArray() : currentMask,
        displayImage, ddepth);

    if ( needColormap && ! displayImage.empty() ) {
      applyColorMap(displayImage, currentMask, displayImage);
    }
  }

//  if ( !currentImage.empty() ) {
//
//    DisplayParams & opts =
//        displayParams();
//
//    c_pixinsight_mtf * mtf =
//        &opts.mtf;
//
//    double imin, imax;
//
//    adjustMtfInputRange(mtf, &imin, &imax);
//
//    mtf->apply(currentImage, displayImage, ddepth);
//
//    restoreMtfInputRange(mtf, imin, imax);
//
//    if ( opts.colormap != COLORMAP_NONE && displayImage.type() == CV_8UC1 ) {
//
//      apply_colormap(displayImage.getMat(),
//          displayImage,
//          opts.colormap);
//
//      if ( !currentMask.empty() ) {
//        displayImage.setTo(0, ~currentMask.getMat());
//      }
//    }
//  }
}


bool QImageViewMtfDisplayFunction::applyMtf(cv::InputArray currentImage, cv::InputArray currentMask,
    cv::OutputArray displayImage, int ddepth)
{
  if( currentImage.empty() ) {
    return false;
  }

  DisplayParams &opts =
      displayParams();

  c_pixinsight_mtf *mtf =
      &opts.mtf;

  double imin, imax;

  adjustMtfInputRange(mtf, &imin, &imax);

  mtf->apply(currentImage, displayImage, ddepth);

  restoreMtfInputRange(mtf, imin, imax);

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


void QImageViewMtfDisplayFunction::adjustMtfInputRange(c_midtones_transfer_function * mtf,
    double * imin, double * imax) const
{
  mtf->get_input_range(imin, imax);

  if( *imin >= *imax && imageViewer_ ) {

    const cv::Mat &currentImage =
        imageViewer_->currentImage();

    if( !currentImage.empty() ) {

      double adjusted_min, adjusted_max;

      const int depth =
          currentImage.depth();

      if( depth == CV_32F || depth == CV_64F ) {
        getminmax(currentImage, &adjusted_min, &adjusted_max, imageViewer_->currentMask());
      }
      else {
        c_midtones_transfer_function::suggest_levels_range(currentImage.depth(),
            &adjusted_min,
            &adjusted_max);
      }

      mtf->set_input_range(adjusted_min, adjusted_max);
    }
  }
}

void QImageViewMtfDisplayFunction::restoreMtfInputRange(c_midtones_transfer_function *mtf, double imin, double imax) const
{
  if( imin >= imax ) {
    mtf->set_input_range(imin, imax);
  }
}
