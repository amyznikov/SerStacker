/*
 * QMtfImageDisplayFunction.cc
 *
 *  Created on: Apr 22, 2022
 *      Author: amyznikov
 */

#include "QMtfImageDisplayFunction.h"
#include <core/mtf/mtf-histogram.h>
#include <core/proc/histogram.h>
#include <core/proc/minmax.h>
#include <core/ssprintf.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QMtfImageDisplaySettings::QMtfImageDisplaySettings(QObject * parent) :
    Base(parent)
{
}

void QMtfImageDisplaySettings::setCurrentImage(cv::InputArray image, cv::InputArray mask)
{
  currentImage_ = image.getMat();
  currentMask_ = mask.getMat();
  emit inputDataChanged();
}

const cv::Mat & QMtfImageDisplaySettings::currentImage() const
{
  return currentImage_;
}

const cv::Mat & QMtfImageDisplaySettings::currentMask() const
{
  return currentMask_;
}

void QMtfImageDisplaySettings::getInputDataRange(double * minval, double * maxval) const
{
  *minval = *maxval = 0;
  if ( !currentImage_.empty() ) {
    getminmax(currentImage_, minval, maxval, currentMask_);
  }
}

void QMtfImageDisplaySettings::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  if ( currentImage_.empty() ) {
    H.release();
  }
  else {

    if( *hmin >= *hmax ) {
      getminmax(currentImage_, hmin, hmax, currentMask_);
    }

    create_histogram(currentImage_, currentMask_,
        H,
        hmin, hmax,
        256,
        false,
        false);
  }
}

void QMtfImageDisplaySettings::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  if ( currentImage_.empty() ) {
    H.release();
  }
  else {
    create_output_histogram(&Base::mtf(),
        currentImage_, currentMask_,
        H,
        hmin, hmax,
        256,
        false,
        false);
  }
}


void QMtfImageDisplaySettings::getDisplayImage(cv::OutputArray displayImage, int ddepth)
{
  if ( !currentImage_.empty() ) {

    const QMtfDisplaySettings::DisplayParams & opts =
        displayParams();

    opts.mtf.apply(currentImage_, displayImage, ddepth);

    if ( opts.colormap != COLORMAP_NONE && displayImage.type() == CV_8UC1 ) {
      apply_colormap(displayImage.getMat(), displayImage, opts.colormap);

      if ( !currentMask_.empty() ) {
        displayImage.setTo(0, ~currentMask_);
      }
    }
  }
}


QMtfImageDisplayFunction::QMtfImageDisplayFunction(QObject * parent) :
  Base(parent)
{
}

QMtfImageDisplayFunction::QMtfImageDisplayFunction(QMtfImageDisplaySettings * settings, QObject * parent) :
  Base(parent),
  displaySettings_(Q_NULLPTR)
{
  setDisplaySettings(settings);
}

void QMtfImageDisplayFunction::setDisplaySettings(QMtfImageDisplaySettings * settings)
{
  if( displaySettings_ ) {
    disconnect(displaySettings_, &QMtfImageDisplaySettings::updateDisplay,
        this, &ThisClass::update);
  }

  if( (displaySettings_  = settings) )  {
    connect(displaySettings_, &QMtfImageDisplaySettings::updateDisplay,
        this, &ThisClass::update);
  }
}

QMtfImageDisplaySettings * QMtfImageDisplayFunction::displaySettings()
{
  return displaySettings_;
}

const QMtfImageDisplaySettings * QMtfImageDisplayFunction::displaySettings() const
{
  return displaySettings_;
}

void QMtfImageDisplayFunction::setCurrentImage(cv::InputArray image, cv::InputArray mask)
{
  if ( displaySettings_ ) {
    displaySettings_->setCurrentImage(image, mask);
  }
}

void QMtfImageDisplayFunction::getDisplayImage(cv::OutputArray image, int ddepth)
{
  if ( displaySettings_ ) {
    displaySettings_->getDisplayImage(image, ddepth);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

QMtfImageDefaultDisplaySettings::QMtfImageDefaultDisplaySettings(QObject * parent)
{
  Base::displayType_ = DISPLAY_PIXEL_VALUE;
  addDisplay(displayParams_, DISPLAY_PIXEL_VALUE, -1, -1);
}

const c_enum_member * QMtfImageDefaultDisplaySettings::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QMtfImageDefaultDisplaySettings::loadParameters()
{
  Base::loadParameters("QMtfImageDefaultDisplaySettings");
}

void QMtfImageDefaultDisplaySettings::saveParameters() const
{
  Base::saveParameters("QMtfImageDefaultDisplaySettings");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
