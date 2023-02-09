/*
 * QFocusMeasureProvider.cc
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#include "QFocusMeasureProvider.h"
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

QFocusMeasureProvider::QFocusMeasureProvider(QObject * parent) :
    Base(parent),
    sprefix_("FocusMeasure")
{
}

c_camera_focus_measure& QFocusMeasureProvider::measure()
{
  return measure_;
}

const c_camera_focus_measure& QFocusMeasureProvider::measure() const
{
  return measure_;
}

int QFocusMeasureProvider::maxMeasurements() const
{
  return max_measurements_;
}

const QVector<double>& QFocusMeasureProvider::measurements(int channel) const
{
  return measurements_[channel];
}

enum COLORID QFocusMeasureProvider::colorid() const
{
  return measure_.avgchannel() ? COLORID_MONO : colorid_;
}

int QFocusMeasureProvider::bpp() const
{
  return bpp_;
}

QMutex& QFocusMeasureProvider::mutex()
{
  return mutex_;
}

bool QFocusMeasureProvider::enabled() const
{
  return enabled_;
}

void QFocusMeasureProvider::setEnabled(bool v)
{
  enabled_ = v;
}

void QFocusMeasureProvider::load_parameters()
{
  if( sprefix_.isEmpty() ) {
    sprefix_ = "FocusMeasure";
  }

//  QSettings settings;

//  measure_.set_avgchannel(settings.value(qsprintf("%s/avgchannel", sprefix_), measure_.avgchannel()).value<bool>());
//  measure_.set_dscale(settings.value(qsprintf("%s/dscale", sprefix_), measure_.dscale()).value<int>());
//  measure_.set_eps(settings.value(qsprintf("%s/eps", sprefix_), measure_.eps()).value<double>());
}

void QFocusMeasureProvider::save_parameters()
{
  if( sprefix_.isEmpty() ) {
    sprefix_ = "FocusMeasure";
  }

//  QSettings settings;

//  settings.setValue(qsprintf("%s/avgchannel", sprefix_), measure_.avgchannel());
//  settings.setValue(qsprintf("%s/dscale", sprefix_), measure_.dscale());
//  settings.setValue(qsprintf("%s/eps", sprefix_), measure_.eps());
}

bool QFocusMeasureProvider::getImageROI(cv::Mat * dst, const cv::Mat & frame, const QRect & rect, bool make_copy) const
{
  if( rect.isEmpty() ) {
    if( make_copy ) {
      frame.copyTo(*dst);
    }
    else {
      *dst = frame;
    }
    return true;
  }

  cv::Rect roi(rect.x() & ~0x1, rect.y() & ~0x1,
      rect.width() & ~0x1, rect.height() & ~0x1);

  if( roi.x + roi.width <= 0 ) {
    roi.width = 0;
  }
  else {
    if( roi.x < 0 ) {
      roi.x = 0;
    }
    if( roi.x + roi.width >= frame.cols ) {
      roi.width = (frame.cols - roi.x) & ~0x1;
    }
  }

  if( roi.y + roi.height <= 0 ) {
    roi.height = 0;
  }
  else {
    if( roi.y < 0 ) {
      roi.y = 0;
    }
    if( roi.y + roi.height >= frame.rows ) {
      roi.height = (frame.rows - roi.y) & ~0x1;
    }
  }

  if( roi.width > 0 && roi.height > 0 ) {
    if( make_copy ) {
      frame(roi).copyTo(*dst);
    }
    else {
      *dst = frame(roi);
    }
    return true;
  }

  return false;
}
