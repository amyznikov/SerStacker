/*
 * QMeasureProvider.cc
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#include "QMeasureProvider.h"
#include "QMeasureMinMax.h"
#include "QMeasureMeanValueStdev.h"
#include "QMeasureLPG.h"
#include "QMeasureLC.h"
#include "QMeasureHarrisCornerResponse.h"
#include "QMeasureNormalizedVariance.h"
#include "QMeasureSharpnessNorm.h"
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

QMeasure * QMeasureProvider::available_measures_[] = {
    new QMeasureMinValue(),   // 0
    new QMeasureMaxValue(),   // 1
    new QMeasureMeanValue(),  // 2
    new QMeasureStdevValue(),
    new QMeasureLPG(),
    new QMeasureLC(),
    new QMeasureHarrisCornerResponse(),
    new QMeasureNormalizedVariance(),
    new QMeasureSharpnessNorm(),
    nullptr // end of array
};

QMeasureProvider::QMeasureProvider(QObject * parent) :
  Base(parent)
{

  // QMeasureMeanValue
  selected_measures_.emplace(available_measures_[2]);
}

const std::deque<QMeasureProvider::MeasuredFrame> & QMeasureProvider::measured_frames() const
{
  return measured_frames_;
}

void QMeasureProvider::clear_measured_frames()
{
  measured_frames_.clear();
  Q_EMIT measurementsChanged();
}

void QMeasureProvider::set_max_measured_frames(int v)
{
  max_measured_frames_ = v;

  while ( measured_frames_.size() > max_measured_frames_ ) {
    measured_frames_.pop_front();
  }

  Q_EMIT measurementsChanged();
}

int QMeasureProvider::max_measured_frames() const
{
  return max_measured_frames_;
}

bool QMeasureProvider::compute(cv::InputArray image, cv::InputArray mask, const QRect & roi)
{
  return compute(image, mask, cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()));
}

bool QMeasureProvider::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi)
{
  if( !selected_measures_.empty() ) {

    MeasuredFrame frame;
    cv::Scalar v;

    for( const QMeasure *m : selected_measures_ ) {
      const int cn = m->compute(image, mask, roi, &v);
      if( cn > 0 ) {
        frame.measurements.emplace_back(m, v, cn);
      }
    }

    if( !frame.empty() ) {

      measured_frames_.emplace_back(frame);

      while ( measured_frames_.size() > max_measured_frames_ ) {
        measured_frames_.pop_front();
      }

      Q_EMIT measurementsChanged();
    }
  }

  return true;
}

QMeasure ** QMeasureProvider::available_measures()
{
  return available_measures_;
}

const std::set<QMeasure*> QMeasureProvider::selected_measures() const
{
  return selected_measures_;
}

void QMeasureProvider::clear_selected_measures()
{
  selected_measures_.clear();
}

void QMeasureProvider::add_selected_measure(QMeasure * m)
{
  selected_measures_.emplace(m);
}
