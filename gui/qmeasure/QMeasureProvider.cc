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

QMeasureProvider::QMeasureProvider(QObject * parent) :
  Base(parent)
{
  measures_.emplace(new QMeasureMinValue());
  measures_.emplace(new QMeasureMaxValue());
  measures_.emplace(new QMeasureMeanValue());
  measures_.emplace(new QMeasureStdevValue());
  measures_.emplace(new QMeasureLPG());
  measures_.emplace(new QMeasureLC());
  measures_.emplace(new QMeasureHarrisCornerResponse());
  measures_.emplace(new QMeasureNormalizedVariance());
  measures_.emplace(new QMeasureSharpnessNorm());
}


const std::set<QMeasure*> QMeasureProvider::measures() const
{
  return measures_;
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
  if( !measures_.empty() ) {

    MeasuredFrame frame;
    cv::Scalar v;

    for( const QMeasure *m : measures_ ) {
      if( m->enabled() ) {
        const int cn = m->compute(image, mask, roi, &v);
        if( cn > 0 ) {
          frame.measurements.emplace_back(m, v, cn);
        }
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

