/*
 * QMeasureProvider.cc
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#include "QMeasureProvider.h"
#include "QMeasureMinMax.h"
#include "QMeasureMeanStdev.h"
#include "QMeasureLPG.h"
#include "QMeasureLC.h"
#include "QMeasureHarrisCornerResponse.h"
#include "QMeasureNormalizedVariance.h"
#include "QMeasureSharpnessNorm.h"
#include "QMeasureNoise.h"
#include "QMeasureLAP.h"

#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

std::set<const std::set<QMeasure*>*> QMeasureProvider::requested_measures_;
std::deque<QMeasureProvider::MeasuredFrame> QMeasureProvider::measured_frames_;
std::vector<QMeasure*> QMeasureProvider::measures_;
int QMeasureProvider::max_measured_frames_ = 200;

QMeasureProvider::QMeasureProvider(QObject * parent) :
  Base(parent)
{
}

QMeasureProvider * QMeasureProvider::instance()
{
  static QMeasureProvider * instance_ =
      new QMeasureProvider();

  return instance_;
}

const std::vector<QMeasure*>& QMeasureProvider::measures()
{
  static std::mutex mtx_;
  std::lock_guard<std::mutex> lock(mtx_);

  if( measures_.empty() ) {
    measures_.emplace_back(new QMeasureCentralPixelValue());
    measures_.emplace_back(new QMeasureMinValue());
    measures_.emplace_back(new QMeasureMinNonZeroValue());
    measures_.emplace_back(new QMeasureMaxValue());
    measures_.emplace_back(new QMeasureMeanValue());
    measures_.emplace_back(new QMeasureStdevValue());
    measures_.emplace_back(new QMeasureLPG());
    measures_.emplace_back(new QMeasureLC());
    measures_.emplace_back(new QMeasureLAP());
    measures_.emplace_back(new QMeasureHarrisCornerResponse());
    measures_.emplace_back(new QMeasureNormalizedVariance());
    measures_.emplace_back(new QMeasureSharpnessNorm());
    measures_.emplace_back(new QMeasureNoise());
  }

  return measures_;
}

const std::deque<QMeasureProvider::MeasuredFrame> & QMeasureProvider::measured_frames()
{
  return measured_frames_;
}

void QMeasureProvider::clear_measured_frames()
{
  measured_frames_.clear();
  Q_EMIT instance()->measurementsChanged();
}

void QMeasureProvider::set_max_measured_frames(int v)
{
  max_measured_frames_ = v;

  while ( measured_frames_.size() > max_measured_frames_ ) {
    measured_frames_.pop_front();
  }

  Q_EMIT instance()->measurementsChanged();
}

int QMeasureProvider::max_measured_frames()
{
  return max_measured_frames_;
}

const std::set<const std::set<QMeasure*>*>& QMeasureProvider::requested_measures()
{
  return requested_measures_;
}

void QMeasureProvider::request_measures(const std::set<QMeasure*> * r)
{
  requested_measures_.emplace(r);
}

void QMeasureProvider::remove_measure_request(const std::set<QMeasure*> * r)
{
  requested_measures_.erase(r);
}

bool QMeasureProvider::compute(cv::InputArray image, cv::InputArray mask, const QRect & roi)
{
  return compute(image, mask, cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()));
}

bool QMeasureProvider::compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi)
{
  if( image.empty() ) {
    return false;
  }

  if( !requested_measures_.empty() ) {

    std::set<QMeasure*> requited_measures;

    for( const auto &r : requested_measures_ ) {
      requited_measures.insert(r->begin(), r->end());
    }

    if ( !requited_measures.empty() ) {

      MeasuredFrame frame;
      cv::Scalar v;

      for( const QMeasure *m : requited_measures ) {
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

        Q_EMIT instance()->measurementsChanged();
      }
    }
  }

  return true;
}

