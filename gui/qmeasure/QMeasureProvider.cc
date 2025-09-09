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

using MeasuresCollection = QMeasureProvider::MeasuresCollection;
using MeasureRequest = QMeasureProvider::MeasureRequest;

QStringList QMeasureProvider::_requested_channels;
MeasureRequest QMeasureProvider::_requested_measurements;
std::vector<QMeasure*> QMeasureProvider::_available_measures;
QMeasure* QMeasureProvider::_valueMeasure;

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

const std::vector<QMeasure*>& QMeasureProvider::available_measures()
{
  static std::mutex mtx_;
  std::lock_guard<std::mutex> lock(mtx_);

  if( _available_measures.empty() ) {
    _available_measures.emplace_back(_valueMeasure = new QMeasureCentralPixelValue());
    _available_measures.emplace_back(new QMeasureMinValue());
    _available_measures.emplace_back(new QMeasureMaxValue());
    _available_measures.emplace_back(new QMeasureMeanValue());
    _available_measures.emplace_back(new QMeasureStdevValue());
    _available_measures.emplace_back(new QMeasureLPG());
    _available_measures.emplace_back(new QMeasureLC());
    _available_measures.emplace_back(new QMeasureLAP());
    _available_measures.emplace_back(new QMeasureHarrisCornerResponse());
    _available_measures.emplace_back(new QMeasureNormalizedVariance());
    _available_measures.emplace_back(new QMeasureSharpnessNorm());
    _available_measures.emplace_back(new QMeasureNoise());
  }

  return _available_measures;
}

const QMeasure* QMeasureProvider::valueMeasure()
{
  return _valueMeasure;
}

const QStringList & QMeasureProvider::requested_channels()
{
  return _requested_channels;
}

void QMeasureProvider::set_requested_channels(QStringList & v)
{
  _requested_channels = v;
}

const MeasureRequest & QMeasureProvider::requested_measures()
{
  return _requested_measurements;
}

void QMeasureProvider::request_measures(const MeasuresCollection * r)
{
  _requested_measurements.emplace(r);
}

void QMeasureProvider::remove_measure_request(const MeasuresCollection * r)
{
  _requested_measurements.erase(r);
}

bool QMeasureProvider::adjust_roi(const cv::Rect & src_roi, const cv::Size & image_size, cv::Rect * dst_roi)
{
  const int l =
      (std::min)(image_size.width - 1, (std::max)(0, src_roi.x));

  const int t =
      (std::min)(image_size.height - 1, (std::max)(0, src_roi.y));

  const int r =
      (std::min)(image_size.width - 1, (std::max)(0, src_roi.x + src_roi.width - 1));

  const int b =
      (std::min)(image_size.height - 1, (std::max)(0, src_roi.y + src_roi.height - 1));

  *dst_roi = cv::Rect(l, t, r - l + 1, b - t + 1);

  return !dst_roi->empty();
}

bool QMeasureProvider::compute(MeasuredFrame * frame, cv::InputArray image, cv::InputArray mask, const QRect & roi)
{
  return compute(frame, image, mask, cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()));
}

bool QMeasureProvider::compute(MeasuredFrame *frame, cv::InputArray image, cv::InputArray mask, const cv::Rect &roi)
{
  if (image.empty() || _requested_measurements.empty()) {
    return false;
  }

  MeasuresCollection requited_measures;
  for (const auto &r : _requested_measurements) {
    requited_measures.insert(r->begin(), r->end());
  }
  if (requited_measures.empty()) {
    return false;
  }

  cv::Rect rc;
  if( !adjust_roi(roi, image.size(), &rc) ) {
    return false;
  }

  if ( mask.empty() ) {

    const cv::Mat img = image.getMat()(rc);
    const cv::Mat msk;

    frame->mcc = cv::Scalar::all(mask.empty() ? rc.area() : cv::countNonZero(mask.getMat()(rc)));
    frame->mcn = 1;

    cv::Scalar v;
    int cn;

    for (const QMeasure *m : requited_measures) {
      if( (cn = m->compute(img, msk, &v)) > 0 ) {
        frame->measurements.emplace_back(m, v, cn);
      }
    }
  }
  else if ( mask.channels() == 1) {

    const cv::Mat img = image.getMat()(rc);
    const cv::Mat msk = mask.getMat()(rc);

    frame->mcc = cv::Scalar::all(mask.empty() ? rc.area() : cv::countNonZero(mask.getMat()(rc)));
    frame->mcn = 1;

    cv::Scalar v;
    int cn;

    for (const QMeasure *m : requited_measures) {
      if( (cn = m->compute(img, msk, &v)) > 0 ) {
        frame->measurements.emplace_back(m, v, 1);
      }
    }
  }
  else if (image.channels() == mask.channels()) {

    std::vector<cv::Mat> image_channels;
    std::vector<cv::Mat> mask_channels;

    cv::split(image, image_channels);
    cv::split(mask, mask_channels);

    const int cn = image_channels.size();

    frame->mcn = cn;
    for (int c = 0; c < cn; ++c) {
      frame->mcc.val[c] = cv::countNonZero(mask_channels[c](rc));
    }

    for (const QMeasure *m : requited_measures) {

      cv::Scalar v;

      for (int c = 0; c < cn; ++c) {
        const cv::Mat img = image_channels[c](rc);
        const cv::Mat msk = mask_channels[c](rc);

        cv::Scalar cv;
        m->compute(img, msk, &cv);
        v.val[c] = cv.val[0];
      }

      frame->measurements.emplace_back(m, v, cn);
    }
  }
  else if (image.channels() == 1 ) {

    const int cn = mask.channels();

    std::vector<cv::Mat> image_channels(cn, image.getMat());
    std::vector<cv::Mat> mask_channels;
    cv::split(mask, mask_channels);

    frame->mcn = cn;
    for (int c = 0; c < cn; ++c) {
      frame->mcc.val[c] = cv::countNonZero(mask_channels[c](rc));
    }

    for (const QMeasure *m : requited_measures) {

      cv::Scalar v;

      for (int c = 0; c < cn; ++c) {
        const cv::Mat img = image_channels[c](rc);
        const cv::Mat msk = mask_channels[c](rc);

        cv::Scalar cv;
        m->compute(img, msk, &cv);
        v.val[c] = cv.val[0];
      }

      frame->measurements.emplace_back(m, v, cn);
    }
  }
  else {
    CF_ERROR("Not supported combination of image.channels()=%d and mask.channels()=%d",
        image.channels(), mask.channels());
    return false;
  }

  return true;
}
