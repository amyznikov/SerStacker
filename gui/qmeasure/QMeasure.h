/*
 * QMeasure.h
 *
 *  Created on: Apr 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasure_h__
#define __QMeasure_h__

#include <gui/widgets/QSettingsWidget.h>
#include <opencv2/opencv.hpp>

class QMeasureSettingsWidget;

class QMeasure
{
public:
  QMeasure(const QString & name, const QString & tooltip) :
      name_(name),
      tooltip_(tooltip)
  {
  }

  virtual ~QMeasure() = default;

  const QString & name() const
  {
    return name_;
  }

  const QString & tooltip() const
  {
    return tooltip_;
  }

  virtual bool hasOptions() const
  {
    return false;
  }

  virtual QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const
  {
    return nullptr;
  }

  int compute(cv::InputArray image, cv::InputArray mask, const QRect& roi, cv::Scalar * output_value) const
  {
    return compute(image, mask, cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()), output_value);
  }

  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * output_value) const
  {
    const cv::Mat src =
        image.getMat();

    const cv::Mat1b msk =
        mask.getMat();

    const int cn =
        src.channels();

    cv::Rect rc;

    if( !adjust_roi(roi, src.size(), &rc) ) {
      return 0;
    }

    return compute_measure(src(rc), msk.empty() ? cv::Mat() : msk(rc), output_value);
  }

  static bool adjust_roi(const cv::Rect & src_roi, const cv::Size & image_size, cv::Rect * dst_roi)
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

protected:
  virtual int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const = 0;

protected:
  const QString name_;
  const QString tooltip_;
};

Q_DECLARE_METATYPE(const QMeasure*);
Q_DECLARE_METATYPE(QMeasure*);


class QMeasureSettingsWidget:
    public QSettingsWidget
{
public:
  typedef QMeasureSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QMeasureSettingsWidget(QWidget * parent = nullptr) :
    Base("", parent)
  {
  }

  virtual void setCurrentMeasure(QMeasure * m) = 0;
  virtual QMeasure * currentMeasure() const = 0;
};

template<class MeasureType>
class QMeasureSettingsWidgetImpl:
    public QMeasureSettingsWidget
{
public:
  typedef QMeasureSettingsWidgetImpl ThisClass;
  typedef QMeasureSettingsWidget Base;

  QMeasureSettingsWidgetImpl(QWidget * parent = nullptr) :
      Base(parent)
  {
  }

  void setMeasure(MeasureType * m)
  {
    measure_ = m;
    updateControls();
  }

  MeasureType* measure() const
  {
    return measure_;
  }

  void setCurrentMeasure(QMeasure * m) override
  {
    setMeasure(dynamic_cast<MeasureType*>(m));
  }

  QMeasure* currentMeasure() const override
  {
    return measure_;
  }

protected:

  // placeholder for overrides
  virtual void update_measure_controls()
  {
    setEnabled(true);
  }

  void onupdatecontrols() override
  {
    if( !measure_ ) {
      setEnabled(false);
    }
    else {
      Q_EMIT Base::populatecontrols();
      update_measure_controls();
    }
  }

protected:
  MeasureType *measure_ = nullptr;
};

class QMeasureCentralPixelValue:
    public QMeasure
{
public:
  typedef QMeasureCentralPixelValue ThisClass;
  typedef QMeasure Base;

  QMeasureCentralPixelValue();

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};





#endif /* __QMeasure_h__ */
