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

  virtual QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const = 0;

  virtual void setAverageColorChannels(bool v)
  {
    average_color_channels_ = v;
  }

  virtual bool averageColorChannels() const
  {
    return average_color_channels_;
  }

  virtual void setSkipZeroPixels(bool v)
  {
    skip_zero_pixels_ = v;
  }

  virtual bool skipZeroPixels() const
  {
    return skip_zero_pixels_;
  }

  int compute(cv::InputArray image, cv::InputArray mask, const QRect& roi, cv::Scalar * output_value) const
  {
    return compute(image, mask, cv::Rect(roi.x(), roi.y(), roi.width(), roi.height()), output_value);
  }

  int compute(cv::InputArray image, cv::InputArray mask, const cv::Rect & roi, cv::Scalar * output_value) const;

  static bool adjust_roi(const cv::Rect & src_roi, const cv::Size & image_size, cv::Rect * dst_roi);

protected:
  virtual int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const = 0;

protected:
  const QString name_;
  const QString tooltip_;
  bool average_color_channels_ = false;
  bool skip_zero_pixels_ = false;
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
class QMeasureSettingsWidgetTemplate:
    public QMeasureSettingsWidget
{
public:
  typedef QMeasureSettingsWidgetTemplate ThisClass;
  typedef QMeasureSettingsWidget Base;

  QMeasureSettingsWidgetTemplate(QWidget * parent = nullptr) :
      Base(parent)
  {
    averageColorChannels_ctl =
        add_checkbox("Average Color Channels",
            "",
            [this](bool checked) {
              if ( measure_ && measure_->averageColorChannels() != checked ) {
                measure_->setAverageColorChannels(checked);
                Q_EMIT parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( measure_ ) {
                * checked = measure_->averageColorChannels();
                return true;
              }
              return false;
            });

    skipZeroPixels_ctl =
        add_checkbox("Skip zero pixels",
            "",
            [this](bool checked) {
              if ( measure_ && measure_->skipZeroPixels() != checked ) {
                measure_->setSkipZeroPixels(checked);
                Q_EMIT parameterChanged();
              }
            },
            [this](bool * checked) {
              if ( measure_ ) {
                * checked =measure_->skipZeroPixels();
                return true;
              }
              return false;
            });
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
  QCheckBox * averageColorChannels_ctl = nullptr;
  QCheckBox * skipZeroPixels_ctl = nullptr;
};

class QMeasureCentralPixelValue:
    public QMeasure
{
public:
  typedef QMeasureCentralPixelValue ThisClass;
  typedef QMeasure Base;

  QMeasureCentralPixelValue();

  QMeasureSettingsWidget * createSettingsWidget(QWidget * parent) const override;

protected:
  int compute_measure(const cv::Mat & image, const cv::Mat & mask, cv::Scalar * output_value) const override;
};


class QMeasureCentralPixelValueSettingsWidget :
    public QMeasureSettingsWidgetTemplate<QMeasureCentralPixelValue>
{
public:
  typedef QMeasureCentralPixelValueSettingsWidget ThisClass;
  typedef QMeasureSettingsWidgetTemplate<QMeasureCentralPixelValue> Base;

  QMeasureCentralPixelValueSettingsWidget(QWidget * parent = nullptr) :
    Base(parent)
  {
    averageColorChannels_ctl->setEnabled(false);
    skipZeroPixels_ctl->setEnabled(false);
    updateControls();
  }
};



#endif /* __QMeasure_h__ */
