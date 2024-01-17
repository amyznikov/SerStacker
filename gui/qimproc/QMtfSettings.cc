/*
 * QMtfSettings.cc
 *
 *  Created on: Jul 30, 2021
 *      Author: amyznikov
 */

#include "QMtfSettings.h"
#include <core/proc/histogram.h>
#include <core/proc/minmax.h>

namespace  {
  enum DISPLAY_TYPE {
    DISPLAY_PIXEL_VALUE
  };
}  // namespace
//
template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static const c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "PIXEL_VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}


QMtfRoutineDisplaySettings::QMtfRoutineDisplaySettings(const c_mtf_routine::ptr & processor, QObject * parent) :
    Base("QMtfRoutineDisplaySettings", parent),
    processor_(processor)
{
  processor_->mtf().set_output_range(0, 255);

  processor_->set_preprocess_notify_callback(
      [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {

        if ( image.empty() ) {
          imin = imax = 0;
          iH.release();
        }
        else {
          getminmax(image, &imin, &imax, mask);

          if ( 0 ) {
            create_histogram(image, mask,
                iH,
                &imin, &imax,
                256,
                false,
                false);
          }
        }
      });

  processor_->set_postprocess_notify_callback(
      [this](c_image_processor_routine * obj, cv::InputArray image, cv::InputArray mask) {

        processor_->mtf().get_output_range(&omin, &omax);

        if ( image.empty() ) {
          oH.release();
        }
        else {
          create_histogram(image, mask,
              oH,
              &omin, &omax,
              256,
              false,
              false);
        }

        Q_EMIT displayImageChanged();
      });


  QMtfDisplay::displayChannel_ =
      DISPLAY_PIXEL_VALUE;

  addDisplay(DISPLAY_PIXEL_VALUE, -1, -1);
}

const c_enum_member * QMtfRoutineDisplaySettings::displayChannels() const
{
  return members_of<DISPLAY_TYPE>();
}

void QMtfRoutineDisplaySettings::setDisplayChannel(int /*v*/)
{
}

int QMtfRoutineDisplaySettings::displayChannel() const
{
  return DISPLAY_PIXEL_VALUE;
}

void QMtfRoutineDisplaySettings::setColormap(COLORMAP /*v*/)
{
}

COLORMAP QMtfRoutineDisplaySettings::colormap() const
{
  return COLORMAP_NONE;
}

void QMtfRoutineDisplaySettings::setInvertColormap(bool /*v*/)
{
}

bool QMtfRoutineDisplaySettings::invertColormap() const
{
  return false;
}

void QMtfRoutineDisplaySettings::setMtfInputRange(double min, double max)
{
  processor_->mtf().set_input_range(min, max);
  Q_EMIT parameterChanged();
}

void QMtfRoutineDisplaySettings::getMtfInputRange(double * min, double * max) const
{
  processor_->mtf().get_input_range(min, max);
}

void QMtfRoutineDisplaySettings::setShadows(double shadows)
{
  processor_->mtf().set_shadows(shadows);
  Q_EMIT parameterChanged();
}

double QMtfRoutineDisplaySettings::shadows() const
{
  return processor_->mtf().shadows();
}

void QMtfRoutineDisplaySettings::setHighlights(double highlights)
{
  processor_->mtf().set_highlights(highlights);
  Q_EMIT parameterChanged();
}

double QMtfRoutineDisplaySettings::highlights() const
{
  return processor_->mtf().highlights();
}

void QMtfRoutineDisplaySettings::setMidtones(double midtones)
{
  processor_->mtf().set_midtones(midtones);
  Q_EMIT parameterChanged();
}

double QMtfRoutineDisplaySettings::midtones() const
{
  return processor_->mtf().midtones();
}

void QMtfRoutineDisplaySettings::setMtf(double shadows, double highlights, double midtones)
{
  processor_->mtf().set_parameters(shadows, highlights, midtones);
  Q_EMIT parameterChanged();
}

void QMtfRoutineDisplaySettings::getMtf(double * shadows, double * highlights, double * midtones) const
{
  return processor_->mtf().get_parameters(shadows, highlights, midtones);
}

void QMtfRoutineDisplaySettings::getInputDataRange(double * minval, double * maxval) const
{
  *minval = imin;
  *maxval = imax;
}

void QMtfRoutineDisplaySettings::getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  *hmin = imin;
  *hmax = imax;
  iH.copyTo(H);
}

void QMtfRoutineDisplaySettings::getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax)
{
  *hmin = omin;
  *hmax = omax;
  oH.copyTo(H);
}
//
//void QMtfRoutineDisplaySettings::loadParameters()
//{
//}
//
//void QMtfRoutineDisplaySettings::saveParameters() const
//{
//}
//
//void QMtfRoutineDisplaySettings::loadParameters(const QString & /*prefix*/)
//{
//}
//
//void QMtfRoutineDisplaySettings::saveParameters(const QString & /*prefix*/) const
//{
//}


QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & routine, QWidget * parent) :
    Base(routine, parent),
    displaySettings_(routine, this)
{
}

void QMtfSettings::setupControls()
{
  mtf_ctl = add_widget<QMtfControl>();
  mtf_ctl->setHistoramViewSizeHint(QSize(80, (int)(80)));
  mtf_ctl->setDisplaySettings(&displaySettings_);

  updateControls();

  connect(&displaySettings_, &QMtfDisplay::displayTypeChanged,
      &displaySettings_, &QMtfDisplay::parameterChanged);

  connect(&displaySettings_, &QMtfRoutineDisplaySettings::parameterChanged,
      this, &ThisClass::parameterChanged,
      Qt::QueuedConnection);
}


//void QMtfSettings::onupdatecontrols()
//{
//  Base::onupdatecontrols();
//}
