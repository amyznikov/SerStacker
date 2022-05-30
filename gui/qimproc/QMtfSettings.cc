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

template<>
const c_enum_member* members_of<DISPLAY_TYPE>()
{
  static constexpr c_enum_member members[] = {
      { DISPLAY_PIXEL_VALUE, "PIXEL_VALUE" },
      { DISPLAY_PIXEL_VALUE }
  };

  return members;
}


QMtfRoutineDisplaySettings::QMtfRoutineDisplaySettings(const c_mtf_routine::ptr & processor, QObject * parent) :
    Base(parent), processor_(processor)
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

          create_histogram(image, mask,
              iH,
              &imin, &imax,
              256,
              false,
              false);
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

        emit inputDataChanged();
      });

}

const c_enum_member * QMtfRoutineDisplaySettings::displayTypes() const
{
  return members_of<DISPLAY_TYPE>();
}

void QMtfRoutineDisplaySettings::setDisplayType(int /*v*/)
{
}

int QMtfRoutineDisplaySettings::displayType() const
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

c_pixinsight_mtf & QMtfRoutineDisplaySettings::mtf()
{
  return processor_->mtf();
}

const c_pixinsight_mtf & QMtfRoutineDisplaySettings::mtf() const
{
  return processor_->mtf();
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

void QMtfRoutineDisplaySettings::loadParameters()
{
}

void QMtfRoutineDisplaySettings::saveParameters() const
{
}

void QMtfRoutineDisplaySettings::loadParameters(const QString & /*prefix*/)
{
}

void QMtfRoutineDisplaySettings::saveParameters(const QString & /*prefix*/) const
{
}


QMtfSettings::QMtfSettings(const c_mtf_routine::ptr & routine, QWidget * parent) :
    Base(routine, parent),
    displaySettings_(routine, this)
{
}

void QMtfSettings::setup_controls()
{
  mtf_ctl = add_widget<QMtfControl>(ctlform);
  mtf_ctl->setDisplaySettings(&displaySettings_);

  updateControls();

  connect(&displaySettings_, &QMtfRoutineDisplaySettings::updateDisplay,
      this, &ThisClass::parameterChanged);
}


void QMtfSettings::onupdatecontrols()
{
  Base::onupdatecontrols();
}
