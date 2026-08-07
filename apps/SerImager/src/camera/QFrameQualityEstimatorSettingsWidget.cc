/*
 * QFrameQualityEstimatorSettingsWidget.cc
 *
 *  Created on: Aug 7, 2026
 *      Author: amyznikov
 */

#include "QFrameQualityEstimatorSettingsWidget.h"
#include <gui/widgets/QSettingsWidgetTemplate.h>

namespace serimager {

QFrameQualityEstimatorSettingsWidget::QFrameQualityEstimatorSettingsWidget(QWidget * parent) :
    Base(parent)
{
  enabled_ctl =
      add_checkbox("Enable smart frame dropping",
          "Enable / disable functionality of QFrameQualityEstimator",
          [this](bool checked) {
            if ( _opts && _opts->isEnabled() != checked ) {
              _opts->setEnabled(checked);
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->isEnabled();
              return true;
            }
            return false;
          });


  c_ctlist<QFrameQualityEstimation> controls;
  QFrameQualityEstimation::getcontrols(controls, c_ctlbind_context<QFrameQualityEstimation>());
  setupControls(this, controls);

  updateControls();
}

void QFrameQualityEstimatorSettingsWidget::setFrameQualityEstimator(QFrameQualityEstimation * estimator)
{
  setOpts(estimator);
}

QFrameQualityEstimation * QFrameQualityEstimatorSettingsWidget::frameQualityEstimator() const
{
  return _opts;
}



} /* namespace serimager */
