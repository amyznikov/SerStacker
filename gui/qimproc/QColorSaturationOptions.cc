/*
 * QColorSaturationOptions.cc
 *
 *  Created on: Aug 16, 2021
 *      Author: amyznikov
 */

#include "QColorSaturationOptions.h"
#include <core/strsplit.h>
#include <core/debug.h>


const QColorSaturationOptions::ClassFactory QColorSaturationOptions::classFactory;

static bool fromString(const QString & text, std::vector<double> * v)
{
  std::vector<std::string> tokens;

  strsplit(text.toStdString(), tokens, " ;:\t\n");

  v->clear();
  v->reserve(tokens.size());

  for ( int i = 0, n = tokens.size(); i < n; ++i ) {
    double value;
    if ( sscanf(tokens[i].c_str(), "%lf", &value) == 1 ) {
      v->emplace_back(value);
    }
    else {
      break;
    }
  }

  return !v->empty();
}

static QString toString(const std::vector<double> & v)
{
  QString s;

  for ( uint i = 0, n = v.size(); i < n; ++i ) {
    s.append(QString("%1 ; ").arg(v[i]));
  }

  return s;
}

QColorSaturationOptions::QColorSaturationOptions(const RoutinePtr & routine, QWidget * parent)
  : Base(&classFactory, routine, parent)
{
  scales_ctl = QSettingsWidget::add_textbox(Base::ctlform,
      "Scales:",
      [this](const QString & s) {
        if ( routine_ ) {
            std::vector<double> values;
            if ( fromString(s, &values)) {
              routine_->set_scales(values);
              emit parameterChanged();
            }
        }
      });

  updateControls();
}

void QColorSaturationOptions::onupdatecontrols()
{
  if ( !routine_ ) {
    setEnabled(false);
  }
  else {

    scales_ctl->setText(toString(routine_->scales()));

    setEnabled(true);
  }

  Base::onupdatecontrols();
}
