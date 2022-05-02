/*
 * QMtfDisplaySettings.cc
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#include "QMtfDisplaySettings.h"


void QMtfDisplaySettingsBase::createLut(COLORMAP colormap, cv::Mat3b & lut)
{
  if( colormap == COLORMAP_NONE ) {
    lut.release();
  }
  else {
    cv::Mat1b M(1, 256);
    for( int i = 0; i < 256; ++i ) {
      M[0][i] = i;
    }
    apply_colormap(M, lut, colormap);
  }
}

QMtfDisplaySettings::QMtfDisplaySettings(QObject * parent) :
  Base(parent)
{
}

void QMtfDisplaySettings::addDisplay(DisplayMap & map, int type, double rmin, double rmax)
{
  DisplayParams p;
  p.mtf.set_input_range(rmin, rmax);
  p.mtf.set_output_range(0, 255);
  if ( p.colormap != COLORMAP_NONE ) {
    createLut(p.colormap, p.lut);
  }
  map.emplace(type, p);
}

void QMtfDisplaySettings::setColormap(COLORMAP v)
{
  const DisplayMap::iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {
    DisplayParams & p = ii->second;
    createLut(p.colormap = v, p.lut);
  }

  updateDisplay();
}

COLORMAP QMtfDisplaySettings::colormap() const
{
  const DisplayMap::const_iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {
    const DisplayParams & p = ii->second;
    return p.colormap;
  }

  return COLORMAP_NONE;
}

void QMtfDisplaySettings::setDisplayType(int v)
{
  if ( displayParams_.find(v) != displayParams_.end() ) {
    displayType_ = v;
    updateDisplay();
  }
}

int QMtfDisplaySettings::displayType() const
{
  return displayType_;
}

QMtfDisplaySettings::DisplayParams & QMtfDisplaySettings::displayParams()
{
  DisplayMap::iterator pos =
      displayParams_.find(displayType_);

  if ( pos == displayParams_.end())  {
    CF_FATAL("APP BUG: displayType_=%d not registered", displayType_);
    exit(1);
  }

  return pos->second;
}

const QMtfDisplaySettings::DisplayParams & QMtfDisplaySettings::displayParams() const
{
  DisplayMap::const_iterator pos =
      displayParams_.find(displayType_);

  if ( pos == displayParams_.end())  {
    CF_FATAL("APP BUG: displayType_=%d not registered", displayType_);
    exit(1);
  }

  return pos->second;
}

c_pixinsight_mtf& QMtfDisplaySettings::mtf()
{
  return displayParams_.find(displayType_)->second.mtf;
}

const c_pixinsight_mtf& QMtfDisplaySettings::mtf() const
{
  return displayParams_.find(displayType_)->second.mtf;
}

void QMtfDisplaySettings::loadParameters(const QString & prefix)
{
  QSettings settings;

  const int displayType =
      settings.value(QString("%1_DisplayType").arg(prefix),
          displayType).toInt();

  if( displayParams_.find(displayType) != displayParams_.end() ) {
    displayType_ = displayType;
    CF_DEBUG("displayType=%d found", displayType);
  }
  else {
    CF_DEBUG("displayType=%d not found", displayType);
  }

  for ( DisplayMap::iterator ii = displayParams_.begin(); ii != displayParams_.end(); ++ii ) {

    const int displayType = ii->first;
    DisplayParams & p = ii->second;

    const QString prefix2 =
        QString("%1_DisplayParams_%2").arg(prefix).arg(displayType);

    double min, max;

    p.mtf.get_input_range(&min, &max);

    min = settings.value(QString("%1/imin").arg(prefix2), min).toDouble();
    max = settings.value(QString("%1/imax").arg(prefix2), max).toDouble();
    p.mtf.set_input_range(min, max);
    p.colormap = fromString(settings.value(QString("%1/cmap").arg(prefix2)).toString().toStdString(), p.colormap);
    createLut(p.colormap, p.lut);
  }
}

void QMtfDisplaySettings::saveParameters(const QString & prefix) const
{
  QSettings settings;

  settings.setValue(QString("%1_DisplayType").arg(prefix),
      displayType_);

  DisplayMap::const_iterator ii =
      displayParams_.find(displayType_);

  if( ii != displayParams_.end() ) {

    const DisplayParams &p =
        ii->second;

    const QString prefix2 =
        QString("%1_DisplayParams_%2").arg(prefix).arg(displayType_);

    double min, max;

    p.mtf.get_input_range(&min, &max);
    settings.setValue(QString("%1/imin").arg(prefix2), min);
    settings.setValue(QString("%1/imax").arg(prefix2), max);
    settings.setValue(QString("%1/cmap").arg(prefix2), toString(p.colormap));
  }
}

void QMtfDisplaySettings::loadParameters()
{
  loadParameters("QMtfDisplaySettings");
}

void QMtfDisplaySettings::saveParameters() const
{
  saveParameters("QMtfDisplaySettings");
}
