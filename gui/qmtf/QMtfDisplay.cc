/*
 * QMtfDisplaySettings.cc
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#include "QMtfDisplay.h"


QMtfDisplay::QMtfDisplay(const QString & prefix, QObject * parent) :
    Base(parent),
    prefix_(prefix.isEmpty() ? "QMtfDisplay" : prefix)
{
}

void QMtfDisplay::addDisplay(DisplayMap & map, int type, double rmin, double rmax)
{
  DisplayParams p;
  p.mtf.set_input_range(rmin, rmax);
  p.mtf.set_output_range(0, 255);
  if ( p.colormap != COLORMAP_NONE ) {
    createLut(p.colormap, p.lut,
        p.invert_colormap);
  }
  map.emplace(type, p);
}

void QMtfDisplay::createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap)
{
  if( colormap == COLORMAP_NONE ) {
    lut.release();
  }
  else {
    cv::Mat1b M(1, 256);

    if( invert_colormap ) {
      for( int i = 0; i < 256; ++i ) {
        M[0][i] = 255 - i;
      }
    }
    else {
      for( int i = 0; i < 256; ++i ) {
        M[0][i] = i;
      }
    }

    apply_colormap(M, lut, colormap);
  }
}

void QMtfDisplay::adjustMtfInputRange(c_midtones_transfer_function *mtf, double * imin, double * imax) const
{
  mtf->get_input_range(imin, imax);
  if( *imin >= *imax ) {
    double min, max;
    getInputDataRange(&min, &max);
    mtf->set_input_range(min, max);
  }
}

void QMtfDisplay::restoreMtfInputRange(c_midtones_transfer_function *mtf, double imin, double imax) const
{
  if( imin >= imax ) {
    mtf->set_input_range(imin, imax);
  }
}

void QMtfDisplay::setDisplayType(int v)
{
  if ( displayParams_.find(v) != displayParams_.end() ) {
    displayType_ = v;
    updateDisplay();
  }
}

int QMtfDisplay::displayType() const
{
  return displayType_;
}

void QMtfDisplay::setColormap(COLORMAP v)
{
  const DisplayMap::iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {
    DisplayParams & p = ii->second;
    createLut(p.colormap = v, p.lut,
        p.invert_colormap);
  }

  updateDisplay();
}

COLORMAP QMtfDisplay::colormap() const
{
  const DisplayMap::const_iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {

    const DisplayParams & p =
        ii->second;

    return p.colormap;
  }

  return COLORMAP_NONE;
}

void QMtfDisplay::setInvertColormap(bool v)
{
  const DisplayMap::iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {

    DisplayParams & p =
        ii->second;

    if ( p.invert_colormap != v ) {
      createLut(p.colormap, p.lut,
          p.invert_colormap = v);
    }
  }

  updateDisplay();
}

bool QMtfDisplay::invertColormap() const
{
  const DisplayMap::const_iterator ii =
      displayParams_.find(displayType_);

  if ( ii != displayParams_.end() ) {

    const DisplayParams & p =
        ii->second;

    return p.invert_colormap;
  }

  return false;
}
QMtfDisplay::DisplayParams & QMtfDisplay::displayParams()
{
  DisplayMap::iterator pos =
      displayParams_.find(displayType_);

  if ( pos == displayParams_.end())  {
    CF_FATAL("FATAL APP BUG: displayType_=%d not registered",
        displayType_);
    exit(1);
  }

  return pos->second;
}

const QMtfDisplay::DisplayParams & QMtfDisplay::displayParams() const
{
  DisplayMap::const_iterator pos =
      displayParams_.find(displayType_);

  if ( pos == displayParams_.end())  {
    CF_FATAL("APP BUG: displayType_=%d not registered", displayType_);
    exit(1);
  }

  return pos->second;
}

c_pixinsight_mtf& QMtfDisplay::mtf()
{
  return displayParams_.find(displayType_)->second.mtf;
}

const c_pixinsight_mtf& QMtfDisplay::mtf() const
{
  return displayParams_.find(displayType_)->second.mtf;
}


void QMtfDisplay::loadParameters()
{
  QSettings settings;
  loadParameters(settings, prefix_);
}

void QMtfDisplay::saveParameters() const
{
  QSettings settings;
  saveParameters(settings, prefix_);
}

void QMtfDisplay::loadParameters(const QSettings & settings, const QString & prefix)
{
  const int displayType =
      settings.value(QString("%1_DisplayType").arg(prefix),
          displayType).toInt();

  if( displayParams_.find(displayType) != displayParams_.end() ) {
    displayType_ = displayType;
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
    p.invert_colormap = settings.value(QString("%1/invert_colormap").arg(prefix2), p.invert_colormap).toBool();

    createLut(p.colormap, p.lut, p.invert_colormap);
  }
}

void QMtfDisplay::saveParameters(QSettings & settings, const QString & prefix) const
{
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
    settings.setValue(QString("%1/invert_colormap").arg(prefix2), p.invert_colormap);
  }
}






