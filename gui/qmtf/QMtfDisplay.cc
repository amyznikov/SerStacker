/*
 * QMtfDisplaySettings.cc
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#include "QMtfDisplay.h"
#include <core/proc/minmax.h>
#include <gui/widgets/settings.h>
#include <core/debug.h>

IMtfDisplay::IMtfDisplay(const QString & prefix) :
  prefix_(prefix.isEmpty() ? "IMtfDisplay" : prefix)
{

}



////////


IMtfDisplay::DisplayParams & IMtfDisplay::addDisplay(DisplayMap & map,
    const QString & displayChannelName, double input_min, double input_max)
{

  DisplayParams & p =
      map[displayChannelName];

  p.mtf.set_input_range(input_min, input_max);
  p.mtf.set_output_range(0, 255);
  if( p.colormap != COLORMAP_NONE ) {
    createLut(p.colormap, p.lut, p.invert_colormap);
  }

  return p;

//  auto pos = map.find(type);
//
//  if ( pos == map.end() ) {
//
//    DisplayParams p;
//
//    p.mtf.set_input_range(input_min, input_max);
//    p.mtf.set_output_range(0, 255);
//    if ( p.colormap != COLORMAP_NONE ) {
//      createLut(p.colormap, p.lut,
//          p.invert_colormap);
//    }
//
//    pos = map.emplace(type, p).first;
//  }
//
//  return pos->second;
}

void IMtfDisplay::addDisplay(const QString & displayChannelName, double input_min, double input_max)
{

  addDisplay(displays_,
      displayChannelName,
      input_min,
      input_max);


//  DisplayParams p;
//  p.mtf.set_input_range(input_min, input_max);
//  p.mtf.set_output_range(0, 255);
//  if ( p.colormap != COLORMAP_NONE ) {
//    createLut(p.colormap, p.lut,
//        p.invert_colormap);
//  }
//  displayParams_.emplace(type, p);
}

void IMtfDisplay::createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap)
{
  if( colormap == COLORMAP_NONE ) {
    lut.release();
  }
  else {
//    cv::Mat1b M(1, 256);
//
//    if( invert_colormap ) {
//      for( int i = 0; i < 256; ++i ) {
//        M[0][i] = 255 - i;
//      }
//    }
//    else {
//      for( int i = 0; i < 256; ++i ) {
//        M[0][i] = i;
//      }
//    }

    cv::Mat1b M(256, 1);

    if( invert_colormap ) {
      for( int i = 0; i < 256; ++i ) {
        M[i][0] = 255 - i;
      }
    }
    else {
      for( int i = 0; i < 256; ++i ) {
        M[i][0] = i;
      }
    }

    apply_colormap(M, lut, colormap);
  }
}

void IMtfDisplay::adjustMtfRange(c_midtones_transfer_function * mtf,
    cv::InputArray currentImage, cv::InputArray currentMask,
    c_mtf_adjustment * a) const
{
  if ( !currentImage.empty() ) {

    mtf->get_input_range(&a->imin, &a->imax);

    if( autoClip_ || a->imin >= a->imax ) {

      a->adjusted_inputs = true;

      double adjusted_min, adjusted_max;

      const int cdepth =
          currentImage.depth();

      if( autoClip_ || cdepth == CV_32F || cdepth == CV_64F ) {
        getminmax(currentImage, &adjusted_min, &adjusted_max, currentMask);
      }
      else {
        c_midtones_transfer_function::suggest_levels_range(cdepth, &adjusted_min, &adjusted_max);
      }

      mtf->set_input_range(adjusted_min, adjusted_max);
      a->adjusted_inputs = true;
    }

    if( currentImage.type() == CV_32FC2 || currentImage.type() == CV_64FC2 ) {
      // assume this is an optical flow image
      mtf->get_output_range(&a->omin, &a->omax);
      mtf->set_output_range(a->imin, a->imax);
      a->adjusted_outputs = true;
    }
  }
}

void IMtfDisplay::restoreMtfRange(c_midtones_transfer_function *mtf, const c_mtf_adjustment & a) const
{
  if ( a.adjusted_inputs ) {
    mtf->set_input_range(a.imin, a.imax);
  }
  if ( a.adjusted_outputs ) {
    mtf->set_output_range(a.omin, a.omax);
  }
}

QStringList IMtfDisplay::displayChannels() const
{
  QStringList sl;

  for ( const auto & p : displays_ ) {
    sl.append(p.first);
  }

  return sl;
}

void IMtfDisplay::setDisplayChannel(const QString & v)
{
  if ( displayChannel_ != v ) {
    displayChannel_ = v;
    Q_EMIT displayChannelsChanged();
  }
}

const QString & IMtfDisplay::displayChannel() const
{
  return displayChannel_;
}

void IMtfDisplay::setMtfInputRange(double min, double max)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {
    ii->second.mtf.set_input_range(min, max);
    Q_EMIT parameterChanged();
  }
}

void IMtfDisplay::getMtfInputRange(double * min, double * max) const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {
    ii->second.mtf.get_input_range(min, max);
  }
}

void IMtfDisplay::setMtf(double shadows, double highlights, double midtones)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    ii->second.mtf.set_parameters(shadows,
        highlights,
        midtones);

    Q_EMIT parameterChanged();
  }
}

void IMtfDisplay::getMtf(double * shadows, double * highlights, double * midtones) const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    ii->second.mtf.get_parameters(shadows,
        highlights,
        midtones);
  }
}

void IMtfDisplay::setShadows(double shadows)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    ii->second.mtf.set_shadows(shadows);

    Q_EMIT parameterChanged();
  }
}

double IMtfDisplay::shadows() const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  return ii != displays_.end() ?
      ii->second.mtf.shadows() :
      0;
}

void IMtfDisplay::setHighlights(double highlights)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    ii->second.mtf.set_highlights(highlights);

    Q_EMIT parameterChanged();
  }
}

double IMtfDisplay::highlights() const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  return ii != displays_.end() ?
      ii->second.mtf.highlights() :
      1;
}

void IMtfDisplay::setMidtones(double midtones)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    ii->second.mtf.set_midtones(midtones);

    Q_EMIT parameterChanged();
  }
}

double IMtfDisplay::midtones() const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  return ii != displays_.end() ?
      ii->second.mtf.midtones() :
      0.5;
}

void IMtfDisplay::setColormap(COLORMAP v)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    DisplayParams & p = ii->second;

    createLut(p.colormap = v, p.lut,
        p.invert_colormap);

    Q_EMIT parameterChanged();
  }
}

COLORMAP IMtfDisplay::colormap() const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    return ii->second.colormap;
  }

  return COLORMAP_NONE;
}

void IMtfDisplay::setInvertColormap(bool v)
{
  const DisplayMap::iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    DisplayParams & p =
        ii->second;

    if ( p.invert_colormap != v ) {

      createLut(p.colormap, p.lut,
          p.invert_colormap = v);

      Q_EMIT parameterChanged();
    }
  }
}

bool IMtfDisplay::invertColormap() const
{
  const DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  if ( ii != displays_.end() ) {

    return ii->second.invert_colormap;
  }

  return false;
}

void IMtfDisplay::setAutoClip(bool v)
{
  if ( autoClip_ != v ) {
    autoClip_ = v;
    Q_EMIT parameterChanged();
  }
}

bool IMtfDisplay::autoClip() const
{
  return autoClip_;
}

IMtfDisplay::DisplayParams & IMtfDisplay::displayParams()
{
  DisplayMap::iterator pos =
      displays_.find(displayChannel_);

  if ( pos == displays_.end())  {
    CF_FATAL("FATAL APP BUG: displayType_='%s' not registered. displays_.size=%zu",
        displayChannel_.toUtf8().constData(),
        displays_.size());
    exit(1);
  }

  return pos->second;
}

const IMtfDisplay::DisplayParams & IMtfDisplay::displayParams() const
{
  DisplayMap::const_iterator pos =
      displays_.find(displayChannel_);

  if ( pos == displays_.end())  {
    CF_FATAL("APP BUG: displayType_='%s' not registered.displays_.size=%zu",
        displayChannel_.toUtf8().constData(),
        displays_.size());
    exit(1);
  }

  return pos->second;
}

//c_pixinsight_mtf& IMtfDisplay::mtf()
//{
//  return displayParams_.find(displayType_)->second.mtf;
//}
//
//const c_pixinsight_mtf& IMtfDisplay::mtf() const
//{
//  return displayParams_.find(displayType_)->second.mtf;
//}


void IMtfDisplay::loadParameters()
{
  QSettings settings;
  loadParameters(settings, prefix_);
}

void IMtfDisplay::saveParameters() const
{
  QSettings settings;
  saveParameters(settings, prefix_);
}

void IMtfDisplay::loadParameters(const QSettings & settings, const QString & prefix)
{
  const QString displayChannel =
      settings.value(QString("%1_DisplayChannel").arg(prefix),
          displayChannel_).toString();

  if( displays_.find(displayChannel) != displays_.end() ) {
    displayChannel_ = displayChannel;
  }

  for ( DisplayMap::iterator ii = displays_.begin(); ii != displays_.end(); ++ii ) {

    const QString displayType = ii->first;
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

void IMtfDisplay::saveParameters(QSettings & settings, const QString & prefix) const
{
  settings.setValue(QString("%1_DisplayChannel").arg(prefix),
      displayChannel_);

  DisplayMap::const_iterator ii =
      displays_.find(displayChannel_);

  if( ii != displays_.end() ) {

    const DisplayParams &p =
        ii->second;

    const QString prefix2 =
        QString("%1_DisplayParams_%2").arg(prefix).arg(displayChannel_);

    double min, max;

    p.mtf.get_input_range(&min, &max);
    settings.setValue(QString("%1/imin").arg(prefix2), min);
    settings.setValue(QString("%1/imax").arg(prefix2), max);
    settings.setValue(QString("%1/cmap").arg(prefix2), toQString(p.colormap));
    settings.setValue(QString("%1/invert_colormap").arg(prefix2), p.invert_colormap);
  }
}


QMtfDisplay::QMtfDisplay(const QString & prefix, QObject * parent) :
    Base(parent),
    IMtfDisplay(prefix)
{
}




