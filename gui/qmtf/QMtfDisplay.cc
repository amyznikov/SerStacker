/*
 * QMtfDisplaySettings.cc
 *
 *  Created on: Apr 19, 2022
 *      Author: amyznikov
 */

#include "QMtfDisplay.h"
#include <core/proc/minmax.h>
#include <gui/widgets/settings.h>
#include <csignal>
#include <core/debug.h>

IMtfDisplay::IMtfDisplay(QObject * parent, const QString & prefix) :
    _events(new QMtfDisplayEvents(parent)),
    _prefix(prefix.isEmpty() ? "IMtfDisplay" : prefix)
{
}


////////


IMtfDisplay::DisplayParams::sptr IMtfDisplay::addDisplay(DisplayMap & map,
    const QString & displayChannelName, double input_min, double input_max)
{
  auto pos = map.find(displayChannelName);
  if ( pos == map.end() ) {
    pos = map.emplace(displayChannelName, new DisplayParams()).first;
  }

  const DisplayParams::sptr & p = pos->second;
  p->mtf.set_input_range(input_min, input_max);
  p->mtf.set_output_range(0, 255);
  if( p->colormap != COLORMAP_NONE ) {
    createLut(p->colormap, p->lut, p->invert_colormap);
  }
  return p;
}

void IMtfDisplay::addDisplay(const QString & displayChannelName, double input_min, double input_max)
{
  addDisplay(_currentDisplays,
      displayChannelName,
      input_min,
      input_max);
}

void IMtfDisplay::createLut(COLORMAP colormap, cv::Mat3b & lut, bool invert_colormap)
{
  if( colormap == COLORMAP_NONE ) {
    lut.release();
  }
  else {

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

void IMtfDisplay::adjustMtfRange(c_mtf * mtf,
    cv::InputArray currentImage, cv::InputArray currentMask,
    c_mtf_adjustment * a) const
{
  if ( !currentImage.empty() ) {

    mtf->get_input_range(&a->imin, &a->imax);

    if( _autoClip || a->imin >= a->imax ) {

      a->adjusted_inputs = true;

      double adjusted_min, adjusted_max;

      const int cdepth =
          currentImage.depth();

      if( _autoClip || cdepth == CV_32F || cdepth == CV_64F ) {
        getMinMax(currentImage, &adjusted_min, &adjusted_max, currentMask);
      }
      else {
        c_mtf::suggest_levels_range(cdepth, &adjusted_min, &adjusted_max);
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

void IMtfDisplay::restoreMtfRange(c_mtf *mtf, const c_mtf_adjustment & a) const
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

  for ( const auto & p : _currentDisplays ) {
    sl.append(p.first);
  }

  return sl;
}

void IMtfDisplay::setDisplayChannel(const QString & v)
{
  if ( _displayChannel != v ) {
    _displayChannel = v;
    Q_EMIT _events->displayChannelsChanged();
  }
}

const QString & IMtfDisplay::displayChannel() const
{
  return _displayChannel;
}

void IMtfDisplay::getMtfInputRange(double * min, double * max) const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.get_input_range(min, max);
  }
}

void IMtfDisplay::setMtf(double imin, double imax, const c_mtf_options * opts)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_input_range(imin, imax);
    if ( opts ) {
      ii->second->mtf.set_opts(*opts);
    }
    Q_EMIT _events->parameterChanged();
  }
}

void IMtfDisplay::getMtf(c_mtf_options * opts) const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    * opts = ii->second->mtf.opts();
  }
}

void IMtfDisplay::setlclip(double v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_lclip(v);
    Q_EMIT _events->parameterChanged();
  }
}

double IMtfDisplay::lclip() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  return ii != _currentDisplays.end() ? ii->second->mtf.lclip() : 0;
}

void IMtfDisplay::sethclip(double v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_hclip(v);
    Q_EMIT _events->parameterChanged();
  }
}

double IMtfDisplay::hclip() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  return ii != _currentDisplays.end() ? ii->second->mtf.hclip() : 1;
}

void IMtfDisplay::setShadows(double v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_shadows(v);
    Q_EMIT _events->parameterChanged();
  }
}

double IMtfDisplay::shadows() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  return ii != _currentDisplays.end() ? ii->second->mtf.shadows() : 1;
}

void IMtfDisplay::setHighlights(double v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_highlights(v);
    Q_EMIT _events->parameterChanged();
  }
}

double IMtfDisplay::highlights() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  return ii != _currentDisplays.end() ? ii->second->mtf.highlights() : 1;
}

void IMtfDisplay::setMidtones(double v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    ii->second->mtf.set_midtones(v);
    Q_EMIT _events->parameterChanged();
  }
}

double IMtfDisplay::midtones() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  return ii != _currentDisplays.end() ? ii->second->mtf.midtones() : 0.5;
}

void IMtfDisplay::setColormap(COLORMAP v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    const auto & p = ii->second;
    createLut(p->colormap = v, p->lut, p->invert_colormap);
    Q_EMIT _events->parameterChanged();
  }
}

COLORMAP IMtfDisplay::colormap() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    return ii->second->colormap;
  }
  return COLORMAP_NONE;
}

void IMtfDisplay::setInvertColormap(bool v)
{
  const DisplayMap::iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    const auto  & p = ii->second;
    if ( p->invert_colormap != v ) {
      createLut(p->colormap, p->lut, p->invert_colormap = v);
      Q_EMIT _events->parameterChanged();
    }
  }
}

bool IMtfDisplay::invertColormap() const
{
  const DisplayMap::const_iterator ii = _currentDisplays.find(_displayChannel);
  if ( ii != _currentDisplays.end() ) {
    return ii->second->invert_colormap;
  }
  return false;
}

void IMtfDisplay::setAutoClip(bool v)
{
  if ( _autoClip != v ) {
    _autoClip = v;
    Q_EMIT _events->parameterChanged();
  }
}

bool IMtfDisplay::autoClip() const
{
  return _autoClip;
}

IMtfDisplay::DisplayParams & IMtfDisplay::displayParams()
{
  DisplayMap::iterator pos = _currentDisplays.find(_displayChannel);
  if ( pos == _currentDisplays.end())  {
    CF_FATAL("FATAL APP BUG: _displayChannel='%s' was not registered. _displays.size=%zu",
        _displayChannel.toUtf8().constData(),
        _currentDisplays.size());
    std::raise(SIGTRAP); // Trapped by GDB/LLDB
    //exit(1);
  }

  auto & res = *pos->second;
  return res;
}

const IMtfDisplay::DisplayParams & IMtfDisplay::displayParams() const
{
  DisplayMap::const_iterator pos = _currentDisplays.find(_displayChannel);
  if ( pos == _currentDisplays.end())  {
    CF_FATAL("FATAL APP BUG: _displayChannel='%s' was not registered. _displays.size=%zu",
        _displayChannel.toUtf8().constData(),
        _currentDisplays.size());
    std::raise(SIGTRAP); // Trapped by GDB/LLDB
    //exit(1);
  }

  const auto & res = *pos->second;
  return res;
}

void IMtfDisplay::loadParameters()
{
  QSettings settings;
  loadParameters(settings, _prefix);
}

void IMtfDisplay::saveParameters() const
{
  QSettings settings;
  saveParameters(settings, _prefix);
}

void IMtfDisplay::loadParameters(const QSettings & settings, const QString & prefix)
{
  const QString displayChannel =
      settings.value(QString("%1_DisplayChannel").arg(prefix),
          _displayChannel).toString();

  if( _currentDisplays.find(displayChannel) != _currentDisplays.end() ) {
    _displayChannel = displayChannel;
  }

  for ( DisplayMap::iterator ii = _currentDisplays.begin(); ii != _currentDisplays.end(); ++ii ) {

    const QString displayName = ii->first;
    const DisplayParams::sptr & p = ii->second;

    const QString prefix2 =
        QString("%1_DisplayParams_%2").arg(prefix).arg(displayName);

    double min, max;

    p->mtf.get_input_range(&min, &max);

    min = settings.value(QString("%1/imin").arg(prefix2), min).toDouble();
    max = settings.value(QString("%1/imax").arg(prefix2), max).toDouble();
    p->mtf.set_input_range(min, max);
    p->colormap = fromString(settings.value(QString("%1/cmap").arg(prefix2)).toString().toStdString(), p->colormap);
    p->invert_colormap = settings.value(QString("%1/invert_colormap").arg(prefix2), p->invert_colormap).toBool();

    createLut(p->colormap, p->lut, p->invert_colormap);
  }
}

void IMtfDisplay::saveParameters(QSettings & settings, const QString & prefix) const
{
  settings.setValue(QString("%1_DisplayChannel").arg(prefix),
      _displayChannel);

  DisplayMap::const_iterator ii =
      _currentDisplays.find(_displayChannel);

  if( ii != _currentDisplays.end() ) {

    const DisplayParams::sptr &p = ii->second;
    const QString prefix2 = QString("%1_DisplayParams_%2").arg(prefix).arg(_displayChannel);

    double min, max;

    p->mtf.get_input_range(&min, &max);
    settings.setValue(QString("%1/imin").arg(prefix2), min);
    settings.setValue(QString("%1/imax").arg(prefix2), max);
    settings.setValue(QString("%1/cmap").arg(prefix2), toQString(p->colormap));
    settings.setValue(QString("%1/invert_colormap").arg(prefix2), p->invert_colormap);
  }
}
