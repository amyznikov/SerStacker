/*
 * QImageFileViewer.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageFileViewer.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/debug.h>

QImageFileViewer::QImageFileViewer(QImageScene * scene, QWidget * parent) :
  Base(scene, parent)
{
  _input_sequence = c_input_sequence::create();
  //input_sequence_->set_auto_debayer(DEBAYER_GB);
  _input_sequence->set_auto_apply_color_matrix(true);

  Base::_layout->insertWidget(2, playControls =
      new QPlaySequenceControl(this));

  connect(playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  playControls->setVisible(false);
}

QImageFileViewer::QImageFileViewer(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

const c_input_sequence::sptr & QImageFileViewer::input_sequence() const
{
  return _input_sequence;
}

void QImageFileViewer::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData, bool make_copy )
{
  closeCurrentSequence();
  Base::setImage(image, mask, imageData, make_copy);
}

void QImageFileViewer::openImage(const std::string & pathfilename)
{
  _input_sequence->clear();
  _input_sequence->add_source(pathfilename);

  startDisplay();
}

void QImageFileViewer::openImage(const QString & pathfilename)
{
  openImage(pathfilename.toStdString());
}

void QImageFileViewer::openImages(const std::vector<std::string> & pathfilenames)
{
  _input_sequence->clear();
  _input_sequence->add_sources(pathfilenames);
  startDisplay();
}

void QImageFileViewer::openImages(const QStringList & pathfilenames)
{
  std::vector<std::string> std_names;

  std_names.reserve(pathfilenames.size());
  for ( const QString & s : pathfilenames ) {
    std_names.emplace_back(s.toStdString());
  }

  return openImages(std_names);
}

void QImageFileViewer::closeCurrentSequence()
{
  if ( playControls->state() != QPlaySequenceControl::Stopped ) {
    playControls->setState(QPlaySequenceControl::Stopped);
  }

  playControls->hide();

  if ( _input_sequence->is_open() ) {
    _input_sequence->clear();
  }

  Q_EMIT currentImageChanged();
}


void QImageFileViewer::startDisplay()
{
  QWaitCursor wait(this);

  Base::setImage(cv::noArray(),cv::noArray(),cv::noArray(), false);

  playControls->setState(QPlaySequenceControl::Stopped);
  playControls->hide();

  if ( !_input_sequence->is_open() && !_input_sequence->open() ) {
    return;
  }

  const int num_frames = _input_sequence->size();

  if ( num_frames < 1 ) {
    return;
  }

  if ( num_frames > 1 ) {
    playControls->show();
    playControls->setSeekRange(0, num_frames - 1);
    playControls->setCurpos(0);
  }

  loadNextFrame();
}

void QImageFileViewer::onSeek(int pos)
{
  if ( _input_sequence && _input_sequence->is_open() ) {
    if ( pos != _input_sequence->current_pos() ) {
      _input_sequence->seek(pos);
    }
    loadNextFrame();
  }
}

void QImageFileViewer::loadNextFrame()
{
  if ( _input_sequence && _input_sequence->is_open() ) {

    c_input_source::sptr current_source =
        _input_sequence->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

      if ( true ) {
        current_image_lock lock(this);
        _input_sequence->read(_currentImage, &_currentMask);
      }
      Base::updateDisplay();
      Q_EMIT currentImageChanged();
    }
  }
}



QString QImageFileViewer::currentFileName() const
{
  if ( _input_sequence->is_open() ) {

    c_input_source::sptr source =
        _input_sequence->current_source();

    if ( source ) {
      return source->filename().c_str();
    }
  }

  return QString();
}

//void QImageFileViewer::showEvent(QShowEvent *e)
//{
//  Base::showEvent(e);
//}

void QImageFileViewer::hideEvent(QHideEvent *e)
{
  closeCurrentSequence();

//  Base::currentImage_.release();
//  Base::currentImageData_.release();

  Base::hideEvent(e);
}

