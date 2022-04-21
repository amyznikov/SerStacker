/*
 * QImageFileViewer.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#include "QImageFileViewer.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/debug.h>

QImageFileViewer::QImageFileViewer(QWidget * parent)
  : Base(parent)
{
  input_sequence_ = c_input_sequence::create();
  input_sequence_->set_auto_debayer(DEBAYER_GB);
  input_sequence_->set_auto_apply_color_matrix(true);

  Base::layout_->insertWidget(2, playControls =
      new QPlaySequenceControl(this));

  connect(playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

  playControls->setVisible(false);
}

const c_input_sequence::ptr & QImageFileViewer::input_sequence() const
{
  return input_sequence_;
}

void QImageFileViewer::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData, bool make_copy )
{
  closeCurrentSequence();
  Base::setImage(image, mask, imageData, make_copy);
}

void QImageFileViewer::openImage(const std::string & pathfilename)
{
  input_sequence_->clear();
  input_sequence_->add_source(pathfilename);

  startDisplay();
}

void QImageFileViewer::openImage(const QString & pathfilename)
{
  openImage(pathfilename.toStdString());
}

void QImageFileViewer::openImages(const std::vector<std::string> & pathfilenames)
{
  input_sequence_->clear();
  input_sequence_->add_sources(pathfilenames);
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

  if ( input_sequence_->is_open() ) {
    input_sequence_->clear();
  }

  emit currentImageChanged();
}


void QImageFileViewer::startDisplay()
{
  QWaitCursor wait(this);

  Base::setImage(cv::noArray(),cv::noArray(),cv::noArray(), false);

  playControls->setState(QPlaySequenceControl::Stopped);
  playControls->hide();

  if ( !input_sequence_->is_open() && !input_sequence_->open() ) {
    return;
  }

  const int num_frames = input_sequence_->size();

  CF_DEBUG("num_frames=%d", num_frames);

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
  if ( input_sequence_ && input_sequence_->is_open() ) {
    if ( pos != input_sequence_->current_pos() ) {
      input_sequence_->seek(pos);
    }
    loadNextFrame();
  }
}

void QImageFileViewer::loadNextFrame()
{
  if ( input_sequence_ && input_sequence_->is_open() ) {

    c_input_source::ptr current_source =
        input_sequence_->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

      input_sequence_->read(currentImage_, &currentMask_);
      Base::updateDisplay();
      emit currentImageChanged();
    }
  }
}



QString QImageFileViewer::currentFileName() const
{
  if ( input_sequence_->is_open() ) {

    c_input_source::ptr source =
        input_sequence_->current_source();

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

