/*
 * QImageFileEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageFileEditor.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/debug.h>

QImageFileEditor::QImageFileEditor(QWidget * parent)
  : Base(parent)
{
  input_sequence_ = c_input_sequence::create();
  input_sequence_->set_auto_debayer(DEBAYER_GB);
  input_sequence_->set_auto_apply_color_matrix(true);

  Base::layout_->insertWidget(2, playControls =
      new QPlaySequenceControl(this));

  connect(playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);

}

const c_input_sequence::ptr & QImageFileEditor::input_sequence() const
{
  return input_sequence_;
}

void QImageFileEditor::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  closeCurrentSequence();
  Base::setImage(image, mask, imageData, make_copy);
}

void QImageFileEditor::editImage(cv::InputArray image, cv::InputArray mask)
{
  closeCurrentSequence();
  Base::editImage(image, mask);
}

void QImageFileEditor::openImage(const std::string & pathfilename)
{
  input_sequence_->close(true);
  input_sequence_->add_source(pathfilename);
  startDisplay();
}

void QImageFileEditor::openImage(const QString & pathfilename)
{
  openImage(pathfilename.toStdString());
}

void QImageFileEditor::openImages(const std::vector<std::string> & pathfilenames)
{
  input_sequence_->close(true);
  input_sequence_->add_sources(pathfilenames);
  startDisplay();
}

void QImageFileEditor::openImages(const QStringList & pathfilenames)
{
  std::vector<std::string> std_names;

  std_names.reserve(pathfilenames.size());
  for ( const QString & s : pathfilenames ) {
    std_names.emplace_back(s.toStdString());
  }

  return openImages(std_names);
}

void QImageFileEditor::closeCurrentSequence()
{
  if ( playControls->state() != QPlaySequenceControl::Stopped ) {
    playControls->setState(QPlaySequenceControl::Stopped);
  }

  playControls->hide();

  if ( input_sequence_->is_open() ) {
    input_sequence_->close(true);
  }

  emit currentImageChanged();
}


void QImageFileEditor::startDisplay()
{
  QWaitCursor wait(this);

  Base::setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  playControls->setState(QPlaySequenceControl::Stopped);
  playControls->hide();

  if ( !input_sequence_->is_open() && !input_sequence_->open() ) {
    return;
  }

  const int num_frames = input_sequence_->size();
  if ( num_frames < 1 ) {
    QMessageBox::critical(this, "ERROR",
        "Can not determine number of frames and seek range for given source.\n"
        "Image can not be displayed correctly, seems such input source is not supported");
    return;
  }

  if ( num_frames > 1 ) {
    playControls->show();
    playControls->setSeekRange(0, num_frames - 1);
    playControls->setCurpos(0);
  }

  loadNextFrame();
}

void QImageFileEditor::onSeek(int pos)
{
  if ( input_sequence_ && input_sequence_->is_open() ) {
    if ( pos != input_sequence_->current_pos() ) {
      input_sequence_->seek(pos);
    }
    loadNextFrame();
  }
}

void QImageFileEditor::loadNextFrame()
{
  if ( input_sequence_ && input_sequence_->is_open() ) {

    c_input_source::ptr current_source =
        input_sequence_->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

      input_sequence_->read(inputImage_, &inputMask_);

      updateImage();
      emit currentImageChanged();
    }
  }
}



QString QImageFileEditor::currentFileName() const
{
  if ( input_sequence_->is_open() ) {

    c_input_source::ptr source =
        input_sequence_->current_source();

    if ( source ) {
      return source->filename().c_str();
    }
  }

  return this->currentFileName_;
}

void QImageFileEditor::setCurrentFileName(const QString & newFileName)
{
  this->currentFileName_ = newFileName;
  emit currentImageChanged();
}


//void QImageFileEditor::showEvent(QShowEvent *e)
//{
//  Base::showEvent(e);
//}

void QImageFileEditor::hideEvent(QHideEvent *e)
{
  closeCurrentSequence();

  Base::currentImage_.release();
  Base::currentImageData_.release();
  Base::inputImage_.release();
  Base::inputMask_.release();

  Base::hideEvent(e);
}

