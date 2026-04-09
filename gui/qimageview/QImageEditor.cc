/*
 * QImageEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageEditor.h"


QImageEditor::QImageEditor(QWidget * parent) :
  Base(parent)
{
}

QImageEditor::QImageEditor(QImageScene * scene, QWidget * parent) :
    Base(scene, parent)
{
}


const cv::Mat & QImageEditor::inputImage() const
{
  return _inputImage;
}

cv::Mat & QImageEditor::inputImage()
{
  return _inputImage;
}

const cv::Mat & QImageEditor::inputMask() const
{
  return _inputMask;
}

cv::Mat & QImageEditor::inputMask()
{
  return _inputMask;
}

void QImageEditor::setCurrentProcessor(const c_image_processor::sptr & processor)
{
  _currentProcessor = processor;
  updateImage();
}

const c_image_processor::sptr & QImageEditor::currentProcessor() const
{
  return _currentProcessor;
}

void QImageEditor::clear()
{
  setCurrentImage(cv::noArray(), cv::noArray(), cv::noArray(), false);
  Q_EMIT currentImageChanged();
  updateDisplay();
}

void QImageEditor::editImage(cv::InputArray image, cv::InputArray mask, bool make_copy)
{
  if ( make_copy ) {
    image.getMat().copyTo(_inputImage);
    mask.getMat().copyTo(_inputMask);
  }
  else {
    _inputImage = image.getMat();
    _inputMask = mask.getMat();
  }

  updateImage();
}

void QImageEditor::updateImage()
{
  if ( !isVisible() ) {
    _hasPendingUpdates = true;
  }
  else {

    //QWaitCursor wait(this);

    _hasPendingUpdates = false;

   // c_current_image_lock lock(this);

    if ( _inputImage.empty() ) {

      current_image_lock lock(this);
      _currentImage.release();
      _currentMask.release();
      _currentImageData.release();
    }
    else {

      current_image_lock lock(this);

      _inputImage.copyTo(_currentImage);

      if( enableEditMask() && keepMaskOnMaskEditMode() && _currentImage.size() == _currentMask.size() ) {
        _inputMask.release();
      }
      else {
        _inputMask.copyTo(_currentMask);
      }

      if ( _currentProcessor && !_currentProcessor->empty() ) {
        _currentProcessor->process(_currentImage, _currentMask);
      }
    }

    Q_EMIT currentImageChanged();
    updateDisplay();
  }

}

void QImageEditor::showEvent(QShowEvent * event)
{
  if( _hasPendingUpdates ) {
    updateImage();
  }
  Base::showEvent(event);
  Q_EMIT visibilityChanged(isVisible());
}

