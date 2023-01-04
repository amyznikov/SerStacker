/*
 * QImageEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageEditor.h"
#include "cv2qt.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/improc/c_unsharp_mask_routine.h>
#include <core/improc/c_anscombe_routine.h>
#include <core/improc/c_autoclip_routine.h>
#include <core/improc/c_noisemap_routine.h>
#include <core/improc/c_align_color_channels_routine.h>
#include <core/average/c_frame_accumulation.h>
#include <core/debug.h>


QImageEditor::QImageEditor(QWidget * parent)
  : Base(parent)
{
}

const cv::Mat & QImageEditor::inputImage() const
{
  return inputImage_;
}

cv::Mat & QImageEditor::inputImage()
{
  return inputImage_;
}

const cv::Mat & QImageEditor::inputMask() const
{
  return inputMask_;
}

cv::Mat & QImageEditor::inputMask()
{
  return inputMask_;
}

void QImageEditor::set_current_processor(const c_image_processor::ptr & processor)
{
  current_processor_ = processor;
  updateImage();
}

const c_image_processor::ptr & QImageEditor::current_processor() const
{
  return current_processor_;
}

void QImageEditor::clear()
{
  setCurrentImage(cv::noArray(), cv::noArray(), cv::noArray(), false);
}

void QImageEditor::editImage(cv::InputArray image, cv::InputArray mask, bool make_copy)
{
  if ( make_copy ) {
    image.getMat().copyTo(inputImage_);
    mask.getMat().copyTo(inputMask_);
  }
  else {
    inputImage_ = image.getMat();
    inputMask_ = mask.getMat();
  }

  updateImage();
}

void QImageEditor::updateImage()
{
  if ( !isVisible() ) {
    hasPendingUpdates_ = true;
  }
  else {

    //QWaitCursor wait(this);

    hasPendingUpdates_ = false;

    if ( inputImage_.empty() ) {
      currentImage_.release();
      currentMask_.release();
      currentImageData_.release();
    }
    else {

      inputImage_.copyTo(currentImage_);
      inputMask_.copyTo(currentMask_);

      if ( current_processor_ && !current_processor_->empty() ) {
        current_processor_->process(currentImage_, currentMask_);
      }
    }

    if ( displayFunction_ ) {
      displayFunction_->setCurrentImage(currentImage_,
          currentMask_);
    }

    updateDisplay();
  }

  Q_EMIT currentImageChanged();
}

void QImageEditor::showEvent(QShowEvent *event)
{
  if ( !hasPendingUpdates_ ) {
    Base::showEvent(event);
  }
  else {
    updateImage();
    QWidget::showEvent(event);
    emit visibilityChanged(isVisible());
  }
}

