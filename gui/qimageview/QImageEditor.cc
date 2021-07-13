/*
 * QImageEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageEditor.h"
#include "cv2qt.h"
#include <core/debug.h>

QImageEditor::QImageEditor(QWidget * parent)
  : Base(parent)
{
  processor_ = c_image_processor_chain::create();
  processor_->emplace_back(c_unsharp_mask_image_processor::create(true));
  processor_->emplace_back(c_align_color_channels_image_processor::create(false));
  processor_->emplace_back(c_unsharp_mask_image_processor::create(false));
  processor_->emplace_back(c_unsharp_mask_image_processor::create(false));
 //processor_->emplace_back(c_test_image_processor::create(false));
  //processor_->emplace_back(c_smap_image_processor::create(false));
  processor_->emplace_back(c_autoclip_image_processor::create(false));
  //processor_->emplace_back(c_mtf_image_processor::create());
  processor_->set_enabled(false);
}

const cv::Mat & QImageEditor::inputImage() const
{
  return inputImage_;
}

const cv::Mat & QImageEditor::editedImage() const
{
  return currentImage_;
}

const cv::Mat & QImageEditor::inputMask() const
{
  return inputMask_;
}

const cv::Mat & QImageEditor::editedMask() const
{
  return editedMask_;
}

//void QImageEditor::setImage(cv::InputArray image, cv::InputArray imageData = cv::noArray(), bool make_copy = true)
//{
//  inputImage_ = image.getMat();
//  inputMask_ = mask.getMat();
//  updateDisplay();
//  emit currentImageChanged();
//
//}

void QImageEditor::editImage(cv::InputArray image, cv::InputArray mask)
{
  inputImage_ = image.getMat();
  inputMask_ = mask.getMat();
  updateDisplay();
  emit currentImageChanged();
}

void QImageEditor::updateDisplay()
{
  if ( inputImage_.empty() || !processor_ || !processor_->enabled() || processor_->empty() ) {
    currentImage_ = inputImage_;
    Base::updateDisplay();
  }
  else {

    if ( currentImage_.data == inputImage_.data ) {
      currentImage_ = cv::Mat();
    }

    inputImage_.copyTo(currentImage_);
    inputMask_.copyTo(editedMask_);
    processor_->process(currentImage_, editedMask_);

    cv::Mat tmp;
    if ( displayFunction_ ) {
      displayFunction_(currentImage_, tmp, CV_8U);
    }
    else {
      tmp = currentImage_;
    }

    cv2qt(tmp, &qimage_);
    view_->scene()->setBackground(qimage_);
  }
}
