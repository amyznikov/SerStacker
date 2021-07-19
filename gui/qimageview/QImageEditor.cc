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

const cv::Mat & QImageEditor::inputMask() const
{
  return inputMask_;
}

void QImageEditor::clear()
{
  editImage(cv::noArray(), cv::noArray());
}

void QImageEditor::editImage(cv::InputArray image, cv::InputArray mask)
{
  inputImage_ = image.getMat();
  inputMask_ = mask.getMat();
  updateImage();
  emit currentImageChanged();
}

void QImageEditor::updateImage()
{
  if ( inputImage_.empty() ) {
    currentImage_.release();
    currentMask_.release();
    currentImageData_.release();
  }
  else if ( processor_ && processor_->enabled() && !processor_->empty() ) {
    inputImage_.copyTo(currentImage_);
    inputMask_.copyTo(currentMask_);
    processor_->process(currentImage_, currentMask_);
  }
  else {
    currentImage_ = inputImage_;
    currentMask_ = inputMask_;
  }

  updateDisplay();
}

