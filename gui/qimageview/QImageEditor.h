/*
 * QImageEditor.h
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#ifndef __QImageEditor_h__
#define __QImageEditor_h__

#include "QImageViewer.h"
#include <core/improc/c_image_processor.h>

class QImageEditor
    : public QImageViewer
{
  Q_OBJECT;
public:
  typedef QImageEditor ThisClass;
  typedef QImageViewer Base;

  QImageEditor(QWidget * parent = Q_NULLPTR);

  const c_image_processor_chain::ptr & processor() const {
    return processor_;
  }

  void set_processor(const c_image_processor_chain::ptr & processor) {
    processor_ = processor;
    updateImage();
  }

  //void setImage(cv::InputArray image, cv::InputArray imageData = cv::noArray(), bool make_copy = true);
  void editImage(cv::InputArray image, cv::InputArray mask = cv::noArray());

  const cv::Mat & inputImage() const;
  //const cv::Mat & editedImage() const;

  const cv::Mat & inputMask() const;
  //const cv::Mat & editedMask() const;

signals:
  void currentImageChanged();

public slots:
  //void updateDisplay() override;
  void updateImage();

protected:

  c_image_processor_chain::ptr processor_;
    // = c_image_processor_chain::create();

  cv::Mat inputImage_;
  cv::Mat inputMask_;

};

#endif /* __QImageEditor_h__ */
