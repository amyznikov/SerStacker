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

  void editImage(cv::InputArray image, cv::InputArray mask = cv::noArray());
  void clear();

  cv::Mat & inputImage();
  const cv::Mat & inputImage() const;

  cv::Mat & inputMask();
  const cv::Mat & inputMask() const;

signals:
  void currentImageChanged();

public slots:
  void updateImage();

protected:
  c_image_processor_chain::ptr processor_;
  cv::Mat inputImage_;
  cv::Mat inputMask_;
};

#endif /* __QImageEditor_h__ */
