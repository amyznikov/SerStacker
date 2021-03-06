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

  void set_current_processor(const c_image_processor::ptr & processor);
  const c_image_processor::ptr & current_processor() const;

  virtual void editImage(cv::InputArray image, cv::InputArray mask = cv::noArray());
  void clear();

  cv::Mat & inputImage();
  const cv::Mat & inputImage() const;

  cv::Mat & inputMask();
  const cv::Mat & inputMask() const;


public slots:
  void updateImage();

protected:
  void showEvent(QShowEvent *event) override;

protected:
  c_image_processor::ptr current_processor_;
  cv::Mat inputImage_;
  cv::Mat inputMask_;
  bool hasPendingUpdates_ = false;
};

#endif /* __QImageEditor_h__ */
