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

class QImageEditor :
    public QImageViewer
{
  Q_OBJECT;
public:
  typedef QImageEditor ThisClass;
  typedef QImageViewer Base;

  QImageEditor(QWidget * parent = nullptr);
  QImageEditor(QImageScene * scene, QWidget * parent = nullptr);

  void set_current_processor(const c_image_processor::sptr & processor);
  const c_image_processor::sptr & current_processor() const;

  virtual void editImage(cv::InputArray image, cv::InputArray mask = cv::noArray(), bool make_copy = false);
  void clear();

  cv::Mat & inputImage();
  const cv::Mat & inputImage() const;

  cv::Mat & inputMask();
  const cv::Mat & inputMask() const;

public Q_SLOTS:
  void updateImage();

protected:
  void showEvent(QShowEvent *event) override;

protected:
  c_image_processor::sptr current_processor_;
  cv::Mat inputImage_;
  cv::Mat inputMask_;
  bool hasPendingUpdates_ = false;
};

#endif /* __QImageEditor_h__ */
