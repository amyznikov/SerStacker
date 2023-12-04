/*
 * QCloudViewImageEditor.h
 *
 *  Created on: Dec 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewImageEditor_h__
#define __QCloudViewImageEditor_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>

namespace cloudview {

class QCloudViewImageEditor :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QCloudViewImageEditor ThisClass;
  typedef QImageEditor Base;

  QCloudViewImageEditor(QWidget * parent = nullptr);

  QImageViewMtfDisplayFunction * mtfDisplayFunction();
  const QImageViewMtfDisplayFunction * mtfDisplayFunction() const;

  QGraphicsRectShape * roiShape() const;

protected:
  void createRoiShape();

protected:
  QImageViewMtfDisplayFunction mtfDisplayFunction_;
  QGraphicsRectShape * roiShape_ = nullptr;
};

} /* namespace cloudview */

#endif /* __QCloudViewImageEditor_h__ */
