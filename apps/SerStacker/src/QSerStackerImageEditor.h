/*
 * QSerStackerImageEditor.h
 *
 *  Created on: May 2, 2022
 *      Author: amyznikov
 */

#pragma once
#ifndef __qserstacker_image_editor_h__
#define __qserstacker_image_editor_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>

namespace serstacker {

class QSerStackerImageEditor:
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QSerStackerImageEditor ThisClass;
  typedef QImageEditor Base;

  QSerStackerImageEditor(QWidget * parent = nullptr);

  QImageViewMtfDisplayFunction * mtfDisplayFunction();
  const QImageViewMtfDisplayFunction * mtfDisplayFunction() const;

  QGraphicsRectShape * roiShape() const;

protected:
  void createRoiShape();

protected:
  QImageViewMtfDisplayFunction mtfDisplayFunction_;
  QGraphicsRectShape * roiShape_ = nullptr;
};

} /* namespace serstacker */

#endif /* __qserstacker_image_editor_h__ */
