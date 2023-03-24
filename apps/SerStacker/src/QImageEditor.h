/*
 * QImageEditor.h
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

class QImageEditor:
    public QImageFileEditor
{
  Q_OBJECT;
public:
  typedef QImageEditor ThisClass;
  typedef QImageFileEditor Base;

  QImageEditor(QWidget * parent = nullptr);

  QImageViewMtfDisplayFunction * mtfDisplayFunction();
  const QImageViewMtfDisplayFunction * mtfDisplayFunction() const;

  QGraphicsRectShape * roiRectShape() const;

protected:
  void createRoiRectShape();

protected:
  QImageViewMtfDisplayFunction mtfDisplayFunction_;
  QGraphicsRectShape * roiRectShape_ = nullptr;
};

} /* namespace serstacker */

#endif /* __qserstacker_image_editor_h__ */
