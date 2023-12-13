/*
 * QInputImageSourceView.h
 *
 *  Created on: Dec 4, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewImageEditor_h__
#define __QCloudViewImageEditor_h__

#include <gui/qimageview/QImageFileEditor.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>

namespace cloudview {

class QImageSourceView :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QImageSourceView ThisClass;
  typedef QImageEditor Base;

  QImageSourceView(QWidget * parent = nullptr);

  QGraphicsRectShape * roiShape() const;

protected:
  void createRoiShape();

protected:
  QGraphicsRectShape * roiShape_ = nullptr;
};

} /* namespace cloudview */

#endif /* __QCloudViewImageEditor_h__ */
