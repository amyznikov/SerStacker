/*
 * QGraphicsShapeSettings.h
 *
 *  Created on: Feb 24, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QGraphicsShapeSettings_h__
#define __QGraphicsShapeSettings_h__

#include <gui/widgets/QSettingsWidget.h>

template<class QGraphicsShapeType>
class QGraphicsShapeSettings :
    public QSettingsWidgetTemplate<QGraphicsShapeType>
{
public:
  typedef QGraphicsShapeSettings ThisClass;
  typedef QSettingsWidgetTemplate<QGraphicsShapeType> Base;
  typedef QGraphicsShapeType ShapeType;

  QGraphicsShapeSettings(QWidget * parent = nullptr) :
    Base(parent)
  {
  }

  void setShape(QGraphicsShapeType * shape)
  {
    ThisClass::setOpts(shape);
  }

  QGraphicsShapeType * shape() const
  {
    return ThisClass::opts();
  }
};


template<class QGraphicsShapeSettingsWidgetType>
class QGraphicsShapeSettingsDialogBox:
    public QSettingsDialogBoxTemplate<QGraphicsShapeSettingsWidgetType>
{
public:
  typedef QGraphicsShapeSettingsDialogBox ThisClass;
  typedef QSettingsDialogBoxTemplate<QGraphicsShapeSettingsWidgetType> Base;
  typedef QGraphicsShapeSettingsWidgetType SettingsWidgetType;
  typedef typename QGraphicsShapeSettingsWidgetType::ShapeType ShapeType;

  QGraphicsShapeSettingsDialogBox(const QString & title, QWidget * parent = nullptr) :
      Base(title, parent)
  {
    Base::setMinimumWidth(256);
  }

  void setShape(ShapeType * shape)
  {
    Base::_settings->setShape(shape);
  }

  ShapeType * shape() const
  {
    return Base::_settings->shape();
  }
};

#endif /* __QGraphicsShapeSettings_h__ */
