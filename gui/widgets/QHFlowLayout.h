/*
 * QHFlowLayout.h
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QHFlowLayout_h__
#define __QHFlowLayout_h__

#include <QtWidgets/QtWidgets>

/**
 * Based on Qt QHFlowLayout example
 */
class QHFlowLayout:
    public QLayout
{
public:
  typedef QHFlowLayout ThisClass;
  typedef QLayout Base;

  explicit QHFlowLayout(QWidget * parent, int margin = -1, int hSpacing = -1, int vSpacing = -1);
  explicit QHFlowLayout(int margin = -1, int hSpacing = -1, int vSpacing = -1);
  ~QHFlowLayout();

  void addItem(QLayoutItem * item) override;
  int horizontalSpacing() const;
  int verticalSpacing() const;
  Qt::Orientations expandingDirections() const override;
  bool hasHeightForWidth() const override;
  int heightForWidth(int) const override;
  int count() const override;
  QLayoutItem* itemAt(int index) const override;
  QSize minimumSize() const override;
  void setGeometry(const QRect & rect) override;
  QSize sizeHint() const override;
  QLayoutItem* takeAt(int index) override;

protected:
  int doLayout(const QRect & rect, bool testOnly) const;
  int smartSpacing(QStyle::PixelMetric pm) const;

protected:
  QList<QLayoutItem*> _items;
  int _hSpace;
  int _vSpace;
};

#endif /* __QHFlowLayout_h__ */
