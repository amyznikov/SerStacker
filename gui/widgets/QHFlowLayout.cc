/*
 * QHFlowLayout.cc
 *
 *  Created on: Apr 4, 2026
 *      Author: amyznikov
 */

#include "QHFlowLayout.h"

QHFlowLayout::QHFlowLayout(QWidget * parent, int margin, int hSpacing, int vSpacing) :
    Base(parent), _hSpace(hSpacing), _vSpace(vSpacing)
{
  setContentsMargins(margin, margin, margin, margin);
}

QHFlowLayout::QHFlowLayout(int margin, int hSpacing, int vSpacing) :
    _hSpace(hSpacing), _vSpace(vSpacing)
{
  setContentsMargins(margin, margin, margin, margin);
}

QHFlowLayout::~QHFlowLayout()
{
  QLayoutItem * item;
  while ((item = takeAt(0))) {
    delete item;
  }
}

void QHFlowLayout::addItem(QLayoutItem * item)
{
  _items.append(item);
}

int QHFlowLayout::horizontalSpacing() const
{
  return (_hSpace >= 0) ? _hSpace : smartSpacing(QStyle::PM_LayoutHorizontalSpacing);
}

int QHFlowLayout::verticalSpacing() const
{
  return (_vSpace >= 0) ? _vSpace : smartSpacing(QStyle::PM_LayoutVerticalSpacing);
}

Qt::Orientations QHFlowLayout::expandingDirections() const
{
  return {};
}

bool QHFlowLayout::hasHeightForWidth() const
{
  return true;
}

int QHFlowLayout::heightForWidth(int width) const
{
  int height = doLayout(QRect(0, 0, width, 0), true);
  return height;
}

int QHFlowLayout::count() const
{
  return _items.size();
}

QLayoutItem* QHFlowLayout::itemAt(int index) const
{
  return _items.value(index);
}

QSize QHFlowLayout::minimumSize() const
{
  QSize size;
  for( const QLayoutItem * item : qAsConst(_items) ) {
    size = size.expandedTo(item->minimumSize());
  }

  const QMargins margins = contentsMargins();
  size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom());
  return size;
}

void QHFlowLayout::setGeometry(const QRect & rect)
{
  Base::setGeometry(rect);
  doLayout(rect, false);
}

QSize QHFlowLayout::sizeHint() const
{
  return minimumSize();
}

QLayoutItem* QHFlowLayout::takeAt(int index)
{
  if( index >= 0 && index < _items.size() ) {
    return _items.takeAt(index);
  }
  return nullptr;
}

int QHFlowLayout::doLayout(const QRect & rect, bool testOnly) const
{
  int left, top, right, bottom;
  getContentsMargins(&left, &top, &right, &bottom);
  QRect effectiveRect = rect.adjusted(+left, +top, -right, -bottom);
  int x = effectiveRect.x();
  int y = effectiveRect.y();
  int lineHeight = 0;

  for( QLayoutItem * item : qAsConst(_items) ) {
    const QWidget * wid = item->widget();
    int spaceX = horizontalSpacing();

    if( spaceX == -1 ) {
      spaceX = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Horizontal);
    }

    int spaceY = verticalSpacing();
    if( spaceY == -1 ) {
      spaceY = wid->style()->layoutSpacing(QSizePolicy::PushButton, QSizePolicy::PushButton, Qt::Vertical);
    }

    int nextX = x + item->sizeHint().width() + spaceX;
    if( nextX - spaceX > effectiveRect.right() && lineHeight > 0 ) {
      x = effectiveRect.x();
      y = y + lineHeight + spaceY;
      nextX = x + item->sizeHint().width() + spaceX;
      lineHeight = 0;
    }

    if( !testOnly ) {
      item->setGeometry(QRect(QPoint(x, y), item->sizeHint()));
    }

    x = nextX;
    lineHeight = qMax(lineHeight, item->sizeHint().height());
  }
  return y + lineHeight - rect.y() + bottom;
}

int QHFlowLayout::smartSpacing(QStyle::PixelMetric pm) const
{
  QObject * parent = this->parent();
  if( !parent ) {
    return -1;
  }

  if( parent->isWidgetType() ) {
    QWidget * pw = static_cast<QWidget*>(parent);
    return pw->style()->pixelMetric(pm, nullptr, pw);
  }

  return static_cast<QLayout*>(parent)->spacing();
}

