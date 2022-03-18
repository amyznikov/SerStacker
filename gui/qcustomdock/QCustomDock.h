/*
 * QCustomDock.h
 *
 *  Created on: Mar 19, 2019
 *      Author: amyznikov
 */

#ifndef __QCustomDock_h__
#define __QCustomDock_h__

#include <QtWidgets/QtWidgets>


class QCustomDockTitleBarLabel;
class QCustomDockTitleBar;


class QCustomDockWidget
    : public QDockWidget
{
  Q_OBJECT;
public:
  typedef QDockWidget Base;

  explicit QCustomDockWidget(const QString & title,
      QWidget * parent = Q_NULLPTR,
      QWidget * view = Q_NULLPTR,
      Qt::WindowFlags flags = Qt::WindowFlags());

  QCustomDockTitleBar * titleBar() const;
};




class QCustomDockTitleBar
    : public QWidget
{
  Q_OBJECT;
public:
  typedef QWidget Base;

  QCustomDockTitleBar(const QString & title = QString());

  QCustomDockTitleBarLabel * titleLabel() const;

  QToolButton * addButton(QToolButton * button);
  QToolButton * addButton(QAction * action);

  QToolButton * addButton(const QIcon & icon, const QString & tooltip = "");
  QToolButton * addButton(const QString & icon, const QString & tooltip = "");


  template<class _Calable>
  QToolButton * addButton(const QIcon & icon, const QString & tooltip, _Calable && slot)
  {
    QToolButton * tb = addButton(icon, tooltip);
    if ( tb ) {
      QObject::connect(tb, &QToolButton::clicked, slot);
    }
    return tb;
  }


  template<class _Calable>
  QToolButton * addButton(const QString & icon, const QString & tooltip, _Calable && slot)
  {
    QToolButton * tb = addButton(icon, tooltip);
    if ( tb ) {
      QObject::connect(tb, &QToolButton::clicked, slot);
    }
    return tb;
  }

  template<class Obj, class _Calable>
  QToolButton * addButton(const QIcon & icon, const QString & tooltip, Obj * obj, _Calable slot)
  {
    QToolButton * tb = addButton(icon, tooltip);
    if ( tb ) {
      QObject::connect(tb, &QToolButton::clicked, obj, slot);
    }
    return tb;
  }

public: // required by QDockWidget
  QSize minimumSizeHint() const override;
  QSize sizeHint() const override;

protected:
  QHBoxLayout * layout_;
  QCustomDockTitleBarLabel * title_;
  QToolButton * close_button_;
  QToolButton * float_button_;
};



class QCustomDockTitleBarLabel
    : public QLabel
{
  Q_OBJECT;
public:
  typedef QLabel Base;

  QCustomDockTitleBarLabel(const QString & title );

protected: // required by QDockWidget
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
#if QT_CONFIG(wheelevent)
  void wheelEvent(QWheelEvent *event) override;
#endif
};




template<class QCustomDockWidgetType, class QClientWidgetType = QWidget>
QCustomDockWidgetType * addDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QClientWidgetType * clientWidget = Q_NULLPTR,
    QMenu * viewMenu = Q_NULLPTR)
{
  QCustomDockWidgetType * dock = new QCustomDockWidgetType(title, parent, clientWidget);
  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if ( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}



inline QCustomDockWidget * addCustomDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QWidget * clientWidget = Q_NULLPTR,
    QMenu * viewMenu = Q_NULLPTR)
{
  return addDock<QCustomDockWidget>(parent, area, dockName, title, clientWidget, viewMenu);
}



#endif /* __QCustomDock_h__ */
