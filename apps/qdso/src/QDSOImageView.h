/*
 * QDSOImageView.h
 *
 *  Created on: Jun 12, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QDSOImageView_h__
#define __QDSOImageView_h__

#include <gui/qimageview/QImageEditor.h>
#include <gui/qcustomdock/QCustomDock.h>

namespace qdso {

class QDSOImageView :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QDSOImageView ThisClass;
  typedef QImageEditor Base;

  QDSOImageView(QWidget * parent = nullptr);

protected:
};

class QDSOImageViewDock:
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QDSOImageViewDock ThisClass;
  typedef QCustomDockWidget Base;

  QDSOImageViewDock(const QString & title,
      QWidget * parent,
      QDSOImageView * view,
      Qt::WindowFlags flags = Qt::WindowFlags());

  QDSOImageView * view() const;

protected:
};

}
#endif /* __QDSOImageView_h__ */
