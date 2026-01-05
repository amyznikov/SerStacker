/*
 * QLCSCTPUrlWidget.h
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLCSCTPUrlWidget_h__
#define __QLCSCTPUrlWidget_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QLineEditBox.h>

namespace serimager {

class QLCSCTPUrlWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QLCSCTPUrlWidget ThisClass;
  typedef QWidget Base;

  QLCSCTPUrlWidget(QWidget * parent = nullptr);

  void setUrl(const QString & v);
  QString url() const;

Q_SIGNALS:
  void urlChanged();

protected:
  QHBoxLayout * hbox = nullptr;
  QLineEditBox * url_ctl = nullptr;
  //QToolButton * browse_ctl = nullptr;
};

} /* namespace serimager */

#endif /* __QLCSCTPUrlWidget_h__ */
