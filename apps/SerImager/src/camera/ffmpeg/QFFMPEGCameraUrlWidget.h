/*
 * QFFMPEGCameraUrlWidget.h
 *
 *  Created on: Mar 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QFFMPEGCameraUrlWidget_h__
#define __QFFMPEGCameraUrlWidget_h__

#include <QtWidgets/QtWidgets>
#include <gui/widgets/QLineEditBox.h>

namespace serimager {

class QFFMPEGCameraUrlWidget :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QFFMPEGCameraUrlWidget ThisClass;
  typedef QWidget Base;

  QFFMPEGCameraUrlWidget(QWidget * parent = nullptr);

  void setUrl(const QString & v);
  QString url() const;

Q_SIGNALS:
  void urlChanged();

protected:
  QHBoxLayout * hbox = nullptr;
  QLineEditBox * url_ctl = nullptr;
  QToolButton * browse_ctl = nullptr;
};

} /* namespace serimager */



#endif /* __QFFMPEGCameraUrlWidget_h__ */
