/*
 * QSignalsBock.h
 *
 *  Created on: May 3, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSignalsBock_h__
#define __QSignalsBock_h__

#include <QtCore/QtCore>

class QSignalsBock
{
public:
  QSignalsBock(QObject * obj)
  {
    if( !obj || obj->signalsBlocked() ) {
      obj_ = nullptr;
    }
    else {
      (obj_ = obj)->blockSignals(true);
    }
  }

  ~QSignalsBock()
  {
    if( obj_ ) {
      obj_->blockSignals(false);
    }
  }

protected:
  QObject * obj_;
};

#endif /* __QSignalsBock_h__ */
