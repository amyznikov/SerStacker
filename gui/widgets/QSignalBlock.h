/*
 * QSignalBlock.h
 *
 *  Created on: Mar 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QSignalBlock__h__
#define __QSignalBlock__h__

#include <QtCore/QtCore>

class QSignalBlock
{
public:
  template<typename ... Args>
  explicit QSignalBlock(Args * ... args) :
      objects( { static_cast<QObject*>(args)... })
  {
    for( auto obj : objects ) {
      if( obj ) {
        obj->blockSignals(true);
      }
    }
  }

  ~QSignalBlock()
  {
    for( auto obj : objects ) {
      if( obj ) {
        obj->blockSignals(false);
      }
    }
  }

  QSignalBlock(const QSignalBlock&) = delete;
  QSignalBlock& operator=(const QSignalBlock&) = delete;

protected:
  std::vector<QObject*> objects;
};



#endif /* __QSignalBlock__h__ */
