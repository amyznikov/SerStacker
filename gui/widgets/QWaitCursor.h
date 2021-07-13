/*
 * QWaitCursor.h
 *
 *  Created on: Jul 16, 2018
 *      Author: amyznikov
 */

#ifndef __CVXQT_QWaitCursor_h__
#define __CVXQT_QWaitCursor_h__

#include <QtWidgets/QtWidgets>

class QWaitCursor {
  bool show_;
public:
  QWaitCursor(QWidget * w = Q_NULLPTR, bool show = true) : show_(show) {
    if ( show_ ) {
      QApplication::setOverrideCursor(Qt::WaitCursor);
    }
  }
  ~QWaitCursor() {
    if ( show_ ) {
      QApplication::restoreOverrideCursor();
    }
  }
};


#endif /* __CVXQT_QWaitCursor_h__ */
