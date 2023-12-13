/*
 * QInputTextSourceView.h
 *
 *  Created on: Dec 10, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputTextSourceView_h__
#define __QInputTextSourceView_h__

#include <gui/qtextview/QTextFileViewer.h>

namespace cloudview {

class QTextSourceView :
    public QTextFileViewer
{
public:
  typedef QTextSourceView ThisClass;
  typedef QTextFileViewer Base;

  QTextSourceView(QWidget * parent = nullptr);

protected:
};

} /* namespace cloudview */

#endif /* __QInputTextSourceView_h__ */
