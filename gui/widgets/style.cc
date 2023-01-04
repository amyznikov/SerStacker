/*
 * style.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */
#include "style.h"
#include <core/debug.h>

static QString styleSelector;

void setIconStyleSelector(const QString & selector)
{
  styleSelector = selector;
}

const QString & iconStyleSelector()
{
  return styleSelector;
}

QPixmap getPixmap(const QString & name)
{
  QPixmap pixmap(name);

  if( pixmap.isNull() && !styleSelector.isEmpty() ) {

    QFileInfo f(name);

    QString styledName =
        QString("%1/%2/%3")
            .arg(f.path())
            .arg(styleSelector)
            .arg(f.fileName());

    pixmap = QPixmap(styledName);
  }

  return pixmap;
}

QIcon getIcon(const QString & name)
{
  return QIcon(getPixmap(name));
}
