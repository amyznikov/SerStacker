/*
 * QToolBarStretch.cc
 *
 *  Created on: May 6, 2023
 *      Author: amyznikov
 */

#include "QToolBarStretch.h"

QToolBarStretch::QToolBarStretch(QWidget * parent) : Base(parent)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
}

