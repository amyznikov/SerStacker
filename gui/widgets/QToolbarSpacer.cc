/*
 * QToolbarSpacer.cc
 *
 *  Created on: Mar 8, 2020
 *      Author: amyznikov
 */

#include "QToolbarSpacer.h"

QToolbarSpacer::QToolbarSpacer(QWidget * parent)
  : Base(parent)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

