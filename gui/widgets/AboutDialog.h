/*
 * AboutDialog.h
 *
 *  Created on: Feb 4, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __AboutDialog_h__
#define __AboutDialog_h__

#include <QtWidgets/QtWidgets>

class AboutDialog: public QDialog
{
  Q_OBJECT;
public:
  typedef AboutDialog ThisClass;
  typedef QDialog Base;

  AboutDialog(const QString appName, const QPixmap & appIcon, QWidget * parent = nullptr);
};

#endif /* __AboutDialog_h__ */
