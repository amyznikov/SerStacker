/*
 * QMeasureSelection.h
 *
 *  Created on: Apr 8, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureSelection_h__
#define __QMeasureSelection_h__

#include "QMeasureProvider.h"


class QSingeMeasureSelectionWidget :
    public QSettingsWidget
{
public:
  typedef QSingeMeasureSelectionWidget ThisClass;
  typedef QSettingsWidget Base;

  QSingeMeasureSelectionWidget(QWidget * parent = nullptr);
  QSingeMeasureSelectionWidget(QMeasureProvider * mp, QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * mp);
  QMeasureProvider * measureProvider() const;

protected:
  void popuatecombo();
  void onupdatecontrols();
  void updatesettingswidget();

protected:
  QMeasureProvider * mp_ = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  QNumericBox * maxMeasurements_ctl = nullptr;
  QScrollArea * scrollArea_ctl = nullptr;
  QList<QMeasureSettingsWidget*> settingsWidgets_;
};


class QSingeMeasureSelectionDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QSingeMeasureSelectionDialogBox ThisClass;
  typedef QDialog Base;

  QSingeMeasureSelectionDialogBox(QMeasureProvider * mp, QWidget * parent = nullptr);
  QSingeMeasureSelectionDialogBox(const QString & title, QMeasureProvider * mp, QWidget * parent = nullptr);

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;
  void closeEvent(QCloseEvent * event) override;

protected:
  QSingeMeasureSelectionWidget * measureSelection_ctl = nullptr;
};


#endif /* __QMeasureSelection_h__ */
