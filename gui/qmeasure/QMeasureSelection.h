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
#include <gui/widgets/UpdateControls.h>


class QSingeMeasureSelectionWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QSingeMeasureSelectionWidget ThisClass;
  typedef QSettingsWidget Base;

  QSingeMeasureSelectionWidget(QWidget * parent = nullptr);
  QSingeMeasureSelectionWidget(QMeasureProvider * mp, QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * mp);
  QMeasureProvider * measureProvider() const;

  QMeasure * currentMeasure() const;

Q_SIGNALS:
  void currentMeasureChanged();

protected:
  void onupdatecontrols();
  void updatesettingswidget();

protected:
  QMeasureProvider * mp_ = nullptr;
  QMeasure * cm_ = nullptr;
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

  QMeasure * currentMeasure() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentMeasureChanged();

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;
  void closeEvent(QCloseEvent * event) override;

protected:
  QSingeMeasureSelectionWidget * measureSelection_ctl = nullptr;
};


class QMultiMeasureSelectionWidget :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMultiMeasureSelectionWidget ThisClass;
  typedef QFrame Base;

  QMultiMeasureSelectionWidget(QWidget * parent = nullptr);
  QMultiMeasureSelectionWidget(QMeasureProvider * mp, QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * mp);
  QMeasureProvider * measureProvider() const;

Q_SIGNALS:
  void selectedMeasuresChanged();

protected:
  void onupdatecontrols();
  void updatesettingswidget();
  void onListViewItemChanged(QListWidgetItem *item);
  void onListViewCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

protected:
  QMeasureProvider * mp_ = nullptr;
  QHBoxLayout * hbox_ = nullptr;
  QVBoxLayout * vbox_ = nullptr;

  QListWidget * listview_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  QNumericBox * maxMeasurements_ctl = nullptr;
  //QScrollArea * scrollArea_ctl = nullptr;
  QMeasureSettingsWidget * cw_ = nullptr;
  QList<QMeasureSettingsWidget*> settingsWidgets_;
};


class QMultiMeasureSelectionDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QMultiMeasureSelectionDialogBox ThisClass;
  typedef QDialog Base;

  QMultiMeasureSelectionDialogBox(QWidget * parent = nullptr);
  QMultiMeasureSelectionDialogBox(QMeasureProvider * mp, QWidget * parent = nullptr);
  QMultiMeasureSelectionDialogBox(const QString & title, QMeasureProvider * mp, QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * mp);
  QMeasureProvider * measureProvider() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void selectedMeasuresChanged();

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;
  void closeEvent(QCloseEvent * event) override;

protected:
  QMultiMeasureSelectionWidget * measureSelection_ctl = nullptr;
};


#endif /* __QMeasureSelection_h__ */
