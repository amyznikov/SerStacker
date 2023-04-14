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


class QMeasureSelectionCombo :
    public QComboBox
{
  Q_OBJECT;
public:
  typedef QMeasureSelectionCombo ThisClass;
  typedef QComboBox Base;

  QMeasureSelectionCombo(QWidget * parent = nullptr);

  QMeasure * currentMeasure();

Q_SIGNALS:
  void currentMeasureChanged();
};


class QMeasureSettingsDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QMeasureSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QMeasureSettingsDialogBox(QWidget * parent = nullptr);
  QMeasureSettingsDialogBox(const QString & title, QWidget * parent = nullptr);

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected Q_SLOTS:
  void onCurrentMeasureChanged();

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;
  void closeEvent(QCloseEvent * event) override;

protected:
  QVBoxLayout * layout_ = nullptr;
  QMeasureSelectionCombo * combobox_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  //QNumericBox * maxMeasurements_ctl = nullptr;
  QScrollArea * scrollArea_ctl = nullptr;
  QList<QMeasureSettingsWidget*> widgets_;
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

  void selectMeasures(std::set<QMeasure*> * r);

Q_SIGNALS:
  void selectedMeasuresChanged();

protected:
  void onupdatecontrols();
  void onListViewItemChanged(QListWidgetItem *item);
  void onListViewCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

protected:
  std::set<QMeasure*> * cm_ = nullptr;
  QHBoxLayout * hbox_ = nullptr;
  QVBoxLayout * vbox_ = nullptr;

  QListWidget * listview_ctl = nullptr;
  QLabel * tooltip_ctl = nullptr;
  QNumericBox * maxMeasurements_ctl = nullptr;
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
  QMultiMeasureSelectionDialogBox(const QString & title, QWidget * parent = nullptr);

  void selectMeasures(std::set<QMeasure*> * r);

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
