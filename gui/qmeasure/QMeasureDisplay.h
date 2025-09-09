/*
 * QMeasureDisplay.h
 *
 *  Created on: Apr 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QMeasureDisplay_h__
#define __QMeasureDisplay_h__

#include "QMeasureSelection.h"
#include <gui/widgets/UpdateControls.h>


class QMeasureDisplay :
    public QFrame
{
  Q_OBJECT;
public:
  typedef QMeasureDisplay ThisClass;
  typedef QFrame Base;

  QMeasureDisplay(QWidget * parent = nullptr);

  QMultiMeasureSelectionDialogBox * measureSelector() const
  {
    return _measureSelectorDialogBox;
  }

  void loadParameters();
  void saveParameters();
  void loadParameters(const QSettings & settings);
  void saveParameters(QSettings & settings);

Q_SIGNALS:
  void updateAvailableMeasureDataChannelsRequired();
  void measureRightNowRequested();

protected:
  void setupToolbar();
  void onSelectMeasuresClicked(bool checked);
  void onEnableMeasurementsClicked(bool checked);

protected:
  void updateVisibleColumns();
  void onFramesMeasured(const QList<QMeasureProvider::MeasuredFrame> & frames);
  void updateEnableMeasurements();
  void clearMeasurements();
  void onTableViewContextMenuRequested(const QPoint &pos);
  void onTableViewCurrentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);

protected:
  void showEvent(QShowEvent * e) final;
  void hideEvent(QHideEvent * e) final;
  void closeEvent(QCloseEvent * event) final;

protected:
  QMeasureProvider::MeasuresCollection _cm;
  bool _measurementsEnabled = false;

  QVBoxLayout * _lv = nullptr;
  QToolBar * _toolbar = nullptr;
  QTableWidget * _table = nullptr;
  QMultiMeasureSelectionDialogBox * _measureSelectorDialogBox = nullptr;

  QAction * _saveToFileAction = nullptr;
  QAction * _copyToClipboardAction = nullptr;
  QAction * _clearTableAction = nullptr;
  QAction * _selectMeasuresAction = nullptr;
  QAction * _incrementalModeAction = nullptr;
  QAction * _enableMeasureAction = nullptr;
};


class QMeasureDisplayDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QMeasureDisplayDialogBox ThisClass;
  typedef QDialog Base;

  QMeasureDisplayDialogBox(QWidget * parent = nullptr);
  QMeasureDisplayDialogBox(const QString & title, QWidget * parent = nullptr);

  QMultiMeasureSelectionDialogBox * measureSelector() const
  {
    return _measureDisplay->measureSelector();
  }

  void loadParameters();
  void saveParameters();
  void loadParameters(const QSettings & settings);
  void saveParameters(QSettings & settings);

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void updateAvailableMeasureDataChannelsRequired();
  void measureRightNowRequested();

protected:
  void showEvent(QShowEvent * e) final;
  void hideEvent(QHideEvent * e) final;

protected:
  QMeasureDisplay * _measureDisplay = nullptr;
};

#endif /* __QMeasureDisplay_h__ */
