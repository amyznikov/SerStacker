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

protected:
  void setupToolbar();
  void setupTableView();
  void onSelectMeasuresClicked(bool checked);

protected:
  void updateVisibleColumns();
  void updateMeasurements();
  void updateEnableMeasurements();
  void onTableViewContextMenuRequested(const QPoint &pos);
  void onTableViewCurrentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;
  void closeEvent(QCloseEvent * event) override;

protected:
  std::set<QMeasure*> cm_;

  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QTableWidget * table_ = nullptr;
  QMultiMeasureSelectionDialogBox * measureSelectorDialog_ = nullptr;

  QAction * saveToFileAction_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * clearTableAction_ = nullptr;
  QAction * selectMeasuresAction_ = nullptr;
  QAction * incrementalModeAction_ = nullptr;
  QAction * enableMeasureAction_ = nullptr;
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

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QMeasureDisplay * measureDisplay_ = nullptr;
};

#endif /* __QMeasureDisplay_h__ */
