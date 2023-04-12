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
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QMeasureDisplay ThisClass;
  typedef QFrame Base;

  QMeasureDisplay(QWidget * parent = nullptr);

  void setMeasureProvider(QMeasureProvider * provider);
  QMeasureProvider* measureProvider() const;

protected:
  void setupToolbar();
  void setupTableView();
  void onSelectMeasuresClicked(bool checked);

protected:
  void onupdatecontrols() override;
  void updateVisibleColumns();

protected:
  QMeasureProvider* mp_ = nullptr;
  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QTableWidget * table_ = nullptr;
  QMultiMeasureSelectionDialogBox * measureSelectorDialog_ = nullptr;

  QAction * saveToFileAction_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * clearTableAction_ = nullptr;
  QAction * selectMeasuresAction_ = nullptr;
  QAction * incrementalMeasurementsAction_ = nullptr;
  QAction * measureAction_ = nullptr;
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

  void setMeasureProvider(QMeasureProvider * provider);
  QMeasureProvider* measureProvider() const;

  QMeasureDisplay * measureDisplay() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QMeasureDisplay * measureDisplay_ = nullptr;
};

#endif /* __QMeasureDisplay_h__ */
