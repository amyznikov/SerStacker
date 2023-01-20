/*
 * QImageStatistics.h
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QImageStatistics_h__
#define __QImageStatistics_h__

#include <QtWidgets/QtWidgets>
#include <opencv2/opencv.hpp>



class QImageStatisticsDisplay :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QImageStatisticsDisplay ThisClass;
  typedef QWidget Base;

  QImageStatisticsDisplay(QWidget * parent = nullptr);

  void loadParameters();
  void saveParameters();

protected:
  void setupToolbar();
  void setupTableView();

protected:
  QVBoxLayout * lv_ = nullptr;
  QToolBar * toolbar_ = nullptr;
  QTableWidget * table_ = nullptr;

  QAction * saveToFileAction_ = nullptr;
  QAction * copyToClipboardAction_ = nullptr;
  QAction * clearTableAction_ = nullptr;
  QAction * incrementalMeasurementsAction_ = nullptr;
  QAction * selectMeasurementsAction_ = nullptr;
};

class QImageStatisticsDisplayDialogBox:
    public QDialog
{
  Q_OBJECT;
public:
  typedef QImageStatisticsDisplayDialogBox ThisClass;
  typedef QDialog Base;

  QImageStatisticsDisplayDialogBox(QWidget * parent = nullptr);
  QImageStatisticsDisplayDialogBox(const QString & title, QWidget * parent = nullptr);

  QImageStatisticsDisplay * display() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void showEvent(QShowEvent * e) override;
  void hideEvent(QHideEvent * e) override;

protected:
  QImageStatisticsDisplay * display_ = nullptr;
};

#endif /* __QImageStatistics_h__ */
