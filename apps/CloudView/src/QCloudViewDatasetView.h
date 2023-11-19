/*
 * QCloudViewDatasetView.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudViewDatasetView_h__
#define __QCloudViewDatasetView_h__

#include <QtWidgets/QtWidgets>
#include <gui/qcustomdock/QCustomDock.h>
#include <gui/widgets/UpdateControls.h>
#include <gui/widgets/QSettingsWidget.h>
#include "c_cloudview_dataset.h"

namespace cloudview {

enum {
  QCloudViewDatasetTreeIemTypeDataset = QTreeWidgetItem::UserType + 1,
  QCloudViewDatasetTreeIemTypeInputSource = QTreeWidgetItem::UserType + 2,
};



class QCloudViewDatasetTreeDatasetItem:
    public QTreeWidgetItem
{
public:
  typedef QCloudViewDatasetTreeDatasetItem ThisClass;
  typedef QTreeWidgetItem Base;

  QCloudViewDatasetTreeDatasetItem(QTreeWidget * treeview, const c_cloudview_dataset::sptr & dataset);

  const c_cloudview_dataset::sptr& dataset() const;

  void refreshInputSources();

protected:
  c_cloudview_dataset::sptr dataset_;
};



class QCloudViewDatasetTreeInputSourceItem:
    public QTreeWidgetItem
{
public:
  typedef QCloudViewDatasetTreeInputSourceItem ThisClass;
  typedef QTreeWidgetItem Base;

  QCloudViewDatasetTreeInputSourceItem(QTreeWidgetItem * parent, const c_cloudview_input_source::sptr & input_source);
  const c_cloudview_input_source::sptr& input_source() const;

protected:
  c_cloudview_input_source::sptr input_source_;
};








class QCloudViewDatasetTreeView :
    public QTreeWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetTreeView ThisClass;
  typedef QTreeWidget Base;

  QCloudViewDatasetTreeView(QWidget * parent = nullptr);

  QCloudViewDatasetTreeDatasetItem * findDatasetItem(const QString & name) const;

  QList<QTreeWidgetItem*> getContextItems(const QPoint & pos) const;

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;
  void onItemChanged(QTreeWidgetItem *item, int column);

protected:
};










class QCloudViewDatasetView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetView ThisClass;
  typedef QWidget Base;

  QCloudViewDatasetView(QWidget * parent = nullptr);

  void onAddDataset();
  void onAddSources(QCloudViewDatasetTreeDatasetItem * datasetItem);

protected:
  void onCustomContextMenuRequested(const QPoint & pos);

protected:
  QToolBar * toolbar_ = nullptr;
  QCloudViewDatasetTreeView * treeView_ = nullptr;
};









class QCloudViewDatasetViewDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetViewDock ThisClass;
  typedef QCustomDockWidget Base;

  QCloudViewDatasetViewDock(const QString &title, QWidget * parent = nullptr);

  QCloudViewDatasetView * datasetView() const;

protected:
  QCloudViewDatasetView * datasetView_ = nullptr;
};


QCloudViewDatasetViewDock * addCloudViewDatasetViewDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu);















class QAddCloudViewDatasetDialogBox :
    public QDialog
{
public:
  typedef QAddCloudViewDatasetDialogBox ThisClass;
  typedef QDialog Base;

  QAddCloudViewDatasetDialogBox(QWidget * parent);

  QString selectedDatasetType() const;
  QString selectedDatasetName() const;

protected:
  void populateDatasetTypesCombo();
  void onDatasetTypeComboCurrentIndexChanged(int);

protected:
  QLineEditBox * datasetName_ctl = nullptr;
  QComboBox * datasetType_ctl = nullptr;
  QSettingsWidget * settings_ctl = nullptr;

  QLabel * datasetTypeTooltip_ctl = nullptr;
  QPushButton * addButton_ctl = nullptr;
  QPushButton * cancelButton_ctl = nullptr;
};



} /* namespace cloudview */

#endif /* __QCloudViewDatasetView_h__ */
