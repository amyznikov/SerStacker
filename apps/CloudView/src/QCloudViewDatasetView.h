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


class QCloudViewDatasetTreeItem :
  public QTreeWidgetItem
{
public:
  typedef QCloudViewDatasetTreeItem ThisClass;
  typedef QTreeWidgetItem Base;

  QCloudViewDatasetTreeItem(QTreeWidget * treeview, int type) :
    Base(treeview, type)
  {
  }
};


class QCloudViewDatasetTreeDatasetItem :
  public QCloudViewDatasetTreeItem
{
public:
  typedef QCloudViewDatasetTreeDatasetItem ThisClass;
  typedef QCloudViewDatasetTreeItem Base;

  QCloudViewDatasetTreeDatasetItem(QTreeWidget * treeview, const c_cloudview_dataset::sptr & dataset) :
    Base(treeview, (int)(QCloudViewDatasetTreeIemTypeDataset)),
    dataset_(dataset)
  {
  }

  const c_cloudview_dataset::sptr & dataset() const
  {
    return dataset_;
  }

protected:
  c_cloudview_dataset::sptr dataset_;
};

class QCloudViewDatasetTreeInputSourceItem :
  public QCloudViewDatasetTreeItem
{
public:
  typedef QCloudViewDatasetTreeInputSourceItem ThisClass;
  typedef QCloudViewDatasetTreeItem Base;

  QCloudViewDatasetTreeInputSourceItem(QTreeWidget * treeview, c_cloudview_input_source::sptr & input_source) :
    Base(treeview, (int)(QCloudViewDatasetTreeIemTypeInputSource)),
    input_source_(input_source)
  {
  }

  const c_cloudview_input_source::sptr & input_source() const
  {
    return input_source_;
  }

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

protected:
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
