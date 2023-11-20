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



class QCloudViewDatasetItem:
    public QTreeWidgetItem
{
public:
  typedef QCloudViewDatasetItem ThisClass;
  typedef QTreeWidgetItem Base;

  QCloudViewDatasetItem(QTreeWidget * treeview, const c_cloudview_dataset::sptr & dataset);

  const c_cloudview_dataset::sptr& dataset() const;

  void refreshInputSources();

protected:
  c_cloudview_dataset::sptr dataset_;
};



class QCloudViewInputSourceItem:
    public QTreeWidgetItem
{
public:
  typedef QCloudViewInputSourceItem ThisClass;
  typedef QTreeWidgetItem Base;

  QCloudViewInputSourceItem(QTreeWidgetItem * parent, const c_cloudview_input_source::sptr & input_source);
  const c_cloudview_input_source::sptr& input_source() const;

protected:
  c_cloudview_input_source::sptr input_source_;
};








class QCloudViewDatasetCollectionsTree :
    public QTreeWidget,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetCollectionsTree ThisClass;
  typedef QTreeWidget Base;

  QCloudViewDatasetCollectionsTree(QWidget * parent = nullptr);

  QCloudViewDatasetItem * findDatasetItem(const QString & name) const;

  QList<QTreeWidgetItem*> getContextItems(const QPoint & pos) const;

protected:
  bool eventFilter(QObject *watched, QEvent *event) override;
  void onItemChanged(QTreeWidgetItem *item, int column);
  void onItemDoubleClicked(QTreeWidgetItem *item, int column);

protected:
};










class QCloudViewDatasetCollectionsView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetCollectionsView ThisClass;
  typedef QWidget Base;

  QCloudViewDatasetCollectionsView(QWidget * parent = nullptr);

  void onAddDataset();
  void onAddSources(QCloudViewDatasetItem * datasetItem);

protected:
  void onCustomContextMenuRequested(const QPoint & pos);

protected:
  QToolBar * toolbar_ = nullptr;
  QCloudViewDatasetCollectionsTree * treeView_ = nullptr;
};









class QCloudViewDatasetCollectionsDock :
    public QCustomDockWidget
{
  Q_OBJECT;
public:
  typedef QCloudViewDatasetCollectionsDock ThisClass;
  typedef QCustomDockWidget Base;

  QCloudViewDatasetCollectionsDock(const QString &title, QWidget * parent = nullptr);

  QCloudViewDatasetCollectionsView * datasetView() const;

protected:
  QCloudViewDatasetCollectionsView * datasetView_ = nullptr;
};


QCloudViewDatasetCollectionsDock * addCloudViewDatasetCollectionsDock(QMainWindow * parent,
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
