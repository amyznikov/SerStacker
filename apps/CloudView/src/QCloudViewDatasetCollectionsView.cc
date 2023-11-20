/*
 * QCloudViewDatasetView.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "QCloudViewDatasetCollectionsView.h"

#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>

#define ICON_add_dataset ":/cloudview/icons/add-dataset.png"


namespace cloudview {

///////////////////////////////////////////////////////////////////////////////////////////////////


static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewDatasetItem::QCloudViewDatasetItem(QTreeWidget * treeview,
    const c_cloudview_dataset::sptr & dataset) :
    Base(treeview, (int) (QCloudViewDatasetTreeIemTypeDataset)),
    dataset_(dataset)
{
  setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
  setText(0, dataset->name().c_str());

  setCheckState(0, Qt::Checked);
  refreshInputSources();
}

const c_cloudview_dataset::sptr& QCloudViewDatasetItem::dataset() const
{
  return dataset_;
}

void QCloudViewDatasetItem::refreshInputSources()
{
  QTreeWidgetItem *childItem;

  while ((childItem = takeChild(0))) {
    delete childItem;
  }

  if( dataset_ ) {
    for( const c_cloudview_input_source::sptr & input_source : dataset_->input_sources() ) {
      new QCloudViewInputSourceItem(this, input_source);
    }
  }
}


QCloudViewInputSourceItem::QCloudViewInputSourceItem(QTreeWidgetItem * parent,
    const c_cloudview_input_source::sptr & input_source) :
    Base(parent, (int) (QCloudViewDatasetTreeIemTypeInputSource)),
    input_source_(input_source)
{

  setFlags(flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
  setText(0, input_source_->filename().c_str());

  setCheckState(0, Qt::Checked);
}

const c_cloudview_input_source::sptr& QCloudViewInputSourceItem::input_source() const
{
  return input_source_;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewDatasetCollectionsTree::QCloudViewDatasetCollectionsTree(QWidget * parent) :
    Base(parent)
{
  setHeaderHidden(true);
  //setRootIsDecorated(false);
  setSelectionMode(QAbstractItemView::ExtendedSelection);
  setSortingEnabled(false);
  setContextMenuPolicy(Qt::CustomContextMenu);

  viewport()->setAcceptDrops(true);
  setDefaultDropAction(Qt::CopyAction);
  setDragDropMode(QAbstractItemView::DropOnly);
  setDropIndicatorShown(true);

  setEditTriggers(QAbstractItemView::EditKeyPressed /*| QAbstractItemView::SelectedClicked*/);

  setExpandsOnDoubleClick(false);

  installEventFilter(this);


  //  connect(this, &QTreeWidget::customContextMenuRequested,
  //      this, &ThisClass::onCustomContextMenuRequested);
  //
  connect(this, &QTreeWidget::itemChanged,
      this, &ThisClass::onItemChanged);
  //
  //  connect(this, &QTreeWidget::currentItemChanged,
  //      this, &ThisClass::onCurrentItemChanged);
  //
    connect(this, &QTreeWidget::itemDoubleClicked,
        this, &ThisClass::onItemDoubleClicked);
}



bool QCloudViewDatasetCollectionsTree::eventFilter(QObject * watched, QEvent * event)
{
  if( watched == this && event->type() == QEvent::KeyPress ) {

    const QKeyEvent *e =
        (const QKeyEvent*) event;

    if( e->key() == Qt::Key_Return && state() != QAbstractItemView::EditingState ) {

      QTreeWidgetItem *item =
          currentItem();

      if( item ) {
        Q_EMIT itemDoubleClicked(item, 0);
        return true;
      }
    }
  }

  return false;
}



void QCloudViewDatasetCollectionsTree::onItemChanged(QTreeWidgetItem *item, int column)
{
  if ( !item || updatingControls() ) {
    return;
  }

  if( QCloudViewDatasetItem *datasetItem =
      dynamic_cast<QCloudViewDatasetItem*>(item) ) {

    if ( column == 0 ) {

      const QString itemName =
          datasetItem->text(0);

      if ( itemName.isEmpty() ) {

        c_update_controls_lock lock(this);

        datasetItem->setText(0, datasetItem->dataset()->cname());
      }
      else {

        const std::string cname =
            itemName.toStdString();

        if ( cname != datasetItem->dataset()->name() ) {
          datasetItem->dataset()->set_name(cname);
        }
      }
    }
  }
  else if( QCloudViewInputSourceItem *sourceItem =
      dynamic_cast<QCloudViewInputSourceItem *>(item) ) {

  }
}

void QCloudViewDatasetCollectionsTree::onItemDoubleClicked(QTreeWidgetItem *item, int column)
{
  if( QCloudViewInputSourceItem *sourceItem =
      dynamic_cast<QCloudViewInputSourceItem*>(item) ) {

  }

}


QCloudViewDatasetItem  * QCloudViewDatasetCollectionsTree::findDatasetItem(const QString & name) const
{
  if( !name.isEmpty() ) {

    const std::string cname =
        name.toStdString();

    for( int i = 0, n = topLevelItemCount(); i < n; ++i ) {

      QCloudViewDatasetItem *item =
          dynamic_cast<QCloudViewDatasetItem*>(topLevelItem(i));

      if( item && item->dataset()->name() == cname ) {
        return item;
      }
    }
  }

  return nullptr;
}

QList<QTreeWidgetItem*> QCloudViewDatasetCollectionsTree::getContextItems(const QPoint & pos) const
{
  QList<QTreeWidgetItem*> contextItems;

  QModelIndexList selectedIndexes =
      selectionModel()->selectedRows();

  if( !selectedIndexes.empty() ) {
    for( int i = 0, n = selectedIndexes.size(); i < n; ++i ) {
      if( selectedIndexes[i].isValid() ) {
        contextItems.append(itemFromIndex(selectedIndexes[i]));
      }
    }
  }
  else {
    QModelIndex index = indexAt(pos);
    if( index.isValid() ) {
      contextItems.append(itemFromIndex(index));
    }
  }

  return contextItems;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewDatasetCollectionsView::QCloudViewDatasetCollectionsView(QWidget * parent) :
    Base(parent)
{

  QVBoxLayout * layout =
      new QVBoxLayout(this);

  layout->addWidget(toolbar_ =
      createToolbar(this), 0,
      Qt::AlignTop);

  layout->addWidget(treeView_ =
      new QCloudViewDatasetCollectionsTree(this));


  toolbar_->addWidget(createToolButton(getIcon(ICON_add_dataset),
      "Add dataset",
      "Add new cloudview dataset",
      this, &ThisClass::onAddDataset));

  connect(treeView_, &QTreeWidget::customContextMenuRequested,
      this, &ThisClass::onCustomContextMenuRequested);
}


void QCloudViewDatasetCollectionsView::onCustomContextMenuRequested(const QPoint & pos)
{
  QMenu menu;
  QAction *action;

  const QList<QTreeWidgetItem*> contextItems =
      treeView_->getContextItems(pos);

  if( contextItems.size() == 1 ) {

    QTreeWidgetItem *item =
        contextItems[0];

    menu.addAction("Copy Name",
        [this, item]() {
          QApplication::clipboard()->setText(item->text(0));
        });

    if( QCloudViewDatasetItem *datasetItem =
        dynamic_cast<QCloudViewDatasetItem*>(item) ) {

      menu.addAction("Rename...",
          [this, datasetItem]() {
            treeView_->editItem(datasetItem);
          });

      menu.addAction("Add sources...",
          [this, datasetItem]() {
            onAddSources(datasetItem);
          });
    }

    else if( QCloudViewInputSourceItem *sourceItem =
        dynamic_cast<QCloudViewInputSourceItem *>(item) ) {

      menu.addAction("Copy full path name",
          [this, sourceItem]() {
            QApplication::clipboard()->setText(sourceItem->input_source()->cfilename());
          });
    }
  }

//  if( contextItems.size() > 0 ) {
//    menu.addAction(deleteItemAction);
//  }

  if( !menu.isEmpty() ) {
    menu.exec(treeView_->mapToGlobal(pos));
  }

}

void QCloudViewDatasetCollectionsView::onAddDataset()
{
  QAddCloudViewDatasetDialogBox dialogBox(this);

  if ( !dialogBox.exec() == QDialog::Accepted ) {
    return;
  }

  const QString selectedDatasetType =
      dialogBox.selectedDatasetType();

  QString selectedDatasetName =
      dialogBox.selectedDatasetName();

  if ( selectedDatasetType.isEmpty() ) {
    return;
  }

  if( selectedDatasetName.isEmpty() ) {

    for ( int i = 0; i < 1000; ++i ) {

      selectedDatasetName =
          QString("%1%2").arg(selectedDatasetType).arg(i);

      if( !treeView_->findDatasetItem(selectedDatasetName) ) {
        break;
      }

    }
  }

  c_cloudview_dataset::sptr dataset =
      c_cloudview_dataset::create(selectedDatasetType.toStdString(),
          selectedDatasetName.toStdString());

  if ( !dataset ) {
    QMessageBox::critical(this, "ERROR",
        "c_cloudview_dataset::create() fails");
    return;
  }

  QCloudViewDatasetItem * datasetItem =
      new QCloudViewDatasetItem(treeView_,
          dataset);

  treeView_->addTopLevelItem(datasetItem);

}

void QCloudViewDatasetCollectionsView::onAddSources(QCloudViewDatasetItem * datasetItem)
{
  if ( !datasetItem ) {
    return;
  }

  const c_cloudview_dataset::sptr & dataset =
      datasetItem->dataset();

  if ( !dataset ) {
    return;
  }


  static const QString savedPathKeyName =
      "lastQCloudViewDatasetAddSourcesPath";

  QSettings settings;

  QString savedPathFileName =
      settings.value(savedPathKeyName).toString();


  const QString filter =
      dataset->file_filter().c_str();

  QStringList selectedFileNames =
      QFileDialog::getOpenFileNames(this,
          "Add sources to dataset",
          savedPathFileName,
          filter,
          nullptr,
          QFileDialog::ReadOnly);


  if ( selectedFileNames.isEmpty() ) {
    return;
  }

  settings.setValue(savedPathKeyName,
      selectedFileNames[0]);

  bool hasChanges = false;

  for( int i = 0, n = selectedFileNames.size(); i < n; ++i ) {

    if( dataset->add_input_source(selectedFileNames[i].toStdString()) ) {
      hasChanges = true;
    }
    else {

      if( i == n - 1 ) {
        QMessageBox::critical(this, "ERROR", QString("Can not load %1.\n"
            "See error log for details.").arg(selectedFileNames[i]));
        break;
      }

      const int responce =
          QMessageBox::critical(this, "ERROR", QString("Can not load %1.\n"
              "See error log for details.\n"
              "Continue loading ?").arg(selectedFileNames[i]),
              QMessageBox::Yes | QMessageBox::No);

      if( responce != QMessageBox::Yes ) {
        break;
      }
    }
  }

  if ( hasChanges ) {
    datasetItem->refreshInputSources();
    treeView_->expandItem(datasetItem);
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////



QCloudViewDatasetCollectionsDock::QCloudViewDatasetCollectionsDock(const QString & title, QWidget * parent) :
    Base(title, parent)
{
  Base::setWidget(datasetView_ = new QCloudViewDatasetCollectionsView(this));

//  const QList<QAction*> actions = datasetView_->toolbarActions();
//  for( int i = 0, n = actions.size(); i < n; ++i ) {
//    titleBar()->addButton(actions[n - i - 1]);
//  }

}

QCloudViewDatasetCollectionsView* QCloudViewDatasetCollectionsDock::datasetView() const
{
  return datasetView_;
}

QCloudViewDatasetCollectionsDock* addCloudViewDatasetCollectionsDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QCloudViewDatasetCollectionsDock *dock =
      new QCloudViewDatasetCollectionsDock(title, parent);

  dock->setObjectName(dockName);
  parent->addDockWidget(area, dock);

  if( viewMenu ) {
    viewMenu->addAction(dock->toggleViewAction());
  }

  return dock;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QAddCloudViewDatasetDialogBox::QAddCloudViewDatasetDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Add CloudView Dataset...");

  QHBoxLayout * h0 =
      new QHBoxLayout(this);

  QVBoxLayout * vl =
      new QVBoxLayout();

  vl->setAlignment(Qt::AlignTop);

  vl->addWidget(settings_ctl =
      new QSettingsWidget("", this));

  settings_ctl->addRow("Dataset Name:", datasetName_ctl =
      new QLineEditBox(this));

  settings_ctl->addRow("Dataset Type: ", datasetType_ctl =
      new QComboBox(this));

  datasetType_ctl->setEditable(false);
  datasetType_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);

  vl->addWidget(datasetTypeTooltip_ctl =
      new QLabel(this));


  QVBoxLayout * vr =
      new QVBoxLayout();

  vr->setAlignment(Qt::AlignTop);

  vr->addWidget(addButton_ctl =
      new QPushButton("Add", this), 0,
      Qt::AlignTop);

  vr->addWidget(cancelButton_ctl =
      new QPushButton("Cancel", this), 0,
      Qt::AlignTop);

  addButton_ctl->setDefault(true);

  ///

  h0->addLayout(vl);
  h0->addLayout(vr);

  ///

  connect(datasetType_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onDatasetTypeComboCurrentIndexChanged);

  connect(addButton_ctl, &QPushButton::clicked,
      this, &ThisClass::accept);

  connect(cancelButton_ctl, &QPushButton::clicked,
      this, &ThisClass::reject);

  ///

  populateDatasetTypesCombo();
}

QString QAddCloudViewDatasetDialogBox::selectedDatasetType() const
{
  return datasetType_ctl->currentText();
}

QString QAddCloudViewDatasetDialogBox::selectedDatasetName() const
{
  return datasetName_ctl->text();
}


void QAddCloudViewDatasetDialogBox::populateDatasetTypesCombo()
{
  datasetType_ctl->clear();

  for ( const auto & supported_type : c_cloudview_dataset::supported_types() ) {

    datasetType_ctl->addItem(supported_type.name().c_str(),
        QVariant::fromValue(QString(supported_type.tooltip().c_str())));
  }

  datasetType_ctl->setCurrentIndex(0);

}

void QAddCloudViewDatasetDialogBox::onDatasetTypeComboCurrentIndexChanged(int)
{
  datasetTypeTooltip_ctl->setText(datasetType_ctl->currentData().toString());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
} /* namespace cloudview */
