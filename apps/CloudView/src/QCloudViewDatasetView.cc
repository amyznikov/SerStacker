/*
 * QCloudViewDatasetView.cc
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#include "QCloudViewDatasetView.h"
#include <gui/widgets/createAction.h>
#include <gui/widgets/style.h>

#define ICON_add_dataset ":/cloudview/icons/add-dataset.png"


namespace cloudview {

static QToolBar * createToolbar(QWidget * parent = nullptr)
{
  QToolBar * toolbar = new QToolBar(parent);
  toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar->setIconSize(QSize(16, 16));
  return toolbar;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewDatasetTreeView::QCloudViewDatasetTreeView(QWidget * parent) :
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

  //  connect(this, &QTreeWidget::customContextMenuRequested,
  //      this, &ThisClass::onCustomContextMenuRequested);
  //
  connect(this, &QTreeWidget::itemChanged,
      this, &ThisClass::onItemChanged);
  //
  //  connect(this, &QTreeWidget::currentItemChanged,
  //      this, &ThisClass::onCurrentItemChanged);
  //
  //  connect(this, &QTreeWidget::itemDoubleClicked,
  //      this, &ThisClass::onItemDoubleClicked);
}


void QCloudViewDatasetTreeView::onItemChanged(QTreeWidgetItem *item, int column)
{
  if ( updatingControls() ) {
    return;
  }

  return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QCloudViewDatasetView::QCloudViewDatasetView(QWidget * parent) :
    Base(parent)
{

  QVBoxLayout * layout =
      new QVBoxLayout(this);

  layout->addWidget(toolbar_ =
      createToolbar(this), 0,
      Qt::AlignTop);

  layout->addWidget(treeView_ =
      new QCloudViewDatasetTreeView(this));


  toolbar_->addWidget(createToolButton(getIcon(ICON_add_dataset),
      "Add dataset",
      "Add new cloudview dataset",
      this, &ThisClass::onAddDataset));

}

void QCloudViewDatasetView::onAddDataset()
{
  QAddCloudViewDatasetDialogBox dialogBox(this);

  dialogBox.exec();

}

///////////////////////////////////////////////////////////////////////////////////////////////////



QCloudViewDatasetViewDock::QCloudViewDatasetViewDock(const QString & title, QWidget * parent) :
    Base(title, parent)
{
  Base::setWidget(datasetView_ = new QCloudViewDatasetView(this));

//  const QList<QAction*> actions = datasetView_->toolbarActions();
//  for( int i = 0, n = actions.size(); i < n; ++i ) {
//    titleBar()->addButton(actions[n - i - 1]);
//  }

}

QCloudViewDatasetView* QCloudViewDatasetViewDock::datasetView() const
{
  return datasetView_;
}

QCloudViewDatasetViewDock* addCloudViewDatasetViewDock(QMainWindow * parent,
    Qt::DockWidgetArea area,
    const QString & dockName,
    const QString & title,
    QMenu * viewMenu)
{
  QCloudViewDatasetViewDock *dock =
      new QCloudViewDatasetViewDock(title, parent);

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

void QAddCloudViewDatasetDialogBox::populateDatasetTypesCombo()
{
  datasetType_ctl->clear();

  for ( const auto & supported_type : c_cloudview_dataset::supported_types() ) {

    datasetType_ctl->addItem(supported_type.name.c_str(),
        QVariant::fromValue(QString(supported_type.tooltip.c_str())));
  }

  datasetType_ctl->setCurrentIndex(0);

}

void QAddCloudViewDatasetDialogBox::onDatasetTypeComboCurrentIndexChanged(int)
{
  datasetTypeTooltip_ctl->setText(datasetType_ctl->currentData().toString());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
} /* namespace cloudview */
