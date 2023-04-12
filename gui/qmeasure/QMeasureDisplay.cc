/*
 * QMeasureDisplay.cc
 *
 *  Created on: Apr 12, 2023
 *      Author: amyznikov
 */

#include "QMeasureDisplay.h"
#include <gui/widgets/style.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

#define ICON_add        ":/qmeasure/icons/add.png"
#define ICON_clear      ":/qmeasure/icons/clear.png"
#define ICON_copy       ":/qmeasure/icons/copy.png"
#define ICON_save       ":/qmeasure/icons/save.png"
#define ICON_select     ":/qmeasure/icons/select.png"
#define ICON_measure    ":/qmeasure/icons/measure.png"


QMeasureDisplay::QMeasureDisplay(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qmeasure_resources);

  lv_ = new QVBoxLayout(this);
  lv_->setContentsMargins(0,0,0,0);

  lv_->addWidget(toolbar_ = new QToolBar(this));
  lv_->addWidget(table_ = new QTableWidget(this));

  table_->setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

  setupToolbar();
  updateControls();
}

void QMeasureDisplay::setMeasureProvider(QMeasureProvider * provider)
{
  mp_ = provider;
  setupTableView();
  updateControls();
}

QMeasureProvider* QMeasureDisplay::measureProvider() const
{
  return mp_;
}


void QMeasureDisplay::setupToolbar()
{
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setIconSize(QSize(16, 16));

  static const auto addAction =
      [](QToolBar * toolbar, const char * icon, const char * text, const char * tooltip, bool checkable) {
        QAction * action = toolbar->addAction(getIcon(icon), text);
        if ( tooltip ) {
          action->setToolTip(tooltip);
        }
        if ( checkable ) {
          action->setCheckable(checkable);
        }
        return action;
      };


  saveToFileAction_ =
      addAction(toolbar_, ICON_save,
          "Save",
          "Save measurements to file...",
          false);

  copyToClipboardAction_ =
      addAction(toolbar_, ICON_copy,
          "Copy",
          "Copy measurements to clipboard",
          false);

  clearTableAction_ =
      addAction(toolbar_, ICON_clear,
          "Clear",
          "Clear measurements",
          false);

  selectMeasuresAction_  =
      addAction(toolbar_, ICON_select,
          "Select measures",
          "Select measures",
          true);

  incrementalMeasurementsAction_ =
      addAction(toolbar_, ICON_add,
          "Incremental",
          "Add measurements",
          true);

  measureAction_ =
      addAction(toolbar_, ICON_measure,
          "Measure",
          "Measure now",
          false);

  connect(selectMeasuresAction_, &QAction::triggered,
      this, &ThisClass::onSelectMeasuresClicked);


  measureAction_->setEnabled(false);

//  connect(measureAction_, &QAction::triggered,
//      this, &ThisClass::measure);

}


void QMeasureDisplay::onSelectMeasuresClicked(bool checked)
{
  if ( !checked ) {
    if ( measureSelectorDialog_ ) {
      measureSelectorDialog_->hide();
    }
  }
  else {

    if( !measureSelectorDialog_ ) {

      measureSelectorDialog_ = new QMultiMeasureSelectionDialogBox(this);

      connect(measureSelectorDialog_, &QMultiMeasureSelectionDialogBox::visibilityChanged,
          selectMeasuresAction_, &QAction::setChecked);

      connect(measureSelectorDialog_, &QMultiMeasureSelectionDialogBox::selectedMeasuresChanged,
          this, &ThisClass::updateVisibleColumns);
    }

    measureSelectorDialog_->setMeasureProvider(mp_);
    measureSelectorDialog_->show();
    measureSelectorDialog_->raise();
    measureSelectorDialog_->setFocus();
  }
}


void QMeasureDisplay::setupTableView()
{
  if ( !mp_ ) {
    return;
  }

  const std::set<QMeasure*> & measures =
      mp_->measures();

  int i = 0;

  table_->setColumnCount(measures.size());

  for( QMeasure *m : measures ) {
    QTableWidgetItem * item = new QTableWidgetItem(m->name());
    item->setData(Qt::UserRole, QVariant::fromValue(m));
    table_->setHorizontalHeaderItem(i, item);
    ++i;
  }

  table_->setRowCount(1);

  updateVisibleColumns();
}

void QMeasureDisplay::updateVisibleColumns()
{
  if  ( mp_ ) {

    for ( int i = 0, n = table_->columnCount(); i < n; ++i )  {

      QMeasure * m =
          table_->horizontalHeaderItem(i)->data(Qt::UserRole).value<QMeasure*>();

      if ( m ) {
        table_->setColumnHidden(i, !m->enabled());
      }
    }
  }
}

void QMeasureDisplay::onupdatecontrols()
{
  setEnabled(mp_ != nullptr);
}


QMeasureDisplayDialogBox::QMeasureDisplayDialogBox(QWidget * parent) :
    ThisClass("Measures", parent)
{
}

QMeasureDisplayDialogBox::QMeasureDisplayDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * lv =
      new QVBoxLayout(this);

  lv->setContentsMargins(0,0,0,0);

  lv->addWidget(measureDisplay_ =
      new QMeasureDisplay(this));
}

void QMeasureDisplayDialogBox::setMeasureProvider(QMeasureProvider * provider)
{
  return measureDisplay_->setMeasureProvider(provider);
}

QMeasureProvider* QMeasureDisplayDialogBox::measureProvider() const
{
  return measureDisplay_->measureProvider();
}

QMeasureDisplay * QMeasureDisplayDialogBox::measureDisplay() const
{
  return measureDisplay_;
}

void QMeasureDisplayDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QMeasureDisplayDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

