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
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);

  setupToolbar();
  setupTableView();
}

void QMeasureDisplay::updateEnableMeasurements()
{
  const bool enable =
      !cm_.empty() &&
      enableMeasureAction_->isChecked() &&
          this->isVisible();

  if( enable ) {
    QMeasureProvider::request_measures(&cm_);
    connect(QMeasureProvider::instance(), &QMeasureProvider::measurementsChanged,
        this, &ThisClass::updateMeasurements);
  }
  else {
    QMeasureProvider::instance()->disconnect(this);
    QMeasureProvider::remove_measure_request(&cm_);
  }

}

void QMeasureDisplay::clearMeasurements()
{
  table_->setRowCount(0);
}


void QMeasureDisplay::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  updateEnableMeasurements();
}

void QMeasureDisplay::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  updateEnableMeasurements();
}

void QMeasureDisplay::closeEvent(QCloseEvent * e)
{
  QMeasureProvider::remove_measure_request(&cm_);
  Base::closeEvent(e);
}

void QMeasureDisplay::setupToolbar()
{
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
  toolbar_->setIconSize(QSize(16, 16));

  static const auto addAction =
      [](QToolBar * toolbar, const char * icon, const char * text, const char * tooltip, bool checkable) {
        QAction * action = toolbar->addAction(getIcon(icon), text);
        action->setToolTip(tooltip);
        action->setCheckable(checkable);
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

  incrementalModeAction_ =
      addAction(toolbar_, ICON_add,
          "Incremental",
          "Incremental mode",
          true);

  enableMeasureAction_ =
      addAction(toolbar_, ICON_measure,
          "Measure",
          "Enable Measurements",
          true);

  connect(selectMeasuresAction_, &QAction::triggered,
      this, &ThisClass::onSelectMeasuresClicked);

  connect(enableMeasureAction_, &QAction::triggered,
      this, &ThisClass::updateEnableMeasurements);

  connect(clearTableAction_, &QAction::triggered,
      this, &ThisClass::clearMeasurements);


  enableMeasureAction_->setChecked(true);
  updateEnableMeasurements();
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

      measureSelectorDialog_->selectMeasures(&cm_);
    }

    measureSelectorDialog_->selectMeasures(&cm_);
    measureSelectorDialog_->show();
    measureSelectorDialog_->raise();
    measureSelectorDialog_->setFocus();
  }
}


void QMeasureDisplay::setupTableView()
{
  const auto & measures =
      QMeasureProvider::measures();

  int i = 0;

  table_->setColumnCount(measures.size());

  for( QMeasure *m : measures ) {
    QTableWidgetItem * item = new QTableWidgetItem(m->name());
    item->setData(Qt::UserRole, QVariant::fromValue(m));
    table_->setHorizontalHeaderItem(i, item);
    ++i;
  }

  table_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

  connect(table_, &QTableWidget::currentCellChanged,
      this, &ThisClass::onTableViewCurrentCellChanged);

  connect(table_, &QTableWidget::customContextMenuRequested,
      this, &ThisClass::onTableViewContextMenuRequested);

  table_->setRowCount(1);

  updateVisibleColumns();
}

void QMeasureDisplay::updateVisibleColumns()
{
  for( int i = 0, n = table_->columnCount(); i < n; ++i ) {

    QMeasure *m =
        table_->horizontalHeaderItem(i)->data(Qt::UserRole).
            value<QMeasure*>();

    table_->setColumnHidden(i,
        cm_.find(m) == cm_.end());
  }

  updateEnableMeasurements();
}

void QMeasureDisplay::updateMeasurements()
{
  if ( !cm_.empty() ) {

    const std::deque<QMeasureProvider::MeasuredFrame> &measured_frames =
        QMeasureProvider::measured_frames();

    if ( measured_frames.empty() ) {
      table_->setRowCount(0);
    }
    else {
      using MeasuredValue = QMeasureProvider::MeasuredValue;
      using MeasuredFrame = QMeasureProvider::MeasuredFrame;

      static const auto find_measurement =
          [](const QMeasure * measure, const MeasuredFrame & frame) -> const MeasuredValue* {
            for ( const MeasuredValue & v : frame.measurements ) {
              if ( v.measure == measure ) {
                return &v;
              }
            }
            return nullptr;
          };

      const QMeasureProvider::MeasuredFrame & frame =
          measured_frames.back();

      const bool incremental_mode =
          incrementalModeAction_->isChecked();

      const int rc =
          table_->rowCount();

      bool row_inserted =
          false;

      for ( int i = 0, n = table_->columnCount(); i < n; ++i )  {
        if ( !table_->isColumnHidden(i) ) {

          const QMeasure *m =
              table_->horizontalHeaderItem(i)->data(Qt::UserRole).
                  value<QMeasure*>();

          if ( m ) {

            const MeasuredValue * v =
                find_measurement(m, frame);

            if( v && v->cn > 0 ) {

              QString text =
                  qsprintf("%+g", v->value(0));

              for( int c = 1; c < v->cn; ++c ) {
                text += qsprintf("; +%g", v->value(c));
              }

              if( incremental_mode ) {

                if( !row_inserted ) {
                  table_->insertRow(rc);
                  row_inserted = true;
                }

                table_->setItem(rc, i,
                    new QTableWidgetItem(text));
              }
              else {

                if( rc != 1 ) {
                  table_->setRowCount(1);
                }

                QTableWidgetItem *item =
                    table_->item(0, i);

                if( item ) {
                  item->setText(text);
                }
                else {
                  table_->setItem(0, i,
                      new QTableWidgetItem(text));
                }
              }
            }
          }
        }
      }

      if ( row_inserted && table_->rowCount() > QMeasureProvider::max_measured_frames() ) {
        table_->removeRow(0);
      }

      table_->scrollToBottom();
    }
  }

}

void QMeasureDisplay::onTableViewCurrentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn)
{
}

void QMeasureDisplay::onTableViewContextMenuRequested(const QPoint &pos)
{
  QMenu menu;

  menu.addAction("Select all",
      [this]() {
        table_->selectAll();
      });

  if( table_->selectionModel()->hasSelection() ) {

    menu.addAction("Delete selected rows",
        [this]() {
          QWaitCursor wait(this);

          QItemSelectionModel * model = table_->selectionModel();
          const QModelIndex parent = QModelIndex();
          for ( int i = table_->rowCount()-1; i >= 0; --i ) {
            if ( model->isRowSelected(i, parent) ) {
              table_->removeRow(i);
            }
          }
        });

    menu.addAction("Copy to clipboard",
        [this]() {

          QWaitCursor wait(this);

          std::vector<int> visible_columns;

          for ( int j = 0, nj = table_->columnCount(); j < nj; ++j ) {
            if ( !table_->isColumnHidden(j) ) {
              visible_columns.emplace_back(j);
            }
          }

          if ( !visible_columns.empty() ) {

            QString text;

            QItemSelectionModel * model = table_->selectionModel();
            const QModelIndex parent = QModelIndex();
            for ( int i = 0, ni = table_->rowCount(); i < ni; ++i ) {
              if ( model->isRowSelected(i, parent) ) {

                for ( int j = 0, nj = visible_columns.size(); j < nj; ++j ) {

                  const QTableWidgetItem * item =
                  table_->item(i, visible_columns[j]);

                  if ( item ) {
                    text.append(item->text());
                  }

                  if ( j < nj - 1 ) {
                    text.append("\t");
                  }
                }

                text.append("\n");
              }
            }

            QApplication::clipboard()->setText(text);
          }
        });

  }


  if ( !menu.isEmpty() ) {
    menu.exec(table_->viewport()->mapToGlobal(pos));
  }
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

