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
#define ICON_clear      ":/qmeasure/icons/clear_measurements.png"
#define ICON_copy       ":/qmeasure/icons/copy.png"
#define ICON_save       ":/qmeasure/icons/save.png"
#define ICON_select     ":/qmeasure/icons/select.png"
#define ICON_measure    ":/qmeasure/icons/measure.png"


QMeasureDisplay::QMeasureDisplay(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qmeasure_resources);

  _lv = new QVBoxLayout(this);
  _lv->setContentsMargins(0,0,0,0);

  _lv->addWidget(_toolbar = new QToolBar(this));
  _lv->addWidget(_table = new QTableWidget(this));

  _table->setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  _table->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  _table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  _table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  _table->horizontalHeader()->setVisible(true);
  _table->verticalHeader()->setVisible(true);

  _table->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);
  _table->setRowCount(1);

  connect(_table, &QTableWidget::currentCellChanged,
      this, &ThisClass::onTableViewCurrentCellChanged);

  connect(_table, &QTableWidget::customContextMenuRequested,
      this, &ThisClass::onTableViewContextMenuRequested);


  setupToolbar();
  updateVisibleColumns();
}

void QMeasureDisplay::onEnableMeasurementsClicked(bool checked)
{
  updateEnableMeasurements();
  if ( _measurementsEnabled ) {
    Q_EMIT measureRightNowRequested();
  }
}

void QMeasureDisplay::updateEnableMeasurements()
{
  const bool enable =
      !_cm.empty() &&
          _enableMeasureAction->isChecked() &&
          this->isVisible();

  if (enable != _measurementsEnabled) {
    if (!(_measurementsEnabled = enable)) {
      QMeasureProvider::instance()->disconnect(this);
    }
    else {
      connect(QMeasureProvider::instance(), &QMeasureProvider::framesMeasured,
          this, &ThisClass::onFramesMeasured);
    }
  }

  if ( _measurementsEnabled ) {
    QMeasureProvider::request_measures(&_cm);
  }
  else {
    QMeasureProvider::remove_measure_request(&_cm);
  }
}

void QMeasureDisplay::clearMeasurements()
{
  _table->setRowCount(0);
  _table->setColumnCount(0);
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
  QMeasureProvider::remove_measure_request(&_cm);
  Base::closeEvent(e);
}

void QMeasureDisplay::setupToolbar()
{
  _toolbar->setContentsMargins(0, 0, 0, 0);
  _toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
  _toolbar->setIconSize(QSize(16, 16));

  static const auto addAction =
      [](QToolBar * toolbar, const char * icon, const char * text, const char * tooltip, bool checkable) {
        QAction * action = toolbar->addAction(getIcon(icon), text);
        action->setToolTip(tooltip);
        action->setCheckable(checkable);
        return action;
      };


  static const auto addStretch =
      [](QToolBar * toolbar) -> QWidget* {
        QWidget *stretch = new QWidget();
        stretch->setStyleSheet("*{background: none;}");
        stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        toolbar->addWidget(stretch);
        return stretch;
      };



  _saveToFileAction =
      addAction(_toolbar, ICON_save,
          "Save",
          "Save measurements to file...",
          false);

  _copyToClipboardAction =
      addAction(_toolbar, ICON_copy,
          "Copy",
          "Copy measurements to clipboard",
          false);

  _toolbar->addSeparator();

  _selectMeasuresAction  =
      addAction(_toolbar, ICON_select,
          "Select measures",
          "Select measures",
          true);

  _incrementalModeAction =
      addAction(_toolbar, ICON_add,
          "Incremental",
          "Incremental mode",
          true);

  _enableMeasureAction =
      addAction(_toolbar, ICON_measure,
          "Measure",
          "Enable Measurements",
          true);

  addStretch(_toolbar);

  _clearTableAction =
      addAction(_toolbar, ICON_clear,
          "Clear",
          "Clear measurements",
          false);

  connect(_selectMeasuresAction, &QAction::triggered,
      this, &ThisClass::onSelectMeasuresClicked);

  connect(_enableMeasureAction, &QAction::triggered,
      this, &ThisClass::onEnableMeasurementsClicked);

  connect(_clearTableAction, &QAction::triggered,
      this, &ThisClass::clearMeasurements);


  _enableMeasureAction->setChecked(true);
  updateEnableMeasurements();
}


void QMeasureDisplay::onSelectMeasuresClicked(bool checked)
{
  if ( !checked ) {
    if ( _measureSelectorDialogBox ) {
      _measureSelectorDialogBox->hide();
    }
  }
  else {

    if( !_measureSelectorDialogBox ) {

      _measureSelectorDialogBox = new QMultiMeasureSelectionDialogBox(this);

      connect(_measureSelectorDialogBox, &QMultiMeasureSelectionDialogBox::visibilityChanged,
          [this](bool visible) {
            _selectMeasuresAction->setChecked(visible);
            if ( visible ) {
              Q_EMIT updateAvailableMeasureDataChannelsRequired();
            }
          });

      connect(_measureSelectorDialogBox, &QMultiMeasureSelectionDialogBox::selectedMeasuresChanged,
          this, &ThisClass::updateVisibleColumns);

      _measureSelectorDialogBox->selectMeasures(&_cm);
    }

    _measureSelectorDialogBox->selectMeasures(&_cm);
    _measureSelectorDialogBox->show();
    _measureSelectorDialogBox->raise();
    _measureSelectorDialogBox->setFocus();
  }
}


void QMeasureDisplay::updateVisibleColumns()
{
  updateEnableMeasurements();
}

void QMeasureDisplay::onFramesMeasured(const QList<QMeasureProvider::MeasuredFrame> &frames)
{
  static const auto findColumn =
      [](QTableWidget *table, const QString &name) {
        for ( int i = 0, n = table->columnCount(); i < n; ++i ) {
          const QTableWidgetItem * hitem = table->horizontalHeaderItem(i);
          if ( hitem && hitem->text() == name ) {
            return i;
          }
        }
        return -1;
      };


  static const auto getColumn =
      [](QTableWidget *table, const QString &name) -> int {
        int currentColumn = findColumn(table, name);
        if (currentColumn < 0) {
          table->setColumnCount((currentColumn = table->columnCount()) + 1);
          table->setHorizontalHeaderItem(currentColumn, new QTableWidgetItem(name));
        }
        return currentColumn;
      };


  static const auto putItem =
      [](QTableWidget *table, const QString &columnName, double value, int currentRow, bool incremental_mode) -> int {

        const int currentColumn = getColumn(table, columnName);
        const QString itemText = qsprintf("%+g", value);

        if (incremental_mode) {
          if (currentRow < 0) {
            table->setRowCount((currentRow = table->rowCount()) + 1);
          }
          table->setItem(currentRow, currentColumn, new QTableWidgetItem(itemText));
        }
        else {

          if (table->rowCount() != 1) {
            table->setRowCount(1);
          }

          QTableWidgetItem *item = table->item(0, currentColumn);
          if (item) {
            item->setText(itemText);
          }
          else {
            table->setItem(0, currentColumn, new QTableWidgetItem(itemText));
          }
        }

        return currentRow;
      };

  const bool incremental_mode =
      _incrementalModeAction->isChecked();

  int currentRow = -1;

  for (int i = 0, n = frames.size(); i < n; ++i) {

    const QMeasureProvider::MeasuredFrame &frame = frames[i];
    const QString &dataChannel = frame.dataChannel;
    const int mcn = frame.mcn;

    if (!frame.measurements.empty() && mcn > 0) {

      for (int c = 0; c < mcn; ++c) {
        const QString columnName =
            QString("%1%2%3").arg(dataChannel).arg("C")
                .arg(mcn < 2 ? QString("") : QString("%1").arg(c));

        currentRow =
            putItem(_table, columnName, frame.mcc[c], currentRow,
                incremental_mode);
      }
    }

    for (int j = 0, m = frame.measurements.size(); j < m; ++j) {

      const QMeasureProvider::MeasuredValue &mv = frame.measurements[j];
      if (!mv.measure) {
        CF_ERROR("APP BUG: mv.measure is NULL");
        continue;
      }

      for (int c = 0; c < mv.cn; ++c) {

        const QString columnName =
            QString("%1%2%3") .arg(dataChannel) .arg(mv.measure->name())
                .arg(mv.cn < 2 ? QString("") : QString("%1").arg(c));

        currentRow =
            putItem(_table, columnName, mv.value[c], currentRow,
                incremental_mode);
      }
    }
  }

  if ( currentRow > 0 ) {
    _table->scrollToBottom();
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
        _table->selectAll();
      });

  if( _table->selectionModel()->hasSelection() ) {

    menu.addAction("Delete selected rows",
        [this]() {
          QWaitCursor wait(this);

          QItemSelectionModel * model = _table->selectionModel();
          const QModelIndex parent = QModelIndex();
          for ( int i = _table->rowCount()-1; i >= 0; --i ) {
            if ( model->isRowSelected(i, parent) ) {
              _table->removeRow(i);
            }
          }
        });


    static const auto copyToClipboard =
        [](QTableWidget *table, bool includeHeaders) {

          const int rowCount = table->rowCount();
          const int columnCount = table->columnCount();

          QString text;

          if ( includeHeaders ) {

            for ( int c = 0; c < columnCount; ++ c) {
              const QTableWidgetItem * hitem = table->horizontalHeaderItem(c);
              if ( hitem ) {
                text.append(hitem->text());
                if ( c < columnCount+1) {
                  text.append("\t");
                }
              }
            }

            text.append("\n");
          }

          const QItemSelectionModel * model = table->selectionModel();
          const QModelIndex parent = QModelIndex();

          for ( int r = 0; r < rowCount; ++ r) {
            if ( model->isRowSelected(r, parent) ) {
              for ( int c = 0; c < columnCount; ++ c) {
                const QTableWidgetItem * hitem = table->item(r,c);
                if ( hitem ) {
                  text.append(hitem->text());
                  if ( c < columnCount+1) {
                    text.append("\t");
                  }
                }
              }
              text.append("\n");
            }
          }

          QApplication::clipboard()->setText(text);
        };

    menu.addAction("Copy to clipboard (with headers)",
        [this]() {
          QWaitCursor wait(this);
          copyToClipboard(_table, true);
        });

    menu.addAction("Copy to clipboard (NO headers)",
        [this]() {
          QWaitCursor wait(this);
          copyToClipboard(_table, false);
        });
  }


  if ( !menu.isEmpty() ) {
    menu.exec(_table->viewport()->mapToGlobal(pos));
  }
}


void QMeasureDisplay::loadParameters()
{
  QSettings settings;
  loadParameters(settings);
}

void QMeasureDisplay::saveParameters()
{
  QSettings settings;
  saveParameters(settings);
}

void QMeasureDisplay::loadParameters(const QSettings & settings)
{
  const QString prefix = "QMeasureDisplay";

  const int num_measures =
      settings.value(QString("%1/selected_measures").arg(prefix), (int) (0)).toInt();

  const std::vector<QMeasure*> & available_measures =
      QMeasureProvider::available_measures();

  _cm.clear();

  for ( int i = 0; i < num_measures; ++i ) {

    const QString name =
        settings.value(QString("%1/measure_%2").arg(prefix).arg(i)).toString();

    if ( name.isEmpty() ) {
      continue;
    }

    const auto mpos =
      std::find_if(available_measures.begin(), available_measures.end(),
          [&name](const QMeasure * obj) {
          return obj->name() == name;
      });

    if ( mpos == available_measures.end() ) {
      continue;
    }

    QMeasure * m = *mpos;
    _cm.emplace(m);
  }
}

void QMeasureDisplay::saveParameters(QSettings & settings)
{
  const QString prefix = "QMeasureDisplay";

  settings.setValue(QString("%1/selected_measures").arg(prefix), (int) (_cm.size()));

  int i = 0;
  for( const auto * m : _cm ) {
    settings.setValue(QString("%1/measure_%2").arg(prefix).arg(i), m->name());
    ++i;
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

  lv->addWidget(_measureDisplay =
      new QMeasureDisplay(this));

  connect(_measureDisplay, &QMeasureDisplay::updateAvailableMeasureDataChannelsRequired,
      this, &ThisClass::updateAvailableMeasureDataChannelsRequired);

  connect(_measureDisplay, &QMeasureDisplay::measureRightNowRequested,
      this, &ThisClass::measureRightNowRequested);


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

void QMeasureDisplayDialogBox::loadParameters()
{
  _measureDisplay->loadParameters();
}

void QMeasureDisplayDialogBox::saveParameters()
{
  _measureDisplay->saveParameters();
}

void QMeasureDisplayDialogBox::loadParameters(const QSettings & settings)
{
  _measureDisplay->loadParameters(settings);

}

void QMeasureDisplayDialogBox::saveParameters(QSettings & settings)
{
  _measureDisplay->saveParameters(settings);
}
