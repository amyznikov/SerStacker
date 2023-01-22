/*
 * QImageStatistics.cc
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#include "QImageStatistics.h"
#include <gui/widgets/style.h>
#include <gui/widgets/QWaitCursor.h>
#include <gui/widgets/qsprintf.h>
#include <core/debug.h>

#define ICON_add        ":/qimagestats/icons/add.png"
#define ICON_clear      ":/qimagestats/icons/clear.png"
#define ICON_copy       ":/qimagestats/icons/copy.png"
#define ICON_save       ":/qimagestats/icons/save.png"
#define ICON_select     ":/qimagestats/icons/select.png"
#define ICON_measure    ":/qimagestats/icons/measure.png"


//////////

QImageStatisticsDisplay::QImageStatisticsDisplay(QWidget * parent) :
  Base(parent)
{

  Q_INIT_RESOURCE(qimagestats_resources);

  lv_ = new QVBoxLayout(this);
  lv_->setContentsMargins(0,0,0,0);

  lv_->addWidget(toolbar_ = new QToolBar(this));
  lv_->addWidget(table_ = new QTableWidget(this));

  setupMeasures();
  setupTableView();
  setupToolbar();
  updateControls();

}

void QImageStatisticsDisplay::setupToolbar()
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

  selectMeasurementsAction_  =
      addAction(toolbar_, ICON_select,
          "Select measurements",
          "Select measurements",
          true);

  incrementalMeasurementsAction_ =
      addAction(toolbar_, ICON_add,
          "Incremental",
          "Add measurements",
          true);

  measureAction_ =
      addAction(toolbar_, ICON_measure,
          "Measure",
          "Masure now",
          false);

  connect(selectMeasurementsAction_, &QAction::triggered,
      [this](bool checked) {

        if ( checked && !metricsSelector_ctl ) {

          metricsSelector_ctl = new QImageMeasuresSelectorDialogBox(this);

          connect(metricsSelector_ctl, &QImageMeasuresSelectorDialogBox::visibilityChanged,
              selectMeasurementsAction_, &QAction::setChecked);

        }

        if ( metricsSelector_ctl )  {
          metricsSelector_ctl->setVisible(checked);
        }
      });


  measureAction_->setEnabled(false);

  connect(measureAction_, &QAction::triggered,
      this, &ThisClass::measure);




  updateControls();
}

void QImageStatisticsDisplay::setupTableView()
{
  QStringList labels;

  for( int i = 0, n = measures_.size(); i < n; ++i ) {
    labels.append(measures_[i].outputs);
  }

  table_->setColumnCount(labels.size());
  table_->setHorizontalHeaderLabels(labels);
  table_->setRowCount(1);
  table_->setSelectionMode(QAbstractItemView::SelectionMode::ExtendedSelection);
  table_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  //table_->resizeColumnsToContents();

  for( int i = 0, j = 0, n = measures_.size(); i < n; ++i ) {

    Measure &m = measures_[i];
    m.column = j;

    for( int c = 0; c < m.outputs.size(); ++c, ++j ) {

      table_->horizontalHeaderItem(j)->setData(Qt::UserRole,
          QVariant::fromValue(i));
    }
  }

  connect(table_, &QTableWidget::currentCellChanged,
      this, &ThisClass::onTableViewCurrentCellChanged);

  table_->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);
  connect(table_, &QTableWidget::customContextMenuRequested,
      this, &ThisClass::onTableViewContextMenuRequested);
}


const std::vector<QImageStatisticsDisplay::Measure> & QImageStatisticsDisplay::measures() const
{
  return measures_;
}

void QImageStatisticsDisplay::setMeasure(int index, bool enable)
{
  if( measures_[index].enabled != enable ) {
    measures_[index].enabled = enable;
    updateControls();
  }
}

bool QImageStatisticsDisplay::hasMeasure(int index) const
{
  return measures_[index].enabled;
}


void QImageStatisticsDisplay::setImage(cv::InputArray image, cv::InputArray mask)
{
  image_ = image.getMat();
  mask_ = mask.getMat();
  measureAction_->setEnabled(!image_.empty());
}

void QImageStatisticsDisplay::setRoi(const QRect & roi)
{
  roi_ = roi;
}

void QImageStatisticsDisplay::adjustCurrentRoi(cv::Rect * rc) const
{
  if( roi_.isEmpty() ) {
    *rc = cv::Rect(0, 0, image_.cols, image_.rows);
  }
  else {

    const int l = std::max(0, std::min(image_.cols - 1, roi_.left()));
    const int t = std::max(0, std::min(image_.rows - 1, roi_.top()));
    const int r = std::max(0, std::min(image_.cols - 1, roi_.right()));
    const int b = std::max(0, std::min(image_.rows - 1, roi_.bottom()));

    rc->x = l;
    rc->y = t;
    rc->width = r - l + 1;
    rc->height = b - t + 1;
  }
}

void QImageStatisticsDisplay::measure()
{
  if ( image_.empty() ) {
    return;
  }


  cv::Rect rc;
  adjustCurrentRoi(&rc);

  if ( rc.empty() ) {
    return;
  }


  const cv::Mat image = image_(rc);
  const cv::Mat mask = mask_.empty() ? cv::Mat() : mask_(rc);

  if( table_->rowCount() < 1 ) {
    table_->setRowCount(1);
  }

  int r = -1;

  for( const Measure & m : measures_ ) {
    if ( m.enabled ) {

      cv::Scalar s[m.outputs.size()];

      m.func(image, mask, s);

      if( r < 0 ) {
        r = std::max(0, table_->rowCount() - 1);
      }

      for ( int i = 0, n = m.outputs.size(); i < n; ++i ) {

        QTableWidgetItem * item =
            table_->item(r, m.column + i);

        if( !item ) {
          table_->setItem(r, m.column + i,
              item = new QTableWidgetItem());
        }

        item->setText(qsprintf("%g", s[i][0]));
      }
    }
  }

  if( r >= 0 && incrementalMeasurementsAction_->isChecked() ) {
    table_->insertRow(r + 1);
    table_->setCurrentCell(r, 0);
  }
}



void QImageStatisticsDisplay::onupdatecontrols()
{
  for( int i = 0, n = table_->columnCount(); i < n; ++i ) {

    const QTableWidgetItem *header =
        table_->horizontalHeaderItem(i);

    const int mindex =
        header->data(Qt::UserRole).value<int>();

    if( hasMeasure(mindex) ) {
      table_->showColumn(i);
    }
    else {
      table_->hideColumn(i);
    }
  }

}

void QImageStatisticsDisplay::onTableViewContextMenuRequested(const QPoint &pos)
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
  }


  if ( !menu.isEmpty() ) {
    menu.exec(table_->viewport()->mapToGlobal(pos));
  }

}

void QImageStatisticsDisplay::onTableViewCurrentCellChanged(int currentRow, int currentColumn, int previousRow, int previousColumn)
{

}



void QImageStatisticsDisplay::loadParameters()
{

}

void QImageStatisticsDisplay::saveParameters()
{

}

void QImageStatisticsDisplay::setupMeasures()
{
  measures_.emplace_back(Measure("meanStdev",
      "Call <strong>cv::meanStdev()</strong> over ROI",
      { "mean", "stdev" },

      measurefunc([](const cv::Mat & image, const cv::Mat & mask, cv::Scalar outputs[]) {
        cv::meanStdDev(image, outputs[0], outputs[1], mask);
      }),

      true));

  measures_.emplace_back(Measure("MinMax",
      "Call <strong>cv::minMaxLoc()</strong> over ROI",
      { "min", "max" },

      measurefunc([](const cv::Mat & image, const cv::Mat & mask, cv::Scalar outputs[]) {
        double min, max;
        cv::minMaxLoc(image, &min, &max, nullptr, nullptr, mask);
        outputs[0] = cv::Scalar::all(min);
        outputs[1] = cv::Scalar::all(max);
      })));

  measures_.emplace_back(Measure("Sum",
      "Call <strong>cv::sum()</strong> over ROI",
      { "sum" },
      measurefunc([](const cv::Mat & image, const cv::Mat & mask, cv::Scalar outputs[]) {
        outputs[0] = cv::sum(image);
      })));

}


//////////

QImageMeasuresSelector::QImageMeasuresSelector(QWidget *  parent) :
    Base(parent)
{
  lv_ = new QVBoxLayout(this);

  lv_->setContentsMargins(0, 0, 0, 0);

  lv_->addWidget(treeview_ = new QTreeWidget(this));
  treeview_->setHeaderHidden(true);
  treeview_->setColumnCount(1);
  treeview_->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
  treeview_->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);


  connect(treeview_, &QTreeWidget::itemChanged,
      this, &ThisClass::onItemChanged);

  updateControls();
}

void QImageMeasuresSelector::setStatisticsDisplay(QImageStatisticsDisplay * statisticsDisplay)
{
  display_ = statisticsDisplay;
  updateControls();
}

QImageStatisticsDisplay * QImageMeasuresSelector::statisticsDisplay() const
{
  return display_;
}

void QImageMeasuresSelector::onupdatecontrols()
{
  treeview_->clear();

  if ( !display_ ) {
    setEnabled(false);
  }
  else {

    const std::vector<QImageStatisticsDisplay::Measure> & measures =
        display_->measures();

    const int num_measures =
        measures.size();

    for ( int i = 0; i < num_measures; ++i ) {

      const QImageStatisticsDisplay::Measure & m =
          measures[i];

      QTreeWidgetItem * item =
          new QTreeWidgetItem(MeasureNameItemType);

      item->setText(0, m.name);
      item->setData(0, Qt::UserRole, QVariant::fromValue(i));
      item->setCheckState(0, display_->hasMeasure(i) ? Qt::Checked : Qt::Unchecked);

      treeview_->addTopLevelItem(item);

      QTreeWidgetItem * child =
          new QTreeWidgetItem(item,
              MeasureDescriptionType);

      QLabel *label = new QLabel(m.description);
      label->setTextFormat(Qt::RichText);
      label->setWordWrap(true);

      treeview_->setItemWidget(child, 0, label);
    }

    setEnabled(true);
  }
}

void QImageMeasuresSelector::onItemChanged(QTreeWidgetItem *item, int column)
{
  if( display_ && item && column == 0 && !updatingControls() && item->type() == MeasureNameItemType ) {
    display_->setMeasure(item->data(0, Qt::UserRole).value<int>(), item->checkState(0) == Qt::Checked);
  }

}

//////////


QImageMeasuresSelectorDialogBox::QImageMeasuresSelectorDialogBox(QImageStatisticsDisplay * statisticsDisplay) :
    ThisClass("Select metrics", statisticsDisplay)
{
}

QImageMeasuresSelectorDialogBox::QImageMeasuresSelectorDialogBox(const QString & title, QImageStatisticsDisplay * statisticsDisplay) :
    Base(statisticsDisplay)
{
  setWindowTitle(title);

  QVBoxLayout *lv =
      new QVBoxLayout(this);

  lv->setContentsMargins(0, 0, 0, 0);

  lv->addWidget(selector_ =
      new QImageMeasuresSelector(this));

  selector_->setStatisticsDisplay(statisticsDisplay);
}


void QImageMeasuresSelectorDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QImageMeasuresSelectorDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

//////////

QImageStatisticsDisplayDialogBox::QImageStatisticsDisplayDialogBox(QWidget * parent) :
    ThisClass("Measure", parent)
{
}

QImageStatisticsDisplayDialogBox::QImageStatisticsDisplayDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);

  QVBoxLayout * lv =
      new QVBoxLayout(this);

  lv->setContentsMargins(0,0,0,0);

  lv->addWidget(display_ =
      new QImageStatisticsDisplay(this));
}

QImageStatisticsDisplay * QImageStatisticsDisplayDialogBox::display() const
{
  return display_;
}

void QImageStatisticsDisplayDialogBox::setImage(cv::InputArray image, cv::InputArray mask)
{
  display_->setImage(image, mask);
}

void QImageStatisticsDisplayDialogBox::setRoi(const QRect & roi)
{
  display_->setRoi(roi);
}

void QImageStatisticsDisplayDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QImageStatisticsDisplayDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

