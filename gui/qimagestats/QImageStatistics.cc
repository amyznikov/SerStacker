/*
 * QImageStatistics.cc
 *
 *  Created on: Jan 20, 2023
 *      Author: amyznikov
 */

#include "QImageStatistics.h"
#include <gui/widgets/style.h>

#define ICON_add        ":/qimagestats/icons/add.png"
#define ICON_clear      ":/qimagestats/icons/clear.png"
#define ICON_copy       ":/qimagestats/icons/copy.png"
#define ICON_save       ":/qimagestats/icons/save.png"
#define ICON_select     ":/qimagestats/icons/select.png"


QImageStatisticsDisplay::QImageStatisticsDisplay(QWidget * parent) :
  Base(parent)
{
  Q_INIT_RESOURCE(qimagestats_resources);

  lv_ = new QVBoxLayout(this);
  lv_->setContentsMargins(0,0,0,0);

  lv_->addWidget(toolbar_ = new QToolBar(this));
  lv_->addWidget(table_ = new QTableWidget(this));

  setupToolbar();
  setupTableView();
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
          false);

  incrementalMeasurementsAction_ =
      addAction(toolbar_, ICON_add,
          "Incremental",
          "Add measurements",
          true);


}

void QImageStatisticsDisplay::setupTableView()
{
  table_->setColumnCount(4);
  table_->setRowCount(10);
}

void QImageStatisticsDisplay::loadParameters()
{

}

void QImageStatisticsDisplay::saveParameters()
{

}


QImageStatisticsDisplayDialogBox::QImageStatisticsDisplayDialogBox(QWidget * parent) :
    ThisClass("Measure")
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

