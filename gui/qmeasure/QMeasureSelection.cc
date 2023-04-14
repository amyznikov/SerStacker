/*
 * QMeasureSelection.cc
 *
 *  Created on: Apr 9, 2023
 *      Author: amyznikov
 */
#include "QMeasureSelection.h"

QMeasureSelectionCombo::QMeasureSelectionCombo(QWidget * parent) :
    Base(parent)
{
  setEditable(false);

  //addItem("NONE", QVariant::fromValue((QMeasure*) nullptr));

  for( QMeasure *m : QMeasureProvider::measures() ) {
    addItem(m->name(),
        QVariant::fromValue(m));
  }

  connect(this, static_cast<void (QComboBox::*)(int)>( &QComboBox::currentIndexChanged ),
      this, &ThisClass::currentMeasureChanged);

}

QMeasure * QMeasureSelectionCombo::currentMeasure()
{
  return currentData().value<QMeasure*>();
}
//
//
//QSingeMeasureSelectionWidget::QSingeMeasureSelectionWidget(QWidget * parent) :
//  Base("", parent)
//{
//  combobox_ctl =
//      add_combobox<QComboBox>("Measure:",
//          "Select measurement",
//          [this](int cursel, QComboBox * combo) {
//
//            bool hasChanges = false;
//
//            if ( cm_ ) {
//              cm_ = nullptr;
//              hasChanges = true;
//            }
//
//            if ( cursel >= 0 ) {
//
//              if ( !(cm_ = combo->itemData(cursel).value<QMeasure *>())) {
//                tooltip_ctl->setText("");
//              }
//              else {
//                tooltip_ctl->setText(cm_->tooltip());
//                hasChanges = true;
//              }
//
//              updatesettingswidget();
//            }
//
//            if ( hasChanges ) {
//              Q_EMIT currentMeasureChanged();
//            }
//          });
//
//  addRow(tooltip_ctl = new QLabel(this));
//
//  maxMeasurements_ctl =
//      add_numeric_box<int>("Max measurements",
//          "Set max queue size",
//          [this](int value) {
//            QMeasureProvider::set_max_measured_frames(value);
//          },
//          [this](int * value) {
//            * value = QMeasureProvider::max_measured_frames();
//            return true;
//          });
//
//
//  scrollArea_ctl = new QScrollArea(this);
//  scrollArea_ctl->setWidgetResizable(true);
//  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
//  scrollArea_ctl->setFrameShape(QFrame::NoFrame);
//  addRow(scrollArea_ctl);
//
//  /////////////
//
//  for( QMeasure *m : QMeasureProvider::measures() ) {
//    combobox_ctl->addItem(m->name(),
//        QVariant::fromValue(m));
//  }
//
//  /////////////
//}
//
//int QSingeMeasureSelectionWidget::indexOf(QMeasure * m) const
//{
//  if( m ) {
//    return combobox_ctl->findData(QVariant::fromValue(m));
//  }
//  return -1;
//}
//
//void QSingeMeasureSelectionWidget::setCurrentMeasure(QMeasure * m)
//{
//  combobox_ctl->setCurrentIndex(indexOf(cm_ = m));
//}
//
//QMeasure * QSingeMeasureSelectionWidget::currentMeasure() const
//{
//  return cm_;
//}
//
//void QSingeMeasureSelectionWidget::onupdatecontrols()
//{
//  Base::onupdatecontrols();
//}
//
//void QSingeMeasureSelectionWidget::updatesettingswidget()
//{
//  QMeasureSettingsWidget *currentWidget =
//      nullptr;
//
//  if( cm_ && cm_->hasOptions() ) {
//
//    const auto pos =
//        std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
//            [this](const QMeasureSettingsWidget * obj) {
//              return cm_ == obj->currentMeasure();
//            });
//
//    if( pos != settingsWidgets_.end() ) {
//      currentWidget = *pos;
//    }
//    else if( !(currentWidget = cm_->createSettingsWidget(this)) ) {
//      CF_ERROR("cm_->createSettingsWidget() fails ");
//    }
//    else {
//      settingsWidgets_.append(currentWidget);
//      currentWidget->setCurrentMeasure(cm_);
//    }
//  }
//
//  if( scrollArea_ctl->widget() != currentWidget ) {
//
//    if( scrollArea_ctl->widget() ) {
//      scrollArea_ctl->widget()->hide();
//    }
//
//    scrollArea_ctl->takeWidget();
//    scrollArea_ctl->setWidget(currentWidget);
//
//    if( scrollArea_ctl->widget() ) {
//      scrollArea_ctl->widget()->show();
//    }
//  }
//}
//


QMeasureSettingsDialogBox::QMeasureSettingsDialogBox(QWidget * parent) :
    ThisClass("Measure options", parent)
{
}

QMeasureSettingsDialogBox::QMeasureSettingsDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);
  setMinimumWidth(QFontMetrics(font(), this).horizontalAdvance(title) + 200);

  layout_ = new QVBoxLayout(this);
  layout_->addWidget(combobox_ctl = new QMeasureSelectionCombo(), 0, Qt::AlignTop);
  layout_->addWidget(tooltip_ctl = new QLabel(), 0, Qt::AlignTop);

  // maxMeasurements_ctl

  scrollArea_ctl = new QScrollArea(this);
  scrollArea_ctl->setWidgetResizable(true);
  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea_ctl->setFrameShape(QFrame::NoFrame);
  layout_->addWidget(scrollArea_ctl, 1000);

  connect(combobox_ctl, &QMeasureSelectionCombo::currentMeasureChanged,
      this, &ThisClass::onCurrentMeasureChanged);

  onCurrentMeasureChanged();
}

void QMeasureSettingsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QMeasureSettingsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QMeasureSettingsDialogBox::closeEvent(QCloseEvent *)
{
  hide();
}

void QMeasureSettingsDialogBox::onCurrentMeasureChanged()
{
  QMeasureSettingsWidget *currentWidget = nullptr;

  QMeasure * m =
      combobox_ctl->currentMeasure();

  if( !m ) {
    tooltip_ctl->setText("");
  }
  else {

    tooltip_ctl->setText(m->tooltip());

    if( m->hasOptions() ) {

      const auto pos =
          std::find_if(widgets_.begin(), widgets_.end(),
              [m](const QMeasureSettingsWidget * obj) {
                return m == obj->currentMeasure();
              });

      if( pos != widgets_.end() ) {
        currentWidget = *pos;
      }
      else if( !(currentWidget = m->createSettingsWidget(this)) ) {
        CF_ERROR("m->createSettingsWidget() fails ");
      }
      else {
        widgets_.append(currentWidget);
        currentWidget->setCurrentMeasure(m);
      }
    }
  }

  if( scrollArea_ctl->widget() != currentWidget ) {

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->hide();
    }

    scrollArea_ctl->takeWidget();
    scrollArea_ctl->setWidget(currentWidget);

    if( scrollArea_ctl->widget() ) {
      scrollArea_ctl->widget()->show();
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QMultiMeasureSelectionWidget::QMultiMeasureSelectionWidget(QWidget * parent) :
    Base(parent)
{
  setMinimumSize(600, 400);

  vbox_ = new QVBoxLayout(this);
  vbox_->setContentsMargins(0, 0,0,0);

  vbox_->addWidget(tooltip_ctl = new QLabel(), 1, Qt::AlignTop);
  vbox_->addLayout(hbox_ = new QHBoxLayout(), 1000);


  listview_ctl = new QListWidget();
  listview_ctl->setViewMode(QListView::ViewMode::ListMode);
  listview_ctl->setSelectionMode(QListView::SelectionMode::SingleSelection);
  listview_ctl->setSelectionBehavior(QListView::SelectionBehavior::SelectRows);
  listview_ctl->setSizeAdjustPolicy(QAbstractScrollArea::SizeAdjustPolicy::AdjustToContents);
  //listview_ctl->setSizePolicy(QSizePolicy::Policy);

  hbox_->addWidget(listview_ctl);

  /////////////////////////////////////

  for( QMeasure *m : QMeasureProvider::measures() ) {

    QListWidgetItem *item =
        new QListWidgetItem(m->name());

    item->setData(Qt::UserRole,
        QVariant::fromValue(m));

    item->setCheckState(Qt::Unchecked);

    listview_ctl->addItem(item);
  }

  /////////////////////////////////////


  connect(listview_ctl, &QListWidget::itemChanged,
      this, &ThisClass::onListViewItemChanged);

  connect(listview_ctl, &QListWidget::currentItemChanged,
      this, &ThisClass::onListViewCurrentItemChanged);


  updateControls();
}


void QMultiMeasureSelectionWidget::selectMeasures(std::set<QMeasure*> * cm)
{
  cm_ = cm;
  updateControls();
}

void QMultiMeasureSelectionWidget::onupdatecontrols()
{
  if( !cm_ ) {
    setEnabled(false);
  }
  else {


    for ( int i = 0, n = listview_ctl->count(); i < n; ++i ) {

      QListWidgetItem *item =
          listview_ctl->item(i);

      QMeasure *m =
          item->data(Qt::UserRole).value<QMeasure *>();

      item->setCheckState(cm_->find(m) != cm_->end() ? Qt::Checked :
          Qt::Unchecked);
    }

    setEnabled(true);
  }
}

void QMultiMeasureSelectionWidget::onListViewItemChanged(QListWidgetItem *item)
{
  if( item && cm_ ) {

    QMeasure *m =
        item->data(Qt::UserRole).value<QMeasure*>();

    if( item->checkState() == Qt::Checked ) {
      cm_->emplace(m);
    }
    else {
      cm_->erase(m);
    }

    Q_EMIT selectedMeasuresChanged();
  }
}

void QMultiMeasureSelectionWidget::onListViewCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
  QMeasureSettingsWidget *currentWidget = nullptr;
  QMeasure * m = nullptr;

  if ( current ) {

    current->setSelected(true);

    m = current->data(Qt::UserRole).value<QMeasure*>();

    if( m && m->hasOptions() ) {

      const auto pos =
          std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
              [m](const QMeasureSettingsWidget * obj) {
                return m == obj->currentMeasure();
              });

      if( pos != settingsWidgets_.end() ) {
        currentWidget = *pos;
      }
      else if( !(currentWidget = m->createSettingsWidget(this)) ) {
        CF_ERROR("m->createSettingsWidget() fails ");
      }
      else {
        settingsWidgets_.append(currentWidget);
        currentWidget->setCurrentMeasure(m);
        currentWidget->setVisible(false);
      }
    }
  }
  else if ( previous ) {
    previous->setSelected(false);
  }

  if ( m ) {
    tooltip_ctl->setText(m->tooltip());
  }
  else {
    tooltip_ctl->setText("");
  }

  if ( cw_ != currentWidget ) {

    if ( cw_ ) {
      cw_->setVisible(false);
      hbox_->removeWidget(cw_);
    }

    if( (cw_ = currentWidget) ) {
      hbox_->addWidget(cw_);
      cw_->setVisible(true);
    }

  }

}

QMultiMeasureSelectionDialogBox::QMultiMeasureSelectionDialogBox(QWidget * parent) :
    ThisClass("Select Measures", parent)
{
}

QMultiMeasureSelectionDialogBox::QMultiMeasureSelectionDialogBox(const QString & title, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);
  setMinimumWidth(QFontMetrics(font(), this).horizontalAdvance(title) + 200);

  QVBoxLayout *lv =
      new QVBoxLayout(this);

  // lv->setContentsMargins(0, 0, 0, 0);

  lv->addWidget(measureSelection_ctl =
      new QMultiMeasureSelectionWidget(this));

  connect(measureSelection_ctl, &QMultiMeasureSelectionWidget::selectedMeasuresChanged,
      this, &ThisClass::selectedMeasuresChanged);

}

void QMultiMeasureSelectionDialogBox::selectMeasures(std::set<QMeasure*> * cm)
{
  measureSelection_ctl->selectMeasures(cm);
}

void QMultiMeasureSelectionDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QMultiMeasureSelectionDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QMultiMeasureSelectionDialogBox::closeEvent(QCloseEvent * )
{
  hide();
}

