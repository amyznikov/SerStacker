/*
 * QMeasureSelection.cc
 *
 *  Created on: Apr 9, 2023
 *      Author: amyznikov
 */
#include "QMeasureSelection.h"

QSingeMeasureSelectionWidget::QSingeMeasureSelectionWidget(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

QSingeMeasureSelectionWidget::QSingeMeasureSelectionWidget(QMeasureProvider * mp, QWidget * parent) :
  Base("", parent)
{
  combobox_ctl =
      add_combobox<QComboBox>("Measure:",
          "Select measurement",
          [this](int cursel, QComboBox * combo) {

            bool hasChanges = false;

            if ( cm_ ) {
              cm_->set_enabled(false);
              cm_ = nullptr;
              hasChanges = true;
            }

            if ( cursel >= 0 && mp_ ) {

              if ( !(cm_ = combo->itemData(cursel).value<QMeasure *>())) {
                tooltip_ctl->setText("");
              }
              else {
                cm_->set_enabled(true);
                tooltip_ctl->setText(cm_->tooltip());
                hasChanges = true;
              }

              updatesettingswidget();
            }

            if ( hasChanges ) {
              Q_EMIT currentMeasureChanged();
              //Q_EMIT parameterChanged();
            }
          });

  addRow(tooltip_ctl = new QLabel(this));

  maxMeasurements_ctl =
      add_numeric_box<int>("Max measurements",
          "Set max queue size",
          [this](int value) {
            if ( mp_ ) {
              mp_->set_max_measured_frames(value);
              Q_EMIT parameterChanged();
            }
          },
          [this](int * value) {
            if ( mp_ ) {
              * value = mp_->max_measured_frames();
              return true;
            }
            return false;
          });


  scrollArea_ctl = new QScrollArea(this);
  scrollArea_ctl->setWidgetResizable(true);
  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
  scrollArea_ctl->setFrameShape(QFrame::NoFrame);
  addRow(scrollArea_ctl);

  setMeasureProvider(mp);
}

void QSingeMeasureSelectionWidget::setMeasureProvider(QMeasureProvider * mp)
{
  if ( cm_ ) {
    cm_->set_enabled(false);
    cm_ = nullptr;
  }

  combobox_ctl->clear();

  if ( (mp_ = mp) ) {

    const std::set<QMeasure*> & measures =
        mp_->measures();

    for ( QMeasure * m : measures ) {
      combobox_ctl->addItem(m->name(),
          QVariant::fromValue(m));
    }
  }

  Q_EMIT currentMeasureChanged();

  updateControls();
}

QMeasureProvider * QSingeMeasureSelectionWidget::measureProvider() const
{
  return mp_;
}

QMeasure * QSingeMeasureSelectionWidget::currentMeasure() const
{
  return cm_;
}

void QSingeMeasureSelectionWidget::onupdatecontrols()
{
  if( !mp_ ) {
    setEnabled(false);
  }
  else {
    Base::onupdatecontrols();
    setEnabled(true);
  }
}

void QSingeMeasureSelectionWidget::updatesettingswidget()
{
  QMeasureSettingsWidget *currentWidget =
      nullptr;

  if( cm_ && cm_->hasOptions() ) {

    const auto pos =
        std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
            [this](const QMeasureSettingsWidget * obj) {
              return cm_ == obj->currentMeasure();
            });

    if( pos != settingsWidgets_.end() ) {
      currentWidget = *pos;
    }
    else if( !(currentWidget = cm_->createSettingsWidget(this)) ) {
      CF_ERROR("cm_->createSettingsWidget() fails ");
    }
    else {
      settingsWidgets_.append(currentWidget);
      currentWidget->setCurrentMeasure(cm_);
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



QSingeMeasureSelectionDialogBox::QSingeMeasureSelectionDialogBox(QMeasureProvider * mp, QWidget * parent) :
    ThisClass("Select metric", mp, parent)
{
}

QSingeMeasureSelectionDialogBox::QSingeMeasureSelectionDialogBox(const QString & title,
    QMeasureProvider * mp, QWidget * parent) :
    Base(parent)
{
  setWindowTitle(title);
  setMinimumWidth(QFontMetrics(font(), this).horizontalAdvance(title) + 200);

  QVBoxLayout *lv =
      new QVBoxLayout(this);

  // lv->setContentsMargins(0, 0, 0, 0);

  lv->addWidget(measureSelection_ctl =
      new QSingeMeasureSelectionWidget(this));

  connect(measureSelection_ctl, &QSingeMeasureSelectionWidget::currentMeasureChanged,
      this, &ThisClass::currentMeasureChanged);

  measureSelection_ctl->setMeasureProvider(mp);
}

QMeasure * QSingeMeasureSelectionDialogBox::currentMeasure() const
{
  return measureSelection_ctl->currentMeasure();
}

void QSingeMeasureSelectionDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QSingeMeasureSelectionDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QSingeMeasureSelectionDialogBox::closeEvent(QCloseEvent *)
{
  hide();
}



QMultiMeasureSelectionWidget::QMultiMeasureSelectionWidget(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

QMultiMeasureSelectionWidget::QMultiMeasureSelectionWidget(QMeasureProvider * mp, QWidget * parent) :
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

//  scrollArea_ctl = new QScrollArea();
//  scrollArea_ctl->setWidgetResizable(true);
//  scrollArea_ctl->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
//  scrollArea_ctl->setFrameShape(QFrame::NoFrame);
//  vbox_->addWidget(scrollArea_ctl, 1000);

  connect(listview_ctl, &QListWidget::itemChanged,
      this, &ThisClass::onListViewItemChanged);

  connect(listview_ctl, &QListWidget::currentItemChanged,
      this, &ThisClass::onListViewCurrentItemChanged);

  setMeasureProvider(mp);
}

void QMultiMeasureSelectionWidget::setMeasureProvider(QMeasureProvider * mp)
{
  for ( int i = 0, n = listview_ctl->count(); i < n; ++i ) {

    QListWidgetItem * item =
        listview_ctl->item(i);

    if ( item->checkState() == Qt::Checked ) {

      QMeasure *m =
          item->data(Qt::UserRole).value<QMeasure*>();

      if ( m ) {
        m->set_enabled(false);
      }
    }
  }

  listview_ctl->clear();

  if( (mp_ = mp) ) {

    const std::set<QMeasure*> & measures =
        mp_->measures();

    for ( QMeasure * m : measures ) {

      QListWidgetItem * item =
          new QListWidgetItem(m->name());

      item->setData(Qt::UserRole,
          QVariant::fromValue(m));

      item->setCheckState(Qt::Unchecked);

      listview_ctl->addItem(item);
    }
  }

  updateControls();
}

QMeasureProvider * QMultiMeasureSelectionWidget::measureProvider() const
{
  return mp_;
}

void QMultiMeasureSelectionWidget::onupdatecontrols()
{
  if ( !mp_ ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}

void QMultiMeasureSelectionWidget::updatesettingswidget()
{
}

void QMultiMeasureSelectionWidget::onListViewItemChanged(QListWidgetItem *item)
{
  if( item ) {
    QMeasure *m =
        item->data(Qt::UserRole).value<QMeasure*>();
    if( m ) {
      m->set_enabled(item->checkState() == Qt::Checked);
      Q_EMIT selectedMeasuresChanged();
    }
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


}

QMultiMeasureSelectionDialogBox::QMultiMeasureSelectionDialogBox(QWidget * parent) :
    ThisClass((QMeasureProvider*) nullptr, parent)
{
}

QMultiMeasureSelectionDialogBox::QMultiMeasureSelectionDialogBox(QMeasureProvider * mp, QWidget * parent) :
    ThisClass("Select Measures", mp, parent)
{
}

QMultiMeasureSelectionDialogBox::QMultiMeasureSelectionDialogBox(const QString & title, QMeasureProvider * mp, QWidget * parent) :
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

  measureSelection_ctl->setMeasureProvider(mp);
}

void QMultiMeasureSelectionDialogBox::setMeasureProvider(QMeasureProvider * mp)
{
  measureSelection_ctl->setMeasureProvider(mp);
}

QMeasureProvider * QMultiMeasureSelectionDialogBox::measureProvider() const
{
  return measureSelection_ctl->measureProvider();
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

