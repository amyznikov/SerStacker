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
            if ( mp_ ) {

              QMeasure * m = nullptr;

              mp_->clear_selected_measures();

              if ( cursel >= 0 ) {
                m = combo->itemData(cursel).value<QMeasure *>();
                mp_->add_selected_measure(m);
              }

              tooltip_ctl->setText( m ? m->tooltip() : "");

              updatesettingswidget();

              Q_EMIT parameterChanged();
            }
          },
          [this](int * cursel, QComboBox * combo) {
            if ( mp_ ) {
              if ( mp_->selected_measures().empty() ) {
                * cursel = -1;
              }
              else {
                * cursel =
                    combo->findData(QVariant::fromValue(*mp_->selected_measures().begin()));
              }
              return true;
            }
            return false;
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

  popuatecombo();
  setMeasureProvider(mp);
}

void QSingeMeasureSelectionWidget::setMeasureProvider(QMeasureProvider * mp)
{
  mp_ = mp;
  updateControls();
}

QMeasureProvider * QSingeMeasureSelectionWidget::measureProvider() const
{
  return mp_;
}

void QSingeMeasureSelectionWidget::popuatecombo()
{
  combobox_ctl->clear();

  QMeasure ** measures =
      QMeasureProvider::available_measures();

  for( int i = 0; measures[i]; ++i ) {
    QMeasure *m = measures[i];
    combobox_ctl->addItem(m->name(), QVariant::fromValue(m));
  }
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

  if ( mp_ ) {

    QMeasure *m =
        mp_->selected_measures().empty() ? nullptr :
            *mp_->selected_measures().begin();


    if ( m && m->hasOptions() ) {

      const auto pos =
          std::find_if(settingsWidgets_.begin(), settingsWidgets_.end(),
              [m](const QMeasureSettingsWidget * obj) {
                return m == obj->currentMeasure();
              });

      if ( pos != settingsWidgets_.end() ) {
        currentWidget = *pos;
      }
      else if( !(currentWidget = m->createSettingsWidget(this)) ) {
        CF_ERROR("m->createSettingsWidget() fails ");
      }
      else {
        settingsWidgets_.append(currentWidget);
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

  measureSelection_ctl->setMeasureProvider(mp);
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

