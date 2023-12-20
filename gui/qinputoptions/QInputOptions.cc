/*
 * QInputOptions.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "QInputOptions.h"
#include <gui/widgets/style.h>


#define BAYER_ICON ":/gui/icons/bayer.png"



QVideoInputOptions::QVideoInputOptions(QWidget * parent) :
    Base(parent)
{
  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Default debayer:",
          "Select debayer algorithm for bayer patterns",
          [this](DEBAYER_ALGORITHM v) {
            if ( options_ && options_->debayer_method != v ) {
              options_->debayer_method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            if ( options_ ) {
              *v = options_->debayer_method;
              return true;
            }
            return false;
          });

  enable_color_maxtrix_ctl =
      add_checkbox("Enable Color Matrix",
          "Enable Color Matrix if provided",
          [this](bool checked) {
            if ( options_ && options_->enable_color_maxtrix != checked ) {
              options_->enable_color_maxtrix = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->enable_color_maxtrix;
              return true;
            }
            return false;
          });

  filter_bad_pixels_ctl =
      add_checkbox("Filter Bad Pixels",
          "Enable Detect and Filter Bad Pixels",
          [this](bool checked) {
            if ( options_ && options_->filter_bad_pixels != checked ) {
              options_->filter_bad_pixels = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( options_ ) {
              *checked = options_->filter_bad_pixels;
              return true;
            }
            return false;
          });

  bad_pixels_variation_threshold_ctl =
      add_numeric_box<double>("Bad Pixels Variation:",
          "bad_pixels_variation_threshold",
          [this](double v) {
            if ( options_ && options_->bad_pixels_variation_threshold != v ) {
              options_->bad_pixels_variation_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( options_ ) {
              *v = options_->bad_pixels_variation_threshold;
              return true;
            }
            return false;

          });

  updateControls();
}

QInputOptions::QInputOptions(QWidget * parent) :
    Base(parent)
{
  addRow(tab_ctl = new QTabWidget(this));

  tab_ctl->addTab(videoOptions_ctl = new QVideoInputOptions(this),
      QIcon(), "Video");

  connect(videoOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);

  updateControls();
}


void QInputOptions::set_options(c_input_options * options)
{
  if ( !(this->options_ = options) ) {
    videoOptions_ctl->set_options(nullptr);
  }
  else {
    videoOptions_ctl->set_options(&options_->video);
  }

  updateControls();
}


QInputOptionsDialogBox::QInputOptionsDialogBox(QWidget * parent) :
    Base(parent)
{
  setWindowTitle("Input Options...");
  setWindowIcon(getIcon(BAYER_ICON));

  QVBoxLayout * vbox =
      new QVBoxLayout(this);

  vbox->addWidget(inputOptions_ctl =
      new QInputOptions(this));

  connect(inputOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);
}

void QInputOptionsDialogBox::setInputOptions(c_input_options * options)
{
  inputOptions_ctl->set_options(options);
}

c_input_options * QInputOptionsDialogBox::inputOptions() const
{
  return inputOptions_ctl->options();
}

void QInputOptionsDialogBox::showEvent(QShowEvent * e)
{
  Base::showEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}

void QInputOptionsDialogBox::hideEvent(QHideEvent * e)
{
  Base::hideEvent(e);
  Q_EMIT visibilityChanged(isVisible());
}
