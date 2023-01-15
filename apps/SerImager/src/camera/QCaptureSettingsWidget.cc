/*
 * QCaptureSettingsWidget.cc
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#include "QCaptureSettingsWidget.h"
#include <gui/widgets/style.h>

#define ICON_start_capture        ":/qserimager/icons/start-capture.png"
#define ICON_stop_capture         ":/qserimager/icons/stop-capture.png"


namespace serimager {

namespace {

QIcon icon_start_capture;
QIcon icon_stop_capture;


void enableControl(QWidget * w, bool enabled)
{
  if ( w->isEnabled() != enabled ) {
    w->setEnabled(enabled);
  }
}

void enableControl(QAbstractButton * w, bool enabled, const QIcon & icon)
{
  if ( w->isEnabled() != enabled ) {
    w->setEnabled(enabled);
  }
  if ( w->icon().cacheKey() != icon.cacheKey() ){
    w->setIcon(icon);
  }
}

void init_resources()
{
  if( icon_start_capture.isNull() ) {
    icon_start_capture = getIcon(ICON_start_capture);
  }
  if( icon_stop_capture.isNull() ) {
    icon_stop_capture = getIcon(ICON_stop_capture);
  }
}

} // namespace


QCaptureLimitsControl::QCaptureLimitsControl(QWidget * parent) :
    Base(parent)
{

  init_resources();

  vbox_ = new QVBoxLayout(this);
  vbox_->setContentsMargins(0,0,0,0);


  vbox_->addLayout(h1_ = new QHBoxLayout());
  h1_->setContentsMargins(0, 0, 0, 0);
  h1_->addWidget(limitsSelection_ctl = new QComboBox(this), 1000);
  h1_->addWidget(startStop_ctl = new QToolButton(this), 1);

  limitsSelection_ctl->setEditable(false);
  limitsSelection_ctl->setFocusPolicy(Qt::StrongFocus);

  startStop_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  startStop_ctl->setIcon(icon_start_capture);

  populateCaptureLimitsCombobox();

  connect(limitsSelection_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onLimitsSelectionChanged);

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopButtonClicked);

}

void QCaptureLimitsControl::populateCaptureLimitsCombobox()
{
  static const c_capture_limits default_capture_limits[] = {
      {
          .type = c_capture_limits::ByTime,
          .value = -1 // unlimited
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 1 // 1 s
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 2
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 5
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 10
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 15
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 30
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 60
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 90
      },
      {
          .type = c_capture_limits::ByTime,
          .value = 120
      },

      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 100
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 300
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 500
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 1000
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 1500
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 2000
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 3000
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 5000
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 10000
      },


  };

  limitsSelection_ctl->clear();

  for ( const c_capture_limits & c : default_capture_limits ) {
    limitsSelection_ctl->addItem(toQString(c), QVariant::fromValue(c));
  }

  limitsSelection_ctl->addItem("Custom...");

  limitsSelection_ctl->setCurrentIndex(0);

}

bool QCaptureLimitsControl::getSelectedCaptureLimits(c_capture_limits * c)
{
  int cursel =
      limitsSelection_ctl->currentIndex();

  if( cursel >= 0 ) {
    *c = limitsSelection_ctl->itemData(cursel).value<c_capture_limits>();
    return true;
  }

  return false;
}

QCameraWriter * QCaptureLimitsControl::cameraWriter() const
{
  return writer_;
}


void QCaptureLimitsControl::setCameraWriter(QCameraWriter * writer)
{
  if ( writer_ ) {
    disconnect(writer, nullptr, this, nullptr);
  }

  if( (writer_ = writer) ) {

    c_capture_limits c;

    if ( getSelectedCaptureLimits(&c) ) {
      writer_->setCaptureLimits(c);
    }

    connect(writer_, &QCameraWriter::stateChanged,
        this, &ThisClass::onUpdateControls);
  }

  updateControls();
}

void QCaptureLimitsControl::onUpdateControls()
{
  updateControls();
}

void QCaptureLimitsControl::onupdatecontrols()
{
  if( !writer_ ) {
    setEnabled(false);
  }
  else {

    switch (writer_->state()) {
      case QCameraWriter::State::Active:
      case QCameraWriter::State::Starting:
      case QCameraWriter::State::Stopping:
        enableControl(startStop_ctl, true, icon_stop_capture);
        enableControl(limitsSelection_ctl, false);
        break;
      case QCameraWriter::State::Idle:
        enableControl(limitsSelection_ctl, true);
        if( !writer_->camera() ) {
          enableControl(startStop_ctl, false, icon_start_capture);
        }
        else {
          switch (writer_->camera()->state()) {
            case QImagingCamera::State_connected:
              case QImagingCamera::State_started:
              case QImagingCamera::State_starting:
              enableControl(startStop_ctl, true, icon_start_capture);
              break;
            default:
              enableControl(startStop_ctl, false, icon_start_capture);
              break;
          }
        }
        break;
    }
  }
}

void QCaptureLimitsControl::onStartStopButtonClicked()
{
  if( writer_ ) {

    switch (writer_->state()) {

      case QCameraWriter::State::Active:
        writer_->stop();
        break;

      case QCameraWriter::State::Idle: {
        c_capture_limits c;
        if( getSelectedCaptureLimits(&c) ) {
          writer_->setCaptureLimits(c);
        }
        writer_->start();
        break;
      }
    }
  }
}

void QCaptureLimitsControl::onLimitsSelectionChanged(int)
{
  if( writer_ && writer_->state() == QCameraWriter::Idle ) {
    c_capture_limits c;
    if( getSelectedCaptureLimits(&c) ) {
      writer_->setCaptureLimits(c);
      Q_EMIT captureLimitChanged();
    }
  }
}


QCaptureSettingsWidget::QCaptureSettingsWidget(QWidget * parent) :
    Base("QCaptureSettingsWidget", parent)
{

  form->setLabelAlignment(Qt::AlignLeft);

  captureLimits_ctl =
      add_widget<QCaptureLimitsControl>("Limit:");

  connect(captureLimits_ctl, &QCaptureLimitsControl::captureLimitChanged,
      [this]() {
        updateControls();
        saveCaptureLimits();
      });

  num_rounds_ctl =
      add_spinbox("Rounds:",
          [this](int value) {
            if ( writer_ ) {
              writer_->setNumRounds(value);
              save_parameter(PREFIX, "numRounds",
                  writer_->numRounds());
            }
          });

  num_rounds_ctl->setRange(1, 10000);
  num_rounds_ctl->setValue(1);

  interval_between_rounds_ctl =
      add_spinbox("Interval [sec]:",
          [this](int value) {
            if ( writer_ ) {
              writer_->setIntervalBetweenRounds(value);
              save_parameter(PREFIX, "intervalBetweenRounds",
                  writer_->intervalBetweenRounds());
            }
          });

  interval_between_rounds_ctl->setRange(0, 3600);
  num_rounds_ctl->setValue(1);

  form->addRow(outpuPath_ctl =
      new QBrowsePathCombo("Output path:",
          QFileDialog::AcceptSave,
          QFileDialog::Directory,
          this));

  connect(outpuPath_ctl, &QBrowsePathCombo::pathSelected,
      [this](const QString & path) {
        if ( writer_ ) {
          writer_->setOutputDirectoty(path);
          save_parameter(PREFIX, "outputDirectoty",
              writer_->outputDirectoty());
        }
      });

  updateControls();
}

QCaptureSettingsWidget::~QCaptureSettingsWidget()
{
}


QCameraWriter * QCaptureSettingsWidget::cameraWriter() const
{
  return writer_;
}

void QCaptureSettingsWidget::setCameraWriter(QCameraWriter * writer)
{
  if ( writer_ ) {
    disconnect(writer, nullptr, this, nullptr);
  }

  captureLimits_ctl->setCameraWriter(writer_ = writer);

  if( (writer_ = writer) ) {

    loadParameters();

    connect(writer_, &QCameraWriter::stateChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}


void QCaptureSettingsWidget::onupdatecontrols()
{
  if ( !writer_ ) {
    setEnabled(false);
  }
  else {

    const bool is_unlimited =
        writer_->captureLimits().value < 0;

    num_rounds_ctl->setValue(writer_->numRounds());
    num_rounds_ctl->setEnabled(!is_unlimited);

    interval_between_rounds_ctl->setValue(writer_->intervalBetweenRounds());
    interval_between_rounds_ctl->setEnabled(!is_unlimited);

    outpuPath_ctl->setCurrentPath(writer_->outputDirectoty(), false);
    outpuPath_ctl->setEnabled(writer_->state() == QCameraWriter::State::Idle);


    setEnabled(true);
  }
}

void QCaptureSettingsWidget::onload(QSettings & settings)
{
  if( writer_ ) {

    QString outputDirectoty = writer_->outputDirectoty();
    if( load_parameter(settings, PREFIX, "outputDirectoty", &outputDirectoty) ) {
      writer_->setOutputDirectoty(outputDirectoty);
    }

    int numRounds = writer_->numRounds();
    if( load_parameter(settings, PREFIX, "numRounds", &numRounds) ) {
      writer_->setNumRounds(numRounds);
    }

    int intervalBetweenRounds = writer_->intervalBetweenRounds();
    if( load_parameter(settings, PREFIX, "intervalBetweenRounds", &intervalBetweenRounds) ) {
      writer_->setIntervalBetweenRounds(intervalBetweenRounds);
    }

    loadCaptureLimits(settings);
  }
}

void QCaptureSettingsWidget::saveCaptureLimits()
{
  if ( writer_ ) {

    QSettings settings;

    const c_capture_limits limits =
        writer_->captureLimits();

    settings.setValue(QString("%1/%2").arg(PREFIX).arg("capture_limits_type"),
        toString(limits.type));

    settings.setValue(QString("%1/%2").arg(PREFIX).arg("capture_limits_value"),
        limits.value);
  }
}

void QCaptureSettingsWidget::loadCaptureLimits(QSettings & settings)
{
  if ( writer_ ) {

    c_capture_limits limits =
        writer_->captureLimits();

    load_parameter(settings, PREFIX, "capture_limits_type",
        &limits.type);

    load_parameter(settings, PREFIX, "capture_limits_value",
        &limits.value);

    writer_->setCaptureLimits(limits);
  }

}

} /* namespace serimager */
