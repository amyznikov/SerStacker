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


////QSpinBox *num_rounds_ctl
//template<class CtrlType>
//class QNamedControl:
//    public QFrame
//{
//public:
//  typedef QNamedControl ThisClass;
//  typedef QFrame Base;
//
//  QNamedControl(const QString & label, QWidget * parent = nullptr) :
//    Base(parent)
//  {
//
//    hbox_ = new QHBoxLayout(this);
//    hbox_->setContentsMargins(0, 0, 0, 0);
//    hbox_->addWidget(label_ctl = new QLabel(label), 0, Qt::AlignLeft);
//    hbox_->addWidget(control_ = new CtrlType(this), 0, Qt::AlignLeft);
//  }
//
//  QLabel * label () const
//  {
//    return label_ctl;
//  }
//
//  CtrlType * control() const
//  {
//    return control_;
//  }
//
//protected:
//  QHBoxLayout * hbox_ = nullptr;
//  QLabel * label_ctl = nullptr;
//  CtrlType * control_ = nullptr;
//};

} // namespace


QCameraCaptureControl::QCameraCaptureControl(QWidget * parent) :
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

  connect(startStop_ctl, &QToolButton::clicked,
      this, &ThisClass::onStartStopButtonClicked);

  //  connect(limitsSelection_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
  //      this, &ThisClass::onLimitsSelectionChanged);

}

void QCameraCaptureControl::populateCaptureLimitsCombobox()
{
  static const c_capture_limits default_capture_limits[] = {
      {
          .type = c_capture_limits::by_time,
          .value = -1 // unlimited
      },
      {
          .type = c_capture_limits::by_time,
          .value = 1 // 1 s
      },
      {
          .type = c_capture_limits::by_time,
          .value = 2
      },
      {
          .type = c_capture_limits::by_time,
          .value = 5
      },
      {
          .type = c_capture_limits::by_time,
          .value = 10
      },
      {
          .type = c_capture_limits::by_time,
          .value = 15
      },
      {
          .type = c_capture_limits::by_time,
          .value = 30
      },
      {
          .type = c_capture_limits::by_time,
          .value = 60
      },
      {
          .type = c_capture_limits::by_time,
          .value = 90
      },
      {
          .type = c_capture_limits::by_time,
          .value = 120
      },

      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 100
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 300
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 500
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 1000
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 1500
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 2000
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 3000
      },
      {
          .type = c_capture_limits::by_number_of_frames,
          .value = 5000
      },
      {
          .type = c_capture_limits::by_number_of_frames,
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

bool QCameraCaptureControl::getSelectedCaptureLimits(c_capture_limits * c)
{
  int cursel =
      limitsSelection_ctl->currentIndex();

  if( cursel >= 0 ) {
    *c = limitsSelection_ctl->itemData(cursel).value<c_capture_limits>();
    CF_DEBUG("capture limits: type=%s value=%d", toString(c->type), c->value);
    return true;
  }

  return false;
}

QCameraWriter * QCameraCaptureControl::cameraWriter() const
{
  return writer_;
}


void QCameraCaptureControl::setCameraWriter(QCameraWriter * writer)
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

void QCameraCaptureControl::onUpdateControls()
{
  updateControls();
}

void QCameraCaptureControl::onupdatecontrols()
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

void QCameraCaptureControl::onStartStopButtonClicked()
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

void QCameraCaptureControl::onLimitsSelectionChanged(int)
{
  if( writer_ && writer_->state() == QCameraWriter::Idle ) {
    c_capture_limits c;
    if( getSelectedCaptureLimits(&c) ) {
      writer_->setCaptureLimits(c);
    }
  }
}


QCaptureSettingsWidget::QCaptureSettingsWidget(QWidget * parent) :
    Base("QCaptureSettingsWidget", parent)
{

  form->setLabelAlignment(Qt::AlignLeft);

  capture_ctl =
      add_widget<QCameraCaptureControl>("Limit:");

  num_rounds_ctl =
      add_spinbox("Rounds:",
          [this](int value) {
            if ( writer_ ) {
              writer_->setRounds(value);
            }
          });

  num_rounds_ctl->setRange(1, 10000);
  num_rounds_ctl->setValue(1);

  interval_between_rounds_ctl =
      add_spinbox("Interval [sec]:",
          [this](int value) {
            if ( writer_ ) {
              writer_->setIntervalBetweenRounds(value);
            }
          });

  interval_between_rounds_ctl->setRange(0, 3600);
  num_rounds_ctl->setValue(1);

  form->addRow(outpuPath_ctl =
      new QBrowsePathCombo("Output path:",
          QFileDialog::Directory,
          this));

  outpuPath_ctl->setAcceptMode(
      QFileDialog::AcceptSave);

  connect(outpuPath_ctl, &QBrowsePathCombo::pathSelected,
      [this](const QString & path) {
        if ( writer_ ) {

          CF_DEBUG("pathSelected: %s", path.toUtf8().constData());

          writer_->setOutputDirectoty(path);
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

  capture_ctl->setCameraWriter(writer_ = writer);

  if( (writer_ = writer) ) {
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

    num_rounds_ctl->setValue(writer_->rounds());
    interval_between_rounds_ctl->setValue(writer_->intervalBetweenRounds());
    outpuPath_ctl->setCurrentPath(writer_->outputDirectoty(), false);

    if ( writer_->state() == QCameraWriter::State::Idle ) {
      num_rounds_ctl->setEnabled(true);
      interval_between_rounds_ctl->setEnabled(true);
      outpuPath_ctl->setEnabled(true);
    }
    else {
      num_rounds_ctl->setEnabled(false);
      interval_between_rounds_ctl->setEnabled(false);
      outpuPath_ctl->setEnabled(false);
    }

    setEnabled(true);
  }
}



} /* namespace serimager */
