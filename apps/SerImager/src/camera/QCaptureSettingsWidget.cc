/*
 * QCaptureSettingsWidget.cc
 *
 *  Created on: Dec 29, 2022
 *      Author: amyznikov
 */

#include "QCaptureSettingsWidget.h"
#include <gui/widgets/style.h>
#include <core/io/c_ffmpeg_file.h>

#define ICON_start_capture        ":/serimager/icons/start-capture.png"
#define ICON_stop_capture         ":/serimager/icons/stop-capture.png"


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

  _vbox = new QVBoxLayout(this);
  _vbox->setContentsMargins(0,0,0,0);


  _vbox->addLayout(_h1 = new QHBoxLayout());
  _h1->setContentsMargins(0, 0, 0, 0);
  _h1->addWidget(limitsSelection_ctl = new QComboBox(this), 1000);
  _h1->addWidget(startStop_ctl = new QToolButton(this), 1);

  limitsSelection_ctl->setEditable(false);
  limitsSelection_ctl->setFocusPolicy(Qt::StrongFocus);

  startStop_ctl->setToolTip("Start / Stop capture (Ctrl + W)");
  startStop_ctl->setToolButtonStyle(Qt::ToolButtonIconOnly);
  startStop_ctl->setIcon(icon_start_capture);

#if QT_VERSION > QT_VERSION_CHECK(5, 16, 0)
  startStopSortuct_ =
      new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_W), this,
          this, &ThisClass::onStartStopButtonClicked,
          Qt::WindowShortcut);
#endif


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
          .value = 600
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 1000
      },
      {
          .type = c_capture_limits::ByNumberOfFrames,
          .value = 1200
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
          .value = 3600
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
  const int cursel = limitsSelection_ctl->currentIndex();
  if( cursel >= 0 ) {
    *c = limitsSelection_ctl->itemData(cursel).value<c_capture_limits>();
    return true;
  }

  return false;
}

bool QCaptureLimitsControl::setSelectedCaptureLimits(const c_capture_limits & c)
{
  int index = limitsSelection_ctl->findData(QVariant::fromValue(c));
  if ( index >= 0 ) {
    QSignalBlocker block(limitsSelection_ctl);
    limitsSelection_ctl->setCurrentIndex(index);
    return true;
  }
  return false;
}

QCameraWriter * QCaptureLimitsControl::cameraWriter() const
{
  return _writer;
}


void QCaptureLimitsControl::setCameraWriter(QCameraWriter * writer)
{
  if ( _writer ) {
    disconnect(writer, nullptr, this, nullptr);
  }

  if( (_writer = writer) ) {

    c_capture_limits c;

    if ( getSelectedCaptureLimits(&c) ) {
      _writer->setCaptureLimits(c);
    }

    connect(_writer, &QCameraWriter::stateChanged,
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
  if( !_writer ) {
    setEnabled(false);
  }
  else {
    switch (_writer->state()) {
      case QCameraWriter::State::Active:
      case QCameraWriter::State::Starting:
      case QCameraWriter::State::Stopping:
        enableControl(startStop_ctl, true, icon_stop_capture);
        enableControl(limitsSelection_ctl, false);
        break;
      case QCameraWriter::State::Idle:
        enableControl(limitsSelection_ctl, true);
        if( !_writer->camera() ) {
          enableControl(startStop_ctl, false, icon_start_capture);
        }
        else {
          switch (_writer->camera()->state()) {
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
  if( _writer ) {

    switch (_writer->state()) {

      case QCameraWriter::State::Active:
        _writer->stop();
        break;

      case QCameraWriter::State::Idle: {
        c_capture_limits c;
        if( getSelectedCaptureLimits(&c) ) {
          _writer->setCaptureLimits(c);
        }
        _writer->start();
        break;
      }
    }
  }
}

void QCaptureLimitsControl::onLimitsSelectionChanged(int)
{
  if( _writer && _writer->state() == QCameraWriter::Idle ) {
    c_capture_limits c;
    if( getSelectedCaptureLimits(&c) ) {
      _writer->setCaptureLimits(c);
      Q_EMIT captureLimitChanged();
    }
  }
}

QStereoStreamCaptureOptions::QStereoStreamCaptureOptions(QWidget * parent) :
    Base(parent)
{
  enable_split_stereo_stream_ctl =
      add_checkbox("Split stereo stream:",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->set_enable_split_stereo_stream(checked);
              //update_control_states();
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->enable_split_stereo_stream();
              return true;
            }
            return false;
          });

  stereo_stream_layout_type_ctl =
      add_enum_combobox<stereo_stream_layout_type>("stereo frame layout:",
          "",
          [this](stereo_stream_layout_type v) {
            if ( _opts ) {
              _opts->stereo_stream_options().layout_type = v;
              Q_EMIT parameterChanged();
            }

          },
          [this](stereo_stream_layout_type * v) {
            if ( _opts ) {
              *v = _opts->stereo_stream_options().layout_type;
              return true;
            }
            return false;
          });

  enable_swap_cameras_ctl =
      add_checkbox("Swap cameras:",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->stereo_stream_options().swap_cameras = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->stereo_stream_options().swap_cameras;
              return true;
            }
            return false;
          });


  downscale_panes_ctl =
      add_checkbox("Downscale panes:",
          "",
          [this](bool checked) {
            if ( _opts ) {
              _opts->stereo_stream_options().downscale_panes = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _opts ) {
              * checked = _opts->stereo_stream_options().downscale_panes;
              return true;
            }
            return false;
          });

  QObject::connect(this, &ThisClass::enablecontrols,
      [this]() {
        const bool enable_split_stereo_stream = _opts && _opts->enable_split_stereo_stream();
        stereo_stream_layout_type_ctl->setEnabled(enable_split_stereo_stream);
        enable_swap_cameras_ctl->setEnabled(enable_split_stereo_stream);
        downscale_panes_ctl->setEnabled(enable_split_stereo_stream);
      });

  updateControls();
}

void QStereoStreamCaptureOptions::setCameraWriter(QCameraWriter * writer)
{
  setOpts(writer);
}

QCameraWriter * QStereoStreamCaptureOptions::cameraWriter() const
{
  return _opts;
}


QCaptureSettingsWidget::QCaptureSettingsWidget(QWidget * parent) :
    Base(parent)
{
  form->setLabelAlignment(Qt::AlignLeft);

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( _opts ) {
          captureLimits_ctl->setSelectedCaptureLimits(_opts->captureLimits());
        }
    });

  connect(this, &ThisClass::enablecontrols,
      [this]() {
        if ( _opts ) {
          const bool is_unlimited = _opts->captureLimits().value < 0;
          num_rounds_ctl->setEnabled(!is_unlimited);
          interval_between_rounds_ctl->setEnabled(!is_unlimited);
          outpuPath_ctl->setEnabled(_opts->state() == QCameraWriter::State::Idle);
          output_format_ctl->setEnabled(_opts->state() == QCameraWriter::State::Idle);
          avi_options_ctl->setEnabled(_opts->state() == QCameraWriter::State::Idle && _opts->outputFormat() == QCameraWriter::FORMAT::AVI);
        }
  });

  captureLimits_ctl =
      add_widget<QCaptureLimitsControl>("Limit:");

  connect(captureLimits_ctl, &QCaptureLimitsControl::captureLimitChanged,
      [this]() {
        updateControls();
        saveSettings();
      });

  num_rounds_ctl =
      add_spinbox("Rounds:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->setNumRounds(value);
              saveSettings();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->numRounds();
              return true;
            }
            return false;
          });

  num_rounds_ctl->setRange(1, 10000);
  num_rounds_ctl->setValue(1);

  interval_between_rounds_ctl =
      add_spinbox("Interval [sec]:",
          "",
          [this](int value) {
            if ( _opts ) {
              _opts->setIntervalBetweenRounds(value);
              saveSettings();
            }
          },
          [this](int * value) {
            if ( _opts ) {
              * value = _opts->intervalBetweenRounds();
              return true;
            }
            return false;
          });

  interval_between_rounds_ctl->setRange(0, 3600);
  num_rounds_ctl->setValue(1);

  outpuPath_ctl =
      add_browse_for_path("Output path:",
          "",
          QFileDialog::AcceptSave,
          QFileDialog::Directory,
          [this](const QString & path) {
            if ( _opts ) {
              _opts->setOutputDirectoty(path);
              saveSettings();
            }
          },
          [this](QString * path) {
            if ( _opts ) {
              *path = _opts->outputDirectoty();
              return true;
            }
            return false;
          });

  output_format_ctl =
      add_enum_combobox<QCameraWriter::FORMAT>("Output format:",
          "",
          [this](QCameraWriter::FORMAT value) {
            if ( _opts ) {
              _opts->setOutputFormat(value);
              avi_options_ctl->setEnabled(_opts->state() == QCameraWriter::State::Idle &&
                  _opts->outputFormat() == QCameraWriter::FORMAT::AVI);
              saveSettings();
            }
          },
          [this](QCameraWriter::FORMAT * value) {
            if ( _opts ) {
              * value  = _opts->outputFormat();
              return true;
            }
            return false;
          });

  avi_options_ctl =
      add_ffmpeg_options_control("ffmpeg options:",
          "",
          [this](const QString & value) {
            if ( _opts ) {
              _opts->setFFmpegOptions(value);
              saveSettings();
            }
          },
          [this](QString * value) {
            if ( _opts ) {
              * value = _opts->ffmpegOptions();
              return true;
            }
            return false;
          });

  filenamePrefix_ctl =
      add_textbox("Prefix:",
          "Optional output file name prefix",
          [this](const QString & value) {
            if ( _opts ) {
              _opts->setFilenamePrefix(value);
              saveSettings();
            }
          },
          [this](QString * value) {
            if ( _opts ) {
              * value = _opts->filenamePrefix();
              return true;
            }
            return false;
          });

  filenameSuffix_ctl =
      add_textbox("Suffix:",
          "Optional output file name suffix",
          [this](const QString & value) {
            if ( _opts ) {
              _opts->setFilenameSuffix(value);
              saveSettings();
            }
          },
          [this](QString * value) {
            if ( _opts ) {
              * value = _opts->filenameSuffix();
              return true;
            }
            return false;
          });

  add_expandable_groupbox("Stereo",
      stereo_stream_ctl = new QStereoStreamCaptureOptions());

  updateControls();
}


void QCaptureSettingsWidget::setCameraWriter(QCameraWriter * writer)
{
  if ( _opts ) {
    disconnect(_opts, nullptr, this, nullptr);
  }

  _opts = writer;

  captureLimits_ctl->setCameraWriter(_opts);
  stereo_stream_ctl->setCameraWriter(_opts);

  if( _opts ) {
    connect(_opts, &QCameraWriter::stateChanged,
        this, &ThisClass::updateControls);
  }

  updateControls();
}

QCameraWriter * QCaptureSettingsWidget::cameraWriter() const
{
  return _opts;
}

void QCaptureSettingsWidget::onload(const QSettings & settings, const QString & prefix)
{
  if( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "CaptureSettings" : prefix;

    QCameraWriter::FORMAT outputFormat = _opts->outputFormat();
    if( load_parameter(settings, PREFIX, "outputFormat", &outputFormat) ) {
      _opts->setOutputFormat(outputFormat);
    }

    QString outputDirectoty = _opts->outputDirectoty();
    if( load_parameter(settings, PREFIX, "outputDirectoty", &outputDirectoty) ) {
      _opts->setOutputDirectoty(outputDirectoty);
    }

    QString filenamePrefix = _opts->filenamePrefix();
    if( load_parameter(settings, PREFIX, "filenamePrefix", &filenamePrefix) ) {
      _opts->setFilenamePrefix(filenamePrefix);
    }

    QString filenameSuffix = _opts->filenameSuffix();
    if( load_parameter(settings, PREFIX, "filenameSuffix", &filenameSuffix) ) {
      _opts->setFilenameSuffix(filenameSuffix);
    }

    int numRounds = _opts->numRounds();
    if( load_parameter(settings, PREFIX, "numRounds", &numRounds) ) {
      _opts->setNumRounds(numRounds);
    }

    int intervalBetweenRounds = _opts->intervalBetweenRounds();
    if( load_parameter(settings, PREFIX, "intervalBetweenRounds", &intervalBetweenRounds) ) {
      _opts->setIntervalBetweenRounds(intervalBetweenRounds);
    }

    c_capture_limits limits = _opts->captureLimits();
    load_parameter(settings, PREFIX, "capture_limits_type", (int*)&limits.type);
    load_parameter(settings, PREFIX, "capture_limits_value", &limits.value);
    _opts->setCaptureLimits(limits);
    updateControls();
  }
}

void QCaptureSettingsWidget::onsave(QSettings & settings, const QString & prefix)
{
  if( _opts ) {

    const QString PREFIX = prefix.isEmpty() ? "CaptureSettings" : prefix;

    save_parameter(settings, PREFIX, "outputFormat",_opts->outputFormat());
    save_parameter(settings, PREFIX, "outputDirectoty", _opts->outputDirectoty());
    save_parameter(settings, PREFIX, "filenamePrefix",_opts->filenamePrefix());
    save_parameter(settings, PREFIX, "filenameSuffix",_opts->filenameSuffix());
    save_parameter(settings, PREFIX, "numRounds",_opts->numRounds());
    save_parameter(settings, PREFIX, "intervalBetweenRounds",_opts->intervalBetweenRounds());

    const c_capture_limits limits = _opts->captureLimits();
    save_parameter(settings, PREFIX, "capture_limits_type", (int)limits.type);
    save_parameter(settings, PREFIX, "capture_limits_value", limits.value);
  }
}

} /* namespace serimager */
