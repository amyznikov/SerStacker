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

  vbox_ = new QVBoxLayout(this);
  vbox_->setContentsMargins(0,0,0,0);


  vbox_->addLayout(h1_ = new QHBoxLayout());
  h1_->setContentsMargins(0, 0, 0, 0);
  h1_->addWidget(limitsSelection_ctl = new QComboBox(this), 1000);
  h1_->addWidget(startStop_ctl = new QToolButton(this), 1);

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

QStereoStreamCaptureOptions::QStereoStreamCaptureOptions(QWidget * parent) :
    Base("QStereoStreamCaptureOptions", parent)
{
  enable_split_stereo_stream_ctl =
      add_checkbox("Split stereo stream:",
          "",
          [this](bool checked) {
            if ( writer_ ) {
              writer_->set_enable_split_stereo_stream(checked);
              update_control_states();
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( writer_ ) {
              * checked = writer_->enable_split_stereo_stream();
              return true;
            }
            return false;
          });

  stereo_stream_layout_type_ctl =
      add_enum_combobox<stereo_stream_layout_type>("stereo frame layout:",
          "",
          [this](stereo_stream_layout_type v) {
            if ( writer_ ) {
              writer_->stereo_stream_options().layout_type = v;
              Q_EMIT parameterChanged();
            }

          },
          [this](stereo_stream_layout_type * v) {
            if ( writer_ ) {
              *v = writer_->stereo_stream_options().layout_type;
              return true;
            }
            return false;
          });

  enable_swap_cameras_ctl =
      add_checkbox("Swap cameras:",
          "",
          [this](bool checked) {
            if ( writer_ ) {
              writer_->stereo_stream_options().swap_cameras = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( writer_ ) {
              * checked = writer_->stereo_stream_options().swap_cameras;
              return true;
            }
            return false;
          });


  downscale_panes_ctl =
      add_checkbox("Downscale panes:",
          "",
          [this](bool checked) {
            if ( writer_ ) {
              writer_->stereo_stream_options().downscale_panes = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( writer_ ) {
              * checked = writer_->stereo_stream_options().downscale_panes;
              return true;
            }
            return false;
          });


  updateControls();
}

void QStereoStreamCaptureOptions::setCameraWriter(QCameraWriter * writer)
{
  writer_ = writer;
  updateControls();
}

QCameraWriter * QStereoStreamCaptureOptions::cameraWriter() const
{
  return writer_;
}

void QStereoStreamCaptureOptions::update_control_states()
{
  stereo_stream_layout_type_ctl->setEnabled(writer_->enable_split_stereo_stream());
  enable_swap_cameras_ctl->setEnabled(writer_->enable_split_stereo_stream());
  downscale_panes_ctl->setEnabled(writer_->enable_split_stereo_stream());
}

void QStereoStreamCaptureOptions::onupdatecontrols()
{
  if ( !writer_  ) {
    setEnabled(false);
  }
  else {

    Base::onupdatecontrols();
    update_control_states();

    setEnabled(true);
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
          "",
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
          "",
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

  connect(outpuPath_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {
        if ( writer_ ) {
          writer_->setOutputDirectoty(outpuPath_ctl->currentPath());
          save_parameter(PREFIX, "outputDirectoty",
              writer_->outputDirectoty());
        }
      });

  output_format_ctl =
      add_enum_combobox<QCameraWriter::FORMAT>("Output format:",
          "",
          [this](QCameraWriter::FORMAT value) {
            if ( writer_ ) {
              writer_->setOutputFormat(value);
              avi_options_ctl->setEnabled(writer_->state() == QCameraWriter::State::Idle &&
                  writer_->outputFormat() == QCameraWriter::FORMAT::AVI);
              save_parameter(PREFIX, "outputFormat",
                  writer_->outputFormat());
            }
          });

  avi_options_ctl =
      add_textbox("ffmpeg options:",
          "",
          [this](const QString & value) {
            if ( writer_ ) {
              writer_->setFFmpegOptions(value);
              save_parameter(PREFIX, "ffmpegOptions",
                  writer_->ffmpegOptions());
            }
          });

  avi_options_menubutton_ctl = new QToolButton();
  avi_options_menubutton_ctl->setText("...");
  avi_options_ctl->layout()->addWidget(avi_options_menubutton_ctl);
  connect(avi_options_menubutton_ctl, &QToolButton::clicked,
      this, &ThisClass::onAviOptionsMenuButtonClicked);

  filenamePrefix_ctl =
      add_textbox("Prefix:",
          "Optional output file name prefix",
          [this](const QString & value) {
            if ( writer_ ) {
              writer_->setFilenamePrefix(value);
              save_parameter(PREFIX, "filenamePrefix",
                  writer_->filenamePrefix());
            }
          });

  filenameSuffix_ctl =
      add_textbox("Suffix:",
          "Optional output file name suffix",
          [this](const QString & value) {
            if ( writer_ ) {
              writer_->setFilenameSuffix(value);
              save_parameter(PREFIX, "filenameSuffix",
                  writer_->filenameSuffix());
            }
          });

  add_expandable_groupbox("Stereo",
      stereo_stream_ctl = new QStereoStreamCaptureOptions());

  updateControls();
}


void QCaptureSettingsWidget::onAviOptionsMenuButtonClicked()
{
  QMenu menu;
  QAction * action = nullptr;


  menu.addAction(action = new QAction("List available encoders..."));
  connect(action, &QAction::triggered,
      []() {

        static QPlainTextEdit * codecListTextBox = nullptr;

        if ( !codecListTextBox ) {

          const std::vector<std::string> & supported_encoders =
              c_ffmpeg_writer::supported_encoders();

          QString text = "Tags:\n"
              "I - AV_CODEC_PROP_INTRA_ONLY, Codec uses only intra compression.\n"
              "L - AV_CODEC_PROP_LOSSY, Codec supports lossy compression. A codec may support both lossy and lossless compression modes\n"
              "S - AV_CODEC_PROP_LOSSLESS, Codec supports lossless compression. \n"
              "\n"
              "";

          for ( const auto & s : supported_encoders ) {
            text.append(s.c_str());
            text.append("\n");
          }


          codecListTextBox = new QPlainTextEdit(text, QApplication::activeWindow());
          codecListTextBox->setWindowFlag(Qt::WindowType::Dialog);
          codecListTextBox->setFont(QFont("Monospace", 14));
          codecListTextBox->setWordWrapMode(QTextOption::WrapMode::NoWrap);
          codecListTextBox->setWindowTitle("Supported ffmpeg video encoders");
        }


        codecListTextBox->show();
        codecListTextBox->raise();
        codecListTextBox->setFocus();
      });


  menu.exec(avi_options_menubutton_ctl->mapToGlobal(
      QPoint(avi_options_menubutton_ctl->width() / 2,
          avi_options_menubutton_ctl->height() / 2)));


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
  stereo_stream_ctl->setCameraWriter(writer_);

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

    output_format_ctl->setValue(writer_->outputFormat());
    output_format_ctl->setEnabled(writer_->state() == QCameraWriter::State::Idle);

    avi_options_ctl->setValue(writer_->ffmpegOptions());
    avi_options_ctl->setEnabled(writer_->state() == QCameraWriter::State::Idle &&
        writer_->outputFormat() == QCameraWriter::FORMAT::AVI);

    filenamePrefix_ctl->setValue(writer_->filenamePrefix());
    filenameSuffix_ctl->setValue(writer_->filenameSuffix());

    setEnabled(true);
  }
}

void QCaptureSettingsWidget::onload(QSettings & settings)
{
  if( writer_ ) {

    QCameraWriter::FORMAT outputFormat = writer_->outputFormat();
    if( load_parameter(settings, PREFIX, "outputFormat", &outputFormat) ) {
      writer_->setOutputFormat(outputFormat);
    }

    QString outputDirectoty = writer_->outputDirectoty();
    if( load_parameter(settings, PREFIX, "outputDirectoty", &outputDirectoty) ) {
      writer_->setOutputDirectoty(outputDirectoty);
    }

    QString filenamePrefix = writer_->filenamePrefix();
    if( load_parameter(settings, PREFIX, "filenamePrefix", &filenamePrefix) ) {
      writer_->setFilenamePrefix(filenamePrefix);
    }

    QString filenameSuffix = writer_->filenameSuffix();
    if( load_parameter(settings, PREFIX, "filenameSuffix", &filenameSuffix) ) {
      writer_->setFilenameSuffix(filenameSuffix);
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
