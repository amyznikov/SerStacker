/*
 * QInputOptions.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "QInputOptions.h"
#include <gui/widgets/style.h>
#include <core/io/hdl/c_hdl_specification.h>



#define ICON_bayer          ":/gui/icons/bayer.png"
#define ICON_settings       ":/qinputoptions/icons/settings.png"
#define ICON_gps            ":/qinputoptions/icons/gps3.png"
#define ICON_lidar          ":/qinputoptions/icons/lidar64.png"


///////////////////////////////////////////////////////////////////////////////////////////////////

static void init_resources()
{
  Q_INIT_RESOURCE(gui_resources);
  Q_INIT_RESOURCE(qinputoptions_resources);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QVideoInputOptions::QVideoInputOptions(QWidget * parent) :
    Base(parent)
{
  init_resources();

  debayer_ctl =
      add_enum_combobox<DEBAYER_ALGORITHM>("Default debayer:",
          "Select debayer algorithm for bayer patterns",
          [this](DEBAYER_ALGORITHM v) {
            if ( _options && _options->debayer_method != v ) {
              _options->debayer_method = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](DEBAYER_ALGORITHM * v) {
            if ( _options ) {
              *v = _options->debayer_method;
              return true;
            }
            return false;
          });

  enable_color_maxtrix_ctl =
      add_checkbox("Enable Color Matrix",
          "Enable Color Matrix if provided",
          [this](bool checked) {
            if ( _options && _options->enable_color_maxtrix != checked ) {
              _options->enable_color_maxtrix = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _options ) {
              *checked = _options->enable_color_maxtrix;
              return true;
            }
            return false;
          });

  filter_bad_pixels_ctl =
      add_checkbox("Filter Bad Pixels",
          "Enable Detect and Filter Bad Pixels",
          [this](bool checked) {
            if ( _options && _options->filter_bad_pixels != checked ) {
              _options->filter_bad_pixels = checked;
              Q_EMIT parameterChanged();
            }
          },
          [this](bool * checked) {
            if ( _options ) {
              *checked = _options->filter_bad_pixels;
              return true;
            }
            return false;
          });

  bad_pixels_variation_threshold_ctl =
      add_numeric_box<double>("Bad Pixels Variation:",
          "bad_pixels_variation_threshold",
          [this](double v) {
            if ( _options && _options->bad_pixels_variation_threshold != v ) {
              _options->bad_pixels_variation_threshold = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](double * v) {
            if ( _options ) {
              *v = _options->bad_pixels_variation_threshold;
              return true;
            }
            return false;

          });

  updateControls();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QHDLConfigOptions::QHDLConfigOptions(QWidget * parent) :
    Base("", parent)
{
  init_resources();

  QWidget *groupBox =
      add_widget<QWidget>("");

  QVBoxLayout * groupBoxLayout =
      new QVBoxLayout(groupBox);

  QHBoxLayout * tooltipLayout =
      new QHBoxLayout();

  QLabel * iconLabel =
      new QLabel(this);

  QPixmap pxmap =
      getPixmap(ICON_lidar);

  // CF_DEBUG("pxmap: isNull=%d %dx%d", pxmap.isNull(), pxmap.width(), pxmap.height());

  iconLabel->setPixmap(pxmap);

  QLabel *tooltipLabel =
      new QLabel("This page will associate specific sensor type with specific lidar settings xml file. "
          "If not explicitly associated then the builtin (hard coded in application) default settings will used.");
  tooltipLabel->setWordWrap(true);

  tooltipLayout->addWidget(iconLabel, 1, Qt::AlignLeft);
  tooltipLayout->addWidget(tooltipLabel, 4);
  groupBoxLayout->addLayout(tooltipLayout, 1);

  ///

  QHBoxLayout * ctrlLayout =
      new QHBoxLayout();

  ctrlLayout->setAlignment(Qt::AlignTop);

  ctrlLayout->addWidget(sensorType_ctl =
      new QEnumComboBox<HDLSensorType>(this), 1, Qt::AlignLeft);
  sensorType_ctl->setToolTip("Select sensor type to associate with config file");

  ctrlLayout->addWidget(configFilePathName_ctl =
      new QBrowsePathCombo("", QFileDialog::AcceptOpen), 100);

  configFilePathName_ctl->setToolTip("Specify path and file name to lidar config file \n"
      "associated with selected sensor type");

  groupBoxLayout->addLayout(ctrlLayout, 100);

  ///

  connect(sensorType_ctl, &QEnumComboBoxBase::currentItemChanged,
      [this]() {
        configFilePathName_ctl->setCurrentPath(
            get_hdl_lidar_specification_config_file(
                sensorType_ctl->currentItem()).c_str(),
            false);
      });

  connect(configFilePathName_ctl, &QBrowsePathCombo::pathChanged,
      [this]() {

        set_hdl_lidar_specification_config_file(
            sensorType_ctl->currentItem(),
            configFilePathName_ctl->currentPath().toStdString());

        saveHDLSensorTypeToConfigFileMapping();
      });

  configFilePathName_ctl->setCurrentPath(
      get_hdl_lidar_specification_config_file(
          sensorType_ctl->currentItem()).c_str(),
      false);

  ///

//
//  nonlive_streams_delay_ctl =
//      add_numeric_box<int>("Nonlive streams delay [usec]",
//          "",
//          [this](int v ) {
//
//            c_hdl_source::set_nonlive_stream_delay_us(v);
//            nonlive_streams_delay_ctl->setValue(c_hdl_source::nonlive_stream_delay_us());
//
//            saveHDLStreamsGlobalOptions();
//          });
//
//  nonlive_streams_delay_ctl->setToolTip(
//      "Use this parameter to setup the read rate of \n"
//          "pre-recorded non-live streams (PCAP files etc).\n"
//          "This interval specifies usleep() time in [us] between reading packets from pcap sources.\n"
//          "This parameter is ignored for live (UDP) streams");

  updateControls();
}



void loadHDLSensorTypeToConfigFileMapping()
{
  QSettings settings;

  const c_enum_member *sensor_type =
      members_of<HDLSensorType>();

  for( ; !sensor_type->name.empty(); ++sensor_type ) {
    set_hdl_lidar_specification_config_file((HDLSensorType) sensor_type->value,
        settings.value(QString("HDLSensorTypeToConfigFileMapping/%1").arg(sensor_type->name.c_str())).toString().toStdString());
  }
}

void saveHDLSensorTypeToConfigFileMapping()
{
  QSettings settings;

  const c_enum_member *sensor_type =
      members_of<HDLSensorType>();

  for( ; !sensor_type->name.empty(); ++sensor_type ) {
    settings.setValue(QString("HDLSensorTypeToConfigFileMapping/%1").arg(sensor_type->name.c_str()),
        get_hdl_lidar_specification_config_file((HDLSensorType) sensor_type->value).c_str());
  }
}


void loadHDLStreamsGlobalOptions()
{
//  QSettings settings;
//
//  const int delay_us =
//      settings.value("HDLStreamsGlobalOptions/nonlive_streams_delay_us",
//          c_hdl_source::nonlive_stream_delay_us()).toInt();
//
//  c_hdl_source::set_nonlive_stream_delay_us(
//      std::max(10, std::min(delay_us, 100 * 1000)));
}

void saveHDLStreamsGlobalOptions()
{
//  QSettings settings;
//  settings.setValue("HDLStreamsGlobalOptions/nonlive_streams_delay_us",
//      c_hdl_source::nonlive_stream_delay_us());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QInputOptions::QInputOptions(QWidget * parent) :
    Base(parent)
{
  init_resources();

  addRow(tab_ctl = new QTabWidget(this));

  tab_ctl->addTab(videoOptions_ctl = new QVideoInputOptions(this),
      QIcon(), "Video");

  tab_ctl->addTab(hdlconfigOptions_ctl = new QHDLConfigOptions(this),
      QIcon(), "HDL");



  connect(videoOptions_ctl, &QSettingsWidget::parameterChanged,
      this, &ThisClass::parameterChanged);



  updateControls();
}


void QInputOptions::set_options(c_input_options * options)
{
  if ( !(this->_options = options) ) {
    videoOptions_ctl->set_options(nullptr);
  }
  else {
    videoOptions_ctl->set_options(&_options->video);
  }

  updateControls();
}


QInputOptionsDialogBox::QInputOptionsDialogBox(QWidget * parent) :
    Base(parent)
{
  init_resources();

  setWindowTitle("Input Options...");
  setWindowIcon(getIcon(ICON_bayer));

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
