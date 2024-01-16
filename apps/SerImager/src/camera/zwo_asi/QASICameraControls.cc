/*
 * QASICameraSettingsWidget.cc
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#include "QASICameraControls.h"
#include <gui/widgets/settings.h>
#include <core/debug.h>

namespace serimager {

///////////////////////////////////////////////////////////////////////////////////////////////////

QASIControlWidget::QASIControlWidget(int CameraID, const ASI_CONTROL_CAPS & ControlCaps, QWidget * parent) :
    Base(parent),
    controlCaps_(ControlCaps),
    iCameraID(CameraID)
{
  setFocusPolicy(Qt::FocusPolicy::StrongFocus);

  bool enable_slider_ctl = false;
  bool enable_scale_ctl = false;

  switch (ControlCaps.ControlType) {
    case ASI_EXPOSURE:
      enable_slider_ctl = true;
      enable_scale_ctl = true;
      break;

    case ASI_GAIN:
    case ASI_GAMMA:
    case ASI_WB_R:
    case ASI_WB_B:
    case ASI_OFFSET:
      enable_slider_ctl = true;
      break;
    default:
      break;
  }

  if( enable_slider_ctl ) {
    lv_ = new QVBoxLayout(this);
    lv_->addLayout(lh_ = new QHBoxLayout());
  }
  else {
    lh_ = new QHBoxLayout(this);
  }

  lh_->addWidget(value_ctl = new QSpinBox(this), 100);
  if ( enable_scale_ctl ) {
    lh_->addWidget(expscale_ctl = new QComboBox(this), 1);
  }
  if ( controlCaps_.IsAutoSupported ) {
    lh_->addWidget(auto_ctl = new QCheckBox("auto", this), 1);
  }
  if ( enable_slider_ctl ) {
    lv_->addWidget(slider_ctl = new QSlider(this));
  }

  value_ctl->setMouseTracking(false);
  value_ctl->setKeyboardTracking(false);
  value_ctl->setTabletTracking(false);
  value_ctl->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

  value_ctl->setToolTip(QString("%1\nMin=%2 Max=%3")
      .arg(controlCaps_.Description)
      .arg(controlCaps_.MinValue)
      .arg(controlCaps_.MaxValue));

  connect(value_ctl, SIGNAL(valueChanged(int)),
      this, SLOT(onValueCtlChanged(int)),
      Qt::QueuedConnection);

  if( auto_ctl ) {
    connect(auto_ctl, &QCheckBox::stateChanged,
        this, &ThisClass::onAutoCtlStateChanged,
        Qt::QueuedConnection);
  }

  if ( slider_ctl ) {

    slider_ctl->setOrientation(Qt::Horizontal);
    slider_ctl->setTracking(false);
    slider_ctl->setMouseTracking(false);
    slider_ctl->setTabletTracking(false);
    slider_ctl->setFocusPolicy(Qt::FocusPolicy::StrongFocus);

    slider_ctl->setToolTip(QString("%1\nMin=%2 Max=%3")
        .arg(controlCaps_.Description)
        .arg(controlCaps_.MinValue)
        .arg(controlCaps_.MaxValue));

    connect(slider_ctl, SIGNAL(valueChanged(int)),
        this, SLOT(onSliderCtlChanged(int)),
        Qt::QueuedConnection);
  }

  if ( expscale_ctl ) {

    expscale_ctl->setEditable(false);


    expscale_ctl->addItem("us ", QVariant::fromValue(QASIExposureScaleParams(controlCaps_.MinValue, 999, 1)));
    expscale_ctl->addItem("ms ", QVariant::fromValue(QASIExposureScaleParams(1, 999, 1000)));
    expscale_ctl->addItem("sec", QVariant::fromValue(QASIExposureScaleParams(1, controlCaps_.MaxValue / 1000000, 1000000)));
    expscale_ctl->setCurrentIndex(1);

    connect(expscale_ctl, SIGNAL(currentIndexChanged(int)),
        this, SLOT(onExpScaleCtlChanged(int)),
        Qt::QueuedConnection);

  }

  updateCtrlRange();
  updateControls();
}

void QASIControlWidget::onupdatecontrols()
{
  c_update_controls_lock lock(this);

  long lValue = 0;
  ASI_BOOL isAuto = ASI_TRUE;

  if ( !getASIControlValue(&lValue, &isAuto) ) {
    CF_ERROR("getASIControlValue() fails");
    return;
  }

  if( expscale_ctl ) {

    for( int i = 0, n = expscale_ctl->count(); i < n; ++i ) {

      const QASIExposureScaleParams p =
          expscale_ctl->itemData(i).value<QASIExposureScaleParams>();

      const long v = lValue / p.scale;

      if( v >= p.minValue && v <= p.maxValue ) {

        lValue = v;

        if ( expscale_ctl->currentIndex() != i ) {
          expscale_ctl->setCurrentIndex(i);
          updateCtrlRange();
        }

        break;
      }
    }
  }

  if ( value_ctl ) {
    value_ctl->setEnabled(!controlCaps_.IsAutoSupported || !isAuto);
    if ( value_ctl->value() != lValue ) {
      value_ctl->setValue(lValue);
    }
  }

  if ( slider_ctl ) {
    slider_ctl->setEnabled(!controlCaps_.IsAutoSupported || !isAuto);
    if ( slider_ctl->value() != lValue ) {
      slider_ctl->setValue(lValue);
    }
  }


  if ( auto_ctl ) {
    auto_ctl->setChecked(isAuto);
  }

}

void QASIControlWidget::onValueCtlChanged(int)
{
  if( !updatingControls() ) {

    const ASI_BOOL isAuto =
        (auto_ctl && auto_ctl->isChecked()) ? ASI_TRUE : ASI_FALSE;

    const long lValue =
        value_ctl->value();

    setASIControlValue(lValue, isAuto);

    setUpdatingControls(true);
    onupdatecontrols();
    setUpdatingControls(false);
  }
}

void QASIControlWidget::onSliderCtlChanged(int)
{
  if( !updatingControls() ) {

    const ASI_BOOL isAuto =
        (auto_ctl && auto_ctl->isChecked()) ? ASI_TRUE : ASI_FALSE;

    const long lValue =
        slider_ctl->value();

    setASIControlValue(lValue, isAuto);

    setUpdatingControls(true);
    onupdatecontrols();
    setUpdatingControls(false);

  }
}

void QASIControlWidget::onAutoCtlStateChanged(int )
{
  if( !updatingControls() ) {

    const ASI_BOOL isAuto =
        (auto_ctl && auto_ctl->isChecked()) ? ASI_TRUE : ASI_FALSE;

    if ( auto_ctl && !isAuto ) {

      ASI_BOOL bAuto = ASI_TRUE;
      long lValue = 0;

      ASI_ERROR_CODE status =
          ASIGetControlValue(iCameraID,
              controlCaps_.ControlType,
              &lValue,
              &bAuto);

      if ( status == ASI_SUCCESS ) {

        c_update_controls_lock lock(this);

        value_ctl->setValue(lValue);

        if ( slider_ctl ) {
          slider_ctl->setValue(lValue);
        }

      }
    }

    value_ctl->setEnabled(!isAuto);
    if ( slider_ctl ) {
      slider_ctl->setEnabled(!isAuto);
    }

    setASIControlValue(value_ctl->value(), isAuto);
  }
}

void QASIControlWidget::onExpScaleCtlChanged(int)
{
  if( !updatingControls() ) {

    c_update_controls_lock lock(this);

    long lValue = 0;
    ASI_BOOL isAuto = ASI_TRUE;

    if ( !getASIControlValue(&lValue, &isAuto) ) {
      CF_ERROR("getASIControlValue() fails");
      return;
    }

    const QASIExposureScaleParams p =
        expscale_ctl->currentData().value<QASIExposureScaleParams>();

    long newValue =
        lValue / p.scale;

    if( newValue < p.minValue ) {
      newValue = p.minValue;
    }

    if( newValue > p.maxValue ) {
      newValue = p.maxValue;
    }

    if( value_ctl ) {
      if( value_ctl->maximum() != p.maxValue || value_ctl->minimum() != p.minValue ) {
        value_ctl->setRange(p.minValue, p.maxValue);
      }
    }

    if( slider_ctl  ) {
      if( slider_ctl->maximum() != p.maxValue || slider_ctl->minimum() != p.minValue ) {
        slider_ctl->setRange(p.minValue, p.maxValue);
      }
    }

    if( (newValue *= p.scale) != lValue ) {

      ASI_ERROR_CODE status =
          ASISetControlValue(iCameraID,
              controlCaps_.ControlType,
              newValue,
              isAuto);

      if( status != ASI_SUCCESS ) {
        CF_ERROR("ASISetControlValue(CameraID=%d, '%s' newValue=%ld isAuto=%d) fails: %d (%s)",
            iCameraID,
            controlCaps_.Name,
            newValue,
            isAuto,
            status,
            toString(status));
      }

      onupdatecontrols();
    }
  }
}

void QASIControlWidget::updateCtrlRange()
{
  c_update_controls_lock lock(this);

  int minValue =
      controlCaps_.MinValue;

  int maxValue =
      controlCaps_.MaxValue;

  if ( expscale_ctl ) {

    const QASIExposureScaleParams p =
        expscale_ctl->currentData().value<QASIExposureScaleParams>();

    if ( p.scale > 0 ) {
      minValue = p.minValue;
      maxValue = p.maxValue;
    }
  }

  if( value_ctl ) {
    if( value_ctl->maximum() != maxValue || value_ctl->minimum() != minValue ) {
      value_ctl->setRange(minValue, maxValue);
    }
  }

  if( slider_ctl  ) {
    if( slider_ctl->maximum() != maxValue || slider_ctl->minimum() != minValue ) {
      slider_ctl->setRange(minValue, maxValue);
    }
  }
}

void QASIControlWidget::setASIControlValue(long lValue, ASI_BOOL isAuto)
{
  if ( expscale_ctl ) {

    const QASIExposureScaleParams p =
        expscale_ctl->currentData().value<QASIExposureScaleParams>();

    if ( p.scale > 0 ) {
      lValue *= p.scale;
    }
  }

  ASI_ERROR_CODE status =
      ASISetControlValue(iCameraID,
          controlCaps_.ControlType,
          lValue,
          isAuto);

  if( status != ASI_SUCCESS ) {
    CF_ERROR("ASISetControlValue(CameraID=%d, '%s' lValue=%ld isAuto=%d) fails: %d (%s)",
        iCameraID,
        controlCaps_.Name,
        lValue,
        isAuto,
        status,
        toString(status));

    onupdatecontrols();
  }

}

bool QASIControlWidget::getASIControlValue(long * lValue, ASI_BOOL * isAuto)
{
  ASI_ERROR_CODE status =
      ASIGetControlValue(iCameraID,
          controlCaps_.ControlType,
          lValue,
          isAuto);

  if( status != ASI_SUCCESS ) {
    CF_ERROR("ASIGetControlValue(CameraID=%d, '%s') fails: %d (%s)",
        iCameraID,
        controlCaps_.Name,
        status,
        toString(status));

    setEnabled(false);
    return false;
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

static QList<QCameraROI> loadUserDefinedCameraROIs()
{
  static const QString keyName =
      "CameraROIs";

  return QSettings().value(keyName,
      QVariant::fromValue(QList<QCameraROI>()))
      .value<QList<QCameraROI>>();
}

static void saveUserDefinedCameraROIs(const QList<QCameraROI> & list)
{
  static const QString keyName =
      "CameraROIs";

  QSettings settings;

  settings.setValue(keyName,
      QVariant::fromValue(list));
}


QASIROIControlWidget::QASIROIControlWidget(const QASICamera::sptr & camera, QWidget * parent) :
  Base(parent),
  camera_(camera)
{
  layout_ = new QHBoxLayout(this);

  layout_->addWidget(frameSize_ctl = new QComboBox(this), 100);
  layout_->addWidget(imageFormat_ctl = new QComboBox(this), 1);
  layout_->addWidget(binning_ctl = new QComboBox(this), 1);

  frameSize_ctl->setEditable(false);
  frameSize_ctl->setToolTip("Set the ROI area before capture.\n"
      "You must stop capture before call it.\n"
      "The width and height is the value after binning,\n"
      "  ie. you need to set width to 640 and height to 480 if you want to run at 640X480@BIN2.\n"
      "\n"
      "Make sure Width % 8 == 0, Height % 2 == 0.\n"
      "Specially, ASI120's data size must be times of 1024 which means width*height % 1024 = 0.\n"
      "Further, for USB2.0 camera ASI120, please make sure that width*height % 1024 = 0.\n");

  imageFormat_ctl->setEditable(false);
  imageFormat_ctl->setToolTip("The output format you want");

  binning_ctl->setEditable(false);
  binning_ctl->setToolTip("Binning method");

  connect(frameSize_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onFrameSizeCtlCurrentIndexChanged);

  prestartproc_ =
      [this]() {
        return setup_camera_capture_format();
      };

  if( camera_ ) {

    camera_->addprestartproc(&prestartproc_);

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged/*,
        Qt::QueuedConnection*/);

    if ( camera_->state() >= QImagingCamera::State_connected ) {
      populate_supported_frame_formats();
    }
  }

  updateControls();
}

QASIROIControlWidget::~QASIROIControlWidget()
{
  if ( camera_ ) {
    camera_->removeprestartproc(&prestartproc_);
  }
}

bool QASIROIControlWidget::setup_camera_capture_format()
{
  if( camera_ ) {

    switch (camera_->state()) {
      case QImagingCamera::State_connected:
        case QImagingCamera::State_starting: {

        const int currentFrameSizeIndex =
            frameSize_ctl->currentIndex();
        if( currentFrameSizeIndex < 0 ) {
          CF_ERROR("No current item selected in frameSize_ctl");
          return false;
        }

        const int currentRoiIndex =
            frameSize_ctl->itemData(currentFrameSizeIndex).value<int>();
        if( currentRoiIndex < 0 ) {
          CF_ERROR("Invalid currentRoiIndex selected");
          return false;
        }

        const QCameraROI &roi =
            currentRoiIndex < predefinedROIs_.size() ?
                predefinedROIs_[currentRoiIndex] :
                userDefinedROIs_[currentRoiIndex - predefinedROIs_.size()];

        ASI_IMG_TYPE iFormat =
            (ASI_IMG_TYPE) imageFormat_ctl->currentData().value<int>();

        int iBin =
            binning_ctl->currentData().value<int>();

        int iX = roi.rect.x();
        int iY = roi.rect.y();
        int iWidth = roi.rect.width();
        int iHeight = roi.rect.height();

        if( !set_capture_format(iX, iY, iWidth, iHeight, iBin, iFormat) ) {

          CF_ERROR("set_capture_format(iX=%d, iY=%d, iWidth=%d, iHeight=%d, iBin=%d, iFormat=%d) fails",
              iX, iY, iWidth, iHeight, iBin, iFormat);

          return false;
        }

        get_capture_format(&iX, &iY, &iWidth, &iHeight, &iBin, &iFormat);

        CF_DEBUG("Capture Format: iX=%d iY=%d iWidth=%d iHeight=%d, iBin=%d iFormat=%d",
            iX, iY, iWidth, iHeight, iBin, iFormat);

        break;
      }

      default:
        break;
    }
  }

  return true;
}


void QASIROIControlWidget::onCameraStateChanged(QImagingCamera::State oldState, QImagingCamera::State newState)
{
  if( camera_ ) {
    CF_DEBUG("%s -> %s", toString(oldState), toString(newState));
    if( oldState < QImagingCamera::State_connected && newState == QImagingCamera::State_connected ) {
      populate_supported_frame_formats();
    }
  }

  updateControls();
}



void QASIROIControlWidget::populate_supported_frame_formats()
{
  c_update_controls_lock lock(this);

  imageFormat_ctl->clear();
  binning_ctl->clear();
  frameSize_ctl->clear();

  if ( camera_ ) {

    const ASI_CAMERA_INFO &c =
        camera_->cameraInfo();

    // iFormat

    for( int i = 0; i < sizeof(c.SupportedVideoFormat) / sizeof(c.SupportedVideoFormat[0]); ++i ) {

      const ASI_IMG_TYPE iFormat =
          c.SupportedVideoFormat[i];

      if( iFormat == ASI_IMG_END ) {
        break;
      }

      imageFormat_ctl->addItem(toQString(iFormat),
          QVariant::fromValue((int) iFormat));
    }


    // iBin
    for( int i = 0; i < sizeof(c.SupportedBins) / sizeof(c.SupportedBins[0]); ++i ) {

      const int iBin =
          c.SupportedBins[i];

      if ( iBin <= 0 ) {
        break;
      }

      binning_ctl->addItem(QString("Bin %1").arg(iBin),
          QVariant::fromValue(iBin));
    }

    // Frame Sizes
    const QSize predefinedFrameSizes[] = {
        QSize(c.MaxWidth, c.MaxHeight),
        QSize(3840, 2160),
        QSize(2560, 2560),
        QSize(2560, 2048),
        QSize(2560, 1140),
        QSize(2048, 2048),
        QSize(2048, 1536),
        QSize(1920, 1080),
        QSize(1600, 1600),
        QSize(1600, 1024),
        QSize(1600, 900),
        QSize(1360, 768),
        QSize(1280, 720),
        QSize(1024, 1024),
        QSize(1024, 768),
        QSize(800, 800),
        QSize(800, 600),
        QSize(640, 480),
        QSize(512, 512),
        QSize(320, 240),
        QSize(256, 256),
        QSize(160, 120),
    };

    // Make sure Width % 8 == 0, Height % 2 == 0

    const QSize maxSize =
        QSize((c.MaxWidth) & ~0x7, (c.MaxHeight) & ~0x1);

    predefinedROIs_.clear();
    for( uint i = 0; i < sizeof(predefinedFrameSizes) / sizeof(predefinedFrameSizes[0]); ++i ) {

      const QSize &size = predefinedFrameSizes[i];
      const QSize s(size.width() & ~0x7, size.height() & ~0x1);

      if( s.width() <= maxSize.width() && s.height() <= maxSize.height() ) {
        predefinedROIs_.append(QCameraROI("", QRect(-1, -1, s.width(), s.height())));
      }
    }

    const QList<QCameraROI> userDefinedCameraROIs =
        loadUserDefinedCameraROIs();

    userDefinedROIs_.clear();
    for( int i = 0, n = userDefinedCameraROIs.size(); i < n; ++i ) {
      userDefinedROIs_.append(userDefinedCameraROIs[i]);
    }


    update_supported_frame_sizes_combobox();
  }

}


void QASIROIControlWidget::update_supported_frame_sizes_combobox()
{
  c_update_controls_lock lock(this);
  frameSize_ctl->clear();

  for( int i = 0, n = predefinedROIs_.size(); i < n; ++i ) {
    frameSize_ctl->addItem(predefinedROIs_[i].toQString(),
          QVariant::fromValue(i));
  }

  for( int i = 0, n = userDefinedROIs_.size(); i < n; ++i ) {
    frameSize_ctl->addItem(userDefinedROIs_[i].toQString(),
        QVariant::fromValue(i + predefinedROIs_.size()));
  }

  frameSize_ctl->addItem("Custom ROIs...",
      QVariant::fromValue(-1));
}

bool QASIROIControlWidget::set_capture_format(int iX, int iY, int iWidth, int iHeight, int iBin, ASI_IMG_TYPE iFormat)
{
  if( !camera_ ) {
    CF_ERROR("Camera is null");
    return false;
  }

  const ASI_CAMERA_INFO &c =
      camera_->cameraInfo();

  ASI_ERROR_CODE status =
      ASISetROIFormat(c.CameraID, iWidth, iHeight, iBin, iFormat);

  if( status != ASI_SUCCESS ) {
    CF_ERROR("ASISetROIFormat(CameraID=%d) fails: %d (%s)",
        c.CameraID,
        status,
        toString(status));
    return false;
  }

  if( iX >= 0 && iY >= 0 ) {
    if( (status = ASISetStartPos(c.CameraID, iX, iY)) != ASI_SUCCESS ) {
      CF_ERROR("ASISetStartPos(CameraID=%d) fails: %d (%s)",
          c.CameraID,
          status,
          toString(status));
      return false;
    }
  }

  return true;
}

bool QASIROIControlWidget::get_capture_format(int * iX, int * iY, int * iWidth, int * iHeight, int * iBin, ASI_IMG_TYPE * iFormat)
{
  if( !camera_ ) {
    CF_ERROR("Camera is null");
    return false;
  }

  const ASI_CAMERA_INFO &c =
      camera_->cameraInfo();

  ASI_ERROR_CODE status =
      ASIGetROIFormat(c.CameraID, iWidth, iHeight, iBin, iFormat);

  if( (status) != ASI_SUCCESS ) {
    CF_ERROR("ASIGetROIFormat(CameraID=%d) fails: %d (%s)",
        c.CameraID,
        status,
        toString(status));
    return false;
  }

  if( (status = ASIGetStartPos(c.CameraID, iX, iY)) ) {
    CF_ERROR("ASIGetStartPos(CameraID=%d) fails: %d (%s)",
        c.CameraID,
        status,
        toString(status));
    return false;
  }

  return true;
}



void QASIROIControlWidget::onFrameSizeCtlCurrentIndexChanged()
{
  if( !updatingControls() ) {

    int currentIndex =
        frameSize_ctl->currentIndex();

    if( currentIndex < 0 ) {
      return;
    }

    const int currentRoiIndex =
        frameSize_ctl->itemData(currentIndex).value<int>();

    if( currentRoiIndex < 0 ) { // Show ROI Edit dialog box;

      c_update_controls_lock lock(this);

      QCameraROISelectionDialog dialogBox(this);

      dialogBox.setROIList(userDefinedROIs_);

      dialogBox.setValidator([](QRect & rc) {

        if ( rc.width() < 1 || rc.height() < 1 ) {
          return false;
        }

        rc.setWidth(rc.width() & ~0x7);
        rc.setHeight(rc.height() & ~0x1);

        return rc.width() > 0 && rc.height() > 0;
      });

      dialogBox.exec();

      if( dialogBox.hasChanges() ) {
        userDefinedROIs_ = dialogBox.roiList();
        saveUserDefinedCameraROIs(userDefinedROIs_);
        update_supported_frame_sizes_combobox();
      }
    }
  }
}



void QASIROIControlWidget::onUpdateControls()
{
  Base::updateControls();
}

void QASIROIControlWidget::onupdatecontrols()
{
  if( !camera_ || camera_->state() != QImagingCamera::State_connected ) {
    setEnabled(false);
  }
  else {
    setEnabled(true);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////

QASICameraExtraContolsWidget::QASICameraExtraContolsWidget(const QASICamera::sptr & camera, QWidget * parent) :
    Base("QASICameraExtraSettings", parent),
    camera_(camera)
{

  if( camera_ ) {

    if( camera_->state() >= QImagingCamera::State_connected) {
      createControls();
    }

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);
  }

  updateControls();
}

QASICameraExtraContolsWidget::~QASICameraExtraContolsWidget()
{
  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }
}

void QASICameraExtraContolsWidget::onCameraStateChanged()
{
  if ( controls_.empty() && camera_ && camera_->state() >= QImagingCamera::State_connected ) {
    createControls();
  }

  updateControls();
}

void QASICameraExtraContolsWidget::createControls()
{
  if ( !camera_ || camera_->state() < QImagingCamera::State_connected ) {
    return;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  static constexpr ASI_CONTROL_TYPE ignore_controls[] = {
      ASI_GAIN,
      ASI_EXPOSURE,
  };

  static constexpr int num_ignore_controls =
      sizeof(ignore_controls) / sizeof(ignore_controls[0]);

  const ASI_CAMERA_INFO & camInfo =
      camera_->cameraInfo();

  const int iCameraID =
      camera_->cameraInfo().CameraID;

  int numControls = 0;

  bool gamma_ctrl_added = false;


  ASI_ERROR_CODE status =
      ASIGetNumOfControls(iCameraID,
          &numControls);

  if( status != ASI_SUCCESS ) {
    CF_ERROR("ASIGetNumOfControls(CameraID=%d) fails: %d (%s)",
        iCameraID,
        status,
        toString(status));
    return;
  }

  for ( int iControlIndex = 0; iControlIndex < numControls; ++iControlIndex ) {

    ASI_CONTROL_CAPS ControlCaps = { 0 };

    status =
        ASIGetControlCaps(iCameraID,
            iControlIndex,
            &ControlCaps);

    if( status != ASI_SUCCESS ) {
      CF_ERROR("ASIGetControlCaps(CameraID=%d, iControlIndex=%d) fails: %d (%s)",
          iCameraID,
          iControlIndex,
          status,
          toString(status));
      continue;
    }

    if ( !ControlCaps.IsWritable ) {
      continue;
    }

    if( std::find(ignore_controls, ignore_controls + num_ignore_controls,
        ControlCaps.ControlType) != ignore_controls + num_ignore_controls ) {
      continue;
    }

    QASIControlWidget *control =
        new QASIControlWidget(iCameraID, ControlCaps, this);

    form->addRow(ControlCaps.Name, control);
    controls_.append(control);

    if ( ControlCaps.ControlType == ASI_GAMMA ) {
      gamma_ctrl_added  = true;
    }
  }

  if( !gamma_ctrl_added ) {

    ASI_CONTROL_CAPS ControlCaps = {
        .Name = "Gamma",    // [64]; //the name of the Control like Exposure, Gain etc..
        .Description = "Gamma", //[128]; //description of this control
        .MaxValue = 100,
        .MinValue = 0,
        .DefaultValue = 50,
        .IsAutoSupported = ASI_TRUE, //support auto set 1, don't support 0
        .IsWritable = ASI_TRUE, //some control like temperature can only be read by some cameras
        .ControlType = ASI_GAMMA, //this is used to get value and set value of the control
        .Unused = "" // [32];
        };

    QASIControlWidget *control =
        new QASIControlWidget(iCameraID, ControlCaps, this);

    form->addRow(ControlCaps.Name, control);
    controls_.append(control);
  }
}

void QASICameraExtraContolsWidget::onload(QSettings & settings)
{
}

void QASICameraExtraContolsWidget::onupdatecontrols()
{
  if( camera_ && camera_->state() == QImagingCamera::State_connected ) {
    for( auto *c : controls_ ) {
      c->updateControls();
    }
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////

QASICameraControls::QASICameraControls(const QASICamera::sptr & camera, QWidget * parent) :
    Base(parent),
    camera_(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  if( camera_ ) {

    if( camera_->state() >= QImagingCamera::State_connected ) {
      create_controls();
    }

    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged,
        Qt::QueuedConnection);
  }

  updateControls();
}

QASICameraControls::~QASICameraControls()
{
  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }
}


void QASICameraControls::onCameraStateChanged()
{
  if ( !roi_ctl && camera_ && camera_->state() >= QImagingCamera::State_connected ) {
    create_controls();
  }

  updateControls();
}


void QASICameraControls::onload(QSettings & settings)
{
}


void QASICameraControls::create_controls()
{
  if ( !camera_ || camera_->state() < QImagingCamera::State_connected ) {
    return;
  }

  if( roi_ctl && exposure_ctl  && gain_ctl && extraSettings_ctl )  {
    return;
  }

  const ASI_CAMERA_INFO & camInfo =
      camera_->cameraInfo();

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if( !roi_ctl ) {
    form->addRow("Format:", roi_ctl =
        new QASIROIControlWidget(camera_,
            this));
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if( !exposure_ctl || !gain_ctl ) {

    const int iCameraID =
        camera_->cameraInfo().CameraID;

    int numControls = 0;

    ASI_ERROR_CODE status =
        ASIGetNumOfControls(iCameraID,
            &numControls);

    if( status != ASI_SUCCESS ) {
      CF_ERROR("ASIGetNumOfControls(CameraID=%d) fails: %d (%s)",
          iCameraID,
          status,
          toString(status));
      return;
    }

    ASI_CONTROL_CAPS exposureControlCaps = { 0 };
    ASI_CONTROL_CAPS gainControlCaps = { 0 };
    int num_controls_found = 0;

    for( int iControlIndex = 0; iControlIndex < numControls; ++iControlIndex ) {

      ASI_CONTROL_CAPS ControlCaps = { 0 };

      status =
          ASIGetControlCaps(iCameraID,
              iControlIndex,
              &ControlCaps);

      if( status != ASI_SUCCESS ) {
        CF_ERROR("ASIGetControlCaps(CameraID=%d, iControlIndex=%d) fails: %d (%s)",
            iCameraID,
            iControlIndex,
            status,
            toString(status));
        continue;
      }

      if( !ControlCaps.IsWritable ) {
        CF_DEBUG("QASICameraControls: NOT Writable '%s' (id=%s)", ControlCaps.Name, toString(ControlCaps.ControlType)); // case ASI_GAMMA:
        continue;
      }

      if( ControlCaps.ControlType == ASI_GAIN ) {
        gainControlCaps = ControlCaps;
        ++num_controls_found;
      }
      else if( ControlCaps.ControlType == ASI_EXPOSURE ) {
        exposureControlCaps = ControlCaps;
        ++num_controls_found;
      }

      if( num_controls_found == 2 ) {
        break;
      }

    }

    if( !exposure_ctl && *exposureControlCaps.Name ) {

      form->addRow(exposureControlCaps.Name, exposure_ctl =
          new QASIControlWidget(iCameraID, exposureControlCaps, this));
    }

    if( !gain_ctl && *gainControlCaps.Name ) {

      form->addRow(gainControlCaps.Name, gain_ctl =
          new QASIControlWidget(iCameraID, gainControlCaps, this));
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if( !extraSettings_ctl ) {
    add_expandable_groupbox("Camera controls", extraSettings_ctl =
        new QASICameraExtraContolsWidget(camera_, this) );
  }

}

void QASICameraControls::onupdatecontrols()
{
  if ( !camera_ ) {
    setEnabled(false);
  }
  else {
    switch (camera_->state()) {
      case QImagingCamera::State_connected:
        exposure_ctl->updateControls();
        gain_ctl->updateControls();
        extraSettings_ctl->updateControls();
        setEnabled(true);

        break;
      case QImagingCamera::State_started:
        setEnabled(true);
        break;
      default:
        setEnabled(false);
        break;
    }
  }
}


} /* namespace serimager */
