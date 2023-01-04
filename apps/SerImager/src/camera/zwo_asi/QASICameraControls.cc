/*
 * QASICameraSettingsWidget.cc
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#include "QASICameraControls.h"
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


    expscale_ctl->addItem("us ", QVariant::fromValue(QASIExposureScaleParams(controlCaps_.MinValue, 1000, 1)));
    expscale_ctl->addItem("ms ", QVariant::fromValue(QASIExposureScaleParams(1, 1000, 1000)));
    expscale_ctl->addItem("sec", QVariant::fromValue(QASIExposureScaleParams(1, controlCaps_.MaxValue / 1000000, 1000000)));
    expscale_ctl->setCurrentIndex(1);

    connect(expscale_ctl, SIGNAL(currentIndexChanged(int)),
        this, SLOT(onExpScaleCtlChanged(int)),
        Qt::QueuedConnection);

  }

  updateCtrlRanges();

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

  if ( value_ctl ) {
    value_ctl->setEnabled(!controlCaps_.IsAutoSupported || !isAuto);
    value_ctl->setValue(lValue);
  }

  if ( slider_ctl ) {
    slider_ctl->setEnabled(!controlCaps_.IsAutoSupported || !isAuto);
    slider_ctl->setValue(lValue);
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
    updateCtrlRanges();
    onupdatecontrols();
  }
}

void QASIControlWidget::updateCtrlRanges()
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

//  CF_DEBUG("ASISetControlValue('%s' %ld auto=%d)",
//      controlCaps_.Name, lValue, isAuto);

  const ASI_ERROR_CODE status =
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

  if( expscale_ctl ) {

    const QASIExposureScaleParams p =
        expscale_ctl->currentData().value<QASIExposureScaleParams>();

    if ( p.scale > 0 ) {
      *lValue /= p.scale;
    }
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

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
      this, &ThisClass::onFrameSizeChanged);

  connect(imageFormat_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onImageFormatCtlChanged);

  connect(binning_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
      this, &ThisClass::onImageBinningCtlChanged);

  if( camera_ ) {

    const ASI_CAMERA_INFO &c =
        camera_->cameraInfo();

    setUpdatingControls(true);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // setup pre-defined frame sizes

    const QSize predefined_sizes[] = {
        QSize(c.MaxWidth, c.MaxHeight),
        QSize(3840, 2160),
        QSize(2560, 1140),
        QSize(2048, 1536),
        QSize(1920, 1080),
        QSize(1600, 900),
        QSize(1360, 768),
        QSize(1280, 720),
        QSize(1024, 768),
        QSize(800, 600),
        QSize(640, 480),
        QSize(512, 512),
        QSize(320, 240),
        QSize(256, 256),
        QSize(160, 120),
    };

    for ( uint i = 0; i < sizeof(predefined_sizes)/sizeof(predefined_sizes[0]); ++i ) {

      const QSize & s =
          predefined_sizes[i];

      if( s.width() > c.MaxWidth || s.height() > c.MaxHeight ) {
        continue;
      }

      const int iX =
          16 * ((c.MaxWidth - s.width()) / 32);

      const int iY =
          4 * ((c.MaxHeight - s.height()) / 8);

      updateFrameSizeCtl(QRect(iX, iY, s.width(), s.height()));
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////

    for( int i = 0; i < sizeof(c.SupportedVideoFormat) / sizeof(c.SupportedVideoFormat[0]); ++i ) {

      const ASI_IMG_TYPE format =
          c.SupportedVideoFormat[i];

      if( format == ASI_IMG_END ) {
        break;
      }

      imageFormat_ctl->addItem(toString(format),
          QVariant::fromValue((int) format));
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////

    for( int i = 0; i < sizeof(c.SupportedBins) / sizeof(c.SupportedBins[0]); ++i ) {

      const int iBin =
          c.SupportedBins[i];

      if ( iBin <= 0 ) {
        break;
      }

      binning_ctl->addItem(QString("Bin %1").arg(iBin),
          QVariant::fromValue(iBin));
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////

    setUpdatingControls(false);


    connect(camera_.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onUpdateControls,
        Qt::QueuedConnection);
  }

  updateControls();
}


void QASIROIControlWidget::onUpdateControls()
{
  updateControls();
}

void QASIROIControlWidget::onupdatecontrols()
{
  if( !camera_ || camera_->state() != QImagingCamera::State_connected ) {
    setEnabled(false);
  }
  else {

    const ASI_CAMERA_INFO & c =
        camera_->cameraInfo();

    int iX = 0;
    int iY = 0;
    int iWidth = 0;
    int iHeight = 0;
    int iBin = 1;
    ASI_IMG_TYPE iFormat = ASI_IMG_END;
    ASI_ERROR_CODE status;

    if( (status = ASIGetROIFormat(c.CameraID, &iWidth, &iHeight, &iBin, &iFormat)) != ASI_SUCCESS ) {

      CF_ERROR("ASIGetROIFormat(CameraID=%d) fails: %d (%s)",
          c.CameraID,
          status,
          toString(status));

      setEnabled(false);
      return;
    }


    if( (status = ASIGetStartPos(c.CameraID, &iX, &iY)) ) {
      CF_ERROR("ASIGetStartPos(CameraID=%d) fails: %d (%s)",
          c.CameraID,
          status,
          toString(status));

      setEnabled(false);
      return;
    }


    updateImageFormatCtl(iFormat);
    updateFrameSizeCtl(QRect(iX, iY, iWidth, iHeight));
    setBinningCtl(iBin);

    setEnabled(true);
  }
}

void QASIROIControlWidget::onSetCameraROI()
{
  if( !updatingControls() && camera_ && camera_->state() == QImagingCamera::State_connected ) {

    const ASI_CAMERA_INFO &c =
        camera_->cameraInfo();

    const QRect rc =
        frameSize_ctl->currentData().value<QRect>();

    const ASI_IMG_TYPE iFormat =
        (ASI_IMG_TYPE) imageFormat_ctl->currentData().value<int>();

    int iBin =
        binning_ctl->currentData().value<int>();

    ASI_ERROR_CODE status;

    CF_DEBUG("ASISetROIFormat: rc=(%d %d %dx%d) iBin=%d iFormat=%s",
        rc.x(), rc.y(), rc.width(), rc.height(), iBin, toString(iFormat));


    if( (status = ASISetROIFormat(c.CameraID, rc.width(), rc.height(), iBin, iFormat)) != ASI_SUCCESS ) {
      CF_ERROR("ASISetROIFormat(CameraID=%d) fails: %d (%s)",
          c.CameraID,
          status,
          toString(status));
    }

    if( (status = ASISetStartPos(c.CameraID, rc.x(), rc.y())) != ASI_SUCCESS ) {
      CF_ERROR("ASISetStartPos(CameraID=%d) fails: %d (%s)",
          c.CameraID,
          status,
          toString(status));
    }

    //if( status != ASI_SUCCESS )
    {
      updateControls();
    }
  }
}

void QASIROIControlWidget::updateFrameSizeCtl(const QRect & rc)
{
  const QVariant v =
      QVariant::fromValue(rc);

  int itemIndex =
      frameSize_ctl->findData(v);

  if( itemIndex < 0 ) {

    const QString itemText =
        QString("%1:%2 %3x%4")
        .arg(rc.x())
        .arg(rc.y())
        .arg(rc.width())
        .arg(rc.height());

    frameSize_ctl->addItem(itemText, v);

    itemIndex =
        frameSize_ctl->count() - 1;
  }

  frameSize_ctl->setCurrentIndex(itemIndex);
}


void QASIROIControlWidget::onFrameSizeChanged(int)
{
  onSetCameraROI();
}

void QASIROIControlWidget::updateImageFormatCtl(ASI_IMG_TYPE iFormat)
{
  const QVariant v =
      QVariant::fromValue((int) (iFormat));

  int itemIndex =
      imageFormat_ctl->findData(v);

  if( itemIndex < 0 ) {
    imageFormat_ctl->addItem(toString(iFormat), v);
    itemIndex = imageFormat_ctl->count() - 1;
  }

  imageFormat_ctl->setCurrentIndex(itemIndex);
}

void QASIROIControlWidget::onImageFormatCtlChanged(int)
{
  onSetCameraROI();
}

void QASIROIControlWidget::setBinningCtl(int iBin)
{
  const QVariant v =
      QVariant::fromValue(iBin);

  int itemIndex =
      binning_ctl->findData(v);

  if( itemIndex < 0 ) {
    binning_ctl->addItem(QString("%1").arg(iBin), v);
    itemIndex = binning_ctl->count() - 1;
  }

  binning_ctl->setCurrentIndex(itemIndex);
}

void QASIROIControlWidget::onImageBinningCtlChanged(int)
{
  onSetCameraROI();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

QASICameraExtraSettingsWidget::QASICameraExtraSettingsWidget(const QASICamera::sptr & camera, QWidget * parent) :
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

QASICameraExtraSettingsWidget::~QASICameraExtraSettingsWidget()
{
  if( camera_ ) {
    disconnect(camera_.get(), nullptr,
        this, nullptr);
  }
}

void QASICameraExtraSettingsWidget::onCameraStateChanged()
{
  if ( controls_.empty() && camera_ && camera_->state() >= QImagingCamera::State_connected ) {
    createControls();
  }

  updateControls();
}

void QASICameraExtraSettingsWidget::createControls()
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

  CF_DEBUG("numControls = %d", numControls);

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
  }
}

void QASICameraExtraSettingsWidget::onload(QSettings & settings)
{
}

void QASICameraExtraSettingsWidget::onupdatecontrols()
{
}



///////////////////////////////////////////////////////////////////////////////////////////////////

QASICameraControls::QASICameraControls(const QASICamera::sptr & camera, QWidget * parent) :
    Base(parent),
    camera_(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  if( camera_ ) {

    if( camera_->state() >= QImagingCamera::State_connected ) {
      createControls();
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
    createControls();
  }

  updateControls();
}


void QASICameraControls::onload(QSettings & settings)
{
}


void QASICameraControls::createControls()
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
    form->addRow("Format:",
        roi_ctl = new QASIROIControlWidget(camera_,
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
        new QASICameraExtraSettingsWidget(camera_, this) );
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
      case QImagingCamera::State_started:
        setEnabled(true);
        break;
      default:
        setEnabled(false);
        break;
    }
  }
}


} /* namespace qserimager */
