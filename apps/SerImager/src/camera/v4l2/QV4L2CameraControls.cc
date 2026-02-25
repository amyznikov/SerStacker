/*
 * QV4L2CameraControls.cc
 *
 *  Created on: Dec 23, 2022
 *      Author: amyznikov
 */

#include "QV4L2CameraControls.h"
#include <linux/version.h>
#include <gui/widgets/style.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#define ICON_enterbutt        ":/serimager/icons/enterbutt.png"

template<>
const c_enum_member* members_of<enum v4l2_ctrl_type>()
{
  int xx = LINUX_VERSION_CODE;

  static const c_enum_member members[] = {
      { V4L2_CTRL_TYPE_INTEGER, "V4L2_CTRL_TYPE_INTEGER", "" },
      { V4L2_CTRL_TYPE_BOOLEAN, "V4L2_CTRL_TYPE_BOOLEAN", "" },
      { V4L2_CTRL_TYPE_MENU, "V4L2_CTRL_TYPE_MENU", "" },
      { V4L2_CTRL_TYPE_BUTTON, "V4L2_CTRL_TYPE_BUTTON", "" },
      { V4L2_CTRL_TYPE_INTEGER64, "V4L2_CTRL_TYPE_INTEGER64", "" },
      { V4L2_CTRL_TYPE_CTRL_CLASS, "V4L2_CTRL_TYPE_CTRL_CLASS", "" },
      { V4L2_CTRL_TYPE_STRING, "V4L2_CTRL_TYPE_STRING", "" },
      { V4L2_CTRL_TYPE_BITMASK, "V4L2_CTRL_TYPE_BITMASK", "" },
      { V4L2_CTRL_TYPE_INTEGER_MENU, "V4L2_CTRL_TYPE_INTEGER_MENU", "" },

      /* Compound types are >= 0x0100 */
      { V4L2_CTRL_COMPOUND_TYPES, "V4L2_CTRL_COMPOUND_TYPES", "" },
      { V4L2_CTRL_TYPE_U8, "V4L2_CTRL_TYPE_U8", "" },
      { V4L2_CTRL_TYPE_U16, "V4L2_CTRL_TYPE_U16", "" },
      { V4L2_CTRL_TYPE_U32, "V4L2_CTRL_TYPE_U32", "" },

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,0,0)
      { V4L2_CTRL_TYPE_AREA, "V4L2_CTRL_TYPE_AREA", "" },
      { V4L2_CTRL_TYPE_HDR10_CLL_INFO, "V4L2_CTRL_TYPE_HDR10_CLL_INFO", "" },
      { V4L2_CTRL_TYPE_HDR10_MASTERING_DISPLAY, "V4L2_CTRL_TYPE_HDR10_MASTERING_DISPLAY", "" },
      { V4L2_CTRL_TYPE_H264_SPS, "V4L2_CTRL_TYPE_H264_SPS", "" },
      { V4L2_CTRL_TYPE_H264_PPS, "V4L2_CTRL_TYPE_H264_PPS", "" },
      { V4L2_CTRL_TYPE_H264_SCALING_MATRIX, "V4L2_CTRL_TYPE_H264_SCALING_MATRIX", "" },
      { V4L2_CTRL_TYPE_H264_SLICE_PARAMS, "V4L2_CTRL_TYPE_H264_SLICE_PARAMS", "" },
      { V4L2_CTRL_TYPE_H264_DECODE_PARAMS, "V4L2_CTRL_TYPE_H264_DECODE_PARAMS", "" },
      { V4L2_CTRL_TYPE_H264_PRED_WEIGHTS, "V4L2_CTRL_TYPE_H264_PRED_WEIGHTS", "" },
      { V4L2_CTRL_TYPE_FWHT_PARAMS, "V4L2_CTRL_TYPE_FWHT_PARAMS", "" },
      { V4L2_CTRL_TYPE_VP8_FRAME, "V4L2_CTRL_TYPE_VP8_FRAME", "" },
      { V4L2_CTRL_TYPE_MPEG2_QUANTISATION, "V4L2_CTRL_TYPE_MPEG2_QUANTISATION", "" },
      { V4L2_CTRL_TYPE_MPEG2_SEQUENCE, "V4L2_CTRL_TYPE_MPEG2_SEQUENCE", "" },
      { V4L2_CTRL_TYPE_MPEG2_PICTURE, "V4L2_CTRL_TYPE_MPEG2_PICTURE", "" },
      { V4L2_CTRL_TYPE_VP9_COMPRESSED_HDR, "V4L2_CTRL_TYPE_VP9_COMPRESSED_HDR", "" },
      { V4L2_CTRL_TYPE_VP9_FRAME, "V4L2_CTRL_TYPE_VP9_FRAME", "" },
      #endif
      { (v4l2_ctrl_type) (0) }
  };

  return members;
}

template<>
const c_enum_member* members_of<enum v4l2_frmivaltypes>()
{
  static const c_enum_member members[] = {
      { V4L2_FRMIVAL_TYPE_DISCRETE, "V4L2_FRMIVAL_TYPE_DISCRETE", "" },
      { V4L2_FRMIVAL_TYPE_CONTINUOUS, "V4L2_FRMIVAL_TYPE_CONTINUOUS", "" },
      { V4L2_FRMIVAL_TYPE_STEPWISE, "V4L2_FRMIVAL_TYPE_STEPWISE", "" },
      { (enum v4l2_frmivaltypes) (0) }
  };

  return members;
}

namespace serimager {
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

template<class C>
typename std::enable_if<std::is_base_of_v<QWidget, C>, void>::type
remove_control(QFormLayout * form, C *& w)
{
  if( w ) {
    form->removeRow(w);
    w = nullptr;
  }
}

void enable_control(QWidget * w, bool enabled)
{
  if( w ) {
    w->setEnabled(enabled);
  }
}

void clearColorspace(cv4l_fmt & fmt)
{
  //if (m_colorspace->currentIndex() == 0)
  fmt.s_colorspace(V4L2_COLORSPACE_DEFAULT);
  //if (m_xferFunc->currentIndex() == 0)
  fmt.s_xfer_func(V4L2_XFER_FUNC_DEFAULT);
  //if (m_ycbcrEnc->currentIndex() == 0)
  fmt.s_ycbcr_enc(V4L2_YCBCR_ENC_DEFAULT);
  //if (m_quantRange->currentIndex() == 0)
  fmt.s_quantization(V4L2_QUANTIZATION_DEFAULT);
}

} // namespace

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QV4L2CameraExtraSettingsWidget::QV4L2CameraExtraSettingsWidget(const QV4L2Camera::sptr & camera, QWidget * parent) :
    Base(parent),
    _camera(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  if( _camera ) {
    if( _camera->state() >= QImagingCamera::State_connected ) {
      createControls();
    }
    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged);
  }

  updateControls();
}

QV4L2CameraExtraSettingsWidget::~QV4L2CameraExtraSettingsWidget()
{
  if( _camera ) {
    disconnect(_camera.get(), nullptr,
        this, nullptr);
  }
}

void QV4L2CameraExtraSettingsWidget::onCameraStateChanged(QImagingCamera::State oldSate, QImagingCamera::State newState)
{
  if( _camera ) {
    if( oldSate == QImagingCamera::State_connecting && newState == QImagingCamera::State_connected ) {
      createControls();
    }
  }

  updateControls();
}

QWidget* QV4L2CameraExtraSettingsWidget::add_ex_ctrl(cv4l_fd & device, const v4l2_query_ext_ctrl & c)
{
  // c.flags & V4L2_CTRL_FLAG_UPDATE;

  static const auto isConnectedState =
      [](const QV4L2Camera::sptr & camera) {
        const QImagingCamera::State state = camera ? camera->state() : QImagingCamera::State_disconnected;
        return state == QImagingCamera::State_connected || state == QImagingCamera::State_started;
  };

//  static const auto isConnected =
//      [](const QV4L2Camera::sptr & camera) {
//        const QImagingCamera::State state = camera ? camera->state() : QImagingCamera::State_disconnected;
//        return state != QImagingCamera::State_disconnected && state != QImagingCamera::State_disconnect;
//  };

  switch (c.type) {
    case V4L2_CTRL_TYPE_INTEGER: {

      QSpinBox *ctrl =
          add_spinbox(c.name,
              "",
              [this, c](int value) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, value);
                  if ( status ) {
                    CF_ERROR("s_ext_ctrl(%s=%u) fails: %d (%s) ", c.name, value,
                        status, strerror(status));
                  }
                }
              },
              [this, c](int * value) -> bool {
                if ( isConnectedState(_camera) ) {
                  const int status = _camera->g_ext_ctrl(c.id, value);
                  if ( !status ) {
                    return true;
                  }
                  CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name, status, strerror(status));
                }
                return false;
              });

      ctrl->setFocusPolicy(Qt::StrongFocus);
      ctrl->setKeyboardTracking(false);
      ctrl->setMouseTracking(false);
      ctrl->setTabletTracking(false);
      ctrl->setRange(c.minimum, c.maximum);
      ctrl->setSingleStep(c.step);
      ctrl->setValue(c.default_value);
      ctrl->setToolTip(QString("%1 : min=%2 max=%3 step=%4 default=%5").arg(c.name)
          .arg(c.minimum)
          .arg(c.maximum)
          .arg(c.step)
          .arg(c.default_value));

      return ctrl;
    }

    case V4L2_CTRL_TYPE_BOOLEAN: {

      QCheckBox *ctrl =
          add_checkbox(c.name,
              "",
              [this, c](bool checked) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, checked);
                  if ( status ) {
                    CF_ERROR("s_ext_ctrl(%s=%u) fails: %d (%s) ", c.name, checked,
                        status, strerror(status));
                  }
                }
              },
              [this, c](bool * checked) -> bool {
                if ( isConnectedState(_camera) ) {
                  const int status = _camera->g_ext_ctrl(c.id, checked);
                  if ( !status ) {
                    return true;
                  }
                  CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name, status, strerror(status));
                }
                return false;
              });

      ctrl->setChecked(c.default_value);
      ctrl->setToolTip(QString("%1: default=%2").arg(c.name)
          .arg(c.default_value ? "checked" : "unchecked"));

      return ctrl;
    }

    case V4L2_CTRL_TYPE_MENU:
      case V4L2_CTRL_TYPE_INTEGER_MENU: {

      QComboBox *ctrl =
          add_combobox<QComboBox>(QString(c.name),
              "",
              false,
              [this, c](int index, QComboBox * combo) -> void {
                if ( _camera ) {
                  int value = combo->itemData(index).value<int>();
                  int status = _camera->s_ext_ctrl(c.id, value);
                  if ( status ) {
                    CF_ERROR("s_ext_ctrl(%s=%u) fails: %d (%s) ", c.name, value,
                        status, strerror(status));
                  }
                }
              },

              [this, c](int * index, QComboBox * combo) -> bool {
                if ( isConnectedState(_camera) ) {
                  int value;
                  const int status = _camera->g_ext_ctrl(c.id, &value);
                  if ( !status ) {
                    * index = combo->findData(QVariant::fromValue(value));
                    return true;
                  }
                  CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                      status, strerror(status));
                }
                return false;
              });

      struct v4l2_querymenu qmenu;

      //device.s_trace(1);

      for( int i = (int) c.minimum; i <= (int) c.maximum; ++i ) {
        qmenu.id = c.id;
        qmenu.index = i;
        if( device.querymenu(qmenu) != 0 ) {
          continue;
        }
        if( c.type == V4L2_CTRL_TYPE_MENU ) {
          ctrl->addItem((const char*) qmenu.name, QVariant::fromValue(i));
        }
        else {
          ctrl->addItem(QString("%1").arg(qmenu.value), QVariant::fromValue(i));
        }
      }

      ctrl->setToolTip(QString("%1: default=%2").arg(c.name)
          .arg(ctrl->itemText(ctrl->findData(c.default_value))));

      return ctrl;
    }

    case V4L2_CTRL_TYPE_BUTTON: {

      QToolButton *ctrl =
          add_tool_button((const char*) c.name,
              getIcon(ICON_enterbutt),
              [this, c](bool checked) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, 1);
                  if ( status ) {
                    CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                        status, strerror(status));
                  }
                }
              });

      ctrl->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      return ctrl;
    }

    case V4L2_CTRL_TYPE_INTEGER64: {

      QNumericBox *ctrl =
          add_numeric_box<int64_t>(c.name,
              "",
              [this, c](int64_t value) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, (__s64)value);
                  if ( status ) {
                    CF_ERROR("s_ext_ctrl(%s=%lld) fails: %d (%s)",
                        c.name, (long long int)value, status, strerror(status));
                  }
                }
              },
              [this, c](int64_t * value) -> bool {
                if ( isConnectedState(_camera) ) {
                  const int status = _camera->g_ext_ctrl(c.id, (__s64*)value);
                  if ( !status ) {
                    return true;
                  }
                  CF_ERROR("g_ext_ctrl(%s) fails: %d (%s)", c.name, status, strerror(status));
                }
                return false;
              }
              );

      ctrl->setValue(c.default_value);
      return ctrl;
    }

    case V4L2_CTRL_TYPE_BITMASK: {
      QLineEditBox *ctrl =
          add_textbox(c.name,
              "",
              [this, c](const QString & text) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, text);
                  if ( status != 0 ) {
                    CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                        status, strerror(status));
                  }
                }},
              [this, c](QString * text) -> bool {
                if ( isConnectedState(_camera) ) {
                  const int status = _camera->g_ext_ctrl(c.id, text);
                  if ( !status ) {
                    return true;
                  }
                  CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                      status, strerror(status));
                }
                return false;
              });

      ctrl->setInputMask("HHHHHHHH");
      return ctrl;
    }

    case V4L2_CTRL_TYPE_STRING: {
      QLineEditBox *ctrl =
          add_textbox(c.name,
              "",
              [this, c](const QString & text) {
                if ( _camera ) {
                  int status = _camera->s_ext_ctrl(c.id, text);
                  if ( status != 0 ) {
                    CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                        status, strerror(status));
                  }
                }
              },

              [this, c](QString * text) -> bool {
                if ( isConnectedState(_camera) ) {
                  const int status = _camera->g_ext_ctrl(c.id, text);
                  if ( !status ) {
                    return true;
                  }
                  CF_ERROR("s_ext_ctrl(%s) fails: %d (%s) ", c.name,
                      status, strerror(status));
                }
                return false;
              });

      ctrl->setMaxLength(c.maximum);
      return ctrl;
    }
  }

  return (QWidget*) nullptr;
}

void QV4L2CameraExtraSettingsWidget::createControls()
{
  if( _camera && _controls.empty() ) {

    c_update_controls_lock lock(this);

    cv4l_fd &device =
        _camera->device();

    v4l2_query_ext_ctrl q = { 0 };

    while (device.query_ext_ctrl(q, true) == 0) {
      if( !(q.flags & V4L2_CTRL_FLAG_DISABLED) ) {

        QWidget *ctrl = add_ex_ctrl(device, q);
        if( ctrl ) {
          _controls.append(ctrl);
        }
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

QV4L2CameraControls::QV4L2CameraControls(const QV4L2Camera::sptr & camera, QWidget * parent) :
    Base(parent),
    _camera(camera)
{
  form->setLabelAlignment(Qt::AlignLeft);

  if( _camera ) {
    if( _camera->state() >= QImagingCamera::State_connected ) {
      createControls();
    }
    connect(_camera.get(), &QImagingCamera::stateChanged,
        this, &ThisClass::onCameraStateChanged);
  }


  connect(this, &ThisClass::populatecontrols,
      [this](){
        if( _camera && extraControls_ctl ) {
          extraControls_ctl->updateControls();
        }
  });

  connect(this, &ThisClass::enablecontrols,
      [this](){
        if( !_camera ) {
          setEnabled(false);
        }
        else {

          bool enable_primary_controls = false;
          bool enable_extended_controls = false;

          switch (_camera->state()) {
            case QImagingCamera::State_disconnected:
              case QImagingCamera::State_connecting:
              case QImagingCamera::State_starting:
              case QImagingCamera::State_stop:
              case QImagingCamera::State_disconnect:
              enable_primary_controls = false;
              enable_extended_controls = false;
              break;

            case QImagingCamera::State_connected:
              enable_primary_controls = true;
              enable_extended_controls = true;
              break;

            case QImagingCamera::State_started:
              enable_primary_controls = false;
              enable_extended_controls = true;
              break;
          }

          enable_control(videoInput_ctl, enable_primary_controls);
          enable_control(fmt_ctl, enable_primary_controls);
          enable_control(frameSize_ctl, enable_primary_controls);
          enable_control(frameWidth_ctl, enable_primary_controls);
          enable_control(frameHeight_ctl, enable_primary_controls);
          enable_control(frameRate_ctl, enable_primary_controls);

          enable_control(extraControls_ctl, enable_extended_controls);

          if( extraControls_ctl && extraControls_ctl->isEnabled() ) {
            extraControls_ctl->enablecontrols();
          }

          setEnabled(true);
        }
  });

  updateControls();
}

QV4L2CameraControls::~QV4L2CameraControls()
{
  if( _camera ) {
    disconnect(_camera.get(), nullptr,
        this, nullptr);
  }
}

void QV4L2CameraControls::onCameraStateChanged(QImagingCamera::State oldSate, QImagingCamera::State newState)
{
  if( _camera ) {
    if( oldSate == QImagingCamera::State_connecting && newState == QImagingCamera::State_connected ) {
      createControls();
    }
  }

  updateControls();
}

void QV4L2CameraControls::deleteControls()
{
  remove_control(form, videoInput_ctl);
  remove_control(form, fmt_ctl);
  remove_control(form, frameSize_ctl);
  remove_control(form, frameWidth_ctl);
  remove_control(form, frameHeight_ctl);
  remove_control(form, frameRate_ctl);
}

void QV4L2CameraControls::createControls()
{
  if( !_camera || _camera->state() != QImagingCamera::State_connected ) {
    return;
  }

  setUpdatingControls(true);

  cv4l_fd &device =
      _camera->device();

  bool needsStd = false;
  bool needsTimings = false;

  v4l2_input vin;
  if( device.enum_input(vin, true) != 0 ) {
    remove_control(form, videoInput_ctl);
  }
  else {

    if( videoInput_ctl ) {
      videoInput_ctl->clear();
    }
    else {
      videoInput_ctl = add_combobox("Input:", "", false);
      connect(videoInput_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &ThisClass::onVideoInputChanged);
    }

    do {
      videoInput_ctl->addItem((const char*) vin.name);
      if( vin.capabilities & V4L2_IN_CAP_STD ) {
        needsStd = true;
      }
      if( vin.capabilities & V4L2_IN_CAP_DV_TIMINGS ) {
        needsTimings = true;
      }
    } while (device.enum_input(vin) == 0);

    __u32 input_index = 0;
    if( device.g_input(input_index) == 0 ) {
      videoInput_ctl->setCurrentIndex(input_index);
    }

  }

  refreshFormats(device);

  if( !extraControls_ctl ) {

    add_expandable_groupbox("Camera controls",
        extraControls_ctl = new QV4L2CameraExtraSettingsWidget(_camera, this));
  }

  setUpdatingControls(false);
}

void QV4L2CameraControls::onVideoInputChanged(int index)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device = _camera->device();

      device.s_input((__u32 ) index);

      c_update_controls_lock lock(this);
      refreshFormats(device);
    }
  }
}

void QV4L2CameraControls::refreshFormats(cv4l_fd & device)
{
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_fmtdesc fmtdesc;
  int status;

  if( (status = device.enum_fmt(fmtdesc, true, 0, type)) ) {

    CF_ERROR("device.enum_fmt() fails: status=%d (%s)",
        status, strerror(status));

    remove_control(form, fmt_ctl);
  }
  else {

    if( fmt_ctl ) {
      fmt_ctl->clear();
    }
    else {
      fmt_ctl = add_combobox("Format:","", false);
      fmt_ctl->setSizeAdjustPolicy(QComboBox::AdjustToContents);

      connect(fmt_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &ThisClass::onInputFormatChanged);
    }

    do {
      fmt_ctl->addItem((const char*) fmtdesc.description,
          QVariant::fromValue(fmtdesc.pixelformat));
    } while (device.enum_fmt(fmtdesc) == 0);

    v4l2_format fmt;

    if( (status = device.g_fmt(fmt, type)) == 0 ) {
      fmt_ctl->setCurrentIndex(fmt_ctl->findData(
          QVariant::fromValue(fmt.fmt.pix.pixelformat)));
    }
    else {
      CF_ERROR("device.g_fmt() fails: status=%d (%s)",
          status, strerror(status));

      fmt_ctl->setEnabled(false);
    }
  }

  refreshFrameSizes(device);
}

void QV4L2CameraControls::onInputFormatChanged(int index)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device =
          _camera->device();

      if( index < 0 ) {
        CF_ERROR("Unhandled case index=%d", index);
      }
      else {

        const uint type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_fmtdesc desc;
        cv4l_fmt fmt;
        int status;

        if( (status = device.enum_fmt(desc, true, index, type)) ) {
          CF_ERROR("device.enum_fmt() fails: status=%d (%s)",
              status, strerror(status));
        }
        else if( (status = device.g_fmt(fmt, type)) ) {
          CF_ERROR("device.g_fmt() fails: status=%d (%s)",
              status, strerror(status));
        }
        else {

          fmt.s_pixelformat(desc.pixelformat);

          if( (status = device.try_fmt(fmt)) ) {
            CF_ERROR("device.try_fmt() fails: status=%d (%s)",
                status, strerror(status));
          }
          else if( (status = device.s_fmt(fmt)) ) {
            CF_ERROR("device.s_fmt() fails: status=%d (%s)",
                status, strerror(status));
          }
          else {
            CF_DEBUG("device.s_fmt() OK");
          }
        }
      }

      c_update_controls_lock lock(this);
      refreshFrameSizes(device);
    }
  }
}

void QV4L2CameraControls::refreshFrameSizes(cv4l_fd & device)
{
  const uint type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cv4l_fmt fmt;
  int status;

  if( (status = device.g_fmt(fmt, type)) ) {

    CF_ERROR("device.g_fmt() fails: status=%d (%s)",
        status, strerror(status));

    remove_control(form, frameSize_ctl);
    remove_control(form, frameWidth_ctl);
    remove_control(form, frameHeight_ctl);
  }
  else {

    v4l2_frmsizeenum frmsize;

    const int width =
        fmt.g_width();

    const int height =
        fmt.g_height();

    if( (status = device.enum_framesizes(frmsize, fmt.g_pixelformat())) ) {
      CF_ERROR("device.enum_framesizes() fails: status=%d (%s)",
          status, strerror(status));
    }

    const bool fok =
        status == 0;

    if( fok && frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE ) {

      remove_control(form, frameWidth_ctl);
      remove_control(form, frameHeight_ctl);

      if( frameSize_ctl ) {
        frameSize_ctl->clear();
      }
      else {
        frameSize_ctl = add_combobox("Frame Size:","", false);
        connect(frameSize_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &ThisClass::onFrameSizeChanged);
      }

      do {

        const QString itemText =
            QString("%1x%2")
                .arg(frmsize.discrete.width)
                .arg(frmsize.discrete.height);

        const QSize frameSize(frmsize.discrete.width,
            frmsize.discrete.height);

        frameSize_ctl->addItem(itemText, QVariant::fromValue(frameSize));

        if( frmsize.discrete.width == width && frmsize.discrete.height == height ) {
          frameSize_ctl->setCurrentIndex(frmsize.index);
        }

      } while (device.enum_framesizes(frmsize) == 0);

      //    updateFrameInterval(device);

    }
    else {

      if( !fok ) {
        frmsize.stepwise.min_width = 8;
        frmsize.stepwise.max_width = 4096;
        frmsize.stepwise.step_width = 1;
        frmsize.stepwise.min_height = 8;
        frmsize.stepwise.max_height = 2160;
        frmsize.stepwise.step_height = 1;
      }

      remove_control(form, frameSize_ctl);

      if( frameWidth_ctl ) {
        frameWidth_ctl->clear();
      }
      else {
        frameWidth_ctl =
            add_spinbox("Frame Width:", "");

        connect(frameWidth_ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            this, &ThisClass::onFrameWidthChanged);

      }

      if( frameHeight_ctl ) {
        frameHeight_ctl->clear();
      }
      else {
        frameHeight_ctl =
            add_spinbox("Frame Height:", "");

        connect(frameHeight_ctl, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            this, &ThisClass::onFrameHeightChanged);
      }

      frameWidth_ctl->blockSignals(true);
      frameWidth_ctl->setMinimum(frmsize.stepwise.min_width);
      frameWidth_ctl->setMaximum(frmsize.stepwise.max_width);
      frameWidth_ctl->setSingleStep(frmsize.stepwise.step_width);
      frameWidth_ctl->setValue(width);
      frameWidth_ctl->blockSignals(false);

      frameHeight_ctl->blockSignals(true);
      frameHeight_ctl->setMinimum(frmsize.stepwise.min_height);
      frameHeight_ctl->setMaximum(frmsize.stepwise.max_height);
      frameHeight_ctl->setSingleStep(frmsize.stepwise.step_height);
      frameHeight_ctl->setValue(height);
      frameHeight_ctl->blockSignals(false);
    }
  }

  refreshFrameRates(device);
}

void QV4L2CameraControls::onFrameSizeChanged(int index)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device =
          _camera->device();

      const uint type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      cv4l_fmt fmt;
      int status;

      if( (status = device.g_fmt(fmt, type)) ) {
        CF_ERROR("device.g_fmt(V4L2_BUF_TYPE_VIDEO_CAPTURE) fails: status=%d (%s)",
            status, strerror(status));
      }
      else {
        v4l2_frmsizeenum frmsize = { 0 };

        if( (status = device.enum_framesizes(frmsize, fmt.g_pixelformat(), index)) ) {
          CF_ERROR("device.enum_framesizes(index=%d, fmt.g_pixelformat()=%u) fails: status=%d (%s)",
              index, fmt.g_pixelformat(), status, strerror(status));
        }
        else {

          fmt.s_width(frmsize.discrete.width);
          fmt.s_height(frmsize.discrete.height);
          clearColorspace(fmt);

          if( (status = device.try_fmt(fmt)) ) {
            CF_ERROR("device.try_fmt() fails: status=%d (%s)",
                status, strerror(status));

          }
          else if( (status = device.s_fmt(fmt)) ) {
            CF_ERROR("device.s_fmt() fails: status=%d (%s)",
                status, strerror(status));
          }
        }
      }

      c_update_controls_lock lock(this);
      refreshFrameRates(device);
    }
  }
}

void QV4L2CameraControls::onFrameWidthChanged(int value)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device =
          _camera->device();
    }
  }

}

void QV4L2CameraControls::onFrameHeightChanged(int value)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device =
          _camera->device();
    }
  }
}

void QV4L2CameraControls::refreshFrameRates(cv4l_fd & device)
{
  const enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  cv4l_fmt fmt;
  int status;

  if( (status = device.g_fmt(fmt, type)) != 0 ) {

    CF_ERROR("device.g_fmt() fails: status=%d (%s)",
        status, strerror(status));

    remove_control(form, frameRate_ctl);
  }
  else {

    v4l2_frmivalenum frmival = { 0 };

    status =
        device.enum_frameintervals(frmival,
            fmt.g_pixelformat(),
            fmt.g_width(),
            fmt.g_height(),
            0);

    if( status ) {
      CF_DEBUG("device.enum_frameintervals(): status=%d (%s) for format '%s' %ux%u",
          status, strerror(status),
          fourccToString(fmt.g_pixelformat()).c_str(), fmt.g_width(), fmt.g_height());
    }

    if( status || frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE ) {
      remove_control(form, frameRate_ctl);
    }
    else {

      if( frameRate_ctl ) {
        frameRate_ctl->clear();
      }
      else {
        frameRate_ctl = add_combobox("Frame Rate:","", false);
        connect(frameRate_ctl, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this, &ThisClass::onFrameRateChanged);
      }

      v4l2_fract curr = { 1, 1 };

      if( (status = device.get_interval(curr, type)) ) {
        CF_DEBUG("device.get_interval() fails: status= %d (%s) curr=%u/%u",
            status, strerror(status),
            curr.numerator, curr.denominator);
      }

      do {

        frameRate_ctl->addItem(QString("%1 fps")
            .arg((double) frmival.discrete.denominator / frmival.discrete.numerator));

        if( status == 0 && frmival.discrete.numerator == curr.numerator &&
            frmival.discrete.denominator == curr.denominator ) {

          frameRate_ctl->setCurrentIndex(frmival.index);
        }

      } while (device.enum_frameintervals(frmival) == 0);

    }
  }

}

void QV4L2CameraControls::onFrameRateChanged(int index)
{
  if( !updatingControls() ) {
    if( _camera && _camera->state() == QImagingCamera::State_connected ) {

      cv4l_fd &device =
          _camera->device();

      const uint type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      cv4l_fmt fmt;
      int status;

      if( (status = device.g_fmt(fmt, type)) ) {
        CF_ERROR("device.g_fmt() fails: status=%d (%s)",
            status, strerror(status));
      }
      else {

        v4l2_frmivalenum frmival = { 0 };

        status =
            device.enum_frameintervals(frmival,
                fmt.g_pixelformat(),
                fmt.g_width(),
                fmt.g_height(),
                index);

        if( status != 0 ) {
          CF_ERROR("device.enum_frameintervals(index=%d) fails: status=%d (%s)",
              index, status, strerror(status));
        }
        else if( frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE ) {
          CF_ERROR("frmival.type = %d (%s) is not V4L2_FRMIVAL_TYPE_DISCRETE",
              frmival.type, toCString((enum v4l2_frmivaltypes )frmival.type));
        }
        else if( (status = device.set_interval(frmival.discrete, type)) ) {

          CF_ERROR("device.set_interval(%u/%u) fails: status=%d (%s)",
              frmival.discrete.numerator, frmival.discrete.numerator,
              status, strerror(status));

          c_update_controls_lock lock(this);
          refreshFrameRates(device);
        }
      }
    }
  }
}

} /* namespace serimager */
