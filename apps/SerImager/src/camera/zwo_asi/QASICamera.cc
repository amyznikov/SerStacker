/*
 * QASICamera.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QASICamera.h"
#include <core/get_time.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<ASI_ERROR_CODE>()
{
  static const c_enum_member members[] = {
      { ASI_SUCCESS, "ASI_SUCCESS", "ASI_SUCCESS" },
      { ASI_ERROR_INVALID_INDEX, "ASI_ERROR_INVALID_INDEX", "no camera connected or index value out of boundary" },
      { ASI_ERROR_INVALID_ID, "ASI_ERROR_INVALID_CONTROL_TYPE", "invalid ID" },
      { ASI_ERROR_INVALID_CONTROL_TYPE, "ASI_ERROR_INVALID_CONTROL_TYPE", "invalid control type" },
      { ASI_ERROR_CAMERA_CLOSED, "ASI_ERROR_CAMERA_CLOSED", "camera didn't open" },
      { ASI_ERROR_CAMERA_REMOVED, "ASI_ERROR_CAMERA_REMOVED",
          "failed to find the camera, maybe the camera has been removed" },
      { ASI_ERROR_INVALID_PATH, "ASI_ERROR_INVALID_PATH", "cannot find the path of the file" },
      { ASI_ERROR_INVALID_FILEFORMAT, "ASI_ERROR_INVALID_FILEFORMAT", "" },
      { ASI_ERROR_INVALID_SIZE, "ASI_ERROR_INVALID_SIZE", "wrong video format size" },
      { ASI_ERROR_INVALID_IMGTYPE, "ASI_ERROR_INVALID_IMGTYPE", "unsupported image formate" },
      { ASI_ERROR_OUTOF_BOUNDARY, "ASI_ERROR_OUTOF_BOUNDARY", "the startpos is out of boundary" },
      { ASI_ERROR_TIMEOUT, "ASI_ERROR_TIMEOUT", "timeout" },
      { ASI_ERROR_INVALID_SEQUENCE, "ASI_ERROR_INVALID_SEQUENCE", "stop capture first" },
      { ASI_ERROR_BUFFER_TOO_SMALL, "ASI_ERROR_BUFFER_TOO_SMALL", "buffer size is not big enough" },
      { ASI_ERROR_VIDEO_MODE_ACTIVE, "ASI_ERROR_VIDEO_MODE_ACTIVE", "" },
      { ASI_ERROR_EXPOSURE_IN_PROGRESS, "ASI_ERROR_EXPOSURE_IN_PROGRESS", "" },
      { ASI_ERROR_GENERAL_ERROR, "ASI_ERROR_GENERAL_ERROR", "general error, eg: value is out of valid range" },
      { ASI_ERROR_INVALID_MODE, "ASI_ERROR_INVALID_MODE", "the current mode is wrong" },
      { ASI_ERROR_END },
  };

  return members;
}

template<>
const c_enum_member* members_of<ASI_BAYER_PATTERN>()
{
  static const c_enum_member members[] = {
      { ASI_BAYER_RG, "ASI_BAYER_RG" },
      { ASI_BAYER_BG, "ASI_BAYER_BG" },
      { ASI_BAYER_GR, "ASI_BAYER_GR" },
      { ASI_BAYER_GB, "ASI_BAYER_GB" },
      { (ASI_BAYER_PATTERN)(-1) }
  };

  return members;
}


template<>
const c_enum_member* members_of<ASI_IMG_TYPE>()
{
  static const c_enum_member members[] = {
      { ASI_IMG_RAW8, "RAW8" },
      { ASI_IMG_RGB24, "RGB24" },
      { ASI_IMG_RAW16, "RAW16" },
      { ASI_IMG_Y8, "Y8" },
      { ASI_IMG_END }
  };

  return members;
}

template<>
const c_enum_member* members_of<ASI_EXPOSURE_STATUS>()
{
  static const c_enum_member members[] = {
      { ASI_EXP_IDLE, "ASI_EXP_IDLE" },
      { ASI_EXP_WORKING, "ASI_EXP_WORKING" },
      { ASI_EXP_SUCCESS, "ASI_EXP_SUCCESS" },
      { ASI_EXP_FAILED, "ASI_EXP_FAILED" },
      { ASI_EXP_IDLE }
  };

  return members;
}

template<>
const c_enum_member* members_of<ASI_CAMERA_MODE>()
{
  static const c_enum_member members[] = {
      { ASI_MODE_NORMAL, "ASI_MODE_NORMAL" },
      { ASI_MODE_TRIG_SOFT_EDGE, "ASI_MODE_TRIG_SOFT_EDGE" },
      { ASI_MODE_TRIG_RISE_EDGE, "ASI_MODE_TRIG_RISE_EDGE" },
      { ASI_MODE_TRIG_FALL_EDGE, "ASI_MODE_TRIG_FALL_EDGE" },
      { ASI_MODE_TRIG_SOFT_LEVEL, "ASI_MODE_TRIG_SOFT_LEVEL"},
      { ASI_MODE_TRIG_HIGH_LEVEL, "ASI_MODE_TRIG_HIGH_LEVEL"},
      { ASI_MODE_TRIG_LOW_LEVEL, "ASI_MODE_TRIG_LOW_LEVEL"},
      { ASI_MODE_END }
  };

  return members;
}


template<>
const c_enum_member* members_of<ASI_CONTROL_TYPE>()
{
  static const c_enum_member members[] = {
      { ASI_GAIN, "GAIN", "" },
      { ASI_EXPOSURE, "EXPOSURE", "" },
      { ASI_GAMMA, "GAMMA", "" },
      { ASI_WB_R, "WB_R", "" },
      { ASI_WB_B, "WB_B", "" },
      { ASI_OFFSET, "OFFSET", "" },
      { ASI_BANDWIDTHOVERLOAD, "BANDWIDTHOVERLOAD", "" },
      { ASI_OVERCLOCK, "OVERCLOCK", "" },
      { ASI_TEMPERATURE, "TEMPERATURE", "" }, // return 10*temperature
      { ASI_FLIP, "FLIP", "" },
      { ASI_AUTO_MAX_GAIN, "AUTO_MAX_GAIN", "" },
      { ASI_AUTO_MAX_EXP, "AUTO_MAX_EXP", "" }, //micro second
      { ASI_AUTO_TARGET_BRIGHTNESS, "AUTO_TARGET_BRIGHTNESS", "" }, //target brightness
      { ASI_HARDWARE_BIN, "HARDWARE_BIN", "" },
      { ASI_HIGH_SPEED_MODE, "HIGH_SPEED_MODE", "" },
      { ASI_COOLER_POWER_PERC, "COOLER_POWER_PERC", "" },
      { ASI_TARGET_TEMP, "TARGET_TEMP", "" }, // not need *10
      { ASI_COOLER_ON, "COOLER_ON", "" },
      { ASI_MONO_BIN, "MONO_BIN", "" }, //lead to less grid at software bin mode for color camera
      { ASI_FAN_ON, "FAN_ON", "" },
      { ASI_PATTERN_ADJUST, "PATTERN_ADJUST", "" },
      { ASI_ANTI_DEW_HEATER, "ANTI_DEW_HEATER", "" },
      { -1 }
  };

  return members;
}



namespace serimager {

QASICamera::QASICamera(const ASI_CAMERA_INFO & camInfo, QObject * parent) :
    Base(parent),
    camInfo_(camInfo)
{
}

QASICamera::~QASICamera()
{
  finish();
}

QASICamera::sptr QASICamera::create(const ASI_CAMERA_INFO & camInfo, QObject * parent)
{
  return sptr(new ThisClass(camInfo, parent));
}

const ASI_CAMERA_INFO & QASICamera::cameraInfo() const
{
  return camInfo_;
}

QString QASICamera::display_name() const
{
  return camInfo_.Name;
}

QString QASICamera::parameters() const
{
  if ( camInfo_.CameraID >= 0 ) {

    //    int SupportedBins[16]; //1 means bin1 which is supported by every camera, 2 means bin 2 etc.. 0 is the end of supported binning method
    //    ASI_IMG_TYPE SupportedVideoFormat[8]; //this array will content with the support output format type.IMG_END is the end of supported video format

    static const auto exposure_string =
        [](long asi_exposure) -> std::string {
          if ( asi_exposure >= 1000000 ) {
            return ssprintf("%g s // exposure in seconds", asi_exposure * 1e-6);
          }
          if ( asi_exposure >= 1000 ) {
            return ssprintf("%g ms // exposure in ms", asi_exposure * 1e-3);
          }
          return ssprintf("%ld us // exposure in us", asi_exposure);
        };


    ASI_IMG_TYPE asiType = ASI_IMG_END;
    int iWidth = 0;
    int iHeight = 0;
    int iX = -1;
    int iY = -1;
    int iBin = 0;

    long exposure = -1;
    ASI_BOOL auto_exposure = ASI_FALSE;

    long gain = -1;
    ASI_BOOL auto_gain = ASI_FALSE;

    long gamma = -1;
    ASI_BOOL auto_gamma;

    long wb_r = -1;
    ASI_BOOL auto_wb_r = ASI_FALSE;

    long wb_b = -1;
    ASI_BOOL auto_wb_b = ASI_FALSE;

    long offset = 0;
    ASI_BOOL auto_offset = ASI_FALSE;

    long temperature = 0;
    ASI_BOOL auto_temperature = ASI_FALSE;

    long asi_bandwidthoverload = -1;
    ASI_BOOL auto_asi_bandwidthoverload = ASI_FALSE;

    long asi_overclock = -1;
    ASI_BOOL auto_asi_overclock = ASI_FALSE;

    ASIGetROIFormat(camInfo_.CameraID,
        &iWidth,
        &iHeight,
        &iBin,
        &asiType);

    ASIGetStartPos(camInfo_.CameraID,
        &iX,
        &iY);

    ASIGetControlValue(camInfo_.CameraID, ASI_EXPOSURE,
        &exposure,
        &auto_exposure);

    ASIGetControlValue(camInfo_.CameraID, ASI_GAIN,
        &gain,
        &auto_gain);

    ASIGetControlValue(camInfo_.CameraID, ASI_GAMMA,
        &gamma,
        &auto_gamma);

    ASIGetControlValue(camInfo_.CameraID, ASI_WB_R,
        &wb_r,
        &auto_wb_r);

    ASIGetControlValue(camInfo_.CameraID, ASI_WB_B,
        &wb_b,
        &auto_wb_b);

    ASIGetControlValue(camInfo_.CameraID, ASI_OFFSET,
        &offset,
        &auto_offset);

    ASIGetControlValue(camInfo_.CameraID, ASI_TEMPERATURE,
        &temperature,
        &auto_temperature);

    ASIGetControlValue(camInfo_.CameraID, ASI_BANDWIDTHOVERLOAD,
        &asi_bandwidthoverload,
        &auto_asi_bandwidthoverload);

    ASIGetControlValue(camInfo_.CameraID, ASI_OVERCLOCK,
        &asi_overclock,
        &auto_asi_overclock);



    std::string text =
        ssprintf(""
            "CameraName         = %s\n"
            "CameraID           = %d\n"
            "MaxHeight          = %ld // the max height of the camera\n"
            "MaxWidth           = %ld //the max width of the camera\n"
            "IsColorCam         = %d\n"
            "BayerPattern       = %s\n"
            "PixelSize          = %gum // the pixel size of the camera, unit is um\n"

            "Format             = %s\n"
            "Binning            = %d\n"
            "iX                 = %d\n"
            "iY                 = %d\n"
            "iWidth             = %d\n"
            "iHeight            = %d\n"
            "Exposure           = %s\n"
            "Gain               = %ld\n"
            "WB_R               = %ld\n"
            "WB_B               = %ld\n"
            "Gamma              = %ld\n"
            "Offset             = %ld\n"

            "AutoExposure       = %d\n"
            "AutoGain           = %d\n"
            "AutoGamma          = %d\n"
            "AutoWB_R           = %d\n"
            "AutoWB_B           = %d\n"
            "AutoOffset         = %d\n"

            "Temperature        = %g\n"

            "MechanicalShutter  = %d\n"
            "ST4Port            = %d\n"
            "IsCoolerCam        = %d\n"
            "IsUSB3Host         = %d\n"
            "IsUSB3Camera       = %d\n"
            "ElecPerADU         = %g\n"
            "BitDepth           = %d\n"
            "IsTriggerCam       = %d\n"
            "\n"
            "ASI_BANDWIDTHOVERLOAD      = %ld\n"
            "ASI_BANDWIDTHOVERLOAD_AUTO = %d\n"
            "ASI_OVERCLOCK              = %ld\n"
            "ASI_OVERCLOCK_AUTO         = %d\n"
            "",
            camInfo_.Name,
            camInfo_.CameraID,
            camInfo_.MaxHeight,
            camInfo_.MaxWidth,
            camInfo_.IsColorCam,
            toString(camInfo_.BayerPattern),
            camInfo_.PixelSize,

            toString(asiType),
            iBin,
            iX, iY,
            iWidth, iHeight,

            exposure_string(exposure).c_str(),
            gain,
            wb_r,
            wb_b,
            gamma,
            offset,

            auto_exposure,
            auto_gain,
            auto_gamma,
            auto_wb_r,
            auto_wb_b,
            auto_offset,

            0.1 * temperature,

            camInfo_.MechanicalShutter,
            camInfo_.ST4Port,
            camInfo_.IsCoolerCam,
            camInfo_.IsUSB3Host,
            camInfo_.IsUSB3Camera,
            camInfo_.ElecPerADU,
            camInfo_.BitDepth,
            camInfo_.IsTriggerCam,

            asi_bandwidthoverload,
            auto_asi_bandwidthoverload,

            asi_overclock,
            auto_asi_overclock
            );

    return QString(text.c_str());

  }

  return QString();

}

bool QASICamera::is_same_camera(const QImagingCamera::sptr & rhs) const
{
  const ThisClass *rhsp =
      dynamic_cast<const ThisClass*>(rhs.get());

  if( rhsp ) {
    return this->camInfo_.CameraID ==
        rhsp->camInfo_.CameraID;
  }

  return false;
}

int QASICamera::drops() const
{
  int drops_ = 0;
  if ( is_asi_open_ ) {

    ASI_ERROR_CODE status =
        ASIGetDroppedFrames(camInfo_.CameraID,
            &drops_);

    if ( status != ASI_SUCCESS ) {
      CF_ERROR("ASIGetDroppedFrames(CameraID=%d) fails: %d (%s)",
          camInfo_.CameraID, status, toString(status));
    }
  }

  return drops_;
}

void QASICamera::asi_close()
{
  unique_lock lock(mtx_);
  if ( is_asi_open_ ) {
    CF_DEBUG("ASICloseCamera(CameraID=%d)", camInfo_.CameraID);
    ASICloseCamera(camInfo_.CameraID);
    is_asi_open_ = false;
  }
}
//
//bool QImagingCameraASI::check_status()
//{
//  if( is_asi_open_ ) {
//
//    CF_DEBUG("ASIGetNumOfConnectedCameras()");
//
//    if( current_state_ != State_started && ASIGetNumOfConnectedCameras() < 1 ) {
//      if( current_state_ <= State_connected ) {
//        asi_close();
//      }
//      return false;
//    }
//
//    if( current_state_ <= State_connected ) {
//
//      long lVal = 0;
//      ASI_BOOL bAuto = ASI_TRUE;
//
//      ASI_ERROR_CODE status =
//          ASIGetControlValue(camInfo_.CameraID,
//              ASI_TEMPERATURE,
//              &lVal,
//              &bAuto);
//
//      CF_DEBUG("ASI_TEMPERATURE: status=%d (%s) vaule=%.1f bAuto=%d\n",
//          status, toString(status), lVal / 10.0, bAuto);
//
//      if( status != ASI_SUCCESS ) {
//        asi_close();
//        return false;
//      }
//    }
//  }
//
//  return true;
//}


QList<QImagingCamera::sptr> QASICamera::detectCameras()
{
  QList<QImagingCamera::sptr> cameras;

  const int numConnectedCameras =
      ASIGetNumOfConnectedCameras();

  ASI_CAMERA_INFO info = {0};

  for( int i = 0; i < numConnectedCameras; ++i ) {

    const ASI_ERROR_CODE status =
        ASIGetCameraProperty(&info, i);

    if ( status != ASI_SUCCESS ) {

      CF_ERROR("ASIGetCameraProperty(index=%d/%d) fails: status=%d",
          i, numConnectedCameras,
          status);

      continue;
    }

    cameras.append(ThisClass::create(info));
  }

  return cameras;
}

void QASICamera::qpool(const QCameraFrame::sptr & frame)
{
  if( frame ) {
    p_.emplace_back(frame);
  }
}

QCameraFrame::sptr QASICamera::dqpool()
{
  if( p_.empty() ) {
    CF_ERROR("APP BUG: frame pool pool devastation, must not happen");
    return nullptr;
  }

  QCameraFrame::sptr frm =
      p_.back();

  p_.pop_back();

  return frm;
}


//void QImagingCameraASI::onStateCanged(State oldSate, State newState)
//{
//  Base::onStateCanged(oldSate, newState);
//}
//
bool QASICamera::device_is_connected() const
{
  return is_asi_open_;
}

bool QASICamera::device_connect()
{
  if( !is_asi_open_ ) {

    ASI_ERROR_CODE status;

    CF_DEBUG("ASIOpenCamera(CameraID=%d)", camInfo_.CameraID);

    if( (status = ASIOpenCamera(camInfo_.CameraID)) != ASI_SUCCESS ) {

      CF_ERROR("QImagingCameraASI:\n"
          "ASIOpenCamera(CameraID=%d) fails.\n"
          "Status=%d (%s)",
          camInfo_.CameraID,
          status,
          toString(status));
    }
    else if( (status = ASIInitCamera(camInfo_.CameraID)) != ASI_SUCCESS ) {

      CF_ERROR("QImagingCameraASI:\n"
          "ASIInitCamera(CameraID=%d) fails.\n"
          "Status=%d (%s)",
          camInfo_.CameraID,
          status,
          toString(status));

      CF_DEBUG("ASICloseCamera(CameraID=%d)", camInfo_.CameraID);
      ASICloseCamera(camInfo_.CameraID);
    }
    else {
      is_asi_open_ = true;
    }
  }

  return is_asi_open_;
}

void QASICamera::device_disconnect()
{
  if ( is_asi_open_ ) {
    CF_DEBUG("ASICloseCamera(CameraID=%d)", camInfo_.CameraID);
    ASICloseCamera(camInfo_.CameraID);
    is_asi_open_ = false;
  }
}

bool QASICamera::device_start()
{
  cv::Size frameSize;
  int iBin = 0;
  ASI_IMG_TYPE asiType = ASI_IMG_END;
  COLORID colorid = COLORID_UNKNOWN;
  int cvType = 0;
  int bpp = 0;

  ASI_ERROR_CODE status;

  CF_DEBUG("BitDepth=%d", camInfo_.BitDepth);

  status =
      ASIGetROIFormat(camInfo_.CameraID,
          &frameSize.width, &frameSize.height,
          &iBin,
          &asiType);

  if( status != ASI_SUCCESS ) {
    CF_ERROR("QImagingCameraASI:\n"
        "ASIGetROIFormat(CameraID=%d) fails.\n"
        "Status=%d (%s)",
        camInfo_.CameraID,
        status,
        toString(status));

    if( status == ASI_ERROR_CAMERA_CLOSED ) {
      asi_close();
    }
    else if( status == ASI_ERROR_INVALID_ID ) {
      is_asi_open_ = false;
    }

    return false;
  }

  switch (asiType) {
    case ASI_IMG_RAW8:
      cvType = CV_8UC1;
      bpp = 8;
      if( !camInfo_.IsColorCam ) {
        colorid = COLORID_MONO;
      }
      else {
        switch (camInfo_.BayerPattern) {
          case ASI_BAYER_RG:
            colorid = COLORID_BAYER_RGGB;
            break;
          case ASI_BAYER_BG:
            colorid = COLORID_BAYER_BGGR;
            break;
          case ASI_BAYER_GR:
            colorid = COLORID_BAYER_GRBG;
            break;
          case ASI_BAYER_GB:
            colorid = COLORID_BAYER_GBRG;
            break;
        }
      }
      break;
    case ASI_IMG_RAW16:
      cvType = CV_16UC1;
      //bpp = camInfo_.BitDepth;
      bpp = 16;
      if( !camInfo_.IsColorCam ) {
        colorid = COLORID_MONO;
      }
      else {
        switch (camInfo_.BayerPattern) {
          case ASI_BAYER_RG:
            colorid = COLORID_BAYER_RGGB;
            break;
          case ASI_BAYER_BG:
            colorid = COLORID_BAYER_BGGR;
            break;
          case ASI_BAYER_GR:
            colorid = COLORID_BAYER_GRBG;
            break;
          case ASI_BAYER_GB:
            colorid = COLORID_BAYER_GBRG;
            break;
        }
      }
      break;
    case ASI_IMG_RGB24:
      cvType = CV_8UC3;
      bpp = 8;
      colorid = COLORID_BGR;
      break;
    case ASI_IMG_Y8:
      bpp = 8;
      cvType = CV_8UC1;
      colorid = COLORID_MONO;
      break;
    default:
      CF_ERROR("Unsupported asi image type %d encountered", asiType);
      return false;
  }

  if( !create_frame_buffers(frameSize, cvType, colorid, bpp, 8) ) {
    CF_ERROR("create_frame_buffers() fails");
    return false;
  }

  if( (status = ASIStartVideoCapture(camInfo_.CameraID)) != ASI_SUCCESS ) {

    CF_ERROR("QImagingCameraASI:\n"
        "ASIStartVideoCapture(CameraID=%d) fails.\n"
        "Status=%d (%s)",
        camInfo_.CameraID,
        status,
        toString(status));

    return false;
  }

  return true;
}

int QASICamera::device_max_qsize()
{
  return p_.size() / 2;
}

void QASICamera::device_stop()
{
  ASI_ERROR_CODE status =
      ASIStopVideoCapture(camInfo_.CameraID);

  if( status != ASI_SUCCESS ) {

    CF_ERROR("ASIStopVideoCapture(CameraID=%d) fails.\n"
        "Status=%d (%s)",
        camInfo_.CameraID,
        status,
        toString(status));
  }

  Q_EMIT exposureStatusUpdate(Exposure_idle, 0, 0);
}

QCameraFrame::sptr QASICamera::device_recv_frame()
{
  INSTRUMENT_REGION("");

  QCameraFrame::sptr frm =
      dqpool();

  if( frm ) {
    INSTRUMENT_REGION("ASIGetVideoData");

    ASI_ERROR_CODE status;

    ASI_BOOL auto_exposure = ASI_FALSE;
    long exposure = -1;

    status =
        ASIGetControlValue(camInfo_.CameraID, ASI_EXPOSURE,
            &exposure,
            &auto_exposure);

    if( status ) {
      CF_ERROR("ASIGetControlValue(ASI_EXPOSURE) fails: status=%d (%s)",
          status, toString(status));
    }

    const bool is_long_exposure =
        exposure > 1 * 1000 * 1000;

    const double exposure_time_ms =
        exposure * 1e-3;

    const double start_time_ms =
        get_realtime_ms();

    if( is_long_exposure ) {
      Q_EMIT exposureStatusUpdate(Exposure_working,
          exposure_time_ms, 0);
    }

    while (is_asi_open_ && current_state_ == State_started) {

      status =
          ASIGetVideoData(camInfo_.CameraID,
              (uint8_t*) frm->data(),
              frm->size(),
              500);

      if( status == ASI_SUCCESS ) {

        if( is_long_exposure ) {
          Q_EMIT exposureStatusUpdate(Exposure_success,
              exposure_time_ms,
              get_realtime_ms() - start_time_ms);
        }

        return frm;
      }

      if( status == ASI_ERROR_TIMEOUT ) {

        ASI_EXPOSURE_STATUS expStatus =
            ASI_EXP_IDLE;

        status =
            ASIGetExpStatus(camInfo_.CameraID,
                &expStatus);

        if( status || expStatus != ASI_EXP_WORKING ) {
          CF_DEBUG("ASIGetExpStatus: %s, expStatus=%d", toString(status), expStatus);
          break;
        }

        if( is_long_exposure ) {
          Q_EMIT exposureStatusUpdate(Exposure_working,
              exposure_time_ms,
              get_realtime_ms() - start_time_ms);
        }

        continue;
      }

      CF_DEBUG("ASIGetVideoData: status=%d (%s) is_open_=%d data: %p size=%d ", status, toString(status), is_asi_open_,
          frm->data(), frm->size());

      if( is_long_exposure ) {
        Q_EMIT exposureStatusUpdate(Exposure_failed,
            exposure_time_ms,
            get_realtime_ms() - start_time_ms);
      }

      break;
    }

    qpool(frm);
  }

  return nullptr;
}

void QASICamera::device_release_frame(const QCameraFrame::sptr & frame)
{
  qpool(frame);
}

bool QASICamera::create_frame_buffers(const cv::Size & imageSize, int cvType, enum COLORID colorid, int bpp,  int num_buffers)
{
  p_.clear();

  for( int i = 0; i < num_buffers; ++i ) {
    p_.emplace_back(QCameraFrame::create(imageSize,
        cvType, colorid, bpp));
  }

  return true;
}

} /* namespace serimager */
