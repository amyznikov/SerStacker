/*
 * QASICamera.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "QASICamera.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<ASI_ERROR_CODE>()
{
  static constexpr c_enum_member members[] = {
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
      { ASI_ERROR_END, nullptr },
  };

  return members;
}

template<>
const c_enum_member* members_of<ASI_IMG_TYPE>()
{
  static constexpr c_enum_member members[] = {
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
  static constexpr c_enum_member members[] = {
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
  static constexpr c_enum_member members[] = {
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




namespace serimager {

QASICamera::QASICamera(const ASI_CAMERA_INFO & camInfo, QObject * parent) :
    Base(parent),
    camInfo_(camInfo)
{
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
  INSTRUMENT_REGION("");
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
}

QCameraFrame::sptr QASICamera::device_recv_frame()
{
  INSTRUMENT_REGION("");

  QCameraFrame::sptr frm =
      dqpool();

  if( frm ) {
    INSTRUMENT_REGION("ASIGetVideoData");

    while (is_asi_open_ && current_state_ == State_started) {

      ASI_ERROR_CODE status =
          ASIGetVideoData(camInfo_.CameraID,
              (uint8_t*) frm->data(),
              frm->size(),
              100);

      if( status == ASI_SUCCESS ) {
        return frm;
      }

      if( status == ASI_ERROR_TIMEOUT ) {
        // CF_DEBUG("ASI_ERROR_TIMEOUT");
        continue;
      }

      CF_DEBUG("ASIGetVideoData: status=%d (%s) is_open_=%d data: %p size=%d ", status, toString(status), is_asi_open_,
          frm->data(), frm->size());
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

} /* namespace qserimager */
