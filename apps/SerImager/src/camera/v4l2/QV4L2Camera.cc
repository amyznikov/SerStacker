/*
 * QV4L2Camera.cc
 *
 *  Created on: Dec 22, 2022
 *      Author: amyznikov
 */

#include "QV4L2Camera.h"
#include "v4l2_list_devices.h"
#include <poll.h>
#include <core/debug.h>

namespace serimager {


QV4L2Camera::QV4L2Camera(const QString & filename) :
    filename_(filename)
{
}

QV4L2Camera::~QV4L2Camera()
{
  if( device_.g_fd() >= 0 ) {
    device_.close();
  }
}

cv4l_fd & QV4L2Camera::device()
{
  return device_;
}

const cv4l_fd & QV4L2Camera::device() const
{
  return device_;
}

int QV4L2Camera::g_ext_ctrl(v4l2_ext_control & c)
{
  struct v4l2_ext_controls ctrls = {0};
  ctrls.which = V4L2_CTRL_ID2WHICH(c.id);
  ctrls.count = 1;
  ctrls.controls = &c;
  return device_.g_ext_ctrls(ctrls);
}

int QV4L2Camera::s_ext_ctrl(const v4l2_ext_control & c)
{
  struct v4l2_ext_controls ctrls = {0};
  ctrls.which = V4L2_CTRL_ID2WHICH(c.id);
  ctrls.count = 1;
  ctrls.controls = const_cast<v4l2_ext_control*>(&c);
  return device_.s_ext_ctrls(ctrls);
}

int QV4L2Camera::s_ext_ctrl(__u32 cid, __s32 value)
{
  v4l2_ext_control ctrl = { .id = cid };
  ctrl.value = value;
  return s_ext_ctrl(ctrl);
}

int QV4L2Camera::g_ext_ctrl(__u32 cid, __s32 * value)
{
  v4l2_ext_control ctrl = { .id = cid };
  int status = g_ext_ctrl(ctrl);
  if( status == 0 ) {
    *value = ctrl.value;
  }
  return status;
}

int QV4L2Camera::s_ext_ctrl(__u32 cid, __s64 value)
{
  v4l2_ext_control ctrl = { .id = cid };
  ctrl.value64 = value;
  return s_ext_ctrl(ctrl);
}

int QV4L2Camera::g_ext_ctrl(__u32 cid, __s64 * value)
{
  v4l2_ext_control ctrl = { .id = cid };
  int status = g_ext_ctrl(ctrl);
  if( status == 0 ) {
    *value = ctrl.value64;
  }
  return status;
}


int QV4L2Camera::s_ext_ctrl(__u32 cid, bool value)
{
  v4l2_ext_control ctrl = { .id = cid };
  ctrl.value = value;
  return s_ext_ctrl(ctrl);
}

int QV4L2Camera::g_ext_ctrl(__u32 cid, bool * value)
{
  v4l2_ext_control ctrl = { .id = cid };
  int status = g_ext_ctrl(ctrl);
  if( status == 0 ) {
    *value = ctrl.value;
  }
  return status;
}

int QV4L2Camera::s_ext_ctrl(__u32 cid, const QString & value)
{
  v4l2_ext_control ctrl = { .id = cid };
  QByteArray data = value.toUtf8();
  ctrl.string = data.data();
  return s_ext_ctrl(ctrl);
}

int QV4L2Camera::g_ext_ctrl(__u32 cid, QString *value)
{
  char buf[4 * 1024] = "";

  v4l2_ext_control ctrl = { .id = cid };
  ctrl.size = sizeof(buf) - 1;
  ctrl.string = buf;

  int status = g_ext_ctrl(ctrl);
  if( status == 0 ) {
    *value = buf;
  }
  return status;
}


QV4L2Camera::sptr QV4L2Camera::create(const QString & filename)
{
  return sptr(new ThisClass(filename));
}


QList<QImagingCamera::sptr> QV4L2Camera::detectCameras()
{
  QList<QImagingCamera::sptr> devices;

  std::vector<std::string> filenames;

  if( v4l2_list_devices(&filenames) ) {

    for( const std::string &filename : filenames ) {

      static std::vector<std::string> blacklist;

      if( std::find(blacklist.begin(), blacklist.end(), filename) != blacklist.end() ) {
        continue;
      }

      cv4l_fd device;
      v4l2_input vin;

      device.s_trace(false);
      device.s_direct(true);

      if( device.open(filename.c_str()) >= 0 ) {
        if ( device.enum_input(vin, true) != 0 ) {
          blacklist.emplace_back(filename);
        }
        else {
          devices.append(ThisClass::create(filename.c_str()));
        }
        device.close();
      }
    }
  }

  return devices;
}

QString QV4L2Camera::display_name() const
{
  return filename_;
}

//bool QImagingCameraV4L2::check_status()
//{
//  return true;
//}

bool QV4L2Camera::is_same_camera(const QImagingCamera::sptr & rhs) const
{
  const ThisClass * rhsp =
      dynamic_cast<const ThisClass * >(rhs.get());

  return rhsp && rhsp->filename_ == this->filename_;
}

int QV4L2Camera::drops() const
{
  return 0;
}

bool QV4L2Camera::device_is_connected() const
{
  return device_.is_open();
}

bool QV4L2Camera::device_connect()
{
  if( !device_.is_open() ) {

    int status =
        device_.open(filename_.toUtf8().constData());

    if( status < 0 ) {
      CF_ERROR("device_.open('%s') fails: errno=%d (%s)",
          filename_.toUtf8().constData(),
          errno,
          strerror(errno));

      return false;
    }
  }

  return true;
}

void QV4L2Camera::device_disconnect()
{
  if( device_.is_open() ) {
    device_.close();
  }
}

bool QV4L2Camera::device_start()
{
  const uint devtype =
      V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int status;

  if( !device_.is_open() ) {
    return false;
  }

  if( device_.g_type() != devtype ) {
    CF_ERROR("Not a video capture device. device_.g_type()=%d", device_.g_type());
    return false;
  }

  if( (status = device_.g_fmt(srcFormat, devtype)) != 0 ) {
    CF_ERROR("Could not obtain a source format: device_.g_fmt() fails. status=%d (%s)\n",
        status, strerror(status));
    return false;
  }

  if( (status = device_.get_interval(interval, devtype)) ) {
    CF_ERROR("device_.get_interval() fails. status=%d (%s)\n",
        status, strerror(status));
  }

  CF_DEBUG("m_capSrcFormat: %u/%u \n"
      "type=%d\n"
      "width=%u\n"
      "height=%u\n"
      "pixelformat=%u\n"
      "field=%u\n"
      "bytesperline=%u\n"
      "sizeimage=%u\n"
      "colorspace=%u\n"
      "priv=%u\n"
      "flags=%u\n"
      "quantization=%u\n"
      "xfer_func=%u\n"
      ,
      interval.numerator,
      interval.denominator,
      srcFormat.type,
      srcFormat.fmt.pix.width,
      srcFormat.fmt.pix.height,
      srcFormat.fmt.pix.pixelformat,
      srcFormat.fmt.pix.field,
      srcFormat.fmt.pix.bytesperline,
      srcFormat.fmt.pix.sizeimage,
      srcFormat.fmt.pix.colorspace,
      srcFormat.fmt.pix.priv,
      srcFormat.fmt.pix.flags,
      srcFormat.fmt.pix.quantization,
      srcFormat.fmt.pix.xfer_func
      );

  cvType_ = -1;
  colorid_ = COLORID_UNKNOWN;
  // __u32 width, height, pixfmt, field;

  status = 0;

  switch (srcFormat.fmt.pix.pixelformat) {
    case V4L2_PIX_FMT_GREY: /*  8  Greyscale     */
      cvType_ = CV_8UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 8;
      break;
    case V4L2_PIX_FMT_Y4:/*  4  Greyscale     */
      cvType_ = CV_8UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 4;
      break;
    case V4L2_PIX_FMT_Y6: /*  6  Greyscale     */
      cvType_ = CV_8UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 6;
      break;
    case V4L2_PIX_FMT_Y10: /*  10  Greyscale     */
      cvType_ = CV_16UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 10;
      break;
    case V4L2_PIX_FMT_Y12:/* 12  Greyscale     */
      cvType_ = CV_16UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 12;
      break;
    case V4L2_PIX_FMT_Y14: /* 14  Greyscale     */
      cvType_ = CV_16UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 14;
      break;
    case V4L2_PIX_FMT_Y16:/* 16  Greyscale     */
      cvType_ = CV_16UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 16;
      break;
    case V4L2_PIX_FMT_BGR24: /* 24  BGR-8-8-8     */
      cvType_ = CV_8UC3;
      colorid_ = COLORID_BGR;
      bpp_ = 8;
      break;
    case V4L2_PIX_FMT_RGB24: /* 24  RGB-8-8-8     */
      cvType_ = CV_8UC3;
      colorid_ = COLORID_RGB;
      bpp_ = 8;
      break;
    case V4L2_PIX_FMT_Z16: /* Depth data 16-bit */
      cvType_ = CV_16UC1;
      colorid_ = COLORID_MONO;
      bpp_ = 16;
      break;
    default:
      break;
  }

  const bool need_conversion =
      cvType_ == -1;

  dstFormat = srcFormat;

  if( need_conversion ) {
    // conversion required

    dstFormat.s_pixelformat(V4L2_PIX_FMT_BGR24);
    cvType_ = CV_8UC3;
    colorid_ = COLORID_BGR;
    bpp_ = 8;

    // Make sure sizeimage is large enough. This is necessary if the mplane
    // plugin is in use since v4lconvert_try_format() bypasses the plugin.


    dstFormat.s_sizeimage(dstFormat.g_width() * dstFormat.g_height() * 3);

    const cv4l_fmt copy =
        srcFormat;

    if( !(convert_ = v4lconvert_create(device_.g_fd())) ) {
      CF_ERROR("v4lconvert_create() fails");
      status = -1;
    }
    else if( (status = v4lconvert_try_format(convert_, &dstFormat, &srcFormat)) ) {
      CF_ERROR("v4lconvert_try_format() fails: errno=%d (%s) %s",
          errno, strerror(errno),
          v4lconvert_get_error_message(convert_));
    }
    else {
      // v4lconvert_try_format sometimes modifies the source format if it thinks
      // that there is a better format available. Restore our selected source
      // format since we do not want that happening.

      if ( srcFormat.fmt.pix.pixelformat != copy.fmt.pix.pixelformat ) {
        CF_ERROR("v4lconvert_try_format suggest another input format: '%s' instead of '%s'",
            fourccToString(srcFormat.fmt.pix.pixelformat).c_str(),
            fourccToString(copy.fmt.pix.pixelformat).c_str());
        // status = -1;
      }

      srcFormat = copy;
    }
  }

  if ( status == 0 && !create_queue() ) {
    CF_ERROR("create_queue() fails");
    status = -1;
  }

  if( status ) {
    q_.free(&device_);
    if( convert_ ) {
      v4lconvert_destroy(convert_);
      convert_ = nullptr;
    }
  }

  return status == 0;
}

void QV4L2Camera::device_stop()
{
  switch (cap_method_) {
  case cap_method_read:
//    if (v4l_type_is_capture(device.g_type())) {
//      memset(&cmd, 0, sizeof(cmd));
//      cmd.cmd = V4L2_ENC_CMD_STOP;
//      encoder_cmd(cmd);
//    }
    break;

  case cap_method_mmap:
  case cap_method_userptr:
    q_.free(&device_);
    break;
  }

  if ( convert_ ) {
    v4lconvert_destroy(convert_);
    convert_ = nullptr;
  }

}

bool QV4L2Camera::create_queue()
{
  const uint devtype =
      V4L2_BUF_TYPE_VIDEO_CAPTURE;

  const int num_buffers = 8;

  const cv::Size imageSize(dstFormat.g_width(),
      dstFormat.g_height());

  int status;

  q_.init(devtype, cap_method_);

  switch (cap_method_) {
    case cap_method_read:
      // device_.s_priority(m_genTab->usePrio());
      /* Nothing to do. */
      CF_ERROR("cap_method_read not implemented");
      status = -1;
      break;

    case cap_method_mmap:
      case cap_method_userptr:
      if( (status = q_.reqbufs(&device_, num_buffers)) ) {
        CF_ERROR("queue_.reqbufs() fails: status=%d (%s)",
            status, strerror(status));
        break;
      }

      if( q_.g_buffers() < 2 ) {
        CF_ERROR("Too few buffers: %u",
            q_.g_buffers());
        status = -1;
        break;
      }

      if( (status = q_.obtain_bufs(&device_)) ) {
        CF_ERROR("queue_.obtain_bufs() fails: status=%d (%s)",
            status, strerror(status));
        break;
      }

      p_.clear();

      for( uint i = 0, n = q_.g_buffers(); i < n; ++i ) {

        QV4L2CameraFrame::sptr frame =
            QV4L2CameraFrame::create(q_, i,
                imageSize,
                cvType_,
                colorid_,
                bpp_,
                convert_ ? nullptr :
                    q_.g_mmapping(i, 0));

        p_.emplace_back(frame);

        if( (status = device_.qbuf(frame->buf())) ) {
          CF_ERROR("device_.qbuf(%u) fails: status=%d (%s)",
              i, status, strerror(status));
          break;
        }
      }

      if( status == 0 && (status = device_.streamon()) != 0 ) {
        CF_ERROR("device_.streamon() fails: %d (%s)",
            status, strerror(status));
      }
      break;
  }

  return status == 0;
}


bool QV4L2Camera::dqbuf(cv4l_buffer & buf)
{
  int status;

  if( (status = device_.dqbuf(buf)) ) {

    if( (status = errno) != EAGAIN ) {
      CF_ERROR("device_.dqbuf() fails: status=%d (%s)",
          status, strerror(status));
    }
    else {

      struct pollfd pfd = {
          .fd = device_.g_fd(),
          .events = POLLIN | POLLPRI,
          .revents = 0
      };

      while (current_state_ == State_started) {

        if( (status = poll(&pfd, 1, 150)) < 0 ) {
          status = errno;
          CF_ERROR("poll() fails: status=%d (%s)", status, strerror(status));
        }

        CF_DEBUG("poll: status=%d revents=0x%0X", status, pfd.revents);

        if( pfd.revents & POLLIN ) {

          if( (status = device_.dqbuf(buf)) ) {
            CF_ERROR("device_.dqbuf() fails with POLLIN: status=%d (%s)",
                status,
                strerror(status));
          }

          break;
        }

        if( pfd.revents & POLLERR ) {

          CF_ERROR("poll() fails with POLLERR: status=%d (%s)",
              status,
              strerror(status));

          break;
        }

      }
    }
  }

  return status == 0;
}

QCameraFrame::sptr QV4L2Camera::device_recv_frame()
{
  QV4L2CameraFrame::sptr frm;
  cv4l_buffer buf(q_);

  int status = 0;

  switch (cap_method_) {

    case cap_method_read:
      //    int s = read(m_frameData, m_capSrcFormat.g_sizeimage(0));
//
//    if (s < 0) {
//      if (errno != EAGAIN) {
//        error("read");
//        m_capStartAct->setChecked(false);
//      }
//      return;
//    }
//    if (m_makeSnapshot)
//      makeSnapshot((unsigned char *)m_frameData, s);
//    if (m_saveRaw.openMode())
//      m_saveRaw.write((const char *)m_frameData, s);
//
//    plane[0] = m_frameData;
//    if (showFrames() && m_mustConvert) {
//      err = v4lconvert_convert(m_convertData, &m_capSrcFormat, &m_capDestFormat,
//             m_frameData, s,
//             m_capImage->bits(), m_capDestFormat.fmt.pix.sizeimage);
//      if (err != -1)
//        plane[0] = m_capImage->bits();
//    }
      status = -1;
      break;

    case cap_method_mmap:
      case cap_method_userptr:

      if( !dqbuf(buf) ) {
        CF_ERROR("dqbuf() fails");
        status = -1;
      }
      else if( buf.g_flags() & V4L2_BUF_FLAG_ERROR ) {

        CF_ERROR("buf.g_flags() & V4L2_BUF_FLAG_ERROR. errno=%d (%s)",
            errno, strerror(errno));

        status = -1;
      }
      else {

        frm =
            p_[buf.g_index()];

        if( !convert_ ) {
          // frm->data() is already mapped to plane[0]]
          // memcpy(frm->data(), plane[0], frm->size());
        }
        else {

          uint8_t *plane[3] = {
              (__u8*) q_.g_dataptr(buf.g_index(), 0) + buf.g_data_offset(0),
              (__u8*) q_.g_dataptr(buf.g_index(), 1),
              (__u8*) q_.g_dataptr(buf.g_index(), 2)
          };

          uint bytesused[3] = {
              buf.g_bytesused(0) - buf.g_data_offset(0)
          };

          if( plane[1] ) {
            plane[1] += buf.g_data_offset(1);
            bytesused[1] = buf.g_bytesused(1) - buf.g_data_offset(1);
          }

          if( plane[2] ) {
            plane[2] += buf.g_data_offset(2);
            bytesused[2] = buf.g_bytesused(2) - buf.g_data_offset(2);
          }

          // CF_DEBUG("dstFormat.fmt.pix.sizeimage=%u frm->size()=%d", dstFormat.fmt.pix.sizeimage, frm->size());

          status = v4lconvert_convert(convert_, &srcFormat, &dstFormat,
              plane[0], bytesused[0], (uint8_t*) frm->data(),
              dstFormat.fmt.pix.sizeimage);

          if( status < 0 ) {
            CF_ERROR("v4lconvert_convert() fails: errno=%d (%s) %s",
                errno, strerror(errno), v4lconvert_get_error_message(convert_));
          }
          else {
            status = 0;
          }
        }
      }

      if( status ) {
        frm.reset();
        if( (status = device_.qbuf(buf)) ) {
          CF_ERROR("device_.qbuf() fails: status=%d (%s)",
              errno, strerror(errno));
        }
      }
      break;
  }

  return frm;
}

void QV4L2Camera::device_release_frame(const QCameraFrame::sptr & frame)
{
  INSTRUMENT_REGION("");

  if( current_state_ == State_started ) {

    QV4L2CameraFrame *frm =
        dynamic_cast<QV4L2CameraFrame*>(frame.get());

    if( frm ) {

      int status =
          device_.qbuf(frm->buf());

      if( status ) {
        CF_ERROR("device_.qbuf() fails: status=%d (%s)",
            status, strerror(status));
      }
    }
  }
}

int QV4L2Camera::device_max_qsize()
{
  return p_.size() / 2;
}

} /* namespace qserimager */
