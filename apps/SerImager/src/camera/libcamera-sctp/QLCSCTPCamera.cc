/*
 * QLCSCTPCamera.cc
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#include "QLCSCTPCamera.h"
#include <netinet/sctp.h>
#include <core/io/iface.h>
#include <core/settings.h>
#include <core/debug.h>
#include "unpack-libcamera-image.h"

namespace serimager {

using c_sctp = QLCSCTPCamera::c_sctp;
using ImageHeader = c_sctp::ImageHeader;

static inline bool load_settings(c_config_setting cfg, const std::string & name, QString * v)
{
  v->clear();

  std::string s;
  if( load_settings(cfg, name, &s) && !s.empty() ) {
    *v = QString::fromStdString(s);
  }

  return !v->isEmpty();
}

static inline bool load_settings(c_config_setting cfg, QString * v)
{
  v->clear();

  std::string s;
  if( load_settings(cfg, &s) && !s.empty() ) {
    *v = QString::fromStdString(s);
  }

  return !v->isEmpty();
}

bool c_sctp::connected() const
{
  return _so >= 0;
}

void c_sctp::disconnect()
{
  if ( _so >= 0 ) {
    shutdown(_so, SHUT_RDWR );
    close(_so);
    _so = -1;
  }
}

int c_sctp::connect(uint32_t address, uint16_t port)
{
  struct sockaddr_in sin = {0};
  struct sctp_event_subscribe events = {0};
  struct sctp_initmsg initmsg = {0};
  bool fOk = false;
  int so = -1;

  disconnect();

  if( (so = socket(AF_INET, SOCK_STREAM, IPPROTO_SCTP)) < 0 ) {
    CF_ERROR("socket(AF_INET, SOCK_STREAM, IPPROTO_SCTP) fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  if ( true ) {
    int buffsize = 8 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if (setsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    buffsize = 0;
    buffsizelen = sizeof(buffsize);
    if (getsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, &buffsizelen) < 0) {
      CF_ERROR("getsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_RCVBUF: %d", buffsize);
  }

  if ( true ) {
    int buffsize = 8 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if (setsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    buffsize = 0;
    buffsizelen = sizeof(buffsize);
    if (getsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, &buffsizelen) < 0) {
      CF_ERROR("getsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_SNDBUF: %d", buffsize);
  }

  if ( true ) {
    // Set to disable (freq=1, delay=0)
    struct sctp_sack_info sack_info = {0};
    socklen_t len = sizeof(sack_info);
    sack_info.sack_delay = 20;
    sack_info.sack_freq = 2; // Sets frequency to 1 for immediate ACKs
    sack_info.sack_assoc_id = SCTP_ALL_ASSOC; // Or specific assoc ID
    if ( setsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, len) < 0 ) {
      CF_DEBUG("setsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno) );
    }
  }
  if ( true ) {
    struct sctp_sack_info sack_info = {0};
    socklen_t len = sizeof(sack_info);
    if ( getsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, &len) < 0 ) {
      CF_DEBUG("getsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno) );
    }
    CF_DEBUG("2: SCTP_DELAYED_SACK: Delay: %u, Freq: %u", sack_info.sack_delay, sack_info.sack_freq);
  }



  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = htonl(address);
  sin.sin_port = htons(port);

  initmsg.sinit_num_ostreams = 2;
  initmsg.sinit_max_instreams = 2;
  initmsg.sinit_max_attempts = 2;
  initmsg.sinit_max_init_timeo = 5000; // [ms][
  if( setsockopt(so, IPPROTO_SCTP, SCTP_INITMSG, &initmsg, sizeof(initmsg)) < 0 ) {
    CF_ERROR("setsockopt(SCTP_INITMSG) fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  events.sctp_data_io_event = 1;      // Required to get stream info in sctp_sndrcvinfo
  events.sctp_association_event = 1;  // Receive notifications about connection status
  events.sctp_shutdown_event = 1;     // Receive notifications when peer shuts down
  if (setsockopt(so, IPPROTO_SCTP, SCTP_EVENTS, &events, sizeof(events)) < 0) {
    CF_ERROR("setsockopt(SCTP_EVENTS) fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  if( ::connect(so, (struct sockaddr*)&sin, sizeof(sin)) < 0 ) {
    CF_ERROR("connect() fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  fOk = true;
__end:

  if( !fOk ) {
    if( so != -1 ) {
      close(so), so = -1;
    }
  }

  return (_so = so);
}

const std::string & c_sctp::smsg() const
{
  return _smsg;
}

const c_sctp::ImageHeader& c_sctp::imghdr() const
{
  return reinterpret_cast<const c_sctp::ImageHeader&>(*_imghdr.data());
}

const std::vector<uint8_t> & c_sctp::imgdata() const
{
  return _imgdata;
}

void c_sctp::clear_smsg()
{
  _smsg.clear();
}

void c_sctp::clear_image()
{
  _imghdr.clear();
  _imgdata.clear();
}

bool c_sctp::sendmsg(const std::string & msg)
{
  if( sctp_sendmsg(_so, msg.data(), msg.size() + 1, nullptr, 0, 0, 0, 0, 0, 0) < 0 ) {
    CF_ERROR("sctp_sendmsg() fails. errno=%d %s", errno, strerror(errno));
    return false;
  }
  return true;
}

int c_sctp::recvmsg()
{
  constexpr uint32_t PPID_TEXT = 0;
  constexpr uint32_t PPID_IMGHDR = 1;
  constexpr uint32_t PPID_IMGCHUNK = 2;
  constexpr uint32_t TMPBUFFSIZE = 256 * 1024;

  if ( _buf.size() < TMPBUFFSIZE) {
    _buf.resize(TMPBUFFSIZE);
  }

  CF_DEBUG("beg loop");
  while (42) {
    struct sctp_sndrcvinfo sinfo = { 0 };
    int flags = 0;
    errno = 0;

    //CF_DEBUG("sctp_recvmsg()");
    const int cb = sctp_recvmsg(_so, _buf.data(), _buf.size(), nullptr, nullptr, &sinfo, &flags);
    //CF_ERROR("cb=%d stream=%u ppid=%u flags=0x%0X errno=%d (%s)", cb, sinfo.sinfo_stream, ntohl(sinfo.sinfo_ppid), flags, errno, strerror(errno));
    if( cb <= 0 ) {
      CF_ERROR("sctp_recvmsg() fails: cb=%d stream=%u flags=0x%0X errno=%d (%s)", cb, sinfo.sinfo_stream, flags, errno, strerror(errno));
      break;
    }

    if ( flags & MSG_NOTIFICATION ) {
      CF_ERROR("MSG_NOTIFICATION: cb=%d stream=%u ppid=%u flags=0x%0X errno=%d (%s)", cb, sinfo.sinfo_stream, ntohl(sinfo.sinfo_ppid), flags, errno, strerror(errno));
      continue;
    }

    const uint32_t ppid = ntohl(sinfo.sinfo_ppid);
    if( sinfo.sinfo_stream == 0 ) {
      _smsg.append(_buf.begin(), _buf.begin() + cb);
      if( flags & MSG_EOR ) {
        CF_DEBUG("MSG_EOR: cb=%d stream=%u flags=0x%0X", cb, sinfo.sinfo_stream, flags);
        return sinfo.sinfo_stream;
      }
      continue;
    }


    if( sinfo.sinfo_stream == 1 ) {

      if( ppid == PPID_IMGHDR ) {
        const size_t old_size = _imghdr.size();
        _imghdr.resize(old_size + cb);
        memcpy(_imghdr.data() + old_size, _buf.data(), cb);

        _imgdata.clear();
        if( (flags & MSG_EOR) && (_imghdr.size() != sizeof(ImageHeader))  ) {
          CF_DEBUG("Bad Image header size: _imghdr.size()=%zu sizeof(ImageHeader)=%zu", _imghdr.size(), sizeof(ImageHeader));
          _imghdr.clear();
          _imghdr.clear();
        }
        else {
          //CF_DEBUG("HOT HDR datasize=%u", imghdr().datasize);
        }

        continue;
      }

      if( ppid == PPID_IMGCHUNK ) {
        if( _imghdr.size() != sizeof(ImageHeader) ) {
          CF_DEBUG("Image chunk with no header - ignored");
        }
        else {
          const size_t old_size = _imgdata.size();
          _imgdata.resize(old_size + cb);
          memcpy(_imgdata.data() + old_size, _buf.data(), cb);

          const ImageHeader & hdr = imghdr();
          //CF_DEBUG("image chunk: hdr.datasize=%u _imgdata.size()=%zu", hdr.datasize, _imgdata.size());

          if( _imgdata.size() == hdr.datasize ) {
            CF_DEBUG("return image");
            return 1;
          }

          if ( _imgdata.size() > hdr.datasize ) {
            CF_DEBUG("bad image size: hdr.datasize=%u _imgdata.size()=%zu", hdr.datasize, _imgdata.size());
            _imgdata.clear();
            _imghdr.clear();
          }
        }
      }
      continue;
    }

    CF_DEBUG("Bad stream index=%u. ignored", sinfo.sinfo_stream);
  }

  CF_DEBUG("end loop");
  return -1;
}

QLCSCTPCamera::QLCSCTPCamera(const QString & name, QObject * parent) :
    ThisClass(name, "", parent)
{
}

QLCSCTPCamera::QLCSCTPCamera(const QString & name, const QString & url, QObject * parent) :
    Base(parent),
    _name(name),
    _url(url)
{
}

QLCSCTPCamera::~QLCSCTPCamera()
{
  finish();
}

void QLCSCTPCamera::setName(const QString & name)
{
  _name = name;
  Q_EMIT parametersChanged();
}

const QString& QLCSCTPCamera::name() const
{
  return _name;
}

void QLCSCTPCamera::setUrl(const QString & v)
{
  _url = v;
  Q_EMIT parametersChanged();
}

const QString & QLCSCTPCamera::url() const
{
  return _url;
}

QLCSCTPCamera::sptr QLCSCTPCamera::create(const QString & name, const QString & url, QObject * parent)
{
  ThisClass::sptr camera(new ThisClass(name, url, parent));
  return camera;
}

QString QLCSCTPCamera::display_name() const
{
  return _name;
}

QString QLCSCTPCamera::parameters() const
{
  return QString();
}

bool QLCSCTPCamera::is_same_camera(const QImagingCamera::sptr & rhs) const
{
  const ThisClass *rhsp =
      dynamic_cast<const ThisClass*>(rhs.get());

  return rhsp == this;
}

int QLCSCTPCamera::drops() const
{
  return 0;
}

const QList<QLCSCTPCamera::QLCCamera> & QLCSCTPCamera::cameras() const
{
  return _cameras;
}

QLCSCTPCamera::QLCCamera * QLCSCTPCamera::selectedCamera()
{
  return _selectedCameraIndex >= 0 && _selectedCameraIndex < _cameras.size() ?
      &_cameras[_selectedCameraIndex] :
      nullptr;
}

const QLCSCTPCamera::QLCCamera * QLCSCTPCamera::selectedCamera() const
{
  return _selectedCameraIndex >= 0 && _selectedCameraIndex < _cameras.size() ?
      &_cameras[_selectedCameraIndex] :
      nullptr;
}

void QLCSCTPCamera::setSelectedCameraIndex(int v)
{
  _selectedCameraIndex = v;
}

int QLCSCTPCamera::selectedCameraIndex() const
{
  return _selectedCameraIndex;
}

QLCSCTPCamera::QLCCameraStream* QLCSCTPCamera::selectedStream()
{
  QLCCamera * cam = selectedCamera();
  return (cam && cam->selectedStreamIndex >= 0 && cam->selectedStreamIndex < cam->streams.size()) ?
      &cam->streams[cam->selectedStreamIndex] :
      nullptr;
}

const QLCSCTPCamera::QLCCameraStream * QLCSCTPCamera::selectedStream() const
{
  const QLCCamera * cam = selectedCamera();
  return (cam && cam->selectedStreamIndex >= 0 && cam->selectedStreamIndex < cam->streams.size()) ?
      &cam->streams[cam->selectedStreamIndex] :
      nullptr;
}

void QLCSCTPCamera::setSelectedStreamIndex(int index)
{
  QLCCamera * cam = selectedCamera();
  if( cam ) {
    cam->selectedStreamIndex = std::min(std::max(0, index), cam->streams.size() - 1);
    CF_DEBUG("cam->selectedStreamIndex=%d", cam->selectedStreamIndex);
  }
}

int QLCSCTPCamera::selectedStreamIndex() const
{
  const QLCCamera * cam = selectedCamera();
  return cam ? cam->selectedStreamIndex : -1;
}

QLCSCTPCamera::QLCCameraPixFormats * QLCSCTPCamera::selectedFormat()
{
  QLCCameraStream * strm = selectedStream();
  return (strm && strm->selectedFormatIndex >= 0 && strm->selectedFormatIndex < strm->formats.size()) ?
      &strm->formats[strm->selectedFormatIndex] :
      nullptr;
}

const QLCSCTPCamera::QLCCameraPixFormats * QLCSCTPCamera::selectedFormat() const
{
  const QLCCameraStream * strm = selectedStream();
  return (strm && strm->selectedFormatIndex >= 0 && strm->selectedFormatIndex < strm->formats.size()) ?
      &strm->formats[strm->selectedFormatIndex] :
      nullptr;
}

void QLCSCTPCamera::setSelectedFormatIndex(int index)
{
  QLCCameraStream * strm = selectedStream();
  if( strm ) {
    strm->selectedFormatIndex = std::min(std::max(0, index), strm->formats.size() - 1);
    CF_DEBUG("strm->selectedFormatIndex=%d", strm->selectedFormatIndex);
  }
}

int QLCSCTPCamera::selectedFormatIndex() const
{
  const QLCCameraStream * strm = selectedStream();
  return strm ? strm->selectedFormatIndex : -1;
}


QString * QLCSCTPCamera::selectedSize()
{
  QLCCameraPixFormats * fmt = selectedFormat();
  return fmt && fmt->selectedSizeIndex >= 0 && fmt->selectedSizeIndex < fmt->sizes.size() ?
      &fmt->sizes[fmt->selectedSizeIndex] :
      nullptr;
}

const QString * QLCSCTPCamera::selectedSize() const
{
  const QLCCameraPixFormats * fmt = selectedFormat();
  return fmt && fmt->selectedSizeIndex >= 0 && fmt->selectedSizeIndex < fmt->sizes.size() ?
      &fmt->sizes[fmt->selectedSizeIndex] :
      nullptr;
}

void QLCSCTPCamera::setSelectedSizeIndex(int index)
{
  QLCCameraPixFormats * fmt = selectedFormat();
  if ( fmt ) {
    fmt->selectedSizeIndex = std::min(std::max(0, index), fmt->sizes.size() - 1);
    CF_DEBUG("fmt->selectedSizeIndex=%d", fmt->selectedSizeIndex);
  }
}

int QLCSCTPCamera::selectedSizeIndex() const
{
  const QLCCameraPixFormats * fmt = selectedFormat();
  return fmt ? fmt->selectedSizeIndex : -1;
}

int QLCSCTPCamera::cameraDeviceBuffers() const
{
  return _cameraDeviceBuffers;
}

void QLCSCTPCamera::setCameraDeviceBuffers(int v)
{
  _cameraDeviceBuffers = v;
}


bool QLCSCTPCamera::device_is_connected() const
{
  return _sctp.connected();
}

bool QLCSCTPCamera::device_connect()
{
  if ( !_sctp.connected() ) {

    uint32_t address = 0;
    uint16_t port = 0;

    if ( !cf_get_iface_address(_url.toUtf8().constData(), &address, &port) ) {
      CF_ERROR("cf_get_iface_address('%s') fails: %s", _url.toUtf8().constData(), strerror(errno));
      return false;
    }

    if ( !_sctp.connect(address, port) ) {
      CF_ERROR("so_sctp_connect('%s') fails: %s", _url.toUtf8().constData(), strerror(errno));
      return false;
    }

    CF_DEBUG("Connected to '%s'", _url.toUtf8().constData());

    if ( !_sctp.sendmsg("command=\"get-cameras\";") ) {
      CF_ERROR("send('%s', get_cameras) fails: %s", _url.toUtf8().constData(), strerror(errno));
      device_disconnect();
      return false;
    }

    CF_DEBUG("msg sent");

    c_config cfg;
    _sctp.clear_smsg();
    const int istream = _sctp.recvmsg();
    if( istream != 0 ) {
      CF_ERROR("_sctp.recvmsg() return bad istream=%d", istream);
      device_disconnect();
      return false;
    }

    if ( !cfg.read_string(_sctp.smsg().c_str()) ) {
      CF_ERROR("cfg.read_string() fails");
      device_disconnect();
      return false;
    }

    CF_DEBUG("config parsed\n%s\n", _sctp.smsg().c_str());

    c_config_setting root = cfg.root();
    c_config_setting cameras_item = root.get_list("cameras");

    QString id, model, role, pixfmt, size;

    _cameras.clear();

    for ( int icam = 0, ncams = cameras_item.length(); icam < ncams; ++icam ) {

      const c_config_setting camera_item = cameras_item[icam];
      load_settings(camera_item, "id", &id);
      load_settings(camera_item, "model", &model);
      if( id.isEmpty() ) {
        continue;
      }

      _cameras.append(QLCCamera(id, model));
      QLCCamera & cam = _cameras.back();

      const c_config_setting roles_item = camera_item["roles"];

      CF_DEBUG("roles_item.length()=%d", roles_item.length());

      for( int irole = 0, nroles = roles_item.length(); irole < nroles; ++irole ) {
        const c_config_setting role_item = roles_item[irole];

        load_settings(role_item, "role", &role);
        if( role.isEmpty() ) {
          continue;
        }

        cam.streams.append(role);
        QLCCameraStream & strm = cam.streams.back();

        const c_config_setting pixfmts_item = role_item["pixfmts"];
        for( int ifmt = 0, nfmts = pixfmts_item.length(); ifmt < nfmts; ++ifmt ) {
          const c_config_setting pixfmt_item = pixfmts_item[ifmt];

          load_settings(pixfmt_item, "format", &pixfmt);
          if( pixfmt.isEmpty() ) {
            continue;
          }

          strm.formats.append(pixfmt);
          QLCCameraPixFormats & fmt = strm.formats.back();

          const c_config_setting sizes_item = pixfmt_item["sizes"];
          for( int isz = 0, nszs = sizes_item.length(); isz < nszs; ++isz ) {
            if ( load_settings(sizes_item[isz], &size) ) {
              fmt.sizes.append(size);
            }
          }

          fmt.selectedSizeIndex = fmt.sizes.empty() ? -1 : 0;
        }
        strm.selectedFormatIndex = strm.formats.empty() ? -1 : 0;
      }
      cam.selectedStreamIndex = cam.streams.empty() ? -1 : 0;
    }

    if( _cameras.isEmpty() ) {
      _selectedCameraIndex = -1;
    }
    else if( _selectedCameraIndex < 0 || _selectedCameraIndex >= _cameras.size() ) {
      _selectedCameraIndex = 0;
    }
  }

  return true;
}

void QLCSCTPCamera::device_disconnect()
{
  _sctp.disconnect();
  CF_DEBUG("Disconnected '%s'", _url.toUtf8().constData());
}

bool QLCSCTPCamera::device_start()
{
  if ( !_sctp.connected() ) {
    CF_ERROR("Device is not connected");
    errno = ENOTCONN;
    return false;
  }


  const auto * cam = selectedCamera();
  const auto * strm = selectedStream();
  const auto * fmt = selectedFormat();
  const auto * size = selectedSize();
  const int nbuffers = std::max(1, cameraDeviceBuffers());

  const std::string command =
      ssprintf("command=\"start-camera\";\n"
          "name=\"%s\";\n"
          "role=\"%s\";\n"
          "format=\"%s\";\n"
          "size=\"%s\"\n"
          "buffers=%d;\n",
          cam ? cam->id.toUtf8().constData() : "",
          strm ? strm->role.toUtf8().constData() : "",
          fmt ? fmt->format.toUtf8().constData() : "",
          size ? size->toUtf8().constData() : "",
          nbuffers);

  CF_DEBUG("send:\n%s\n",command.c_str());
  if( !_sctp.sendmsg(command) ) {
    CF_ERROR("send(start-camera) fails. errno=%d (%s)", errno, strerror(errno));
    device_disconnect();
    return false;
  }

  return true;
}

void QLCSCTPCamera::device_stop()
{
  CF_DEBUG("device_stop()");
  if( !_sctp.sendmsg("command=\"stop-camera\";") ) {
    CF_ERROR("_sctp.sendmsg(stop-camera) fails. errno=%d (%s)", errno, strerror(errno));
    device_disconnect();
  }
  CF_DEBUG("_sctp.connected=%d", _sctp.connected());
}

int QLCSCTPCamera::device_max_qsize()
{
  return 2;//_p.size() / 2;
}

void QLCSCTPCamera::device_release_frame(const QCameraFrame::sptr & frame)
{
  //qpool(frame);
  //frame.reset();
}

bool QLCSCTPCamera::device_recv_frame(QCameraFrame::sptr & ofrm)
{
  if ( !_sctp.connected() ) {
    CF_ERROR("Device is not connected");
    errno = ENOTCONN;
    return false;
  }

  while (_sctp.connected()) {

    const int istream = _sctp.recvmsg();
    if (istream < 0 || istream > 1 ) {
      CF_ERROR("_sctp.recvmsg(): bad istream=%d", istream);
      break;
    }
    if (istream == 0 ) {
      CF_DEBUG("smsg:\n%s\n", _sctp.smsg().c_str());
      _sctp.clear_smsg();
      continue;
    }
    if (istream == 1 ) {
      const ImageHeader & hdr = _sctp.imghdr();
      const std::vector<uint8_t> & data = _sctp.imgdata();

      CF_DEBUG("image: %ux%u size=%u data: %zu", hdr.width, hdr.height, hdr.datasize, data.size());


      cv::Mat image;
      COLORID colorid = COLORID_UNKNOWN;
      int bpp = 0;

      bool fOk =
          unpack_libcamera_image(data,
              hdr.width, hdr.height, hdr.stride,
              hdr.fourcc, hdr.fourcc_modifier,
              image,
              &colorid,
              &bpp);

      if ( !fOk ) {
        CF_ERROR("libcamera_unpack_image() fails");
      }
      else {
        CF_DEBUG("unpack ok");
        ofrm = QCameraFrame::create(image, colorid, hdr.bpp);
      }

      _sctp.clear_image();
      return true;
    }
  }

  return false;
}

} // namespace serimager
