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

//using c_sctp = QLCSCTPCamera::c_sctp_connection;
//using ImageHeader = c_sctp::ImageHeader;

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
  }
}

int QLCSCTPCamera::selectedSizeIndex() const
{
  const QLCCameraPixFormats * fmt = selectedFormat();
  return fmt ? fmt->selectedSizeIndex : -1;
}


QLCSCTPCamera::QLCCameraControl * QLCSCTPCamera::QLCCamera::getControl(const QString & id)
{
  for( int i = 0, n = contols.size(); i < n; ++i ) {
    if( contols[i].id == id ) {
      return &contols[i];
    }
  }
  return nullptr;
}

const QLCSCTPCamera::QLCCameraControl * QLCSCTPCamera::QLCCamera::getControl(const QString & id) const
{
  for( int i = 0, n = contols.size(); i < n; ++i ) {
    if( contols[i].id == id ) {
      return &contols[i];
    }
  }
  return nullptr;
}

QLCSCTPCamera::QLCCameraControl* QLCSCTPCamera::getControl(const QString & id)
{
  QLCCamera * cam = selectedCamera();
  return cam ? cam->getControl(id) : nullptr;
}

const QLCSCTPCamera::QLCCameraControl* QLCSCTPCamera::getControl(const QString & id) const
{
  const QLCCamera * cam = selectedCamera();
  return cam ? cam->getControl(id) : nullptr;
}

int QLCSCTPCamera::cameraDeviceBuffers() const
{
  return _cameraDeviceBuffers;
}

void QLCSCTPCamera::setCameraDeviceBuffers(int v)
{
  _cameraDeviceBuffers = v;
}


/////////////////////////////////////////////////////////////////////////////////////////////

bool QLCSCTPCamera::sctp_connect(const QString & url)
{
  struct sctp_event_subscribe events = { 0 };
  struct sctp_initmsg initmsg = { 0 };
  bool fOk = false;
  int so = -1;

  sctp_disconnect();

  if( !cf_get_iface_address(url.toUtf8().constData(), &saddrs) ) {
    CF_ERROR("cf_get_iface_address('%s') fails: %s", url.toUtf8().constData(), strerror(errno));
    goto __end;
  }

  if( (so = socket(AF_INET, SOCK_STREAM, IPPROTO_SCTP)) < 0 ) {
    CF_ERROR("socket(AF_INET, SOCK_STREAM, IPPROTO_SCTP) fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  if( true ) {
    int buffsize = 8 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if( setsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    buffsize = 0;
    buffsizelen = sizeof(buffsize);
    if( getsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, &buffsizelen) < 0 ) {
      CF_ERROR("getsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_RCVBUF: %d", buffsize);
  }

  if( true ) {
    int buffsize = 8 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if( setsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    buffsize = 0;
    buffsizelen = sizeof(buffsize);
    if( getsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, &buffsizelen) < 0 ) {
      CF_ERROR("getsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_SNDBUF: %d", buffsize);
  }

  if( true ) {
    // Set to disable (freq=1, delay=0)
    struct sctp_sack_info sack_info = { 0 };
    socklen_t len = sizeof(sack_info);
    sack_info.sack_delay = 20;
    sack_info.sack_freq = 2; // Sets frequency to 1 for immediate ACKs
    sack_info.sack_assoc_id = SCTP_ALL_ASSOC; // Or specific assoc ID
    if( setsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, len) < 0 ) {
      CF_DEBUG("setsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno));
    }
  }
  if( true ) {
    struct sctp_sack_info sack_info = { 0 };
    socklen_t len = sizeof(sack_info);
    if( getsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, &len) < 0 ) {
      CF_DEBUG("getsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("2: SCTP_DELAYED_SACK: Delay: %u, Freq: %u", sack_info.sack_delay, sack_info.sack_freq);
  }

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
  if( setsockopt(so, IPPROTO_SCTP, SCTP_EVENTS, &events, sizeof(events)) < 0 ) {
    CF_ERROR("setsockopt(SCTP_EVENTS) fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  if( ::connect(so, (struct sockaddr*) &saddrs, sizeof(saddrs)) < 0 ) {
    CF_ERROR("connect() fails.errno=%d (%s)", errno, strerror(errno));
    goto __end;
  }

  so_set_recv_timeout(_so, 1);

  fOk = true;
__end:

  if( !fOk && so != -1 ) {
    close(so), so = -1;
  }

  return (_so = so);
}

void QLCSCTPCamera::sctp_disconnect()
{
  CF_DEBUG("_so=%d", _so);
  if ( _so != -1 ) {
    so_close(_so, true);
    _so = -1;
  }
}

bool QLCSCTPCamera::sctp_sendmsg(const std::string & msg)
{
  if( ::sctp_sendmsg(_so, msg.data(), msg.size() + 1, nullptr, 0, 0, 0, 0, 0, 0) < 0 ) {
    CF_ERROR("sctp_sendmsg() fails. errno=%d %s", errno, strerror(errno));
    return false;
  }
  return true;
}

void QLCSCTPCamera::sctp_threadproc()
{
  static const auto stop_requested =
      [](int camera_state) -> bool {
        return camera_state == State_disconnect || camera_state == State_disconnected;
      };


  CF_DEBUG("enter");

  constexpr uint32_t PPID_TEXT = 0;
  constexpr uint32_t PPID_IMGHDR = 1;
  constexpr uint32_t PPID_IMGCHUNK = 2;
  constexpr uint32_t TMPBUFFSIZE = 256 * 1024;

  unique_lock lock(_mtx);

  if( _buf.size() < TMPBUFFSIZE ) {
    _buf.resize(TMPBUFFSIZE);
  }

  while (!stop_requested(_current_state)) {

    if( true ) {
      temporary_unlock unlok(lock);

      struct sctp_sndrcvinfo sinfo = { 0 };
      int flags = 0;
      errno = 0;

      const int cb = sctp_recvmsg(_so, _buf.data(), _buf.size(), nullptr, nullptr, &sinfo, &flags);
      if( cb <= 0 ) {
        CF_ERROR("sctp_recvmsg() fails: cb=%d stream=%u flags=0x%0X errno=%d (%s)",
            cb, sinfo.sinfo_stream, flags, errno, strerror(errno));
        if( errno == EAGAIN || errno == EWOULDBLOCK ) {
          continue;
        }
        break;
      }

      if( flags & MSG_NOTIFICATION ) {
        CF_ERROR("MSG_NOTIFICATION: cb=%d stream=%u ppid=%u flags=0x%0X errno=%d (%s)",
            cb, sinfo.sinfo_stream, ntohl(sinfo.sinfo_ppid), flags, errno, strerror(errno));
        continue;
      }

      const uint32_t ppid = ntohl(sinfo.sinfo_ppid);

      if( sinfo.sinfo_stream == 0 ) {
        _smsg.append(_buf.begin(), _buf.begin() + cb);
        if( flags & MSG_EOR ) {
          if ( _smsg.size() < 1024 ) {
            CF_DEBUG("MSG_EOR: cb=%d stream=%u flags=0x%0X msg=\n%s\n", cb, sinfo.sinfo_stream, flags, _smsg.c_str());
          }
          else {
            fprintf(stderr, "MSG_EOR: cb=%d stream=%u flags=0x%0X msg=\n'%s'\n", cb, sinfo.sinfo_stream, flags, _smsg.c_str());
          }
          if ( _current_state == State_connecting ) {
            _smsgs.emplace_back(_smsg);
            _condvar.notify_all();
          }
          _smsg.clear();
        }
        continue;
      }

      if( sinfo.sinfo_stream == 1 ) {

        if( ppid == PPID_IMGHDR ) {
          const size_t old_size = _imghdr.size();
          _imghdr.resize(old_size + cb);
          memcpy(_imghdr.data() + old_size, _buf.data(), cb);

          _imgdata.clear();
          if( (flags & MSG_EOR) && (_imghdr.size() != sizeof(ImageHeader)) ) {
            CF_DEBUG("Bad Image header size: _imghdr.size()=%zu sizeof(ImageHeader)=%zu", _imghdr.size(), sizeof(ImageHeader));
            _imghdr.clear();
            _imghdr.clear();
          }
//          else {
//            const ImageHeader & hdr = reinterpret_cast<const ImageHeader&>(*_imghdr.data());
//            CF_DEBUG("HOT HDR datasize=%u", hdr.datasize);
//          }

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

            const ImageHeader & hdr = reinterpret_cast<const ImageHeader&>(*_imghdr.data());
            //CF_DEBUG("image chunk: hdr.datasize=%u _imgdata.size()=%zu", hdr.datasize, _imgdata.size());

            if( _imgdata.size() > hdr.datasize ) {
              CF_DEBUG("bad image size: hdr.datasize=%u _imgdata.size()=%zu", hdr.datasize, _imgdata.size());
              _imgdata.clear();
              _imghdr.clear();
            }

            if( _imgdata.size() == hdr.datasize ) {

              cv::Mat image;
              COLORID colorid = COLORID_UNKNOWN;
              int bpp = 0;

              bool fOk =
                  unpack_libcamera_image(_imgdata,
                      hdr.width, hdr.height, hdr.stride,
                      hdr.fourcc, hdr.fourcc_modifier,
                      image,
                      &colorid,
                      &bpp);

              if( !fOk ) {
                CF_ERROR("unpack_libcamera_image() fails");
              }
              else {
                //CF_DEBUG("unpack ok");
                _frms.emplace_back(QCameraFrame::create(image, colorid, hdr.bpp));
                _condvar.notify_all();
              }
              _imgdata.clear();
              _imghdr.clear();
            }
          }
        }

        continue;
      }

      CF_DEBUG("Bad stream index=%u. ignored", sinfo.sinfo_stream);
    }
  }

  CF_DEBUG("sctp_disconnect(). _current_state=%d (%s)", _current_state, toCString(_current_state));
  sctp_disconnect();
  CF_DEBUG("leave");
}


/////////////////////////////////////////////////////////////////////////////////////////////


bool QLCSCTPCamera::device_is_connected() const
{
  return _so >= 0;
}

bool QLCSCTPCamera::device_connect()
{
  unique_lock lock(_mtx);

  if ( device_is_connected() ) {
    CF_DEBUG("already connected");
    return true;
  }

  if ( true ) {
    temporary_unlock unlock(lock);
    CF_DEBUG("Connecting to '%s'", _url.toUtf8().constData());
    if ( !sctp_connect(_url) ) {
      CF_DEBUG("sctp_connect('%s') fails", _url.toUtf8().constData());
      return false;
    }
    CF_DEBUG("Connected to '%s'", _url.toUtf8().constData());
  }
  if ( _current_state != State_connecting ) {
    CF_DEBUG("Disconnect requested");
    sctp_disconnect();
    return false;
  }

  _sctp_thread.reset(new std::thread(&ThisClass::sctp_threadproc, this));

  if ( true ) {
    temporary_unlock unlock(lock);
    if ( !sctp_sendmsg("command=\"get-cameras\";") ) {
      CF_ERROR("sctp_sendmsg('%s', get_cameras) fails: %s", _url.toUtf8().constData(), strerror(errno));
      sctp_disconnect();
      return false;
    }
  }

  while (_current_state == State_connecting) {

    _condvar.wait(lock, [this]() {
      return (_current_state != State_connecting || !_smsgs.empty() );
    });

    if( _current_state != State_connecting ) {
      break;
    }

    if( !_smsgs.empty() ) {

      const std::string smsg = _smsgs.front();
      _smsgs.pop_front();

      //CF_DEBUG("server message:\n%s\n", smsg.c_str());

      c_config cfg;
      if( !cfg.read_string(smsg.c_str()) ) {
        CF_ERROR("cfg.read_string() fails. smsg='\n%s\n'", smsg.c_str());
        sctp_disconnect();
        return false;
      }

      c_config_setting root = cfg.root();
      c_config_setting cameras_item = root.get_list("cameras");

      QString id, model, role, pixfmt, size, type, value, minval, maxval, defval;
      std::vector<std::string> values;

      _cameras.clear();

      for( int icam = 0, ncams = cameras_item.length(); icam < ncams; ++icam ) {

        const c_config_setting camera_item = cameras_item[icam];
        load_settings(camera_item, "id", &id);
        load_settings(camera_item, "model", &model);
        if( id.isEmpty() ) {
          continue;
        }

        _cameras.append(QLCCamera(id, model));
        QLCCamera & cam = _cameras.back();

        const c_config_setting controls_item = camera_item["controls"];
        //CF_DEBUG("controls_item.length()=%d", controls_item.length());
        for( int ictrl = 0, nctrls = controls_item.length(); ictrl < nctrls; ++ictrl ) {
          const c_config_setting control_item = controls_item[ictrl];
          if( load_settings(control_item, "id", &id) ) {
            values.clear();
            load_settings(control_item, "type", &type);
            load_settings(control_item, "min", &minval);
            load_settings(control_item, "max", &maxval);
            load_settings(control_item, "def", &defval);
            load_settings(control_item, "val", &value);
            load_settings(control_item, "values", &values);
            cam.contols.append(QLCCameraControl(id, type, value, minval, maxval, defval, values));
          }
        }

        const c_config_setting roles_item = camera_item["roles"];

        for( int irole = 0, nroles = roles_item.length(); irole < nroles; ++irole ) {
          const c_config_setting role_item = roles_item[irole];

          if( !load_settings(role_item, "role", &role) ) {
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
              if( load_settings(sizes_item[isz], &size) ) {
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

      CF_DEBUG("break");
      break;
    }
  }

  if( _current_state != State_connecting ) {
    CF_DEBUG("sctp_disconnect()");
    sctp_disconnect();
    CF_DEBUG("return false");
    return false;
  }

//  CF_DEBUG("setState(State_connected)");
//  setState(State_connected);

  CF_DEBUG("return true");
  return true;
}

void QLCSCTPCamera::device_disconnect()
{
  CF_DEBUG("sctp_disconnect()");
  sctp_disconnect();

  CF_DEBUG("unique_lock lock(_mtx)");
  unique_lock lock(_mtx);
  CF_DEBUG("_sctp_thread=%p", _sctp_thread.get());
  if ( _sctp_thread ) {
    if ( _current_state != State_disconnect ) {
      CF_DEBUG("setState(State_disconnect)");
      setState(State_disconnect, "QLCSCTPCamera::device_disconnect");
      _condvar.notify_all();
    }
    CF_DEBUG("_sctp_thread->join()");
    temporary_unlock unlock(lock);
    _sctp_thread->join();
    CF_DEBUG("_sctp_thread->join() OK");
  }

  CF_DEBUG("leave");
}

bool QLCSCTPCamera::device_start()
{
  unique_lock lock(_mtx);

  if ( !device_is_connected() ) {
    CF_ERROR("Device is not connected.");
    errno = ENOTCONN;
    return false;
  }

  const auto * cam = selectedCamera();
  const auto * strm = selectedStream();
  const auto * fmt = selectedFormat();
  const auto * size = selectedSize();
  const int nbuffers = std::max(1, cameraDeviceBuffers());

  std::string command =
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

  if( cam ) {

    std::string controls = "";

    for( const auto & ctl : cam->contols ) {
      if( !ctl.value.isEmpty() ) {
        controls +=
            ssprintf("%s{id=\"%s\"; value=\"%s\"}",
                controls.empty() ? "" : ",\n",
                ctl.id.toUtf8().constData(),
                ctl.value.toUtf8().constData());
      }
    }

    if ( !controls.empty() ) {
      command += ssprintf("controls=(\n%s\n);",controls.c_str());
    }
  }


  if ( true ) {
    temporary_unlock unlock(lock);
    CF_DEBUG("send:\n%s\n",command.c_str());
    if( !sctp_sendmsg(command) ) {
      CF_ERROR("send(start-camera) fails. errno=%d (%s)", errno, strerror(errno));
      return false;
    }
  }

  // Fixme: receive server response here

  return true;
}

void QLCSCTPCamera::device_stop()
{
  CF_DEBUG("sctp_sendmsg(stop-camera)");
  if( !sctp_sendmsg("command=\"stop-camera\";") ) {
    CF_ERROR("sctp_sendmsg(stop-camera) fails. errno=%d (%s)", errno, strerror(errno));
  }
}

int QLCSCTPCamera::device_max_qsize()
{
  return 2;//_p.size() / 2;
}

void QLCSCTPCamera::device_release_frame(const QCameraFrame::sptr & frame)
{
  // qpool(frame);
  // frame.reset();
}

bool QLCSCTPCamera::device_recv_frame(QCameraFrame::sptr & ofrm)
{
  unique_lock lock(_mtx);

  while (_current_state == State_started) {
    if( _frms.empty() ) {
      _condvar.wait(lock, [this]() {
        return _current_state != State_started || !_frms.empty();
      });
    }
    if( _current_state != State_started ) {
      break;
    }
    if( !_frms.empty() ) {
      ofrm = _frms.front();
      _frms.pop_front();
      return true;
    }
  }
  return false;
}

void QLCSCTPCamera::applyDeviceControl(const QLCCameraControl & ctl)
{
  if ( _so >= -1 )  {
    const std::string command =
        ssprintf("command=\"set-control\";\n"
            "id=\"%s\";\n"
            "value=\"%s\";\n",
            ctl.id.toUtf8().constData(),
            ctl.value.toUtf8().constData());

    CF_DEBUG("sctp_sendmsg(set-control)");

    if( !sctp_sendmsg(command) ) {
      CF_ERROR("sctp_sendmsg(set-control) fails. errno=%d (%s)", errno, strerror(errno));
    }
  }
}

} // namespace serimager
