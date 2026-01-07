/*
 * QLCSCTPCamera.h
 *
 *  Created on: Jan 1, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLCSCTPCamera_h__
#define __QLCSCTPCamera_h__

#include "QImagingCamera.h"
#include <core/io/sockopt.h>

namespace serimager {

class QLCSCTPCamera :
    public QImagingCamera
{
  Q_OBJECT;
public:
  typedef QLCSCTPCamera ThisClass;
  typedef QImagingCamera Base;
  typedef std::shared_ptr<ThisClass> sptr;

#pragma pack(push, 1)
  struct ImageHeader
  {
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t datasize;
    uint32_t bpp;
    uint32_t fourcc;
    uint64_t fourcc_modifier;
    uint64_t timestamp;
  };
#pragma pack(pop)

  struct QLCCameraPixFormats
  {
    QString format;
    QList<QString> sizes;
    int selectedSizeIndex = -1;
    QLCCameraPixFormats() = default;
    QLCCameraPixFormats(const QString & name ) : format(name) {}
  };

  struct QLCCameraStream
  {
    QString role;
    QList<QLCCameraPixFormats> formats;
    int selectedFormatIndex = -1;
    QLCCameraStream() = default;
    QLCCameraStream(const QString & name) : role(name) {}
  };

  struct QLCCameraControl
  {
    QString id;
    QString type;
    QString value;
    QString minval;
    QString maxval;
    QString defval;
    std::vector<std::string> values;
    QLCCameraControl() = default;
    QLCCameraControl(const QString & _id, const QString & _type, const QString & _value, const QString & _minval,
        const QString & _maxval, const QString & _defval, const std::vector<std::string> &_values ) :
        id(_id), type(_type), value(_value), minval(_minval), maxval(_maxval), defval(_defval), values(_values)
    {}
  };

  struct QLCCamera
  {
    QString id;
    QString model;
    QList<QLCCameraControl> contols;
    QList<QLCCameraStream> streams;
    int selectedStreamIndex = -1;
    QLCCamera() = default;
    QLCCamera(const QString & name, const QString & _model) : id(name), model(_model) {}
    QLCCameraControl * getControl(const QString & id);
    const QLCCameraControl * getControl(const QString & id) const;
  };

//  class c_sctp_connection
//  {
//  public:
//    typedef c_sctp_connection this_class;
//
//    int connect(const QString & url);
//    void disconnect();
//    bool connected() const;
//    bool sendmsg(const std::string & msg);
//    int recvmsg();
//    const std::string & smsg() const;
//    const ImageHeader & imghdr() const;
//    const std::vector<uint8_t> & imgdata() const;
//    void clear_smsg();
//    void clear_image();
//
//  protected:
//    int _so = -1;
//    struct sockaddr_in saddrs = {0};
//    std::string _smsg;
//    std::vector<uint8_t> _imghdr;
//    std::vector<uint8_t> _imgdata;
//    std::vector<uint8_t> _buf;
//    std::deque<std::string> _smsgq;
//    std::deque<QCameraFrame::sptr> _frmq;
//  };

  ~QLCSCTPCamera();

  static sptr create(const QString & cameraName, const QString & url,
      QObject * parent = nullptr);

  void setName(const QString & name);
  const QString & name() const;

  void setUrl(const QString & v);
  const QString & url() const;

  QString display_name() const override;
  QString parameters() const override;

  bool is_same_camera(const QImagingCamera::sptr & rhs) const override;
  int drops() const override;

  const QList<QLCCamera> & cameras() const;

  void setSelectedCameraIndex(int v);
  int selectedCameraIndex() const;
  QLCCamera * selectedCamera();
  const QLCCamera * selectedCamera() const;

  void setSelectedStreamIndex(int index);
  int selectedStreamIndex() const;
  QLCCameraStream * selectedStream();
  const QLCCameraStream * selectedStream() const;

  void setSelectedFormatIndex(int index);
  int selectedFormatIndex() const;
  QLCCameraPixFormats * selectedFormat();
  const QLCCameraPixFormats * selectedFormat() const;

  void setSelectedSizeIndex(int index);
  int selectedSizeIndex() const;
  QString * selectedSize();
  const QString * selectedSize() const;

  QLCCameraControl * getControl(const QString & id);
  const QLCCameraControl * getControl(const QString & id) const;

  int cameraDeviceBuffers() const;
  void setCameraDeviceBuffers(int v);

  void applyDeviceControl(const QLCCameraControl & ctl);

Q_SIGNALS:
  void parametersChanged();

protected:
  QLCSCTPCamera(const QString & name, const QString & url, QObject * parent = nullptr);
  QLCSCTPCamera(const QString & name, QObject * parent = nullptr);

  bool device_is_connected() const override;
  bool device_connect() override;
  void device_disconnect() override;
  bool device_start() override;
  void device_stop() override;
  int device_max_qsize() override;
  void device_release_frame(const QCameraFrame::sptr & frame) override;
  bool device_recv_frame(QCameraFrame::sptr & frm) override;

protected:
  QString _name;
  QString _url;
  QList<QLCCamera> _cameras;
  int _selectedCameraIndex = -1;
  int _cameraDeviceBuffers = 2;

protected:
  std::unique_ptr<std::thread> _sctp_thread;
  struct sockaddr_in saddrs = {0};
  int _so = -1;
  std::string _smsg;
  std::vector<uint8_t> _imghdr;
  std::vector<uint8_t> _imgdata;
  std::vector<uint8_t> _buf;
  std::deque<std::string> _smsgs;
  std::deque<QCameraFrame::sptr> _frms;
  void sctp_threadproc();
  bool sctp_connect(const QString & url);
  void sctp_disconnect();
  bool sctp_sendmsg(const std::string & msg);
};


/**
 * This struct is just used to serialize QLCSCTPCamera parameters with QSettings data stream
 * */
struct QLCSCTPCameraParameters {
  QString name;
  QString url;
};

} /* namespace serimager */

Q_DECLARE_METATYPE(serimager::QLCSCTPCamera::sptr);
Q_DECLARE_METATYPE(serimager::QLCSCTPCameraParameters);
Q_DECLARE_METATYPE(QList<serimager::QLCSCTPCameraParameters>);

inline QDataStream& operator << (QDataStream & out, const serimager::QLCSCTPCameraParameters & v)
{
  return (out << v.name << v.url);
}

inline QDataStream& operator >> (QDataStream & in, serimager::QLCSCTPCameraParameters & v)
{
  return (in >> v.name >> v.url);
}

inline QDataStream& operator << (QDataStream & out, const QList<serimager::QLCSCTPCameraParameters> & list)
{
  out << list.size();
  for( int i = 0, n = list.size(); i < n; ++i ) {
    out << list[i];
  }
  return out;
}

inline QDataStream& operator >> (QDataStream & in, QList<serimager::QLCSCTPCameraParameters> & list)
{
  list.clear();

  int n = 0;
  if( (in >> n).status() == QDataStream::Ok ) {
    for( int i = 0; i < n; ++i ) {
      serimager::QLCSCTPCameraParameters p;
      if( (in >> p).status() != QDataStream::Ok ) {
        break;
      }
      list.append(p);
    }
  }
  return in;
}

#endif /* __QLCSCTPCamera_h__ */
