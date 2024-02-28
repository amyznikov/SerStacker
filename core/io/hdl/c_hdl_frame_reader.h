/*
 * c_hdl_frame_reader.h
 *
 *  Created on: Feb 27, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_frame_reader_h__
#define __c_hdl_frame_reader_h__

// Must be set from CMakeList.txt
// #define HAVE_PCAP 1
#if HAVE_PCAP

#include <core/io/c_pcap_file.h>
#include "c_hdl_packet_parser.h"
#include <memory>
#include <vector>
#include <functional>



///////////////////////////////////////////////////////////////

/**
 * Offline pcap file reader and HDL parser
 * */
class c_hdl_offline_pcap_reader
{
public:
  typedef c_hdl_offline_pcap_reader this_class;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  struct parsed_frame
  {
    c_hdl_packet_parser::State parser_state;
    ssize_t filepos;
  };

  struct parsed_stream
  {
    std::vector<parsed_frame> frames;
    uint32_t addrs;
  };

  ~c_hdl_offline_pcap_reader();

  const c_hdl_packet_parser & hdl_parser() const;

  bool open(const std::string & filename, const std::string & options = "");
  void close();
  bool is_open() const;
  c_hdl_frame::sptr read();

  const std::vector<parsed_stream> & streams() const;

  bool select_stream(int index);
  int current_stream() const;

  ssize_t num_frames() const; // number of frames in current stream

  bool seek(int32_t frame_index_in_current_stream);
  int curpos() const;


protected:
  c_pcap_reader pcap_;
  c_hdl_packet_parser hdl_parser_;
  std::vector<parsed_stream> parsed_streams_;
  int32_t current_stream_index_ = -1;
  int32_t current_pos_ = -1;
};



#if 0

// Old code kept here for some time

///////////////////////////////////////////////////////////////
// New (multi-lidar) API


class c_hdl_source;
class c_hdl_connection;
class c_hdl_frame_processor;

class c_hdl_connection
{
public:
  typedef c_hdl_connection this_class;
  typedef std::shared_ptr<this_class> sptr;

  c_hdl_connection(c_hdl_source * hdl_source,
      uint32_t src_address,
      uint16_t src_port,
      HDLSensorType sensor_type,
      HDLReturnMode return_mode);

  virtual ~c_hdl_connection() = default;

  c_hdl_source * hdl_source() const;
  uint32_t address() const;
  uint16_t port() const;

  void set_enabled(bool v);
  bool enabled() const;

  void set_hdl_framing_mode(enum HDLFramingMode v);
  enum HDLFramingMode hdl_framing_mode() const;

  void set_hdl_frame_seam_azimuth(double azimuth_in_degrees);
  double hdl_frame_seam_azimuth() const;

  void set_lidar_config_xml(const std::string & v);
  const std::string & lidar_config_xml() const;

  HDLSensorType sensor_type() const;
  HDLReturnMode return_mode() const;
  const c_hdl_specification * lidar_specification() const;

//  c_notification<void()> state_chagned;
//  c_notification<void(const c_lidar_frame::sptr & frame)> frame_received;
//  c_notification<void()> sensor_changed;

protected:
  friend class c_hdl_source;
  void reset();
  void set_hdl_source(c_hdl_source * );
  bool parse_packet(const uint8_t data[], ssize_t size);
  bool lidar_sensor_changed() const;
  size_t queued_frames() const;
  c_hdl_frame::sptr pop_queued_frame();
  void clear_queued_frames();

protected:
  virtual void state_chagned()
  {
  }

  virtual void frame_received(const c_hdl_frame::sptr & frame)
  {
  }

  virtual void sensor_changed()
  {
  }

protected:
  c_hdl_source * hdl_source_ = nullptr;
  uint32_t address_ = 0;
  uint16_t port_  = 0;
  bool enabled_ = false;

  c_hdl_packet_parser hdl_parser_;
  HDLSensorType sensor_type_ = HDLSensor_unknown;
  HDLReturnMode return_mode_ = HDLReturnMode_unknown;
  bool sensor_changed_ = false;
};

class c_hdl_source
{
public:
  typedef c_hdl_source this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::function<void(const c_hdl_source::sptr &)> slot;
  typedef std::shared_ptr<slot> slotptr;

  enum State {
    State_disconnected,
    State_connecting,
    State_connected,
    State_starting,
    State_started,
    State_stopping,
    State_disconnecting,
  };


  c_hdl_source(const std::string & address);
  virtual ~c_hdl_source();

  const std::string & address() const;
  const std::string & display_name() const;
  State state() const;
  const std::string & reason() const;
  std::mutex & mutex();

  static void set_nonlive_stream_delay_us(int v);
  static int nonlive_stream_delay_us();

  const std::vector<c_hdl_connection::sptr> & hdl_connections() const;

  c_hdl_connection::sptr hdl_connection(uint32_t address, uint16_t port) const;

  c_hdl_connection::sptr add_hdl_connection(uint32_t address, uint16_t port,
      HDLSensorType sensor_type = HDLSensor_unknown,
      HDLReturnMode return_mode = HDLReturnMode_unknown);

  bool open(bool start_after_connect = false);
  void start(bool lock_mutex = true);
  void stop();
  void step();
  void close(bool sync = false);


public: // notifications
//  c_notification<void(this_class *, State)> state_chagned;
//  c_notification<void(const c_hdl_connection::sptr & connection)> connection_created;
//  c_notification<void(const c_hdl_connection::sptr & connection)> connection_destroyed;

protected:

  virtual c_hdl_connection::sptr create_hdl_connection(uint32_t src_address, uint16_t src_port,
      HDLSensorType sensor_type = HDLSensor_unknown, HDLReturnMode return_mode = HDLReturnMode_unknown);

  virtual bool open_file() = 0;

  virtual void close_file() = 0;

  virtual ssize_t read_packet(const uint8_t ** payload,
      uint32_t * src_address,
      uint16_t * src_port) = 0;

  void set_state(State state,const char * reasonmsg = nullptr, ...);

protected:

  virtual void state_chagned(this_class *, State)
  {
  }

  virtual void connection_created(const c_hdl_connection::sptr & connection)
  {
  }

  virtual void connection_destroyed(const c_hdl_connection::sptr & connection)
  {
  }

protected:
  std::string address_;
  std::string display_name_;
  std::vector<c_hdl_connection::sptr> hdl_connections_;
  static int nonlive_stream_delay_us_; // [usec]

  bool auto_start_after_connect_ = false;
  State current_state_ = State_disconnected;
  std::string reason_;
  std::mutex mtx_;

};

class c_hdl_pcap_file_reader :
    public c_hdl_source
{
public:
  typedef c_hdl_pcap_file_reader this_class;
  typedef c_hdl_source base;

  c_hdl_pcap_file_reader(const std::string & address,
      const std::string & filename,
      const std::string & options);

  ~c_hdl_pcap_file_reader();

  const std::string & filename() const;

protected:
  bool open_file() override;
  void close_file() override;
  ssize_t read_packet(const uint8_t ** payload, uint32_t * src_address, uint16_t * src_port) override;

protected:
  std::string filename_;
  std::string options_;
  c_pcap_reader pcap_;
};


class c_hdl_udp_port_reader :
    public c_hdl_source
{
public:
  typedef c_hdl_udp_port_reader this_class;
  typedef c_hdl_source base;

  c_hdl_udp_port_reader(const std::string & address,
      uint16_t port,
      const std::string & options);

  ~c_hdl_udp_port_reader();

  uint16_t port() const;

protected:
  bool open_file() override;
  void close_file() override;
  ssize_t read_packet(const uint8_t ** payload, uint32_t * src_address, uint16_t * src_port) override;

protected:
  uint16_t port_ = 2368; // host byte order
  std::string options_;
  int so_ = -1;

  static constexpr int max_packet_size = 2048;
  uint8_t payload[max_packet_size];
};


///////////////////////////////////////////////////////////////
// Old (single lidar) API



class c_hdl_frame_reader
{
public:
  typedef c_hdl_frame_reader this_class;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  c_hdl_frame_reader();
  virtual ~c_hdl_frame_reader() = default;

  void set_hdl_framing_mode(enum HDLFramingMode v);
  enum HDLFramingMode hdl_framing_mode() const;

  void set_hdl_frame_seam_azimuth(double azimuth_in_degrees);
  double hdl_frame_seam_azimuth() const;

  void set_lidar_config_xml(const std::string & v);
  const std::string & lidar_config_xml() const;

  HDLSensorType sensor_type() const;
  HDLReturnMode return_mode() const;
  const c_hdl_specification * lidar_specification() const;

  void set_current_lidar(in_addr_t v);
  in_addr_t current_lidar() const;

  const std::map<in_addr_t, double> & available_lidars() const;

  bool available_lidars_changed() const;

  virtual bool open(const std::string & device, const std::string & options = "") = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual c_hdl_frame::sptr recv() = 0;

protected:
  c_hdl_packet_parser hdl_parser_;
  std::map<in_addr_t, double> available_lidars_;
  mutable bool available_lidars_changed_ = false;
  in_addr_t current_lidar_ = INADDR_ANY; // network byte order

};


class c_hdl_pcap_reader :
    public c_hdl_frame_reader
{
public:
  typedef c_hdl_pcap_reader this_class;
  typedef c_hdl_frame_reader base;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  ~c_hdl_pcap_reader();

  bool open(const std::string & pcapfile, const std::string & options = "") override;
  void close() override;
  bool is_open() const override;
  c_hdl_frame::sptr recv() override;

protected:
  c_pcap_reader pcap_;
};

class c_hdl_udp_reader :
    public c_hdl_frame_reader
{
public:
  typedef c_hdl_udp_reader this_class;
  typedef c_hdl_frame_reader base;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  ~c_hdl_udp_reader();

  bool open(const std::string & address, const std::string & options = "") override;
  void close() override;
  bool is_open() const override;
  c_hdl_frame::sptr recv() override;
  void stop();

protected:
  int so_ = -1;
  uint32_t bind_address_ = INADDR_ANY; // host byte order
  uint16_t bind_port_ = 2368; // host byte order
  volatile bool stop_ = false;
};

#endif // 0

#endif // HAVE_PCAP


#endif /* __c_hdl_frame_reader_h__ */
