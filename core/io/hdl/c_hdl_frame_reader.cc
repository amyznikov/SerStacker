/*
 * c_hdl_frame_reader.cc
 *
 *  Created on: Feb 27, 2024
 *      Author: amyznikov
 */

#include "c_hdl_frame_reader.h"

#if HAVE_PCAP

#include <core/debug.h>

///////////////////////////////////////////////////////////////
// Static pcap file reader

static int read_next_hdl_frame(c_pcap_reader & pcap, uint32_t * src_addrs, const uint8_t ** payload)
{
  int status;

  const pcap_pkthdr * pkt_header = nullptr;
  const c_pcap_data_header * data_header = nullptr;

  while ((status = pcap.read(&pkt_header, &data_header, payload)) >= 0) {

    if( !payload ) {
      continue;
    }

    const uint payload_size =
        pkt_header->len - pcap.data_header_size();

    if( payload_size != hdl_lidar_packet_size() ) {
      continue;
    }

    *src_addrs = 0;

    switch (pcap.datalinktype()) {
      case DLT_NULL:
        *src_addrs = data_header->loopbak.ip.ip_src.s_addr;
        break;
      case DLT_EN10MB:
        *src_addrs = data_header->en10mb.ip.ip_src.s_addr;
        //port = data_header->en10mb.udp.dest;
        break;
      case DLT_LINUX_SLL:
        *src_addrs = data_header->sll.ip.ip_src.s_addr;
        break;
      case DLT_LINUX_SLL2:
        *src_addrs = data_header->sll2.ip.ip_src.s_addr;
        break;
      case DLT_USER1:
        *src_addrs = data_header->user1.ip.ip_src.s_addr;
        break;
      default:
        break;
    }

    if( *src_addrs ) {
      break;
    }
  }

  return status;
}

c_hdl_offline_pcap_reader::~c_hdl_offline_pcap_reader()
{
  close();
}

void c_hdl_offline_pcap_reader::close()
{
  pcap_.close();
  hdl_parser_.reset();
}

bool c_hdl_offline_pcap_reader::is_open() const
{
  return pcap_.is_open();
}

bool c_hdl_offline_pcap_reader::open(const std::string & filename, const std::string & options )
{
  close();

  if( !pcap_.open(filename, options) ) {
    CF_ERROR("pcap_.open('%s') fals", filename.c_str());
    return false;
  }

  struct c_hdl_stream
  {
    c_hdl_packet_parser parser;
    std::vector<parsed_frame> frames;
    uint32_t addrs;
  };

  std::vector<c_hdl_stream> hdl_streams;

  const uint8_t * payload = nullptr;
  uint32_t addrs;

  int status;

  for( long filepos = pcap_.tell(); (status = read_next_hdl_frame(pcap_, &addrs, &payload)) >= 0;
      filepos = pcap_.tell() ) {

    auto current_hdl_stream =
        std::find_if(hdl_streams.begin(), hdl_streams.end(),
            [addrs](const auto & s) {
              return s.addrs == addrs;
            });

    if( current_hdl_stream == hdl_streams.end() ) {

      hdl_streams.emplace_back();
      current_hdl_stream = hdl_streams.end() - 1;

      current_hdl_stream->addrs = addrs;
      current_hdl_stream->parser.set_hdl_framing_mode(HDLFraming_Rotation);
      current_hdl_stream->parser.set_only_extract_frame_seams(true);

      current_hdl_stream->parser.set_frame_created_callback(
          [&](const c_hdl_packet_parser & p, const c_hdl_frame::sptr & f) {

            const c_hdl_packet_parser::State & state = p.state();

            current_hdl_stream->frames.emplace_back();

            auto & frame =
                current_hdl_stream->frames.back();

            frame.filepos = filepos;
            frame.parser_state = state;
          });
    }

    if( !current_hdl_stream->parser.parse(payload, hdl_lidar_packet_size()) ) {
      CF_ERROR("packet_parser_.parse() fails");
      continue;
    }
  }

  CF_DEBUG("streams.size=%d", hdl_streams.size());

  parsed_streams_.clear();
  parsed_streams_.resize(hdl_streams.size());

  for( int i = 0, n = hdl_streams.size(); i < n; ++i ) {
    parsed_streams_[i].addrs = hdl_streams[i].addrs;
    parsed_streams_[i].frames = std::move(hdl_streams[i].frames);
  }

  return true;
}

c_hdl_frame::sptr c_hdl_offline_pcap_reader::read()
{
  const pcap_pkthdr * pkt_header = nullptr;
  const c_pcap_data_header * data_header =  nullptr;
  const uint8_t * payload = nullptr;


  if ( !pcap_.is_open() ) {
    CF_ERROR("pcap file is not open");
    return nullptr;
  }

  if ( current_stream_index_ < 0 || current_stream_index_ >= (int)parsed_streams_.size() ) {
    CF_ERROR("current_stream_=%d is invalid", current_stream_index_);
    return nullptr;
  }

  const parsed_stream & stream =
      parsed_streams_[current_stream_index_];

  if ( current_pos_ < 0 || current_pos_ >= (int)stream.frames.size() ) {
    CF_ERROR("current_pos_=%d is invalid", current_pos_);
    return nullptr;
  }

  const parsed_frame & f =
      stream.frames[current_pos_];

  hdl_parser_.clear(&f.parser_state);

  c_hdl_frame::sptr frame;

  hdl_parser_.set_frame_populated_callback(
      [&](const c_hdl_packet_parser&, const c_hdl_frame::sptr & populated_frame) {
        frame = populated_frame;
      });

  pcap_.seek(f.filepos);

  int start_block =
      f.parser_state.start_block;

  uint32_t addrs = 0;

  int status;

  while ((status = read_next_hdl_frame(pcap_, &addrs, &payload)) >= 0) {

    if( addrs != stream.addrs ) {
      continue;
    }

    if( !hdl_parser_.parse(payload, hdl_lidar_packet_size(), start_block) ) {
      CF_ERROR("hdl_parser_.parse() fails");
      break;
    }

    if( frame ) {
      break;
    }

    start_block = 0;
  }

  if( !frame ) {
    frame = hdl_parser_.current_frame();
  }

  ++current_pos_;

  return frame;
}

const std::vector<c_hdl_offline_pcap_reader::parsed_stream> & c_hdl_offline_pcap_reader::streams() const
{
  return parsed_streams_;
}

ssize_t c_hdl_offline_pcap_reader::num_frames() const
{
  return ((current_stream_index_ >= 0) && (current_stream_index_ < (int) parsed_streams_.size())) ?
      parsed_streams_[current_stream_index_].frames.size() : -1;
}

int c_hdl_offline_pcap_reader::current_stream() const
{
  return current_stream_index_;
}

bool c_hdl_offline_pcap_reader::select_stream(int index)
{
  if( index != current_stream_index_ ) {

    if( (index < 0) || (index > (int) parsed_streams_.size()) ) {
      return false;
    }

    current_stream_index_ = index;
    current_pos_ = 0;
    hdl_parser_.reset();
    hdl_parser_.set_hdl_framing_mode(HDLFraming_Rotation);
    hdl_parser_.set_only_extract_frame_seams(false);
  }

  return true;
}

bool c_hdl_offline_pcap_reader::seek(int32_t pos)
{
  if( (current_stream_index_ >= 0) && (current_stream_index_ < (int) parsed_streams_.size()) ) {

    const parsed_stream & s =
        parsed_streams_[current_stream_index_];

    if( (pos >= 0) && (pos < (int) (s.frames.size())) ) {
      current_pos_ = pos;
      return true;
    }
  }

  return false;
}

int c_hdl_offline_pcap_reader::curpos() const
{
  return current_pos_;
}




#if 0
// Old code kept here for some time

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <fcntl.h>
//#include <core/get_time.h>
//#include <core/readdir.h>
//#include <core/ssprintf.h>
//#include <core/get_time.h>



#define INET_ADDR(a,b,c,d) \
  (uint32_t)((((uint32_t)(a))<<24)|(((uint32_t)(b))<<16)|(((uint32_t)(c))<<8)|(d))


template<>
const c_enum_member * members_of<c_hdl_source::State>()
{
  static const c_enum_member members[] = {
      {c_hdl_source::State_disconnected,"disconnected", "" },
      {c_hdl_source::State_connecting,"connecting", "" },
      {c_hdl_source::State_connected,"connected", "" },
      {c_hdl_source::State_starting, "starting", "" },
      {c_hdl_source::State_started, "started", "" },
      {c_hdl_source::State_stopping,"stopping", "" },
      {c_hdl_source::State_disconnecting,"disconnecting", "" },
      {c_hdl_source::State_disconnected,},
  };

  return members;
}



// assuming lidar_address has format "[ip]:[port]"
static bool aton(const std::string & address_string,
  /*out*/ uint32_t * address, /*out*/ uint16_t * port)
{
  * address = INADDR_ANY;
  * port = 2368;

  const char * s =
      address_string.c_str();

  if( *s && *s != ':' ) {

    uint8_t b1, b2, b3, b4;
    if( sscanf(s, "%hhu.%hhu.%hhu.%hhu", &b1, &b2, &b3, &b4) != 4 ) {
      errno = EINVAL;
      return false;
    }

    *address = INET_ADDR(b1, b2, b3, b4);

    while (s && *s != ':') {
      ++s;
    }
  }

  if ( port ) {

    if( *s && *s == ':' && *++s ) {
      if( sscanf(s, "%hu", port) != 1 ) {
        errno = EINVAL;
        return false;
      }
    }
  }

  return true;
}


static std::string ntoa(in_addr_t addrs)
{
  const uint8_t b1 = ((addrs>>0) & 0xFF);
  const uint8_t b2 = ((addrs>>8) & 0xFF);
  const uint8_t b3 = ((addrs>>16) & 0xFF);
  const uint8_t b4 = ((addrs>>24) & 0xFF);

  return ssprintf("%u.%u.%u.%u", b1, b2, b3, b4);
}

//static inline int64_t mksrchash(in_addr_t addrs, in_port_t port)
//{
//  return (((int64_t) addrs) << 32) | ((int64_t) port);
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// New (multi-lidar) API


c_hdl_connection::c_hdl_connection(c_hdl_source * hdl_source,
    uint32_t src_address, uint16_t src_port,
    HDLSensorType sensor_type,
    HDLReturnMode return_mode) :
    hdl_source_(hdl_source),
    address_(src_address),
    port_(src_port),
    sensor_type_(sensor_type),
    return_mode_(return_mode)
{
}

c_hdl_source * c_hdl_connection::hdl_source() const
{
  return hdl_source_;
}

void c_hdl_connection::set_hdl_source(c_hdl_source * hdl_source)
{
  hdl_source_ = hdl_source;
}

uint32_t c_hdl_connection::address() const
{
  return address_;
}

uint16_t c_hdl_connection::port() const
{
  return port_;
}

void c_hdl_connection::set_enabled(bool v)
{
  if ( enabled_ != v ){

    enabled_ = v;
    // hdl_parser_.reset();

    state_chagned();
  }
}

bool c_hdl_connection::enabled() const
{
  return enabled_;
}

bool c_hdl_connection::parse_packet(const uint8_t data[], ssize_t size)
{
  bool fOK = hdl_parser_.parse(data, size);
  if( (sensor_changed_ = hdl_parser_.sensor_changed()) ) {
    sensor_type_ = hdl_parser_.sensor_type();
    return_mode_ = hdl_parser_.return_mode();
  }
  return fOK;
}

bool c_hdl_connection::lidar_sensor_changed() const
{
  return sensor_changed_;// hdl_parser_.sensor_changed();
}

void c_hdl_connection::reset()
{
  hdl_parser_.reset();
}

size_t c_hdl_connection::queued_frames() const
{
  return hdl_parser_.frames.size();
}

c_hdl_frame::sptr c_hdl_connection::pop_queued_frame()
{
  if( hdl_parser_.frames.empty() ) {
    return nullptr;
  }

  c_hdl_frame::sptr frame =
      hdl_parser_.frames.front();

  hdl_parser_.frames.erase(
      hdl_parser_.frames.begin());

  return frame;
}

void c_hdl_connection::clear_queued_frames()
{
  while (!hdl_parser_.frames.empty()) {
    hdl_parser_.frames.erase(
        hdl_parser_.frames.begin());
  }
}


void c_hdl_connection::set_hdl_framing_mode(enum HDLFramingMode v)
{
  return hdl_parser_.set_hdl_framing_mode(v);
}

enum HDLFramingMode c_hdl_connection::hdl_framing_mode() const
{
  return hdl_parser_.hdl_framing_mode();
}

void c_hdl_connection::set_hdl_frame_seam_azimuth(double azimuth_in_degrees)
{
  return hdl_parser_.set_hdl_frame_seam_azimuth(azimuth_in_degrees);
}

double c_hdl_connection::hdl_frame_seam_azimuth() const
{
  return hdl_parser_.hdl_frame_seam_azimuth();
}

void c_hdl_connection::set_lidar_config_xml(const std::string & v)
{
  return hdl_parser_.set_lidar_config_xml(v);
}

const std::string & c_hdl_connection::lidar_config_xml() const
{
  return hdl_parser_.lidar_config_xml();
}

HDLSensorType c_hdl_connection::sensor_type() const
{
  return sensor_type_;
}

HDLReturnMode c_hdl_connection::return_mode() const
{
  return return_mode_;
}

const c_hdl_specification * c_hdl_connection::lidar_specification() const
{
  return hdl_parser_.lidar_specification();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int c_hdl_source::nonlive_stream_delay_us_ = 10;

c_hdl_source::c_hdl_source(const std::string & address) :
    address_(address)
{
}

c_hdl_source::~c_hdl_source()
{
  mtx_.lock();

  for( const c_hdl_connection::sptr & connection : hdl_connections_ ) {
    connection->set_hdl_source(nullptr);
    connection_destroyed(connection);
  }
  hdl_connections_.clear();

  mtx_.unlock();
}

const std::string & c_hdl_source::address() const
{
  return address_;
}

const std::string & c_hdl_source::display_name() const
{
  return display_name_;
}

c_hdl_connection::sptr  c_hdl_source::create_hdl_connection(uint32_t src_address, uint16_t src_port, HDLSensorType sensor_type, HDLReturnMode return_mode)
{
  return std::make_shared<c_hdl_connection>(this, src_address, src_port, sensor_type, return_mode);
}

const std::vector<c_hdl_connection::sptr> & c_hdl_source::hdl_connections() const
{
  return hdl_connections_;
}

c_hdl_connection::sptr c_hdl_source::hdl_connection(uint32_t src_address, uint16_t src_port) const
{
  for( const c_hdl_connection::sptr & lidar : hdl_connections_ ) {
    if( lidar->address() == src_address && lidar->port() == src_port ) {
      return lidar;
    }
  }
  return nullptr;
}

c_hdl_connection::sptr c_hdl_source::add_hdl_connection(uint32_t address, uint16_t port, HDLSensorType sensor_type, HDLReturnMode return_mode)
{
  c_hdl_connection::sptr connection =
      hdl_connection(address, port);

  if( connection ) {
    CF_ERROR("ERROR: HDL connection %s:%u already exists",
        ntoa(address).c_str(), port);
    return nullptr;
  }

  if( !(connection = create_hdl_connection(address, port, sensor_type, return_mode)) ) {
    CF_ERROR("create_hdl_connection(%s:%u) fails",
        ntoa(address).c_str(),
        port);
    return nullptr;
  }

  mtx_.lock();
  hdl_connections_.emplace_back(connection);
  mtx_.unlock();
  connection_created(connection);

  return connection;
}

void c_hdl_source::set_state(State state, const char * reasonmsg, ...)
{
  if( state != current_state_  ) {

    CF_DEBUG("STATE CHANGE : %s -> %s", toString(current_state_), toString(state));

    current_state_ = state;

    if( !reasonmsg || !*reasonmsg ) {
      reason_.clear();
    }
    else {
      va_list arglist;
      va_start(arglist, reasonmsg);
      reason_ = vssprintf(reasonmsg, arglist);
      va_end(arglist);
    }

    if( state == State_disconnected ) {
      close_file();
      for( const c_hdl_connection::sptr & connection : hdl_connections_ ) {
        connection->reset();
      }
    }

    state_chagned(this, current_state_);
  }
}

c_hdl_source::State c_hdl_source::state() const
{
  return current_state_;
}

const std::string & c_hdl_source::reason() const
{
  return reason_;
}

std::mutex & c_hdl_source::mutex()
{
  return mtx_;
}

void c_hdl_source::set_nonlive_stream_delay_us(int v)
{
  nonlive_stream_delay_us_ =
      std::max(1, std::min(v, 100 * 1000));
}

int c_hdl_source::nonlive_stream_delay_us()
{
  return nonlive_stream_delay_us_;
}

bool c_hdl_source::open(bool start_after_connect /*= false*/)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if ( current_state_ != State_disconnected ) {
    CF_ERROR("Not appropriate state: %s, must be disconnected",
        toString(current_state_));
    return false;
  }

  auto_start_after_connect_ = start_after_connect;
  set_state(State_connecting);

  std::thread([this]() {

    if( !open_file() ) {

      std::lock_guard<std::mutex> lock(mtx_);

      set_state(State_disconnected,
          "Can not open file");
      return;
    }

    mtx_.lock();

    while ( current_state_ == State_connecting ) {

      const uint8_t * payload = nullptr;
      ssize_t payload_size = 0;
      uint32_t src_address;
      uint16_t src_port;
      bool new_lidar_discovered;

      mtx_.unlock();

      payload_size =
          read_packet(&payload,
              &src_address,
              &src_port);

      mtx_.lock();

      if ( payload_size < 0 ) {
        CF_ERROR("read_packet() fails");
        break;
      }

      if ( payload_size == 0 ) {
        continue;
      }

      c_hdl_connection::sptr connection =
          hdl_connection(src_address, src_port);

      if ( (new_lidar_discovered = !connection) ) {

        CF_DEBUG("new_lidar_discovered: %s:%u", ntoa(src_address).c_str(), src_port);

        if ( !(connection = create_hdl_connection(src_address, src_port)) ) {
          CF_ERROR("create_hdl_connection(%s:%u) fails",
              ntoa(src_address).c_str(),
              src_port);
          continue;
        }
      }

      if ( !connection->parse_packet(payload, payload_size) ) {

        CF_ERROR("lidar->parse_packet() fails for lidar %s:%u",
            ntoa(src_address).c_str(),
            src_port);

        continue;
      }

      if ( new_lidar_discovered ) {

        hdl_connections_.emplace_back(connection);

        mtx_.unlock();
        connection_created(connection);
        mtx_.lock();
      }

      if ( connection->lidar_sensor_changed() ) {
        CF_DEBUG("SENSOR CHANGED FOR LIDAR %s:%u", ntoa(connection->address()).c_str(), connection->port());
        mtx_.unlock();
        connection->sensor_changed();
        mtx_.lock();
      }

      if ( connection->queued_frames() > 0 ) {
        set_state(State_connected);
        break;
      }
    }

    switch (current_state_) {
      case State_connected:
        if ( auto_start_after_connect_ ) {
          start(false);
        }
        break;

      case State_connecting:
      case State_disconnecting:
        set_state(State_disconnected,
          "Can not read very first frames from source");
        break;

      case State_disconnected:
        break;

      case State_starting:
      case State_started:
      case State_stopping:
        CF_ERROR("Unexpected state change: %s",
          toString(current_state_));
        break;
    }

    mtx_.unlock();

  }).detach();

  return true;
}

void c_hdl_source::start(bool lock_mutex /*= true*/)
{
  if ( lock_mutex ) {
    mtx_.lock();
  }

  if( current_state_ == State_disconnected ) {

    if ( lock_mutex ) {
      mtx_.unlock();
    }

    open(true);
    return;
  }

  if ( current_state_ != State_connected ) {

    CF_ERROR("Invalid state: %s. Must be %s",
        toString(current_state_),
        toString(State_connected));
  }
  else {

    set_state(State_starting);


    std::thread([this]() {

      mtx_.lock();

      if ( current_state_ == State_starting ) {

        set_state(State_started);

        while ( current_state_ == State_started ) {

          c_hdl_frame::sptr frame;
          const uint8_t * payload = nullptr;
          ssize_t payload_size = 0;
          uint32_t src_address;
          uint16_t src_port;
          bool new_lidar_discovered;

          mtx_.unlock();

          payload_size =
              read_packet(&payload,
                  &src_address,
                  &src_port);

          mtx_.lock();

          if ( payload_size < 0 ) {
            CF_ERROR("read_packet() fails");
            break;
          }

          if ( payload_size == 0 ) {
            continue;
          }

          c_hdl_connection::sptr connection =
              hdl_connection(src_address, src_port);

          if ( (new_lidar_discovered = !connection) ) {

            CF_DEBUG("new_lidar_discovered: %s:%u", ntoa(src_address).c_str(), src_port);

            if ( !(connection = create_hdl_connection(src_address, src_port, HDLSensor_unknown, HDLReturnMode_unknown)) ) {
              CF_ERROR("create_hdl_connection(%s:%u) fails",
                  ntoa(src_address).c_str(),
                  src_port);
              continue;
            }
          }

          if ( (connection->enabled() || new_lidar_discovered) && !connection->parse_packet(payload, payload_size) ) {

            CF_ERROR("lidar->parse_packet() fails for lidar %s:%u",
                ntoa(src_address).c_str(),
                src_port);

            continue;
          }


          if ( new_lidar_discovered ) {

            hdl_connections_.emplace_back(connection);

            mtx_.unlock();
            connection_created(connection);
            mtx_.lock();
          }


          if ( connection->lidar_sensor_changed() ) {
            CF_DEBUG("SENSOR CHANGED FOR LIDAR %u:%u", connection->address(), connection->port());
            mtx_.unlock();
            connection->sensor_changed();
            mtx_.lock();
          }

          while ( (frame = connection->pop_queued_frame()) ) {
            if ( connection->enabled() ) {
              mtx_.unlock();
              connection->frame_received(frame);
              mtx_.lock();
            }
          }
        }
      }

      switch (current_state_) {
        case State_disconnected:
        case State_connecting:
        case State_connected:
        case State_starting:
          CF_ERROR("Unexpected state change: %s",
              toString(current_state_));
          break;
        case State_started:
          set_state(State_connected,
            "read_packet() fails, possible EOF");
          break;
        case State_stopping:
          set_state(State_connected);
        break;
        case State_disconnecting:
          set_state(State_disconnected);
          break;
      }

      mtx_.unlock();

    }).detach();
  }

  if ( lock_mutex ) {
    mtx_.unlock();
  }
}

void c_hdl_source::stop()
{
  std::lock_guard<std::mutex> lock(mtx_);

  switch (current_state_) {
  case State_connecting:
  case State_connected:
  case State_starting:
    CF_ERROR("APP BUG: Inappropriate state: %s", toString(current_state_));
    break;
  case State_started:
    set_state(State_stopping);
    break;
  case State_stopping:
  case State_disconnecting:
  case State_disconnected:
    break;
  }

  return;
}

void c_hdl_source::step()
{
  mtx_.lock();

  if( current_state_ == State_connected ) {
    // TODO:Implement c_hdl_source::step() in more efficient way avoiding code duplicate from start()

    std::thread([this]() {

      mtx_.lock();

      if ( current_state_ == State_connected ) {

        set_state(State_started);

        while ( current_state_ == State_started ) {

          c_hdl_frame::sptr frame;
          const uint8_t * payload = nullptr;
          ssize_t payload_size = 0;
          uint32_t src_address;
          uint16_t src_port;
          bool new_lidar_discovered;

          mtx_.unlock();

          payload_size =
              read_packet(&payload,
                  &src_address,
                  &src_port);

          mtx_.lock();

          if ( payload_size < 0 ) {
            CF_ERROR("read_packet() fails");
            break;
          }

          if ( payload_size == 0 ) {
            continue;
          }

          c_hdl_connection::sptr connection =
              hdl_connection(src_address, src_port);

          if ( (new_lidar_discovered = !connection) ) {

            CF_DEBUG("new_lidar_discovered: %s:%u", ntoa(src_address).c_str(), src_port);

            if ( !(connection = create_hdl_connection(src_address, src_port, HDLSensor_unknown, HDLReturnMode_unknown)) ) {
              CF_ERROR("create_hdl_connection(%s:%u) fails",
                  ntoa(src_address).c_str(),
                  src_port);
              continue;
            }
          }

          if ( (connection->enabled() || new_lidar_discovered) && !connection->parse_packet(payload, payload_size) ) {

            CF_ERROR("lidar->parse_packet() fails for lidar %s:%u",
                ntoa(src_address).c_str(),
                src_port);

            continue;
          }


          if ( new_lidar_discovered ) {

            hdl_connections_.emplace_back(connection);

            mtx_.unlock();
            connection_created(connection);
            mtx_.lock();
          }


          if ( connection->lidar_sensor_changed() ) {
            CF_DEBUG("SENSOR CHANGED FOR LIDAR %u:%u", connection->address(), connection->port());
            mtx_.unlock();
            connection->sensor_changed();
            mtx_.lock();
          }

          bool gotFrame = false;
          while ( (frame = connection->pop_queued_frame()) ) {
            if ( connection->enabled() ) {
              mtx_.unlock();
              connection->frame_received(frame);
              gotFrame = true;
              mtx_.lock();
            }
          }

          if ( gotFrame ) {
            break;
          }
        }
      }

      switch (current_state_) {
        case State_disconnected:
        case State_connecting:
        case State_connected:
        case State_starting:
          CF_ERROR("Unexpected state change: %s",
              toString(current_state_));
          break;
        case State_started:
          set_state(State_connected,
            "read_packet() fails, possible EOF");
          break;
        case State_stopping:
          set_state(State_connected);
        break;
        case State_disconnecting:
          set_state(State_disconnected);
          break;
      }

      mtx_.unlock();

    }).detach();
  }

  mtx_.unlock();
}

void c_hdl_source::close(bool sync)
{
  mtx_.lock();

  switch (current_state_) {
  case State_disconnected:
  case State_disconnecting:
    break;
  case State_connecting:
  case State_starting:
  case State_started:
  case State_stopping:
    set_state(State_disconnecting);
    break;
  case State_connected:
    set_state(State_disconnected);
    break;
  }

  if ( sync ) {
    while (current_state_ != State_disconnected) {
      mtx_.unlock();
      usleep(10 * 1000);
      mtx_.lock();
    }
  }

  mtx_.unlock();
  CF_DEBUG("c_hdl_source::close() OK");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_hdl_pcap_file_reader::c_hdl_pcap_file_reader(const std::string & address, const std::string & filename, const std::string & options) :
    base(address),
    filename_(filename),
    options_(options)
{
  display_name_ =
      ssprintf("PCAP: %s",
          get_file_name(filename).c_str());

}

c_hdl_pcap_file_reader:: ~c_hdl_pcap_file_reader()
{
  close(true);
}

const std::string & c_hdl_pcap_file_reader:: filename() const
{
  return filename_;
}

bool c_hdl_pcap_file_reader::open_file()
{
#if HAVE_PCAP
  if( !pcap_.open(filename_, options_) ) {
    CF_ERROR("pcap_.open('%s') fails", filename_.c_str());
    return false;
  }
  return true;
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
  return false;
#endif
}

void c_hdl_pcap_file_reader:: close_file()
{
#if HAVE_PCAP
  pcap_.close();
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
#endif
}

ssize_t c_hdl_pcap_file_reader::read_packet(const uint8_t ** ppayload,
    uint32_t * src_address, uint16_t * src_port)
{
#if HAVE_PCAP
  const pcap_pkthdr * pkt_header = nullptr;
  const c_pcap_data_header * data_header = nullptr;
  const uint8_t * payload = nullptr;
  in_addr_t ipsrc = 0;
  in_port_t port = 0;
  int status;

  if( !pcap_.is_open() ) {
    CF_ERROR("pcap file is not open");
    return -1;
  }

  if( !hdl_connections_.empty() ) {
    usleep(std::max(10, nonlive_stream_delay_us_));
  }

  if( (status = pcap_.read(&pkt_header, &data_header, &payload)) <= 0 ) {
    CF_ERROR("pcap_.read() fails");
    return -1;
  }

  if( !payload ) {
    return 0;
  }

  const uint payload_size =
      pkt_header->len - pcap_.data_header_size();

  if( payload_size != hdl_lidar_packet_size() ) {
    // Not HDL packet
    return 0;
  }

  // CF_DEBUG("pcap_.datalinktype()=%d", pcap_.datalinktype());
  switch (pcap_.datalinktype()) {
  case DLT_NULL:
    ipsrc = data_header->loopbak.ip.ip_src.s_addr;
    port = data_header->loopbak.udp.dest;
    break;
  case DLT_EN10MB:
    ipsrc = data_header->en10mb.ip.ip_src.s_addr;
    port = data_header->en10mb.udp.dest;
    break;
  case DLT_LINUX_SLL:
    ipsrc = data_header->sll.ip.ip_src.s_addr;
    port = data_header->sll.udp.dest;
    break;
  case DLT_LINUX_SLL2:
    ipsrc = data_header->sll2.ip.ip_src.s_addr;
    port = data_header->sll2.udp.dest;
    break;
  case DLT_USER1:
    ipsrc = data_header->user1.ip.ip_src.s_addr;
    port = data_header->user1.udp.dest;
    break;
  default:
    break;
  }

  if( !ipsrc || !port ) {
    // unparsed packet
    return 0;
  }

  *ppayload = payload;
  *src_address = ntohl(ipsrc);
  *src_port = ntohs(port);

  return payload_size;
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
  return -1;
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_hdl_udp_port_reader::c_hdl_udp_port_reader(const std::string & address, uint16_t port, const std::string & options) :
    base(address),
    port_(port),
    options_(options)
{
  display_name_ =
      ssprintf("UDP: %u",
          port);
}

c_hdl_udp_port_reader::~c_hdl_udp_port_reader()
{
  close(true);
}

uint16_t c_hdl_udp_port_reader::port() const
{
  return port_;
}

bool c_hdl_udp_port_reader::open_file()
{
  static const auto set_reuse_address =
      [](int so, int reuse) -> bool {
        return setsockopt(so, SOL_SOCKET, SO_REUSEADDR,
            &reuse, sizeof(reuse)) != -1;
      };

  static const auto set_recv_timeout =
      [](int so, int seconds) -> bool {
#ifdef _WIN32
          // WINDOWS
          DWORD timeout = seconds * 1000;
          return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof (timeout)) == 0;
#else
          // LINUX / MAC
          struct timeval timeout = {.tv_sec = seconds, .tv_usec = 0};
          return setsockopt(so, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
#endif
        };

  bool fOk = false;
  sockaddr_in bind_addrs = { 0 };

  if( (so_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
    CF_ERROR("socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP) fails: %s",
        strerror(errno));
    goto end;
  }

  if( !set_reuse_address(so_, true) ) {
    CF_ERROR("set_reuse_address(so=%d) fails: %s",
        so_, strerror(errno));
    goto end;
  }

  if( !set_recv_timeout(so_, 1) ) {
    CF_ERROR("set_recv_timeout(so=%d, tmout = 1 sec) fails: %s",
        so_, strerror(errno));
    goto end;
  }

  bind_addrs.sin_family = AF_INET;
  bind_addrs.sin_port = htons(port_);
  bind_addrs.sin_addr = { INADDR_ANY };

  if( bind(so_, (sockaddr*) &bind_addrs, sizeof(bind_addrs)) < 0 ) {
    CF_ERROR("bind(so=%d, port=%u) fails: %s",
        so_, port_, strerror(errno));
    goto end;
  }

  fOk = true;

  end:

  if( !fOk ) {
    close_file();
  }

  return fOk;
}

void c_hdl_udp_port_reader::close_file()
{
  if( so_ >= 0 ) {
    ::shutdown(so_, SHUT_RDWR);
    ::close(so_);
    so_ = -1;
  }

}

ssize_t c_hdl_udp_port_reader::read_packet(const uint8_t ** ppayload, uint32_t * src_address, uint16_t * src_port)
{
  sockaddr_in sender_address;

  static const auto recvpkt =
      [](int so, sockaddr_in * sender_address, uint8_t pkt[], int max_packet_size) -> int {

        socklen_t sender_address_len =
            sizeof(*sender_address);

        return recvfrom(so, pkt,
            max_packet_size, 0,
            (sockaddr*) sender_address,
            &sender_address_len);
      };

  if ( so_ < 0 ) {
    CF_ERROR("UDP socket is not in listening mode");
    return -1;
  }

  const ssize_t payload_size =
      recvpkt(so_,
          &sender_address,
          payload,
          max_packet_size);

  if( payload_size <= 0 ) {

    switch (errno) {
    case EAGAIN:
      case ETIMEDOUT:
      return 0;
    default:
      CF_ERROR("recvpkt() fails with errno=%d (%s)",
          errno, strerror(errno));
      return -1;
    }
  }

  if( payload_size != hdl_lidar_packet_size() ) {
    // Not an HDL packet
    return 0;
  }

  const in_addr_t ipsrc =
      sender_address.sin_addr.s_addr;

  const in_port_t port =
      sender_address.sin_port;

  if ( !ipsrc || !port ) {
    return 0;
  }

  *ppayload = payload;
  *src_address = ntohl(ipsrc);
  *src_port = ntohs(port);

  return payload_size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Old (single lidar) API

c_hdl_frame_reader::c_hdl_frame_reader()
{
}

const std::map<in_addr_t, double> & c_hdl_frame_reader::available_lidars() const
{
  return available_lidars_;
}

bool c_hdl_frame_reader::available_lidars_changed() const
{
  const bool retval = available_lidars_changed_;
  available_lidars_changed_ = false;
  return retval;
}


void c_hdl_frame_reader::set_current_lidar(in_addr_t v)
{
  current_lidar_ = v;
  hdl_parser_.reset();
}

in_addr_t c_hdl_frame_reader::current_lidar() const
{
  return current_lidar_;
}

void c_hdl_frame_reader::set_hdl_framing_mode(enum HDLFramingMode v)
{
  return hdl_parser_.set_hdl_framing_mode(v);
}

enum HDLFramingMode c_hdl_frame_reader::hdl_framing_mode() const
{
  return hdl_parser_.hdl_framing_mode();
}

void c_hdl_frame_reader::set_hdl_frame_seam_azimuth(double azimuth_in_degrees)
{
  return hdl_parser_.set_hdl_frame_seam_azimuth(azimuth_in_degrees);
}

double c_hdl_frame_reader::hdl_frame_seam_azimuth() const
{
  return hdl_parser_.hdl_frame_seam_azimuth();
}

void c_hdl_frame_reader::set_lidar_config_xml(const std::string & v)
{
  return hdl_parser_.set_lidar_config_xml(v);
}

const std::string & c_hdl_frame_reader::lidar_config_xml() const
{
  return hdl_parser_.lidar_config_xml();
}

HDLSensorType c_hdl_frame_reader::sensor_type() const
{
  return hdl_parser_.sensor_type();
}

HDLReturnMode c_hdl_frame_reader::return_mode() const
{
  return hdl_parser_.return_mode();
}

const c_hdl_specification * c_hdl_frame_reader::lidar_specification() const
{
  return hdl_parser_.lidar_specification();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_hdl_pcap_reader::~c_hdl_pcap_reader()
{
  close();
}

bool c_hdl_pcap_reader::open(const std::string & pcapfile, const std::string & options)
{
#if HAVE_PCAP

  close();

  hdl_parser_.reset();

  if( !pcap_.open(pcapfile, options) ) {
    CF_ERROR("pcap_.open('%s') fals", pcapfile.c_str());
    return false;
  }

  return true;
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
  return false;
#endif
}

bool c_hdl_pcap_reader::is_open() const
{
#if HAVE_PCAP
  return pcap_.is_open();
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
  return false;
#endif
}

void c_hdl_pcap_reader::close()
{
#if HAVE_PCAP
  pcap_.close();
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
#endif
}

c_hdl_frame::sptr c_hdl_pcap_reader::recv()
{
#if HAVE_PCAP

  if ( !pcap_.is_open() ) {
    CF_ERROR("pcap file is not open");
    return nullptr;
  }

  static const auto pop_frame =
      [](c_hdl_packet_parser & hdl_parser) -> c_hdl_frame::sptr {
        c_hdl_frame::sptr frame = hdl_parser.frames.front();
        hdl_parser.frames.erase(hdl_parser.frames.begin());
        return frame;
      };

  const pcap_pkthdr * pkt_header = nullptr;
  const c_pcap_data_header * data_header =  nullptr;
  const uint8_t * payload = nullptr;

  c_hdl_frame::sptr frame;

  const HDLFramingMode framing_mode =
      hdl_parser_.hdl_framing_mode();

  int status;

  if( hdl_parser_.frames.size() > 0 ) {
    if( hdl_parser_.frames.size() > 1 ) {
      return pop_frame(hdl_parser_);
    }
    if( framing_mode != HDLFraming_Rotation ) {
      return pop_frame(hdl_parser_);
    }
  }

  while ((status = pcap_.read(&pkt_header, &data_header, &payload)) >= 0) {

    if ( !payload ) {
      continue;
    }

    const uint payload_size =
        pkt_header->len - pcap_.data_header_size();

    if( payload_size != hdl_lidar_packet_size() ) {
        //      CF_DEBUG("payload_size=%u ignored (expected size is %u) datalinktype()=%d data_header_size=%d",
        //          payload_size,
        //          hdl_lidar_packet_size(),
        //          pcap_.datalinktype(),
        //          pcap_.data_header_size());
        //      if ( payload_size == 512 ) {
        //         NMEA GPRMC sentence ?
        //      }

      continue;
    }

    in_addr_t ipsrc = 0;
    //uint16_t port = 0;

    //CF_DEBUG("pcap_.datalinktype()=%d", pcap_.datalinktype());
    switch (pcap_.datalinktype()) {
    case DLT_NULL:
      ipsrc = data_header->loopbak.ip.ip_src.s_addr;
      break;
    case DLT_EN10MB:
      ipsrc = data_header->en10mb.ip.ip_src.s_addr;
      //port = data_header->en10mb.udp.dest;
      break;
    case DLT_LINUX_SLL:
      ipsrc = data_header->sll.ip.ip_src.s_addr;
      break;
    case DLT_LINUX_SLL2:
      ipsrc = data_header->sll2.ip.ip_src.s_addr;
      break;
    case DLT_USER1:
      ipsrc = data_header->user1.ip.ip_src.s_addr;
      break;
    default:
      break;
    }

    if ( !ipsrc ) {
      continue;
    }

    //CF_DEBUG("pkt: size=%u from %s:%u", payload_size, ntoa(ipsrc).c_str(), ntohs(port));

    const auto pos =
        available_lidars_.find(ipsrc);

    if ( pos != available_lidars_.end() ) {
      pos->second = get_realtime_ms();
    }
    else {
      available_lidars_[ipsrc] = get_realtime_ms();
      available_lidars_changed_ = true;
    }

    if( !current_lidar_ ) {
      current_lidar_ = ipsrc;
    }
    else if( ipsrc != current_lidar_ ) {
      continue;
    }

    if( !hdl_parser_.parse(payload, payload_size) ) {
      CF_ERROR("packet_parser_.parse() fails");
      break;
    }

    if( hdl_parser_.frames.size() > 1 ) {
      frame = pop_frame(hdl_parser_);
      break;
    }
  }

  if ( status <= 0 ) {
    CF_ERROR("pcap_.read() fails");
  }

  if( !frame && hdl_parser_.frames.size() > 0 ) {
    frame = pop_frame(hdl_parser_);
  }

  return frame;
#else
  CF_ERROR("ERROR: pcap reader functionality is disabled");
  return nullptr;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



c_hdl_udp_reader::~c_hdl_udp_reader()
{
  close();
}

bool c_hdl_udp_reader::open(const std::string & lidar_address, const std::string & options)
{
  static const auto set_reuse_address =
      [](int so, int reuse) -> bool {
        return setsockopt(so, SOL_SOCKET, SO_REUSEADDR,
            &reuse, sizeof(reuse)) != -1;
      };

  static const auto set_non_blocking =
      [](int so, bool optval) -> bool {

        int flags, status;

        if ( (flags = fcntl(so, F_GETFL, 0)) < 0 ) {
          status = -1;
        }
        else if ( optval ) {
          status = fcntl(so, F_SETFL, flags | O_NONBLOCK);
        }
        else {
          status = fcntl(so, F_SETFL, flags & ~O_NONBLOCK);
        }

        return status != -1;
      };

  static const auto set_recv_timeout =
      [](int so, int seconds) -> bool {
#ifdef _WIN32
          // WINDOWS
        DWORD timeout = seconds * 1000;
        return setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof (timeout)) == 0;
#else
        // LINUX / MAC
        struct timeval timeout = {.tv_sec = seconds, .tv_usec = 0};
        return setsockopt(so, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0;
#endif
      };


  bool fOk = false;
  sockaddr_in bind_addrs = {0};

  close();

  stop_ = false;

  if( !aton(lidar_address, &bind_address_, &bind_port_) ) {
    CF_ERROR("parse_lidar_address(%s) fails: %s",
        lidar_address.c_str(),
        strerror(errno));
    goto end;
  }

  if( (so_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 ) {
    CF_ERROR("socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP) fails: %s",
        strerror(errno));
    goto end;
  }

  if( !set_reuse_address(so_, true) ) {
    CF_ERROR("set_reuse_address(so=%d) fails: %s",
        so_, strerror(errno));
    goto end;
  }


  bind_addrs.sin_family = AF_INET;
  bind_addrs.sin_port = htons(bind_port_);
  bind_addrs.sin_addr = { INADDR_ANY };

  if( bind(so_, (sockaddr*) &bind_addrs, sizeof(bind_addrs)) < 0 ) {
    CF_ERROR("bind(so=%d, port=%u) fails: %s",
        so_, bind_port_, strerror(errno));
    goto end;
  }


  fOk = true;

end:
  if ( !fOk ) {
    if ( so_ >= 0 ) {
      ::close(so_);
      so_ = -1;
    }
  }

  return fOk;
}

bool c_hdl_udp_reader::is_open() const
{
  return so_ >= 0;
}

void c_hdl_udp_reader::close()
{
  stop();

  if( so_ >= 0 ) {
    ::shutdown(so_, SHUT_RDWR);
    ::close(so_);
    so_ = -1;
  }
}

void c_hdl_udp_reader::stop()
{
  stop_ = true;
}


c_hdl_frame::sptr c_hdl_udp_reader::recv()
{
  if ( so_ < 0 ) {
    CF_ERROR("UDP socket is not listening");
    return nullptr;
  }

  static const auto pop_frame =
      [](c_hdl_packet_parser & hdl_parser) -> c_hdl_frame::sptr {
        c_hdl_frame::sptr frame = hdl_parser.frames.front();
        hdl_parser.frames.erase(hdl_parser.frames.begin());
        return frame;
      };


  static const auto recvpkt =
      [](int so, sockaddr_in * sender_address, uint8_t pkt[], int max_packet_size) -> int {

    //        memset(sender_address, 0,
    //            sizeof(*sender_address));

        socklen_t sender_address_len =
            sizeof(*sender_address);

        return recvfrom(so, pkt,
            max_packet_size, 0,
            (sockaddr*) sender_address,
            &sender_address_len);
      };


  c_hdl_frame::sptr frame;

  const HDLFramingMode framing_mode =
      hdl_parser_.hdl_framing_mode();

  if( hdl_parser_.frames.size() > 0 ) {
    if( hdl_parser_.frames.size() > 1 ) {
      return pop_frame(hdl_parser_);
    }
    if( framing_mode != HDLFraming_Rotation ) {
      return pop_frame(hdl_parser_);
    }
  }


  const int max_packet_size = 2048;

  sockaddr_in sender_address;
  uint8_t pkt[max_packet_size];

  while (!stop_) {

    const ssize_t pktsize =
        recvpkt(so_,
            &sender_address,
            pkt,
            max_packet_size);

    if( pktsize <= 0 ) {

      CF_ERROR("recvpkt() fails with errno=%d (%s)",
          errno, strerror(errno));

      if( errno == ETIMEDOUT ) {
        continue;
      }

      break;
    }

    //CF_DEBUG("pkt: size=%zd from %s:%u", pktsize, ntoa(sender_address.sin_addr.s_addr).c_str(), ntohs(sender_address.sin_port));

    if( pktsize != hdl_lidar_packet_size() ) {
      continue;
    }

    const in_addr_t ipsrc =
        sender_address.sin_addr.s_addr;
    if ( !ipsrc ) {
      continue;
    }

    const auto pos =
        available_lidars_.find(ipsrc);

    if ( pos != available_lidars_.end() ) {
      pos->second = get_realtime_ms();
    }
    else {
      available_lidars_[ipsrc] = get_realtime_ms();
      available_lidars_changed_ = true;
    }

    if( !current_lidar_ ) {
      current_lidar_ = ipsrc;
    }
    else if( ipsrc != current_lidar_ ) {
      continue;
    }

    if( !hdl_parser_.parse(pkt, pktsize) ) {
      CF_ERROR("packet_parser_.parse() fails");
      break;
    }

    if( hdl_parser_.frames.size() > 1 ) {
      frame = pop_frame(hdl_parser_);
      break;
    }
  }

  if( !frame && hdl_parser_.frames.size() > 0 ) {
    frame = pop_frame(hdl_parser_);
  }

  return frame;
}

#endif
#endif // HAVE_PCAP

