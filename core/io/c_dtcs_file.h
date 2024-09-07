/*
 * c_dtcs_file.h
 *
 *  Created on: Jul 22, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_datc_file_h__
#define __c_datc_file_h__


#include "c_file_handle.h"
#include <algorithm>
#include <cinttypes>
#include <functional>

class c_dtcs_file
{
public:
  typedef c_dtcs_file this_class;

  struct c_file_header
  {
    uint64_t file_tag = 0;
    uint64_t index_offset = 0 ;
  };

  struct c_index_header
  {
    uint32_t streams_count = 0;
  };

  struct c_stream_header
  {
    uint32_t chunks_count = 0;
    char stream_name[128] = {0};
  };

  struct c_chunk_header
  {
    uint32_t stream_index = 0;
    uint32_t payload_size = 0;
  };

  struct c_chunk_index_item
  {
    c_chunk_header chunk_header;
    uint64_t payload_offset = 0;
  };

  c_dtcs_file(const std::string & filename = "") :
      _filename(filename)
  {
  }

  const std::string & filename() const
  {
    return _filename;
  }

  const char * cfilename() const
  {
    return _filename.c_str();
  }

  bool is_open() const
  {
    return _fd.is_open();
  }

  void set_file_tag(uint64_t v)
  {
    _file_tag = v;
  }

  uint64_t file_tag() const
  {
    return _file_tag;
  }

  static inline constexpr uint64_t mktag(char a0, char a1, char a2, char a3, char a4, char a5, char a6, char a7)
  {
    return (uint64_t(a7) << 56) |
        (uint64_t(a6) << 48) |
        (uint64_t(a5) << 40) |
        (uint64_t(a4) << 32) |
        (uint64_t(a3) << 24) |
        (uint64_t(a2) << 16) |
        (uint64_t(a1) << 8) |
        (uint64_t(a0) << 0);
  }


protected:
  c_file_handle _fd;
  std::string _filename;
  uint64_t _file_tag = mktag('c', '_', 'd', 't', 'c', 's', '_', 'f');
};

class c_dtcs_writer :
    public c_dtcs_file
{
public:
  typedef c_dtcs_writer this_class;
  typedef c_dtcs_file base;

  struct c_stream
  {
    std::string name;
    std::vector<c_chunk_index_item> chunks;
  };

  c_dtcs_writer(const std::string & filename = "");
  ~c_dtcs_writer();

  bool create(const std::string & filename = "");
  void close();

  int add_stream(const std::string & name);
  bool write(uint32_t stream_index, const void * payload, uint32_t size);
  bool writex(uint32_t stream_index, const std::function<bool(c_file_handle & fd)> & wfn);

  template<class T>
  bool write(uint32_t stream_index, const T & data)
  {
    return write(stream_index, &data, sizeof(data));
  }

  uint32_t num_streams() const
  {
    return (uint32_t )_streams.size();
  }

  const std::vector<c_stream> & streams() const
  {
    return _streams;
  }

protected:
  std::vector<c_stream> _streams;
};

class c_dtcs_reader :
    public c_dtcs_file
{
public:
  typedef c_dtcs_reader this_class;
  typedef c_dtcs_file base;

  struct c_stream
  {
    std::string name;
    std::vector<c_chunk_index_item> chunks;
    uint32_t curpos = 0;
  };

  c_dtcs_reader(const std::string & filename = "") :
      base(filename)
  {
  }

  ~c_dtcs_reader()
  {
    close();
  }

  uint32_t num_streams() const
  {
    return (uint32_t )_streams.size();
  }

  uint32_t num_frames(uint32_t stream_index) const
  {
    return stream_index < _streams.size() ? _streams[stream_index].chunks.size() : 0;
  }

  uint32_t num_frames() const
  {
    return num_frames(_selected_stream);
  }

  uint32_t curpos(uint32_t stream_index) const
  {
    return stream_index < _streams.size() ? _streams[stream_index].curpos : 0;
  }

  uint32_t curpos() const
  {
    return curpos(_selected_stream);
  }

  const std::vector<c_stream> & streams() const
  {
    return _streams;
  }

  const c_stream & streams(uint32_t index) const
  {
    return _streams[index];
  }

  int find_stream(const std::string & stream_name) const
  {
    const auto pos =
        std::find_if(_streams.begin(), _streams.end(),
            [&](const c_stream & s) {
              return s.name == stream_name;
            });

    return pos == _streams.end() ? -1 :
        pos - _streams.begin();
  }

  bool select_stream(uint32_t stream_index)
  {
    if ( stream_index < _streams.size() ) {
      _selected_stream = stream_index;
      return true;
    }
    errno = EINVAL;
    return false;
  }

  bool select_stream(const std::string & stream_name)
  {
    const int stream_index =
        find_stream(stream_name);

    if ( stream_index < 0 ) {
      errno = EINVAL;
      return false;
    }

    return select_stream(stream_index);
  }

  uint32_t selected_stream() const
  {
    return _selected_stream;
  }

  bool seek(uint32_t chunk_index_in_current_stream)
  {
    if ( _selected_stream < _streams.size() ) {
      if ( chunk_index_in_current_stream < _streams[_selected_stream].chunks.size() ) {
        _streams[_selected_stream].curpos = chunk_index_in_current_stream;
        return true;
      }
    }
    errno = EINVAL;
    return false;
  }

  bool seek(uint32_t stream_index, uint32_t chunk_index)
  {
    if( stream_index < _streams.size() ) {
      if( chunk_index < _streams[stream_index].chunks.size() ) {
        _streams[stream_index].curpos = chunk_index;
        return true;
      }
    }
    errno = EINVAL;
    return false;
  }

  bool open(const std::string & filename = "");
  void close();

  ssize_t read(uint32_t stream_index, void * buff, size_t maxsize);
  bool readx(uint32_t stream_index, const std::function<bool(c_file_handle & fd)> & rfn);

  ssize_t read(void * buff, size_t maxsize)
  {
    return read(_selected_stream, buff, maxsize);
  }

  bool readx(const std::function<bool(c_file_handle & fd)> & rfn)
  {
    return readx(_selected_stream, rfn);
  }

protected:
  c_file_header _file_header;
  std::vector<c_stream> _streams;
  uint32_t _selected_stream = 0;
};




#endif /* __c_datc_file_h__ */
