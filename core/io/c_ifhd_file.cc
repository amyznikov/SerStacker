/*
 * c_ifhd_file.cc
 *
 *  Created on: Oct 25, 2023
 *      Author: amyznikov
 */

#include "c_ifhd_file.h"
#include <string.h>
#include <fcntl.h>
#include <limits>
#include <cmath>
#include <core/debug.h>


#ifdef _MSC_VER
# pragma warning (disable:4996)
#endif

#if ! __DEBUG_H_INCLUDED__
#include <stdio.h>

#define CF_ERROR(...) \
    fprintf(stderr, "%s(): %d ", __func__, __LINE__), \
    fprintf(stderr, __VA_ARGS__), \
    fprintf(stderr, "\n"), \
    fflush(stderr)

#endif

// check if it is a IFHD file
static bool check_file_header(const ifhd::FileHeader & file_header)
{
  const char * id =
      (const char * )&file_header.file_id;

  if( id[0] != 'I' || id[1] != 'F' || id[2] != 'H' || id[3] != 'D' ) {
    return false;
  }

  return true;
}

c_ifhd_file::c_ifhd_file()
{
}

c_ifhd_file::c_ifhd_file(const std::string & filename) :
    _filename(filename)
{
}

const std::string& c_ifhd_file::filename() const
{
  return _filename;
}


bool c_ifhd_reader::_print_stream_names_on_file_open = false;

c_ifhd_reader::c_ifhd_reader() :
    this_class("")
{
}

c_ifhd_reader::c_ifhd_reader(const std::string & filename) :
    base(filename)
{
  memset(&_file_header, 0, sizeof(_file_header));
}

c_ifhd_reader::~c_ifhd_reader()
{
  close();
}


void c_ifhd_reader::set_print_stream_names_on_file_open(bool v)
{
  _print_stream_names_on_file_open = v;
}

bool c_ifhd_reader::print_stream_names_on_file_open()
{
  return _print_stream_names_on_file_open;
}

const std::vector<c_ifhd_reader::IfhdStream> & c_ifhd_reader::streams() const
{
  return _file_streams;
}

const c_ifhd_reader::IfhdStream & c_ifhd_reader::stream(int index) const
{
  return _file_streams[index];
}

bool c_ifhd_reader::is_open() const
{
  return _fd.is_open();
}

void c_ifhd_reader::close()
{
  if ( _fd.is_open() ) {
    _fd.close();
    _current_stream_index = -1;
    _current_frame_index_in_current_stream = -1;
  }
}

bool c_ifhd_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    _filename = filename;
  }

  if( _filename.empty() ) {
    CF_ERROR("c_ifhd_reader::open() : no filename specified");
    return false;
  }

  const char *fname = _filename.c_str();
  bool fOk = false;
  uint16_t format_version = 0;

#if _WIN32 || _WIN64
  constexpr int openflags =
      O_RDONLY | O_BINARY;

#else
  constexpr int openflags =
      O_RDONLY | O_NOATIME;
#endif

  if( !_fd.open(fname, openflags) ) {
    CF_ERROR("fd_.open('%s') fails: %s", fname, strerror(errno));
    goto end;
  }

  if( _fd.read(&_file_header, sizeof(_file_header)) != sizeof(_file_header) ) {
    CF_ERROR("read('%s', file_header_) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if ( !check_file_header(_file_header) ) {

    if (_print_stream_names_on_file_open ) {
      CF_ERROR("check_file_header('%s') fails", fname);
    }

    errno = ENODATA;
    goto end;
  }

  _file_streams.clear();
  _file_streams.reserve(_file_header.extension_count);

  if (_print_stream_names_on_file_open ) {
    CF_DEBUG("'%s': file_header_.extension_count = %u", fname, (uint32_t)_file_header.extension_count);
  }

  for( size_t i = 0; i < _file_header.extension_count; ++i ) {

    IfhdStream stream;

    const ssize_t offset =
        _file_header.extension_offset + i * sizeof(FileExtension);

    if( _fd.readfrom(offset, &stream.extension, sizeof(stream.extension)) != sizeof(stream.extension) ) {
      CF_ERROR("readfrom('%s',offset=%zd, size=%zu) fails: %s", fname, offset, sizeof(stream.extension),
          strerror(errno));
      goto end;
    }

    if( _fd.readfrom(stream.extension.data_pos, &stream.header, sizeof(stream.header)) != sizeof(stream.header) ) {
      CF_ERROR("readfrom('%s',offset=%zd, size=%zu) fails: %s", fname, stream.extension.data_pos, sizeof(stream.header),
          strerror(errno));
      goto end;
    }

    _file_streams.emplace_back(stream);

    if ( _print_stream_names_on_file_open ) {
      CF_DEBUG("stream[%zu]: name=\"%s\" frames=%llu", i, stream.header.stream_name,
          (unsigned long long) stream.header.stream_index_count);
    }
  }

  if( _fd.seek(0, SEEK_SET) != 0 ) {
    CF_ERROR("fd_.seek('%s', offset=0, SEEK_SET) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  fOk = true;

end:
  if ( !fOk ) {
    close();
  }

  return fOk;
}



int c_ifhd_reader::current_stream() const
{
  return _current_stream_index;
}

bool c_ifhd_reader::select_stream(int index)
{
  if (index == _current_stream_index) {
    return true;
  }

  if (index < 0 || index >= streams().size()) {
    CF_ERROR("invalid stream index %d requested", index);
    errno = EINVAL;
    return false;
  }

  _current_stream_index = index;
  _current_frame_index_in_current_stream = -1;

  IfhdStream &stream =
      _file_streams[_current_stream_index];

  if (!stream.chunks.empty()) {
    _current_frame_index_in_current_stream = 0;
    return true;
  }

  ////////////////////////////////////////////////////////

  ChunkHeader chunk_header;
  memset(&chunk_header, 0, sizeof(chunk_header));

  stream.chunks.reserve(stream.header.stream_index_count);

  const ssize_t file_header_data_size = (ssize_t) (_file_header.data_size);

  for (uint64_t ichunk = 0, current_chunk_offset = 0; ichunk < _file_header.chunk_count;
      ++ichunk, current_chunk_offset += 16U * ((chunk_header.size + 15U) / 16U)) {

    const ssize_t current_offset =
        (ssize_t) (_file_header.data_offset + current_chunk_offset);

    if (_fd.readfrom(current_offset, &chunk_header, sizeof(ifhd::ChunkHeader)) != sizeof(ifhd::ChunkHeader)) {
      CF_ERROR("readfrom(fd=%d, offset=%zd, size=%zu) fails: %s", _fd, current_offset, sizeof(ifhd::ChunkHeader),
          strerror(errno));
      break;
    }

    if (chunk_header.stream_id == stream.extension.stream_id) {
      chunk_header.payload_size = chunk_header.size - sizeof(ifhd::ChunkHeader) - 5 - 17;
      chunk_header.payload_offset = _file_header.data_offset + current_chunk_offset + sizeof(ifhd::ChunkHeader) + 17;
      stream.chunks.emplace_back(chunk_header);
    }
  }

  if (!stream.chunks.empty()) {
    _current_frame_index_in_current_stream = 0;
  }

  return true;
}

bool c_ifhd_reader::select_stream(const std::string & stream_name)
{
  for( int i = 0, n = _file_streams.size(); i < n; ++i ) {
    if( _file_streams[i].header.stream_name == stream_name ) {
      return select_stream(i);
    }
  }

  // CF_ERROR("stream not found: '%s'", stream_name.c_str());
  errno = EINVAL;
  return false;
}

/// @brief get number of frames in current stream
ssize_t c_ifhd_reader::num_frames() const
{
  return _current_stream_index >= 0 ?
      _file_streams[_current_stream_index].chunks.size() :
      -1;
}

bool c_ifhd_reader::seek(int32_t frame_index_in_current_stream)
{
  if ( frame_index_in_current_stream == _current_frame_index_in_current_stream ) {
    return true;
  }

  if ( _current_stream_index < 0 ) {
    CF_ERROR("No current stream selected");
    errno = EINVAL;
    return false;
  }

  const IfhdStream & stream =
      _file_streams[_current_stream_index];

  if ( frame_index_in_current_stream < 0 || frame_index_in_current_stream >= (int32_t)stream.chunks.size() ) {
    CF_ERROR("Seek to invalid position %d requested", frame_index_in_current_stream);
    errno = EINVAL;
    return false;
  }

  _current_frame_index_in_current_stream =
      frame_index_in_current_stream;

  return true;
}

int32_t c_ifhd_reader::curpos() const
{
  return _current_frame_index_in_current_stream;
}

ssize_t c_ifhd_reader::current_payload_size() const
{
  if( _current_stream_index < 0 || _current_stream_index >= (ssize_t) _file_streams.size() ) {
    CF_ERROR("ERROR current stream was not selected");
    errno = EINVAL;
    return -1;
  }

  const IfhdStream & stream =
      _file_streams[_current_stream_index];

  if( _current_frame_index_in_current_stream < 0 || _current_frame_index_in_current_stream >= stream.chunks.size() ) {
    errno = EINVAL;
    return -1;
  }

  return stream.chunks[_current_frame_index_in_current_stream].payload_size;
}

ssize_t c_ifhd_reader::read_payload(void * data, size_t max_size)
{
  if( _current_stream_index < 0 || _current_stream_index >= (ssize_t) _file_streams.size() ) {
    CF_ERROR("ERROR current stream was not selected");
    errno = EINVAL;
    return -1;
  }

  const IfhdStream &stream =
      _file_streams[_current_stream_index];

  if( _current_frame_index_in_current_stream < 0 ) {
    CF_DEBUG("current_frame_index_in_current_stream_=%zd", _current_frame_index_in_current_stream);
    errno = EINVAL;
    return -1;
  }

  if (_current_frame_index_in_current_stream >= stream.chunks.size()) {
    CF_DEBUG("current_frame_index_in_current_stream_=%zd >= stream.chunks.size()-%zu",
        _current_frame_index_in_current_stream, stream.chunks.size());
    errno = EINVAL;
    return 0;
  }


  const ChunkHeader &chunk =
      stream.chunks[_current_frame_index_in_current_stream];

  if( max_size > (size_t) chunk.payload_size ) {
    max_size = chunk.payload_size;
  }

  const ssize_t bytes_read =
      _fd.readfrom(chunk.payload_offset, data,
            max_size);

  if( bytes_read >= 0 ) {
    ++_current_frame_index_in_current_stream;
  }

  return bytes_read;
}


