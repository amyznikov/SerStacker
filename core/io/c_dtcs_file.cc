/*
 * c_dtcs_file.cc
 *
 *  Created on: Jul 22, 2024
 *      Author: amyznikov
 */

#include "c_dtcs_file.h"
#include <core/debug.h>

////////////////////////////////////////////////////////////////////////////////////////////

c_dtcs_writer::c_dtcs_writer(const std::string & filename ) :
    base(filename)
{

}

c_dtcs_writer::~c_dtcs_writer()
{
  close();
}

bool c_dtcs_writer::create(const std::string & filename /*= ""*/)
{
  close();

  if ( !filename .empty() ) {
    _filename  = filename;
  }

  if( !_fd.open(_filename, O_CREAT | O_TRUNC | O_WRONLY) ) {
    CF_ERROR("_fd.open(_filename='%s') fails : %s", _filename.c_str(), strerror(errno));
    return false;
  }

  c_file_header file_header;

  file_header.file_tag = _file_tag;

  if ( !_fd.write(file_header) ) {
    CF_ERROR("_fd.write(file_header) fails: %s", strerror(errno));
    return false;
  }

  return true;
}

void c_dtcs_writer::close()
{
  if ( _fd.is_open() ) {
    // write trailer and update header

    const ssize_t endpos =
        _fd.whence();

    c_file_header file_header;
    c_index_header index_header;
    c_stream_header stream_header;
    c_chunk_header chunk_header;
    bool fOk = true;


    file_header.file_tag = _file_tag;
    file_header.index_offset = endpos;

    if ( _fd.seek(0) != 0 ) {
      CF_ERROR("_fd.seek(0) fails: %s", strerror(errno));
      fOk = false;
      goto end;
    }

    if( !(fOk = _fd.write(file_header)) ) {
      CF_ERROR("_fd.write(file_header) fails: %s", strerror(errno));
      goto end;
    }

    if( _fd.seek(endpos) != endpos ) {
      CF_ERROR("_fd.seek(endpos=%zd) fails: %s", endpos, strerror(errno));
      fOk = false;
      goto end;
    }




    index_header.streams_count =
        _streams.size();

    if( !(fOk = _fd.write(index_header)) ) {
      CF_ERROR("_fd.write(index_header) fails: %s", strerror(errno));
      goto end;
    }




    for ( uint32_t i = 0, n = _streams.size(); i < n; ++i ) {

      const c_stream & stream =
          _streams[i];

      const std::vector<c_chunk_index_item> & chunks =
          stream.chunks;

      stream_header.chunks_count =
          chunks.size();

      strncpy(stream_header.stream_name, stream.name.c_str(),
          sizeof(stream_header.stream_name)-1);

      if( !(fOk = _fd.write(stream_header)) ) {
        CF_ERROR("_fd.write(stream_header) fails: %s", strerror(errno));
        break;
      }

      for ( uint32_t j = 0, m = chunks.size(); j < m; ++j ) {

        if( !(fOk = _fd.write(chunks[j])) ) {
          CF_ERROR("_fd.write(chunk_header) fails: %s", strerror(errno));
          break;
        }
      }

      if ( !fOk ) {
        break;
      }
    }

end:
    _fd.close();
  }
}

int c_dtcs_writer::add_stream(const std::string & stream_name)
{
  if( !_streams.empty() ) {

    const auto pos =
        std::find_if(_streams.begin(), _streams.end(),
            [stream_name](const c_stream & s) {
              return stream_name == s.name;
            });

    if( pos != _streams.end() ) {
      CF_ERROR("steam '%s' already exist", stream_name.c_str());
      errno = EINVAL;
      return false;
    }
  }


  _streams.emplace_back();
  _streams.back().name = stream_name;

  return _streams.size() - 1;
}

bool c_dtcs_writer::write(uint32_t stream_index, const void * payload, uint32_t size)
{
  if( stream_index >= _streams.size() ) {
    CF_ERROR("Invalid stream index  %zu specified", stream_index);\
    errno = EINVAL;
    return false;
  }

  c_stream & stream =
      _streams[stream_index];

  const ssize_t curpos =
      _fd.whence();

  c_chunk_index_item chunk_item;
  chunk_item.chunk_header.stream_index = stream_index;
  chunk_item.chunk_header.payload_size = size;
  chunk_item.payload_offset = curpos + sizeof(chunk_item.chunk_header);

  if( !_fd.write(chunk_item.chunk_header) ) {
    CF_ERROR("_fd.write(chunk_header) fails: %s", strerror(errno));
    _fd.seek(curpos);
    return false;
  }

  if( _fd.write(payload, size) != size ) {
    CF_ERROR("_fd.write(payload, size=%zu) fails: %s", size, strerror(errno));
    _fd.seek(curpos);
    return false;
  }

  stream.chunks.emplace_back(chunk_item);

  return true;
}

bool c_dtcs_writer::writex(uint32_t stream_index, const std::function<bool(c_file_handle & fd)> & wfn)
{
  if( stream_index >= _streams.size() ) {
    CF_ERROR("Invalid stream index  %zu specified", stream_index);\
    errno = EINVAL;
    return false;
  }

  c_stream & stream =
      _streams[stream_index];

  const ssize_t curpos =
      _fd.whence();

  c_chunk_index_item chunk_item;
  chunk_item.chunk_header.stream_index = stream_index;
  chunk_item.chunk_header.payload_size = 0;
  chunk_item.payload_offset = curpos + sizeof(chunk_item.chunk_header);

  if( !_fd.write(chunk_item.chunk_header) ) {
    CF_ERROR("_fd.write(chunk_header) fails: %s", strerror(errno));
    _fd.seek(curpos);
    return false;
  }

  if ( !wfn(_fd) ) {
    _fd.seek(curpos);
    return false;
  }

  const ssize_t newpos =
      _fd.whence();

  if ( newpos < chunk_item.payload_offset ) {
    _fd.seek(curpos);
    return false;
  }


  chunk_item.chunk_header.payload_size =
      newpos - chunk_item.payload_offset;

  _fd.seek(curpos);
  if( !_fd.write(chunk_item.chunk_header) ) {
    CF_ERROR("_fd.write(chunk_header) fails: %s", strerror(errno));
    _fd.seek(curpos);
    return false;
  }

  _fd.seek(newpos);

  stream.chunks.emplace_back(chunk_item);

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////

bool c_dtcs_reader::open(const std::string & filename)
{
  c_index_header index_header;
  c_stream_header stream_header;
  c_chunk_index_item chunk_item;

  bool fOk = false;

  close();

  if ( !filename.empty() ) {
    _filename = filename;
  }

  if( !_fd.open(_filename, O_RDONLY) ) {
    CF_ERROR("_fd.open(_filename='%s') fails: %s", _filename.c_str(),
        strerror(errno));
    goto end;
  }

  if( !_fd.read(_file_header) ) {
    CF_ERROR("_fd.read(_file_header, _filename='%s') fails: %s", _filename.c_str(),
        strerror(errno));
    goto end;
  }

  if( _file_header.file_tag != _file_tag ) {
    CF_ERROR("_file_tag not match");
    errno = ENODATA;
    goto end;
  }

 //  CF_DEBUG("_file_header.index_offset=%zu", (size_t)_file_header.index_offset);

  if( _file_header.index_offset < sizeof(_file_header) ) {
    // no stream index available
    fOk = true;
    goto end;
  }

  if( !_fd.seek(_file_header.index_offset) ) {
    CF_ERROR("_fd.seek('%s', index_offset=%zu) fails: %s", _filename.c_str(),
        (size_t )_file_header.index_offset,
        strerror(errno));
    goto end;
  }

  if( !_fd.read(index_header) ) {
    CF_ERROR("_fd.read(index_header) fails: %s", strerror(errno));
    goto end;
  }

  _streams.reserve(index_header.streams_count);

  fOk = true;

  for ( uint32_t i = 0, n = index_header.streams_count; i < n; ++i ) {

    memset(&stream_header, 0, sizeof(stream_header));

    if( !(fOk = _fd.read(stream_header)) ) {
      CF_ERROR("_fd.read(stream_header) fails: %s", strerror(errno));
      break;
    }

    // CF_DEBUG("stream_header.stream_name='%s'", stream_header.stream_name);

    _streams.emplace_back();

    c_stream & stream =
        _streams.back();

    stream.name =
        stream_header.stream_name;

    std::vector<c_chunk_index_item> & chunks =
        stream.chunks;

    chunks.reserve(stream_header.chunks_count);

    for( uint32_t j = 0; j < stream_header.chunks_count && fOk; ++j ) {

      if( !(fOk = _fd.read(chunk_item)) ) {
        CF_ERROR("_fd.read(chunk_item) fails: %s", strerror(errno));
        break;
      }

      chunks.emplace_back(chunk_item);
    }
  }

  if( fOk && !(fOk = _fd.seek(sizeof(_file_header))) ) {
    CF_ERROR("_fd.seek('%s', index_offset=%zu) fails: %s", _filename.c_str(),
        (size_t )sizeof(_file_header),
        strerror(errno));
    goto end;
  }

end:
  if ( !fOk ) {
    close();
  }

  return fOk;
}

void c_dtcs_reader::close()
{
  if ( _fd.is_open() ) {
    _fd.close();
  }
  _streams.clear();
}

ssize_t c_dtcs_reader::read(uint32_t stream_index, void * buff, size_t maxsize)
{
  // check if have index
  if( _streams.empty() ) {
    CF_ERROR("Read with no index is does not work now");
    return -1;
  }

  ssize_t cb = -1;

  if( stream_index >= _streams.size() ) {
    CF_ERROR("Invalid stream index %u specified. _streams.size()=%zu", stream_index, _streams.size());
    errno = EINVAL;
  }
  else {

    c_stream & stream =
        _streams[stream_index];

    if( stream.curpos >= stream.chunks.size() ) {
      errno = ENODATA;
      cb = 0;
    }
    else {

      const c_chunk_index_item & chunk =
          stream.chunks[stream.curpos];

      if( maxsize > chunk.chunk_header.payload_size ) {
        maxsize = chunk.chunk_header.payload_size;
      }

      if( _fd.seek(chunk.payload_offset) && (cb = _fd.read(buff, maxsize)) == maxsize ) {
        ++stream.curpos;
      }
    }
  }

  return cb;
}

bool c_dtcs_reader::readx(uint32_t stream_index, const std::function<bool(c_file_handle & fd)> & rfn)
{
  // check if have index
  if( _streams.empty() ) {
    CF_ERROR("Read with no index is does not work now");
    return false;
  }

  bool fOk = false;

  if( stream_index >= _streams.size() ) {
    CF_ERROR("Invalid stream index %u specified. _streams.size()=%zu", stream_index, _streams.size());
    errno = EINVAL;
  }
  else {

    c_stream & stream =
        _streams[stream_index];

    if( stream.curpos >= stream.chunks.size() ) {
      errno = ENODATA;
    }
    else {

      const c_chunk_index_item & chunk =
          stream.chunks[stream.curpos];

      if( _fd.seek(chunk.payload_offset) == chunk.payload_offset ) {

        if( (fOk = rfn(_fd)) ) {
          ++stream.curpos;
        }
      }
    }
  }

  return fOk;
}
