/*
 * c_ifhd_file.cc
 *
 *  Created on: Oct 25, 2023
 *      Author: amyznikov
 */

#include "c_ifhd_file.h"
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits>
#include <cmath>
#include <core/ssprintf.h>
#include <core/debug.h>


//@brief get current file position
static inline ssize_t whence(int fd)
{
  return ::lseek64(fd, 0, SEEK_CUR);
}

static inline ssize_t readfrom(int fd, ssize_t offset, void * data, size_t size)
{
  if( ::lseek64(fd, offset, SEEK_SET) != offset ) {
    return -1;
  }

  return ::read(fd, data, size);
}


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
    filename_(filename)
{
}

const std::string& c_ifhd_file::filename() const
{
  return filename_;
}

c_ifhd_reader::c_ifhd_reader() :
    this_class("")
{
}

c_ifhd_reader::c_ifhd_reader(const std::string & filename) :
    base(filename)
{
  memset(&file_header_, 0, sizeof(file_header_));
}

c_ifhd_reader::~c_ifhd_reader()
{
  close();
}

const std::vector<c_ifhd_reader::IfhdStream> & c_ifhd_reader::streams() const
{
  return file_streams_;
}

bool c_ifhd_reader::is_open() const
{
  return fd_ >= 0;
}

void c_ifhd_reader::close()
{
  if ( fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
    current_stream_index_ = -1;
    current_frame_index_in_current_stream_ = -1;
  }
}

bool c_ifhd_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_ifhd_reader::open() : no filename specified");
    return false;
  }

  const char *fname = filename_.c_str();
  bool fOk = false;
  uint16_t format_version = 0;

#if _WIN32 || _WIN64
  constexpr int openflags = O_RDONLY | O_BINARY;
#else
  constexpr int openflags = O_RDONLY | O_NOATIME;
#endif

  if( (fd_ = ::open(fname, openflags)) < 0 ) {
    CF_ERROR("open('%s') fails: %s", fname, strerror(errno));
    goto end;
  }


  if( ::read(fd_, &file_header_, sizeof(file_header_)) != sizeof(file_header_) ) {
    CF_ERROR("read('%s', file_header_) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if ( !check_file_header(file_header_) ) {
    // CF_ERROR("check_file_header('%s') fails", fname);
    errno = ENODATA;
    goto end;
  }

//  CF_DEBUG("file_header_: extension_count=%u",
//      file_header_.extension_count);


  file_streams_.clear();
  file_streams_.reserve(file_header_.extension_count);

  for( std::size_t i = 0; i < file_header_.extension_count; ++i ) {

    IfhdStream stream;

    const ssize_t offset =
        file_header_.extension_offset + i * sizeof(FileExtension);

    if( readfrom(fd_, offset, &stream.extension, sizeof(stream.extension)) != sizeof(stream.extension) ) {
      CF_ERROR("readfrom('%s',offset=%zd, size=%zu) fails: %s", fname, offset, sizeof(stream.extension),
          strerror(errno));
      goto end;
    }

    if( readfrom(fd_, stream.extension.data_pos, &stream.header, sizeof(stream.header)) != sizeof(stream.header) ) {
      CF_ERROR("readfrom('%s',offset=%zd, size=%zu) fails: %s", fname, stream.extension.data_pos, sizeof(stream.header),
          strerror(errno));
      goto end;
    }

    file_streams_.emplace_back(stream);

    //CF_DEBUG("stream: name='%s'", stream.header.stream_name);

  }

  if( ::lseek64(fd_, 0, SEEK_SET) != 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
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
  return current_stream_index_;
}

bool c_ifhd_reader::select_stream(int index)
{
  if ( index < 0 || index >= streams().size() ) {
    CF_ERROR("invalid stream index %d requested", index);
    errno = EINVAL;
    return false;
  }

  current_stream_index_ = index;
  current_frame_index_in_current_stream_ = -1;

  IfhdStream & stream =
      file_streams_[current_stream_index_];

  if ( !stream.chunks.empty() ) {
    current_frame_index_in_current_stream_ = 0;
    return true;
  }


  ChunkHeader chunk_header;
  ssize_t current_chunk_offset = 0;

  stream.chunks.reserve(stream.header.stream_index_count);

  //  CF_DEBUG("file_header_: data_size=%zu chunk_count=%zu stream.header.stream_index_count=%zu",
  //      (size_t ) file_header_.data_size,
  //      (size_t ) file_header_.chunk_count,
  //      (size_t ) stream.header.stream_index_count);

  for( ; current_chunk_offset < file_header_.data_size;  current_chunk_offset += 16 * ((chunk_header.size + 15) / 16) ) {

    const ssize_t offset =
        file_header_.data_offset + current_chunk_offset;

    if( readfrom(fd_, offset, &chunk_header, sizeof(ifhd::ChunkHeader)) != sizeof(ifhd::ChunkHeader) ) {
      CF_ERROR("readfrom(fd=%d, offset=%zd, size=%zu) fails: %s", fd_, offset, sizeof(ifhd::ChunkHeader),
          strerror(errno));
      break;
    }


    if( chunk_header.stream_id == stream.extension.stream_id ) {
      chunk_header.payload_size = chunk_header.size - sizeof(ifhd::ChunkHeader) - 5 - 17;
      chunk_header.payload_offset = file_header_.data_offset + current_chunk_offset + sizeof(ifhd::ChunkHeader) + 17;
      stream.chunks.emplace_back(chunk_header);

      //      CF_DEBUG("chunk_header.size=%u payload_size=%zd payload_offset=%zd",
      //          chunk_header.size,
      //          chunk_header.payload_size,
      //          chunk_header.payload_offset);
    }

  }

//  CF_DEBUG("%zu chunks was read. stream.header.stream_index_count=%zu",
//      stream.chunks.size(),
//      (uint64_t)stream.header.stream_index_count);
//
//  CF_DEBUG("scan sizes: scan1=%zu scan3=%zu scan5=%zu ",
//      sizeof(c_vlo_scan1),
//      sizeof(c_vlo_scan3),
//      sizeof(c_vlo_scan5));

  if ( !stream.chunks.empty() ) {
    current_frame_index_in_current_stream_ = 0;
  }

  return true;
}

bool c_ifhd_reader::select_stream(const std::string & stream_name)
{
  for( int i = 0, n = file_streams_.size(); i < n; ++i ) {
    if( file_streams_[i].header.stream_name == stream_name ) {
      return select_stream(i);
    }
  }

  CF_ERROR("stream not found: '%s'", stream_name.c_str());
  errno = EINVAL;
  return false;
}

/// @brief get number of frames in current stream
ssize_t c_ifhd_reader::num_frames() const
{
  return current_stream_index_ >= 0 ?
      file_streams_[current_stream_index_].chunks.size() :
      -1;
}

bool c_ifhd_reader::seek(int32_t frame_index_in_current_stream)
{
  if ( current_stream_index_ < 0 ) {
    CF_ERROR("No current stream selected");
    errno = EINVAL;
    return false;
  }

  const IfhdStream & stream =
      file_streams_[current_stream_index_];

  if ( frame_index_in_current_stream < 0 || frame_index_in_current_stream >= (int32_t)stream.chunks.size() ) {
    CF_ERROR("Seek to invalid position %d requested", frame_index_in_current_stream);
    errno = EINVAL;
    return false;
  }

  current_frame_index_in_current_stream_ =
      frame_index_in_current_stream;

  return true;
}

int32_t c_ifhd_reader::curpos() const
{
  return current_frame_index_in_current_stream_;
}

ssize_t c_ifhd_reader::current_payload_size() const
{
  if( current_stream_index_ < 0 || current_stream_index_ >= (ssize_t) file_streams_.size() ) {
    CF_ERROR("ERROR current stream was not selected");
    errno = EINVAL;
    return -1;
  }

  const IfhdStream & stream =
      file_streams_[current_stream_index_];

  if( current_frame_index_in_current_stream_ < 0 || current_frame_index_in_current_stream_ >= stream.chunks.size() ) {
    errno = EINVAL;
    return -1;
  }

  return stream.chunks[current_frame_index_in_current_stream_].payload_size;
}

size_t c_ifhd_reader::read_payload(void * data, size_t max_size)
{
  if( current_stream_index_ < 0 || current_stream_index_ >= (ssize_t) file_streams_.size() ) {
    CF_ERROR("ERROR current stream was not selected");
    errno = EINVAL;
    return -1;
  }

  const IfhdStream &stream =
      file_streams_[current_stream_index_];

  if( current_frame_index_in_current_stream_ < 0 ) {
    errno = EINVAL;
    return -1;
  }

  if( current_frame_index_in_current_stream_ >= stream.chunks.size() ) {
    return 0;
  }


  const ChunkHeader &chunk =
      stream.chunks[current_frame_index_in_current_stream_];

  if( max_size > (size_t) chunk.payload_size ) {
    max_size = chunk.payload_size;
  }

  const ssize_t bytes_read =
      readfrom(fd_, chunk.payload_offset, data,
            max_size);

  if( bytes_read >= 0 ) {
    ++current_frame_index_in_current_stream_;
  }

  return bytes_read;
}


