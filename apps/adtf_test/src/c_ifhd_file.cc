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
#include <core/ssprintf.h>
#include <core/debug.h>


static bool check_file_header(const ifhd::FileHeader & file_header)
{
  // check if it is a dat file
  char id[4];
  memcpy(id, &file_header.file_id, 4);

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

bool c_ifhd_reader::is_open() const
{
  return fd_ >= 0;
}

void c_ifhd_reader::close()
{
  if ( fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
    current_stream_id_ = -1;
    current_chunk_offset_ = 1;
    curpos_ = -1;
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
    CF_ERROR("check_file_header('%s') fails: %s", fname);
    errno = ENODATA;
    goto end;
  }

  CF_DEBUG("file_header_: extension_count=%u", file_header_.extension_count);


  file_exstensions_.clear();
  file_exstensions_.reserve(file_header_.extension_count);

  for( std::size_t idx = 0; idx < file_header_.extension_count; ++idx ) {

    file_exstensions_.emplace_back();

    const ssize_t offset =
        file_header_.extension_offset + sizeof(FileExtension) * idx;

    if( ::lseek64(fd_, offset, SEEK_SET) != offset ) {
      CF_ERROR("lseek('%s', offset=%zd, SEEK_SET) fails: %s", fname, offset,
          strerror(errno));
      goto end;
    }

    FileExtension &file_exstension =
        file_exstensions_.back();

    if( ::read(fd_, &file_exstension, sizeof(FileExtension)) != sizeof(FileExtension) ) {
      CF_ERROR("read('%s', file_exstension index %zu) fails: %s", fname, idx,
          strerror(errno));
      goto end;
    }
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


bool c_ifhd_reader::set_stream(const std::string & stream_name)
{
  if( !is_open() ) {
    CF_ERROR("File is not open");
    errno = EBADF;
    return false;
  }

  current_stream_id_ = -1;
  current_chunk_offset_ = -1;

  for( const auto &file_extension : file_exstensions_ ) {
    if( file_extension.data_size >= sizeof(StreamInfoHeader) ) {

      StreamInfoHeader stream_info;

      if( ::lseek64(fd_, file_extension.data_pos, SEEK_SET) != file_extension.data_pos ) {
        CF_ERROR("lseek('%s', offset=%zd, SEEK_SET) fails: %s", filename_.c_str(),
            file_extension.data_pos,
            strerror(errno));
        return false;
      }

      if( ::read(fd_, &stream_info, sizeof(stream_info)) != sizeof(stream_info) ) {
        CF_ERROR("read('%s', stream_info) fails: %s", filename_.c_str(), strerror(errno));
        return false;
      }

      if( stream_name.compare((const char*) (stream_info.stream_name)) == 0 ) {
        current_stream_id_ = file_extension.stream_id;
        current_chunk_offset_ = 0U;
        return true;
      }
    }
  }

  errno = ENODATA;
  return false;
}

bool c_ifhd_reader::seek(int32_t frame_index)
{

}

int32_t c_ifhd_reader::curpos() const
{

}

ssize_t c_ifhd_reader::current_chunk_size()
{

}

bool c_ifhd_reader::read_current_chunk(void * data)
{

}


