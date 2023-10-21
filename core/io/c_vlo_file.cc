/*
 * c_vlo_file.cc
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#include "c_vlo_file.h"
#include <fcntl.h>
#include <string.h>
#include <core/proc/bswap.h>
#include <core/debug.h>

//@brief get current file position
static inline ssize_t whence(int fd)
{
  return ::lseek(fd, 0, SEEK_CUR);
}

c_vlo_reader::c_vlo_reader()
{
}

c_vlo_reader::c_vlo_reader(const std::string & filename) :
    base(filename)
{
  if ( !filename_.empty() && !open() ) {
    CF_ERROR("open('%s') fails: %s", filename_.c_str(),
        strerror(errno));
  }
}

bool c_vlo_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_vlo_reader::open() : no filename specified");
    return false;
  }

  const char *fname = filename_.c_str();
  bool fOk = false;
  uint16_t format_version = 0;

  if( (fd_ = ::open(fname, O_RDONLY | O_NOATIME)) < 0 ) {
    CF_ERROR("open('%s') fails: %s", fname, strerror(errno));
    goto end;
  }

  if( ::read(fd_, &format_version, sizeof(format_version)) != sizeof(format_version) ) {
    CF_ERROR("read('%s', format_version) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  CF_DEBUG("vlo format version %u in '%s'",
      format_version, fname);

  switch ((version_ = VLO_VERSION_UNKNOWN)) {
    case VLO_VERSION_5:
      break;
    default:
      CF_ERROR("Not supported format version in vlo file '%s' : '%u' ", fname,
          format_version);
      errno = ENOMSG;
      goto end;
  }

  if( ::lseek(fd_, 0, SEEK_SET) != 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if( (file_size_ = ::lseek(fd_, 0, SEEK_END)) < 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_END) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  if( ::lseek(fd_, 0, SEEK_SET) != 0 ) {
    CF_ERROR("lseek('%s', offset=0, SEEK_SET) fails: %s", fname,
        strerror(errno));
    goto end;
  }

  num_frames_ = file_size_ / frame_size();

  if ( num_frames_ * frame_size() != file_size_ ) { // temporary ignore this error
    CF_ERROR("vlo file '%s': file size = %zd bytes not match to expected number of frames %zd in", fname,
        file_size_, num_frames_);
  }

  fOk = true;

end:
  if( !fOk && fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
  }

  return fOk;
}

void c_vlo_reader::close()
{
  if( fd_ >= 0 ) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool c_vlo_reader::is_open() const
{
  return fd_ >= 0;
}

/// @brief get total file size in bytes
ssize_t c_vlo_reader::file_size() const
{
  return file_size_;
}

/// @brief get frame size in bytes
ssize_t c_vlo_reader::frame_size() const
{
  return sizeof(c_vlo_scan);
}

/// @brief get number of framesin this file
ssize_t c_vlo_reader::num_frames() const
{
  return num_frames_;
}

bool c_vlo_reader::seek(int32_t frame_index)
{
  if( fd_ >= 0 ) {
    return ::lseek(fd_, frame_index * frame_size(), SEEK_CUR) >= 0;
  }
  errno = EBADF;
  return false;
}

int32_t c_vlo_reader::curpos() const
{
  if( fd_ >= 0 ) {
    return whence(fd_) / frame_size();
  }
  errno = EBADF;
  return false;
}

bool c_vlo_reader::read(c_vlo_scan * scan)
{
  return ::read(fd_, scan, sizeof(*scan)) == sizeof(*scan);
}

