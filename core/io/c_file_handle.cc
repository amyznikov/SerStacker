/*
 * c_file_handle.cc
 *
 *  Created on: Nov 19, 2023
 *      Author: amyznikov
 */

#include "c_file_handle.h"
#include <core/debug.h>

c_file_handle::~c_file_handle()
{
  close();
}

bool c_file_handle::open(const std::string & filename, int openflags)
{
  close();

  if( filename.empty() ) {
    CF_ERROR("c_file_handle: No file name specified");
    return false;
  }

#if _MSC_VER
  DWORD accessFlags = GENERIC_READ;
  DWORD shareMode = 0;
  DWORD creationDisposition = 0;

  if( openflags & O_CREAT ) {
    creationDisposition = CREATE_ALWAYS;
  }
  else {
    creationDisposition = OPEN_EXISTING;
  }

  if( !(openflags & (O_WRONLY | O_RDWR)) ) {
    accessFlags |= GENERIC_WRITE;
  }

  fd_ =
      CreateFileA(filename,
          accessFlags,
          shareMode,
          (LPSECURITY_ATTRIBUTES) nullptr,
          creationDisposition,
          FILE_ATTRIBUTE_NORMAL,
          (HANDLE) nullptr);

  if( fd_ == INVALID_FILE_DESCRIPTOR ) {
    CF_ERROR("CreateFileA() fails");
    return false;
  }

#else

  if( (fd_ = ::open(filename.c_str(), openflags)) == INVALID_FILE_DESCRIPTOR ) {
    CF_ERROR("open('%s') fails", filename.c_str());
    return false;
  }

#endif

  return true;
}



void c_file_handle::close()
{
#if _MSC_VER
  if ( fd_ != INVALID_FILE_DESCRIPTOR ) {
    CloseHandle(fd_);
    fd_ = INVALID_FILE_DESCRIPTOR;
  }
#else
  if( fd_ != INVALID_FILE_DESCRIPTOR ) {
    ::close(fd_);
    fd_ = INVALID_FILE_DESCRIPTOR;
  }
#endif
}

bool c_file_handle::is_open() const
{
  return fd_ != INVALID_FILE_DESCRIPTOR;
}

ssize_t c_file_handle::read(void * buf, size_t nbytes)
{
  if ( !is_open() ) {
    errno = EBADF;
    return -1;
  }

#if _MSC_VER

  DWORD numberOfBytesRead = 0;

  ReadFile(fd, buf, (DWORD) nbytes, &numberOfBytesRead, nullptr);

  return (ssize_t)(numberOfBytesRead);
#else

  return ::read(fd_, buf, nbytes);

#endif
}

ssize_t c_file_handle::readfrom(ssize_t offset, void * data, size_t size)
{
  if( seek(offset, SEEK_SET) != offset ) {
    return -1;
  }

  return read(data, size);
}

ssize_t c_file_handle::size()
{
  if ( !is_open() ) {
    errno = EBADF;
    return -1;
  }

  const ssize_t saved_pos =
      whence();

  const ssize_t last_pos =
      seek(0, SEEK_END);

  seek(saved_pos, SEEK_SET);

  return last_pos;
}

ssize_t c_file_handle::seek(ssize_t offset, int whence)
{
#if _MSC_VER

  LARGE_INTEGER  DistanceToMove = {0};
  LARGE_INTEGER  NewFilePointer = {0};
  DWORD  dwMoveMethod = 0;

  DistanceToMove.QuadPart = offset;

  switch (whence) {
    case SEEK_SET:
      dwMoveMethod = FILE_BEGIN;
      break;
    case SEEK_CUR:
      dwMoveMethod = FILE_CURRENT;
      break;
    case SEEK_END:
      dwMoveMethod = FILE_END;
      break;
  }

  if( !SetFilePointerEx(fd_, DistanceToMove, &NewFilePointer, dwMoveMethod) ) {
    return -1;
  }

  return NewFilePointer.QuadPart;

#else

  return ::lseek64 (fd_, offset, whence);

#endif
}

ssize_t c_file_handle::whence()
{
  return seek(0, SEEK_CUR);
}