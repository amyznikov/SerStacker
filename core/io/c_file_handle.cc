/*
 * c_file_handle.cc
 *
 *  Created on: Nov 19, 2023
 *      Author: amyznikov
 */

#include "c_file_handle.h"
#include <sys/stat.h>
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


c_file_handle::~c_file_handle()
{
  close();
}

bool c_file_handle::create(const std::string & filename, bool write_only)
{
  return open(filename, O_CREAT | O_TRUNC | (write_only ? O_WRONLY : O_RDWR));
}

bool c_file_handle::open(const std::string & filename, int openflags)
{
  close();

  if( filename.empty() ) {
    errno = EINVAL;
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

  _fd =
      CreateFileA(filename,
          accessFlags,
          shareMode,
          (LPSECURITY_ATTRIBUTES) nullptr,
          creationDisposition,
          FILE_ATTRIBUTE_NORMAL,
          (HANDLE) nullptr);

  if( _fd == INVALID_FILE_DESCRIPTOR ) {
    CF_ERROR("CreateFileA() fails");
    return false;
  }

#else

  const int mode =
      S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH;

#if _WIN32 || _WIN64
  openflags |= O_BINARY;
#endif

  if( (_fd = ::open(filename.c_str(), openflags, mode)) == INVALID_FILE_DESCRIPTOR ) {
    CF_ERROR("open('%s') fails", filename.c_str());
    return false;
  }

#endif

  return true;
}



void c_file_handle::close()
{
#if _MSC_VER
  if ( _fd != INVALID_FILE_DESCRIPTOR ) {
    CloseHandle(_fd);
    _fd = INVALID_FILE_DESCRIPTOR;
  }
#else
  if( _fd != INVALID_FILE_DESCRIPTOR ) {
    ::close(_fd);
    _fd = INVALID_FILE_DESCRIPTOR;
  }
#endif
}

bool c_file_handle::is_open() const
{
  return _fd != INVALID_FILE_DESCRIPTOR;
}

ssize_t c_file_handle::read(void * buf, size_t nbytes)
{
  if ( !is_open() ) {
    errno = EBADF;
    return -1;
  }

#if _MSC_VER

  DWORD numberOfBytesRead = 0;

  ReadFile(_fd, buf, (DWORD) nbytes, &numberOfBytesRead, nullptr);

  return (ssize_t)(numberOfBytesRead);
#else

  return ::read(_fd, buf, nbytes);

#endif
}

ssize_t c_file_handle::readfrom(ssize_t offset, void * data, size_t size)
{
  if( seek(offset, SEEK_SET) != offset ) {
    return -1;
  }

  return read(data, size);
}


ssize_t c_file_handle::write(const void * buf, size_t nbytes)
{
  if ( !is_open() ) {
    errno = EBADF;
    return -1;
  }

#if _MSC_VER

  DWORD numberOfBytesWritten = 0;

  WriteFile(_fd, buf, (DWORD) nbytes, &numberOfBytesWritten, nullptr);

  return (ssize_t)(numberOfBytesWritten);
#else

  return ::write(_fd, buf, nbytes);

#endif
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

  if( !SetFilePointerEx(_fd, DistanceToMove, &NewFilePointer, dwMoveMethod) ) {
    return -1;
  }

  return NewFilePointer.QuadPart;

#else

  return ::lseek64 (_fd, offset, whence);

#endif
}

ssize_t c_file_handle::whence()
{
  return seek(0, SEEK_CUR);
}

bool c_file_handle::flush()
{
  if( _fd != INVALID_FILE_DESCRIPTOR ) {
#if _MSC_VER
  return FlushFileBuffers(_fd);
#else
#if _WIN32 || _WIN64
    _commit(_fd);
#else
    fdatasync(_fd);
#endif
    return true;
#endif
  }
  return false;
}

