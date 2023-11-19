/*
 * c_file_handle.h
 *
 *  Created on: Nov 19, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_file_handle_h__
#define __c_file_handle_h__

#include <string>
#include <sys/types.h>
#include <fcntl.h>

#if _MSC_VER
# define _CRT_DECLARE_NONSTDC_NAMES  1
# include <windows.h>

typedef SSIZE_T size_t;

#else
# include <unistd.h>
# include <stddef.h>
#endif

class c_file_handle
{
public:

#if _MSC_VER
  typedef HANDLE FILE_DESCRIPTOR;
  static const HANDLE INVALID_FILE_DESCRIPTOR =
      INVALID_HANDLE_VALUE;
#else
  typedef int FILE_DESCRIPTOR;
  static constexpr int INVALID_FILE_DESCRIPTOR = -1;
#endif

  ~c_file_handle();

  bool open(const std::string & filename, int openflags);
  void close();
  bool is_open() const;
  ssize_t read(void * buf, size_t nbytes);
  ssize_t readfrom(ssize_t offset, void * data, size_t size);
  ssize_t size();
  ssize_t seek(ssize_t, int whence);
  ssize_t whence();

protected:
  FILE_DESCRIPTOR fd_ = INVALID_FILE_DESCRIPTOR;
};

#endif /* __c_file_handle_h__ */
