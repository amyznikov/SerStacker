/*
 * c_stdio_file.h
 *
 *  Created on: Jun 9, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_stdio_file_h__
#define __c_stdio_file_h__

#include <cstdio>
#include <cstdarg>
#include <string>
#include <cerrno>

class c_stdio_file
{
public:
  c_stdio_file();
  c_stdio_file(const std::string & filename, const std::string & mode);
  ~c_stdio_file();

  const std::string & filename() const;
  const std::string & mode() const;
  bool is_open() const;
  FILE * fp() const;

  bool open(const std::string & filename = "", const std::string & mode = "");
  void close();

  int vfprintf(const char * format, va_list arglist);

#ifdef _MSC_VER
  int fprintf(const char * format, ...);
#else
  int fprintf(const char * format, ...) __attribute__ ((__format__ (printf, 2, 3)));
#endif

protected:
  FILE * fp_;
  std::string filename_;
  std::string mode_;
};

#endif /* __c_stdio_file_h__ */
