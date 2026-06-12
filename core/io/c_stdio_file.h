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
#include <string>
#include <cerrno>

class c_stdio_file
{
public:

  c_stdio_file() :
    _fp(nullptr)
  {
  }

  c_stdio_file(const std::string & filename, const std::string & mode) :
      _fp(nullptr),
      _filename(filename),
      _mode(mode)
  {
    if( !filename.empty() && !mode.empty() ) {
      open();
    }
  }

  ~c_stdio_file()
  {
    close();
  }

  const std::string & filename() const
  {
    return _filename;
  }

  const char * cfilename() const
  {
    return _filename.c_str();
  }

  const std::string & mode() const
  {
    return _mode;
  }

  bool is_open() const
  {
    return _fp != nullptr;
  }

  FILE * fp() const
  {
    return _fp;
  }

  operator FILE * () const
  {
    return _fp;
  }

  bool open(const std::string & filename = "", const std::string & mode = "")
  {
    close();

    if ( !filename.empty() ) {
      _filename = filename;
    }

    if ( !mode.empty() ) {
      _mode = mode;
    }

    if ( _filename.empty() ) {
      errno = EINVAL;
      return false;
    }

    if ( _mode.empty() ) {
      errno = EINVAL;
      return false;
    }

    if( !(_fp = fopen(_filename.c_str(), _mode.c_str())) ) {
      return false;
    }

    return true;
  }

  void close()
  {
    if( _fp ) {
      fclose(_fp);
      _fp = nullptr;
    }
  }

protected:
  FILE * _fp;
  std::string _filename;
  std::string _mode;
};

#endif /* __c_stdio_file_h__ */
