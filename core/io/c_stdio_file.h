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
    fp_(nullptr)
  {
  }

  c_stdio_file(const std::string & filename, const std::string & mode) :
      fp_(nullptr),
      filename_(filename),
      mode_(mode)
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
    return filename_;
  }

  const std::string & mode() const
  {
    return mode_;
  }

  bool is_open() const
  {
    return fp_ != nullptr;
  }

  FILE * fp() const
  {
    return fp_;
  }

  operator FILE * () const
  {
    return fp_;
  }

  bool open(const std::string & filename = "", const std::string & mode = "")
  {
    close();

    if ( !filename.empty() ) {
      filename_ = filename;
    }

    if ( !mode.empty() ) {
      mode_ = mode;
    }

    if ( filename_.empty() ) {
      errno = EINVAL;
      return false;
    }

    if ( mode_.empty() ) {
      errno = EINVAL;
      return false;
    }

    if( !(fp_ = fopen(filename_.c_str(), mode_.c_str())) ) {
      return false;
    }

    return true;
  }

  void close()
  {
    if( fp_ ) {
      fclose(fp_);
      fp_ = nullptr;
    }
  }

protected:
  FILE * fp_;
  std::string filename_;
  std::string mode_;
};

#endif /* __c_stdio_file_h__ */
