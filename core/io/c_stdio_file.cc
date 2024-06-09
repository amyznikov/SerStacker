/*
 * c_stdio_file.cc
 *
 *  Created on: Jun 9, 2024
 *      Author: amyznikov
 */

#include "c_stdio_file.h"
#include <cstring>
#include <errno.h>
#include <core/debug.h>

c_stdio_file::c_stdio_file() :
  fp_(nullptr)
{
}

c_stdio_file::c_stdio_file(const std::string & filename, const std::string & mode) :
    fp_(nullptr),
    filename_(filename),
    mode_(mode)
{
  if( !filename_.empty() ) {
    open();
  }
}

c_stdio_file::~c_stdio_file()
{
  close();
}

const std::string & c_stdio_file::filename() const
{
  return filename_;
}

const std::string & c_stdio_file::mode() const
{
  return mode_;
}

bool c_stdio_file::is_open() const
{
  return fp_ != nullptr;
}

FILE * c_stdio_file::fp() const
{
  return fp_;
}


bool c_stdio_file::open(const std::string & filename, const std::string & mode)
{
  close();

  if ( !filename.empty() ) {
    filename_ = filename;
  }

  if ( !mode.empty() ) {
    mode_ = mode;
  }

  if ( filename_.empty() ) {
    CF_ERROR("c_stdio_file: No path file name specified");
    errno = EINVAL;
    return false;
  }

  if ( mode_.empty() ) {
    CF_ERROR("c_stdio_file: No open mode specified");
    errno = EINVAL;
    return false;
  }

  if( !(fp_ = fopen(filename_.c_str(), mode_.c_str())) ) {

    CF_ERROR("c_stdio_file: fopen('%s', '%s') fails: %s",
        filename_.c_str(),
        mode_.c_str(),
        strerror(errno));

    return false;
  }

  return true;
}

void c_stdio_file::close()
{
  if ( fp_ ) {
    fclose(fp_);
    fp_ = nullptr;
  }
}

bool c_stdio_file::eof() const
{
  return fp_ ? ::feof(fp_) : true;
}

int c_stdio_file::vfprintf(const char * format, va_list arglist)
{
  return fp_ ? ::vfprintf(fp_, format, arglist) : errno = EINVAL, -1;
}

int c_stdio_file::fprintf(const char * format, ...)
{
  va_list arglist;
  int status;

  va_start(arglist, format);
  status = vfprintf(format, arglist);
  va_end(arglist);

  return status;
}

char * c_stdio_file::fgets(char buff[], int max_size)
{
  if ( !fp_ ) {
    errno = EINVAL;
    return nullptr;
  }

  return ::fgets(buff, max_size, fp_);
}

