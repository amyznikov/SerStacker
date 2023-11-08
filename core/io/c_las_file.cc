/*
 * c_las_file.cc
 *
 *  Created on: Nov 7, 2023
 *      Author: amyznikov
 */

#include "c_las_file.h"
#include <core/debug.h>

#if HAVE_LASLIB


///////////////////////////////////////////////////////////////////////////////////////////////////

c_las_file::c_las_file()
{
}

const std::string & c_las_file::filename() const
{
  return filename_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_las_reader::c_las_reader()
{
}

bool c_las_reader::open(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_las_reader::open() : no filename specified");
    return false;
  }

  LASreadOpener lasread;
  bool fOK = false;

  lasread.set_file_name(filename_.c_str());

  if( !(las_ = lasread.open()) ) {
    CF_ERROR("lasread.open('%s') fails", filename_.c_str());
    return false;
  }

  CF_DEBUG("LAS: %lld points in '%s'",
      las_->npoints,
      filename_.c_str());

  las_->point;

  fOK = true;

end:
  if ( !fOK ) {
    close();
  }


  return fOK;
}

void c_las_reader::close()
{
  if( las_ ) {
    las_->close();
    delete las_;
    las_ = nullptr;
  }
}

bool c_las_reader::is_open() const
{
  return las_ != nullptr;
}

const LASheader * c_las_reader::header() const
{
  return las_ ? &las_->header : nullptr;
}

const LASpoint * c_las_reader::read_point()
{
  if ( !las_ ) {
    errno = EBADF;
    return nullptr;
  }

  errno = 0;

  if ( las_->read_point() ) {
    return & las_->point;
  }

  return nullptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_las_writer::c_las_writer()
{
}

bool c_las_writer::create(const std::string & filename)
{
  close();

  if( !filename.empty() ) {
    filename_ = filename;
  }

  if( filename_.empty() ) {
    CF_ERROR("c_las_writer::open() : no filename specified");
    return false;
  }


  return false;
}

void c_las_writer::close()
{
  if( las_ ) {
    las_->close();
    delete las_;
    las_ = nullptr;
  }
}

bool c_las_writer::is_open() const
{
  return las_ != nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
#endif // HAVE_LASLIB
