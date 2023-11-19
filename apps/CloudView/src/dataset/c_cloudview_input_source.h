/*
 * c_cloudview_input_source.h
 *
 *  Created on: Nov 18, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_cloudview_input_source_h__
#define __c_cloudview_input_source_h__

#include <string>
#include <memory>

namespace cloudview {

class c_cloudview_input_source
{
public:
  typedef c_cloudview_input_source this_class;
  typedef std::shared_ptr<this_class> sptr;

  virtual ~c_cloudview_input_source() = default;

  const std::string & filename() const
  {
    return filename_;
  }

  const char * cfilename() const
  {
    return filename_.c_str();
  }

  virtual bool open(const std::string & filename) = 0;
  virtual void close() = 0;
  virtual bool is_open() = 0;
  virtual ssize_t size() = 0;
  virtual bool seek(ssize_t pos) = 0;
  virtual ssize_t curpos() = 0;

protected:
  c_cloudview_input_source()
  {
  }

protected:
  std::string filename_;
};

} /* namespace cloudview */

#endif /* __c_cloudview_input_source_h__ */
