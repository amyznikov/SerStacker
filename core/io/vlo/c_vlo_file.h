/*
 * c_vlo_file.h
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_file_h__
#define __c_vlo_file_h__

#include <core/io/c_ifhd_file.h>
#include "c_vlo_scan.h"


class c_vlo_file
{
public:
  typedef c_vlo_file this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_file();
  c_vlo_file(const std::string & filename);

  const std::string & filename() const;
  VLO_VERSION version() const;

  static cv::Mat get_thumbnail_image(const c_vlo_scan & scan);

protected:
  std::string filename_;
  VLO_VERSION version_ = VLO_VERSION_UNKNOWN;
};

class c_vlo_reader :
    public c_vlo_file
{
public:
  typedef c_vlo_reader this_class;
  typedef c_vlo_file base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_reader();
  c_vlo_reader(const std::string & filename);
  ~c_vlo_reader();

  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;
  bool seek(int32_t frame_index);
  int32_t curpos();

  /// @brief get frame size in bytes
  ssize_t frame_size() const;

  /// @brief get number of frames in this file
  ssize_t num_frames() const;

  bool read(c_vlo_scan * scan);

protected:
  template<class ScanType> std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION > 0),
    bool> read(ScanType * scan);

protected:
  c_file_handle fd_;
  c_ifhd_reader ifhd_;
  ssize_t num_frames_ = -1;
  ssize_t frame_size_ = -1;
};



#endif /* __c_vlo_file_h__ */
