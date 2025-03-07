/*
 * c_sply_file.h
 *
 *  Created on: Jul 22, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sply_file_h__
#define __c_sply_file_h__

#include <opencv2/opencv.hpp>
#include "c_dtcs_file.h"


class c_sply_file
{
public:
  typedef c_sply_file this_class;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  // The data size is computed as rows * cols * CV_ELEM_SIZE(type);
  struct c_sply_mat_header
  {
    int32_t rows = 0;
    int32_t cols = 0;
    int32_t type = -1;
    int32_t data_start = 0;
  };

  struct c_sply_cloud_header
  {
    double cloud_timestamp = 0;
    c_sply_mat_header points;
    c_sply_mat_header colors;
    c_sply_mat_header timestamps;
  };

  struct c_sply_frame_header2
  {
    int32_t num_clouds = 0;
  };



  struct c_sply_frame_header
  {
    double frame_timestamp = 0;

    int32_t points_rows = 0;
    int32_t points_cols = 0;
    int32_t points_type = -1;

    int32_t colors_rows = 0;
    int32_t colors_cols = 0;
    int32_t colors_type = -1;

    int32_t timestamps_rows = 0;
    int32_t timestamps_cols = 0;
    int32_t timestamps_type = -1;
  };

  static inline constexpr uint64_t mktag(char a0, char a1, char a2, char a3, char a4, char a5, char a6, char a7)
  {
    return c_dtcs_file::mktag(a0, a1, a2, a3, a4, a5, a6, a7);
  }

  static inline constexpr uint64_t default_sply_file_tag()
  {
    return mktag('c', '_', 's', 'p', 'l', 'y', '_', 'f');
  }

protected:
};


class c_sply_writer :
    public c_sply_file
{
public:
  typedef c_sply_writer this_class;
  typedef c_sply_file base;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  c_sply_writer(const std::string & filename = "");
  ~c_sply_writer();

  bool create(const std::string & filename = "");
  void close();
  bool is_open() const;

  int add_stream(const std::string & stream_name);

  bool write(uint32_t stream_index, cv::InputArrayOfArrays points,
      cv::InputArrayOfArrays colors = cv::noArray(),
      cv::InputArrayOfArrays timestamps = cv::noArray());

  const std::string & filename() const
  {
    return _datafile.filename();
  }

  const char * cfilename() const
  {
    return _datafile.cfilename();
  }

  void set_file_tag(uint64_t v)
  {
    _datafile.set_file_tag(v);
  }

  uint64_t file_tag() const
  {
    return _datafile.file_tag();
  }

protected:
  c_dtcs_writer _datafile;
};


class c_sply_reader :
    public c_sply_file
{
public:
  typedef c_sply_reader this_class;
  typedef c_sply_file base;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::shared_ptr<this_class> sptr;

  c_sply_reader(const std::string & filename = "");
  ~c_sply_reader();

  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;

  int find_stream(const std::string & stream_name);

  bool select_stream(uint32_t index);

  uint32_t num_frames(uint32_t stream_index) const;
  uint32_t num_frames() const;

  uint32_t curpos(uint32_t stream_index) const;
  uint32_t curpos() const;

  bool seek(uint32_t stream_index, uint32_t pos);
  bool seek(uint32_t pos);


  bool read(uint32_t stream_index, cv::OutputArrayOfArrays points,
      cv::OutputArrayOfArrays colors = cv::noArray(),
      cv::OutputArrayOfArrays timestamps = cv::noArray());

  bool read(cv::OutputArrayOfArrays points,
      cv::OutputArrayOfArrays colors = cv::noArray(),
      cv::OutputArrayOfArrays timestamps = cv::noArray());

  const std::string & filename() const
  {
    return _datafile.filename();
  }

  const char * cfilename() const
  {
    return _datafile.cfilename();
  }

  void set_file_tag(uint64_t v)
  {
    _datafile.set_file_tag(v);
  }

  uint64_t file_tag() const
  {
    return _datafile.file_tag();
  }

protected:
  c_dtcs_reader _datafile;
};


#endif /* __c_sply_file_h__ */
