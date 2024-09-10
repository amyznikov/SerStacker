/*
 * c_sply_video_file.h
 *
 *  Created on: Sep 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sply_video_file_h__
#define __c_sply_video_file_h__

#include <opencv2/opencv.hpp>
#include <core/io/c_dtcs_file.h>

struct c_sply_video_frame
{
  double ts, lat, lon, elev;
  cv::Mat image;
  cv::Mat mask;
  cv::Mat cloud3d;
  cv::Mat colors3d;
};

class c_sply_video_file
{
public:
  typedef c_sply_video_file this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

#pragma pack(push, 1)
  struct gps_storage
  {
    double ts, lat, lon, elev;
  };

  struct cv_mat_header_storage
  {
    int32_t rows = 0;
    int32_t cols = 0;
    int32_t type = 0;
    int32_t datasize = 0;
  };
#pragma pack(pop)

  static inline constexpr uint64_t mktag(char a0, char a1, char a2, char a3, char a4, char a5, char a6, char a7)
  {
    return c_dtcs_file::mktag(a0, a1, a2, a3, a4, a5, a6, a7);
  }

  static inline constexpr uint64_t default_file_tag()
  {
    return mktag('c', '_', 's', 'p', 'l', 'y', '_', 'v');
  }
};




class c_sply_video_writer:
    public c_sply_video_file
{
public:
  typedef c_sply_video_writer this_class;
  typedef c_sply_video_file base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  enum CompressionType
  {
    CompressionNone,
    CompressionJpeg,
    CompressionPng,
  };

  c_sply_video_writer(const std::string & filename = "");

  const std::string& filename() const;
  const char* cfilename() const;

  void set_file_tag(uint64_t v);
  uint64_t file_tag() const;

  bool create(const std::string & filename = "");

  void close();

  bool is_open() const;

  bool write(const c_sply_video_frame & frame);

protected:
  static bool write_gps(c_file_handle & fd, const c_sply_video_frame & frame);
  static bool write_mat(c_file_handle & fd, const cv::Mat & image, CompressionType compression = CompressionNone);
  static bool write_image(c_file_handle & fd, const cv::Mat & image);
  static bool write_mask(c_file_handle & fd, const cv::Mat & mask);

protected:
  c_dtcs_writer _dtcs;
};



class c_sply_video_reader:
    public c_sply_video_file
{
public:
  typedef c_sply_video_reader this_class;
  typedef c_sply_video_file base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_sply_video_reader(const std::string & filename = "");

  const std::string& filename() const;
  const char* cfilename() const;

  void set_file_tag(uint64_t v);
  uint64_t file_tag() const;

  bool open(const std::string & filename = "");

  void close();

  bool is_open() const;

  bool read(c_sply_video_frame & frame);

protected:
  static bool read_gps(c_file_handle & fd, c_sply_video_frame & frame);
  static bool read_mat(c_file_handle & fd, cv::Mat & image);

protected:
  c_dtcs_reader _dtcs;
};

#endif /* __c_sply_video_file_h__ */
