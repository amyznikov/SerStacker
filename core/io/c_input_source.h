/*
 * c_input_source.h
 *
 *  Created on: Jan 11, 2021
 *      Author: amyznikov
 */

#ifndef __c_input_source_h__
#define __c_input_source_h__

#include "c_ser_file.h"
#include "c_fits_file.h"
#include "c_ffmpeg_file.h"
#include "c_raw_file.h"

class c_input_source
{
public:
  typedef c_input_source this_class;
  typedef std::shared_ptr<this_class> ptr;

  enum source_type {
    UNKNOWN = 0,
    SER = 1,
    FITS = 2,
    MOVIE = 3,
    REGULAR_IMAGE = 4,
    RAW_IMAGE = 5,
  };

  static ptr create(const std::string & filename);

  static ptr create(source_type type,
      const std::string & filename);

  static enum source_type suggest_source_type(
      const std::string & filename);

  const std::string & filename() const
  {
    return filename_;
  }

  enum source_type type() const
  {
    return type_;
  }

  int size() const
  {
    return size_;
  }

  bool empty() const
  {
    return size_ < 1;
  }


  void set_global_pos(int pos)
  {
    global_pos_ = pos;
  }

  int global_pos() const
  {
    return global_pos_;
  }

  bool has_color_matrix() const
  {
    return has_color_matrix_;
  }

  const cv::Matx33f & color_matrix() const
  {
    return color_matrix_;
  }

  virtual ~c_input_source() = default;

  virtual bool open() = 0;

  virtual void close() = 0;

  virtual bool seek(int pos) = 0;

  virtual bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) = 0;

protected:
  c_input_source(enum source_type type, const std::string & filename);

  std::string filename_;
  enum source_type type_;
  int size_ = 0;
  int global_pos_ = 0;

  bool has_color_matrix_ = false;
  cv::Matx33f color_matrix_ = cv::Matx33f::eye();
};

class c_ser_input_source
  : public c_input_source
{
public:
  typedef c_ser_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> ptr;

  c_ser_input_source(const std::string & filename);

  static ptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_ser_reader ser_;
};

class c_fits_input_source
  : public c_input_source
{
public:
  typedef c_fits_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> ptr;

  c_fits_input_source(const std::string & filename);

  static ptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_fits_reader fits_;
  int curpos_ = -1;
};

class c_movie_input_source
  : public c_input_source
{
public:
  typedef c_movie_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> ptr;

  c_movie_input_source(const std::string & filename);

  static ptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_ffmpeg_reader ffmpeg_;
};


class c_regular_image_input_source
  : public c_input_source
{
public:
  typedef c_regular_image_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> ptr;

  c_regular_image_input_source(const std::string & filename);

  static ptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  int curpos_ = -1;
};

class c_raw_image_input_source
  : public c_input_source
{
public:
  typedef c_raw_image_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> ptr;

  c_raw_image_input_source(const std::string & filename);

  static ptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_raw_file_reader raw_;
  int curpos_ = -1;
};

#endif /* __c_input_source_h__ */
