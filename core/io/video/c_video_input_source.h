/*
 * c_video_input_source.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_video_input_source_h__
#define __c_video_input_source_h__

#include <core/io/c_input_source.h>
#include <core/io/c_ser_file.h>
#include <core/io/c_fits_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/c_raw_file.h>



class c_video_input_source :
    public c_input_source
{
public:
  typedef c_video_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  bool read(c_data_frame::sptr & output_frame) override;

protected:
  c_video_input_source(/*enum source_type type, */const std::string & filename);
};


class c_ser_input_source :
    public c_video_input_source
{
public:
  typedef c_ser_input_source this_class;
  typedef c_video_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_ser_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  c_ser_reader ser_;
};



#if HAVE_CFITSIO
class c_fits_input_source :
    public c_video_input_source
{
public:
  typedef c_fits_input_source this_class;
  typedef c_video_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_fits_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  c_fits_reader fits_;
  int curpos_ = -1;
};
#endif // HAVE_CFITSIO


class c_movie_input_source :
    public c_video_input_source
{
public:
  typedef c_movie_input_source this_class;
  typedef c_video_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_movie_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  c_ffmpeg_reader ffmpeg_;
};



class c_regular_image_input_source :
    public c_video_input_source
{
public:
  typedef c_regular_image_input_source this_class;
  typedef c_video_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_regular_image_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  int curpos_ = -1;
};

#ifdef HAVE_LIBRAW
class c_raw_image_input_source :
    public c_video_input_source
{
public:
  typedef c_raw_image_input_source this_class;
  typedef c_video_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_raw_image_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  c_raw_file_reader raw_;
  int curpos_ = -1;
};
#endif // HAVE_LIBRAW

#endif /* __c_video_input_source_h__ */
