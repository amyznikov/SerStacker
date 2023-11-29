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
#include "c_vlo_file.h"

class c_input_source
{
public:
  typedef c_input_source this_class;
  typedef std::shared_ptr<this_class> sptr;

  enum source_type
  {
    UNKNOWN = 0,
    SER = 1,
#if HAVE_CFITSIO
    FITS = 2,
#endif
    MOVIE = 3,
    REGULAR_IMAGE = 4,
#if HAVE_LIBRAW
    RAW_IMAGE = 5,
#endif
    CAMERA = 6,
#if HAVE_VLO_FILE
    VLO = 7,
#endif
  };

  enum OUTPUT_TYPE {
    OUTPUT_TYPE_IMAGE,
    OUTPUT_TYPE_CLOUD3D
  };

  static sptr create(const std::string & filename);

  static sptr create(source_type type,
      const std::string & filename);

  static enum source_type suggest_source_type(
      const std::string & filename);

  const std::string & filename() const
  {
    return filename_;
  }

  const char * cfilename() const
  {
    return filename_.c_str();
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

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  virtual ~c_input_source() = default;

  virtual bool open() = 0;

  virtual void close() = 0;

  virtual bool seek(int pos) = 0;

  virtual int curpos() = 0;

  virtual bool is_open() const = 0;

  virtual bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) = 0;

  const std::vector<uint> & badframes() const;
  bool is_badframe(uint index) const;
  void set_badframe(uint index, bool is_bad);
  void set_badframes(const std::vector<uint> & indexes);
  const std::vector<uint> & load_badframes(const std::string & fname = "");
  void save_badframes(const std::string & fname = "") const;



protected:
  c_input_source(enum source_type type, const std::string & filename);

  std::string filename_;
  enum source_type type_;
  int size_ = 0;
  int global_pos_ = 0;
  bool enabled_ = true;

  bool has_color_matrix_ = false;
  cv::Matx33f color_matrix_ = cv::Matx33f::eye();

  std::vector<uint> badframes_;
};

class c_ser_input_source :
    public c_input_source
{
public:
  typedef c_ser_input_source this_class;
  typedef c_input_source base;
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
    public c_input_source
{
public:
  typedef c_fits_input_source this_class;
  typedef c_input_source base;
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
    public c_input_source
{
public:
  typedef c_movie_input_source this_class;
  typedef c_input_source base;
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
    public c_input_source
{
public:
  typedef c_regular_image_input_source this_class;
  typedef c_input_source base;
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

#if HAVE_LIBRAW
class c_raw_image_input_source :
    public c_input_source
{
public:
  typedef c_raw_image_input_source this_class;
  typedef c_input_source base;
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

#if HAVE_VLO_FILE

class c_vlo_input_source :
    public c_input_source
{
public:
  typedef c_vlo_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool read_cloud3d(cv::OutputArray points,
      cv::OutputArray colors);

  bool is_open() const override;

  void set_read_channel(c_vlo_file::DATA_CHANNEL v);
  c_vlo_file::DATA_CHANNEL read_channel() const;

  VLO_VERSION version() const;

  bool read(c_vlo_scan * scan);

protected:
  c_vlo_reader vlo_;

  c_vlo_file::DATA_CHANNEL read_channel_ =
      c_vlo_file::DATA_CHANNEL_AMBIENT;
};

#endif // HAVE_VLO_FILE


#endif /* __c_input_source_h__ */
