/*
 * c_input_source.h
 *
 *  Created on: Jan 11, 2021
 *      Author: amyznikov
 */

#ifndef __c_input_source_h__
#define __c_input_source_h__

#include "debayer.h"
#include "c_data_frame.h"


struct c_input_options;

class c_input_source
{
public:
  typedef c_input_source this_class;
  typedef std::shared_ptr<this_class> sptr;

  static sptr create(const std::string & filename);
  static sptr open(const std::string & filename);

  const std::string & filename() const
  {
    return filename_;
  }

  const char * cfilename() const
  {
    return filename_.c_str();
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

  bool has_last_ts() const
  {
    return _has_last_ts;
  }

  double last_ts() const
  {
    return _last_ts;
  }

  void set_enabled(bool v)
  {
    enabled_ = v;
  }

  bool enabled() const
  {
    return enabled_;
  }

  void set_input_options(const c_input_options * options)
  {
    _input_options = options;
  }

  const c_input_options * input_options() const
  {
    return _input_options;
  }

  virtual ~c_input_source() = default;

  virtual bool open() = 0;

  virtual void close() = 0;

  virtual bool seek(int pos) = 0;

  virtual int curpos() = 0;

  virtual bool is_open() const = 0;

  virtual bool read(c_data_frame::sptr & output_frame) = 0;

  virtual bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) = 0;

  const std::vector<uint> & badframes() const;
  bool is_badframe(uint index) const;
  void set_badframe(uint index, bool is_bad);
  void set_badframes(const std::vector<uint> & indexes);
  const std::vector<uint> & load_badframes(const std::string & fname = "");
  void save_badframes(const std::string & fname = "") const;


  static enum COLORID suggest_colorid(int cn);
  static int suggest_bpp(int ddepth);


protected:
  c_input_source(const std::string & filename);

  std::string filename_;
  std::vector<uint32_t> badframes_;
  const c_input_options * _input_options = nullptr;

  int size_ = 0;
  int global_pos_ = 0;
  bool enabled_ = true;

  cv::Matx33f color_matrix_ = cv::Matx33f::eye();
  bool has_color_matrix_ = false;

  double _last_ts = 0;
  bool _has_last_ts = false;
};



#endif /* __c_input_source_h__ */
