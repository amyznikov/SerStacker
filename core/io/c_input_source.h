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
    return _filename;
  }

  const char * cfilename() const
  {
    return _filename.c_str();
  }

  int size() const
  {
    return _size;
  }

  bool empty() const
  {
    return _size < 1;
  }

  void set_global_pos(int pos)
  {
    _global_pos = pos;
  }

  int global_pos() const
  {
    return _global_pos;
  }

  bool has_color_matrix() const
  {
    return _has_color_matrix;
  }

  const cv::Matx33f & color_matrix() const
  {
    return _color_matrix;
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
    _enabled = v;
  }

  bool enabled() const
  {
    return _enabled;
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

  const std::vector<int> & badframes() const;
  bool is_badframe(int index) const;
  void set_badframe(int index, bool is_bad);
  void set_badframes(const std::vector<int> & indexes);
  const std::vector<int> & load_badframes(const std::string & fname = "");
  void save_badframes(const std::string & fname = "") const;


  static enum COLORID suggest_colorid(int cn);
  static int suggest_bpp(int ddepth);


protected:
  c_input_source(const std::string & filename);

  std::string _filename;
  std::vector<int> _badframes;
  const c_input_options * _input_options = nullptr;

  int _size = 0;
  int _global_pos = 0;
  bool _enabled = true;

  cv::Matx33f _color_matrix = cv::Matx33f::eye();
  bool _has_color_matrix = false;

  double _last_ts = 0;
  bool _has_last_ts = false;
};



#endif /* __c_input_source_h__ */
