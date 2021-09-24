/*
 * c_input_sequence.h
 *
 *  Created on: Sep 13, 2019
 *      Author: amyznikov
 */

#ifndef __c_input_sequence_h__
#define __c_input_sequence_h__

#include "c_input_source.h"
#include <core/settings.h>


class c_input_sequence
{
public:
  typedef c_input_sequence this_class;
  typedef std::shared_ptr<this_class> ptr;

public:

  static ptr create();
  static ptr create(const std::string & sourcefilename);
  static ptr create(const std::vector<std::string> & sourcefilename);
  virtual ~c_input_sequence();

  void set_auto_debayer(enum DEBAYER_ALGORITHM algo);
  enum DEBAYER_ALGORITHM auto_debayer() const;

  void set_auto_apply_color_matrix(bool v);
  bool auto_apply_color_matrix() const;

  c_input_source::ptr add_source(const std::string & pathfilename, int insert_pos = -1);
  bool add_sources(const std::vector<std::string> & pathfilenames);

  void remove_source(int index);
  void remove_source(const c_input_source::ptr & source);
  void remove_source(const std::string & sourcefilename);

  const std::vector<c_input_source::ptr> & sources() const;
  const c_input_source::ptr & source(int index) const;
  c_input_source::ptr source(const std::string & pathfilename) const;

  int indexof(const std::string & pathfilename) const;
  int indexof(const c_input_source::ptr & source) const;

  static int indexof(const std::string & pathfilename,
      const std::vector<c_input_source::ptr> & list) ;
  static int indexof(const c_input_source::ptr & source,
      const std::vector<c_input_source::ptr> & list);

  void clear();
  bool empty() const;

  bool open();
  bool is_open() const;
  void close(bool clear = false);
  int size();
  bool seek(int pos);
  bool read(cv::Mat & output_frame,
      cv::Mat * output_mask = nullptr);

  c_input_source::ptr current_source() const;
  int current_pos() const;
  int global_pos(int source_index, int source_frame_index) const;

  int pixel_depth() const;
  enum COLORID colorid() const;
  const cv::Matx33f & color_matrix() const;
  bool has_color_matrix() const;

  bool serialize(c_config_setting settings) const;
  bool deserialize(c_config_setting settings);

protected:
  bool open_source(int source_index);
  void close_source(int source_index);
  bool seek_current_source(int relative_pos);
  bool read_current_source(cv::Mat & output_frame, cv::Mat * output_mask);

protected:

  std::vector<c_input_source::ptr> all_sources_;
  std::vector<c_input_source::ptr> enabled_sources_;

  enum DEBAYER_ALGORITHM auto_debayer_ = DEBAYER_GBNR;
  bool auto_apply_color_matrix_ = true;

  int total_frames_ = -1;  // in enabled_sources_
  int current_source_ = -1; // in enabled_sources_
  int current_global_pos_ = -1; // in enabled_sources_


  enum COLORID last_colorid_ = COLORID_UNKNOWN;
  int last_pixel_depth_ = 0;
  cv::Matx33f last_color_matrix_ = cv::Matx33f::eye() ;
  bool has_last_color_matrix_ = false;
};

class c_auto_close_input_sequence {
  const c_input_sequence::ptr & input_sequence_;
public:
  c_auto_close_input_sequence(const c_input_sequence::ptr & input_sequence)
    : input_sequence_(input_sequence) {
  }
  ~c_auto_close_input_sequence() {
    if ( input_sequence_ ) {
      input_sequence_->close();
    }
  }
};


#endif /* __c_input_sequence_h__ */
