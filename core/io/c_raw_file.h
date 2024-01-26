/*
 * c_raw_file.h
 *
 *  Created on: Dec 5, 2020
 *      Author: amyznikov
 */

#ifndef __c_raw_file_h__
#define __c_raw_file_h__

#include "debayer.h"

#ifdef HAVE_LIBRAW

#include <libraw/libraw.h>

class c_raw_file_reader
{
public:

  enum COLORID colorid() const {
    return colorid_;
  }

  int bpc () const {
    return bpc_;
  }

  double black_level() const {
    return black_level_;
  }

  const cv::Matx33f & color_matrix() const {
    return color_matrix_;
  }

  bool has_color_matrix() const {
    return has_color_maxtrix_;
  }

  const cv::Vec4f & channel_multipliers() const {
    return channel_multipliers_;
  }

  bool has_channel_multipliers() const {
    return has_channel_multipliers_;
  }

  void set_auto_apply_channel_multipliers(bool v) {
    auto_apply_channel_multipliers_ = v;
  }

  bool auto_apply_channel_multipliers() const {
    return auto_apply_channel_multipliers_;
  }

  bool read(const std::string & filename,
      cv::Mat & output_image,
      enum COLORID * output_colorid = nullptr,
      int * output_bpc = nullptr);


  void recycle();

private:
  LibRaw raw;
  enum COLORID colorid_ = COLORID_UNKNOWN;
  int bpc_ = 0;
  cv::Matx33f color_matrix_;
  cv::Vec4f channel_multipliers_;
  double black_level_ = 0;
  bool has_color_maxtrix_ = false;
  bool has_channel_multipliers_ = false;
  bool auto_apply_channel_multipliers_ = true;

private:
  int raw2mat(cv::Mat & output_image);
};

#endif // HAVE_LIBRAW

#endif /* __c_raw_file_h__ */
