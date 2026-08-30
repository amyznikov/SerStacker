/*
 * c_raw_file.h
 *
 *  Created on: Dec 5, 2020
 *      Author: amyznikov
 */

#ifndef __c_raw_file_h__
#define __c_raw_file_h__

#if HAVE_LIBRAW

#include "debayer.h"
#include <libraw/libraw.h>

class c_raw_file_reader
{
public:

  enum COLORID colorid() const {
    return _colorid;
  }

  int bpc () const {
    return _bpc;
  }

  double black_level() const {
    return _black_level;
  }

  const cv::Matx33f & color_matrix() const {
    return _color_matrix;
  }

  bool has_color_matrix() const {
    return _has_color_maxtrix;
  }

  const cv::Vec4f & channel_multipliers() const {
    return _channel_multipliers;
  }

  bool has_channel_multipliers() const {
    return _has_channel_multipliers;
  }

  void set_auto_apply_channel_multipliers(bool v) {
    _auto_apply_channel_multipliers = v;
  }

  bool auto_apply_channel_multipliers() const {
    return _auto_apply_channel_multipliers;
  }

  bool read(const std::string & filename, cv::OutputArray output_image,cv::OutputArray output_mask,
      enum COLORID * output_colorid, int * output_bpc);


  void recycle();

private:
  LibRaw raw;
  enum COLORID _colorid = COLORID_UNKNOWN;
  int _bpc = 0;
  cv::Matx33f _color_matrix;
  cv::Vec4f _channel_multipliers;
  double _black_level = 0;
  bool _has_color_maxtrix = false;
  bool _has_channel_multipliers = false;
  bool _auto_apply_channel_multipliers = true;

private:
  int raw2mat(cv::OutputArray output_image);
};

#endif // HAVE_LIBRAW

#endif /* __c_raw_file_h__ */
