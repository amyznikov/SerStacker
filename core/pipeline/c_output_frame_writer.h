/*
 * c_output_frame_writer.h
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_output_frame_writer_h__
#define __c_output_frame_writer_h__

#include <core/io/c_ser_file.h>
#include <core/io/c_ffmpeg_file.h>
#include <core/io/save_image.h>
#include <core/proc/pixtype.h>
#include <core/improc/c_image_processor.h>
#include <core/ctrlbind/ctrlbind.h>

struct c_output_frame_writer_options
{
  std::string output_filename;
  std::string ffmpeg_opts = "-r 10 -c huffyuv -f avi" ; // "-r 10 -c rawvideo -pix_fmt rgb24";
  c_image_processor::sptr output_image_processor;
  PIXEL_DEPTH output_pixel_depth = PIXEL_DEPTH_NO_CHANGE;
  bool save_frame_mapping = false;

  c_output_frame_writer_options();
};

class c_output_frame_writer
{
public:
  c_output_frame_writer();
  ~c_output_frame_writer();

  static void set_default_ffmpeg_opts(const std::string & opts = "-r 10 -c huffyuv");
  static const std::string & default_ffmpeg_opts();

  const std::string & filename() const;
  const std::string & ffmpeg_opts() const;

  bool open(const std::string & filename,
      const std::string & ffmpeg_opts = "",
      bool write_frame_mapping = false);

  bool open(const std::string & filename,
      const std::string & ffmpeg_opts,
      const c_image_processor::sptr & output_image_processor,
      PIXEL_DEPTH output_depth,
      bool write_frame_mapping = false);

  bool write(cv::InputArray currenFrame,
      cv::InputArray currentMask = cv::noArray(),
      bool with_alpha_mask = false,
      int seqindex = -1);

  bool is_open() const;
  void close();

protected:
  static bool create_output_frame(const cv::Mat & src, const cv::Mat & src_mask,
      cv::Mat & dst, cv::Mat & dst_mask,
      const c_image_processor::sptr & processor,
      PIXEL_DEPTH ddepth);

protected:
  c_ffmpeg_writer ffmpeg;
  c_ser_writer ser;
  cv::Mat tmp;

  enum
  {
    output_type_unknown,
    output_type_video,
    output_type_ser,
    output_type_images,
  } output_type = output_type_unknown;

  std::string output_file_name;
  FILE *frame_mapping_fp = nullptr;
  int current_frame_index = 0;
  int current_input_sequence_index = 0;
  int64_t pts = 0;

  c_image_processor::sptr _output_image_processor;
  PIXEL_DEPTH _output_pixel_depth = PIXEL_DEPTH_NO_CHANGE;

  std::string _ffmpeg_opts;

  static std::string _default_ffmpeg_opts;
};


class c_output_text_writer
{
public:
  c_output_text_writer();
  ~c_output_text_writer();

  const std::string & filename() const;

  bool open(const std::string & filename);

  bool vprintf(const char * format, va_list arglist);

#if _MSC_VER
  bool printf(const char * format, ...);
#else
  bool printf(const char * format, ...) __attribute__ ((__format__ (printf, 2, 3)));
#endif

  bool is_open() const;
  void close();
  void flush();

protected:
  std::string _filename;
  FILE * _fp = nullptr;
};

bool load_settings(c_config_setting settings,
    c_output_frame_writer_options * opts);

bool save_settings(c_config_setting settings,
    const c_output_frame_writer_options & opts);



template<class RootObjectType>
inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_output_frame_writer_options> & ctx)
{
  using S = c_output_frame_writer_options;
  ctlbind_browse_for_file(ctls, "output_filename", ctx(&S::output_filename), "");
  ctlbind(ctls, "ffmpeg_opts", ctx(&S::ffmpeg_opts), "");
  ctlbind(ctls, "output_image_processor", ctx(&S::output_image_processor), "");
  ctlbind(ctls, "output_pixel_depth", ctx(&S::output_pixel_depth), "");
  ctlbind(ctls, "save_frame_mapping", ctx(&S::save_frame_mapping), "");
}

#endif /* __c_output_frame_writer_h__ */
