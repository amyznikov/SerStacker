/*
 * c_ffmpeg_file.h
 *
 *  Created on: December 1, 2020
 *      Author: amyznikov
 */

#ifndef __c_ffmpeg_file_h__
#define __c_ffmpeg_file_h__

#include <opencv2/opencv.hpp>

extern "C" {
  #include <libavformat/avformat.h>
  #include <libavcodec/avcodec.h>
  #include <libswscale/swscale.h>
  #include <libavutil/avutil.h>
  #include <libavutil/opt.h>
#if HAVE_AVDEVICE
  #include <libavdevice/avdevice.h>
#endif

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100 ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
# include <libavutil/imgutils.h>
#endif

#ifndef ff_const59
# if LIBAVFORMAT_BUILD > AV_VERSION_INT(58, 79, 100)
#   define ff_const59  const
# else
#   define ff_const59
# endif
#endif
}


class c_ffmpeg_reader
{
public:
  typedef c_ffmpeg_reader
      this_class;

  struct timeout_interrupt_callback {
    AVIOInterruptCB icb;
    int64_t end_time;
  };


  ~c_ffmpeg_reader();

  ///@brief open video file
  bool open(const std::string & input_url,
      const std::string & input_opts = "");

  ///@brief return true if video file is open
  bool is_open() const;

  // the output pts is returned in stream time_base units
  bool read(cv::Mat & outframe, int64_t * outpts = nullptr);

  ///@brief estimate video duration in seconds
  double duration() const;

  ///@brief estimate fps
  double fps() const;

  ///@brief set output size
  void set_frame_size(const cv::Size &size);

  ///@brief get scalled output size
  cv::Size frame_size() const;

  ///@brief get coded image size
  cv::Size coded_size() const;

  ///@brief estimate number of video frames in video stream
  int num_frames() const;

  ///@brief try seek to given frame
  bool seek_frame(int index);

  int64_t curpos() const;


  void close();

  const AVStream * stream() const;


  static const std::vector<std::string> & supported_input_formats();

  void set_stream_name(const std::string & v);
  const std::string & stream_name() const;

  void set_rcvtmo(int64_t v);
  int64_t rcvtmo() const;


protected:
  std::string stream_name_;
  timeout_interrupt_callback tcb;
  int64_t rcvtmo_ = 10 * 1000000; // 10 sec

  int  video_stream_index = -1;
  AVFormatContext * ic = nullptr;
  ff_const59 AVCodec * codec = nullptr;
  AVCodecContext *  cctx = nullptr;
  AVStream *        istream = nullptr;
  AVPacket *        avpacket = nullptr;
  AVFrame *         avframe = nullptr;
  AVFrame *         rgbpicture = nullptr;
  SwsContext *      swsctx = nullptr;
  cv::Size          scaledSize_;

  struct cvframe {
    cv::Mat image;
    int64_t pts; // in stream time_base units
  };
  std::vector<cvframe> received_frames;
  int64_t last_ts = 0;
};

/**
 * c_ffmpeg_writer
 *
 * Based on code exampe
 *  <https://ffmpeg.org/doxygen/0.6/output-example_8c-source.html>
 */
class c_ffmpeg_writer
{
public:
  typedef c_ffmpeg_writer
      this_class;

  ~c_ffmpeg_writer();

  ///@brief open (create) video file
  bool open(const std::string & filename, const cv::Size & frame_size,
      bool iscolor, const std::string & ffmpeg_opts = "");

  ///@brief return true if video file is open
  bool is_open() const;

  ///@brief close current output file
  void close();

  ///@brief write framw with pts in stream time_base units
  bool write(const cv::Mat frame, int64_t pts);

  ///@brief access to stream time_base units to allow caller to precompute pts from it's real timestamp
  const AVStream * stream() const;

  ///@brief access to lists of supported formats
  static const std::vector<std::string> & supported_output_formats();
  static const AVPixelFormat * supported_codec_pix_formats(AVCodecID codec_id);

protected:
  /// write a frame with given pts in stream time_base units
  bool write_frame(const uint8_t * data, int step, int width, int height, int cn, int origin, int64_t pts);
  int encode_and_send_frame(AVFrame * picture);

protected:
  std::string output_filename_;
  AVFormatContext * octx = nullptr;
  AVStream * ostream = nullptr;
  AVCodecContext * codec_ctx = nullptr;
  AVFrame * input_frame = nullptr;
  AVFrame * output_frame = nullptr;
  enum AVPixelFormat input_pix_fmt = AV_PIX_FMT_NONE;
  uint8_t * aligned_input =  nullptr;
  size_t  aligned_input_size = 0;
  cv::Size frame_size;
  int frames_written_ = 0;
  bool header_written_ = false;
  int64_t start_pts_ = 0;

  AVPacket * encoded_pkt = nullptr;
  SwsContext * sws_ctx = nullptr;

};

#endif /* __c_ffmpeg_file_h__ */
