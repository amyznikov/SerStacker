/*
 * c_ffmpeg_file.h
 *
 *  Created on: December 1, 2020
 *      Author: amyznikov
 */

#ifndef __c_ffmpeg_file_h__
#define __c_ffmpeg_file_h__

#include <opencv2/opencv.hpp>
#include <deque>

// Must be set from CMakeLists.txt
// #define HAVE_AVCODEC  1
// #define HAVE_AVFORMAT 1
// #define HAVE_AVUTIL 1
// #define HAVE_SWSCALE  1
// #define HAVE_AVDEVICE 1
#define HAVE_FFMPEG ((HAVE_AVCODEC) && (HAVE_AVFORMAT) && (HAVE_AVUTIL) && (HAVE_SWSCALE))
#if HAVE_FFMPEG


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

  // the output pts measure is returned in seconds
  bool read(cv::Mat & outframe, double * outpts = nullptr);

  ///@brief estimate video duration in seconds
  double duration() const;

  ///@brief estimate fps
  double fps() const;

  ///@brief set output size
  void set_frame_size(const cv::Size &size);

  ///@brief get scaled output size
  cv::Size frame_size() const;

  ///@brief get coded image size
  cv::Size coded_size() const;

  ///@brief estimate number of video frames in video stream
  int num_frames() const;

  ///@brief try seek to given frame
  bool seek_frame(int64_t index);

  int64_t curpos() const;

  void close();

  const AVStream * stream() const;

  static const std::vector<std::string> & supported_input_formats();
  static const std::vector<std::string> & supported_decoders();
  static const std::vector<std::string> & supported_pixel_formats();

  void set_stream_name(const std::string & v);
  const std::string & stream_name() const;

  void set_rcvtmo(int64_t v);
  int64_t rcvtmo() const;

protected:
  std::string _stream_name;
  timeout_interrupt_callback _tcb;
  int64_t _rcvtmo = 20 * 1000000; // 10 sec

  int _video_stream_index = -1;
  AVFormatContext * _ic = nullptr;
  const AVCodec *   _codec = nullptr;
  AVCodecContext *  _cctx = nullptr;
  AVStream *        _istream = nullptr;
  AVPacket *        _avpacket = nullptr;
  AVFrame *         _avframe = nullptr;
  AVFrame *         _rgbpicture = nullptr;
  SwsContext *      _swsctx = nullptr;
  cv::Size          _scaledSize;

  struct cvframe {
    cv::Mat image;
    int64_t pts; // in stream time_base units
  };
  std::deque<cvframe> _received_frames;
  int64_t _last_ts = -1;
  double _timescale = 0;

  struct c_gif_frame {
    std::vector<uint8_t> data;
    double ts = 0;
  };
  std::vector<c_gif_frame> _gifcache;
  int64_t _gifpos = -1;
};

/**
 * c_ffmpeg_writer
 *
 * Based on code example
 *  <https://ffmpeg.org/doxygen/0.6/output-example_8c-source.html>
 */
class c_ffmpeg_writer
{
public:
  typedef c_ffmpeg_writer
      this_class;

  ~c_ffmpeg_writer();

  ///@brief open (create) video file
  bool open(const std::string & filename, const cv::Size & image_size,
      bool iscolor, const std::string & ffmpeg_opts = "");

  ///@brief return true if video file is open
  bool is_open() const;

  ///@brief close current output file
  void close();

  ///@brief write framw with pts stream time base units
  bool write(const cv::Mat & frame, int64_t pts);

  const std::string & filename() const;

  const std::string & opts() const;

  ///@brief access to stream time_base units to allow caller to precompute pts from it's real timestamp
  const AVStream * stream() const;

  ///@brief access to lists of supported formats
  static const std::vector<std::string> & supported_output_formats();
  static const std::vector<std::string> & supported_encoders();
  static const std::vector<std::string> & supported_pixel_formats();
  static const AVPixelFormat * supported_codec_pix_formats(AVCodecID codec_id);

  int frames_written() const
  {
    return _frames_written;
  }

protected:
  int encode_and_send_frame(AVFrame * picture);

protected:
  std::string _output_filename;
  std::string _opts;
  AVFormatContext * _octx = nullptr;
  AVStream * _ostream = nullptr;
  AVCodecContext * _codec_ctx = nullptr;
  AVFrame * _input_frame = nullptr;
  AVFrame * _output_frame = nullptr;
  enum AVPixelFormat _input_pix_fmt = AV_PIX_FMT_NONE;
  cv::Size _frame_size;
  int64_t _frames_written = 0;
  int64_t _start_pts = 0;
  int64_t _ppts = 0;
  bool _header_written = false;

  AVPacket * _encoded_pkt = nullptr;
  SwsContext * _swsctx = nullptr;
};

#endif // HAVE_FFMPEG
#endif /* __c_ffmpeg_file_h__ */
