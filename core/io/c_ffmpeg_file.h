/*
 * c_ffmpeg_file.h
 *
 *  Created on: December 1, 2020
 *      Author: amyznikov
 */

#ifndef __c_ffmpeg_file_h__
#define __c_ffmpeg_file_h__

#include <opencv2/opencv.hpp>

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

  // the output pts measure is returned in seconds
  bool read(cv::Mat & outframe, double * outpts = nullptr);

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
  static const std::vector<std::string> & supported_decoders();
  static const std::vector<std::string> & supported_pixel_formats();

  void set_stream_name(const std::string & v);
  const std::string & stream_name() const;

  void set_rcvtmo(int64_t v);
  int64_t rcvtmo() const;


protected:
  std::string stream_name_;
  timeout_interrupt_callback tcb;
  int64_t rcvtmo_ = 20 * 1000000; // 10 sec

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
  int64_t last_ts = -1;
  double timescale_ = 0;
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
    return frames_written_;
  }

protected:
  /// write a frame with given pts in stream time_base units
  bool write_frame(const uint8_t * data, int step, int width, int height, int cn, int origin, int64_t pts);
  int encode_and_send_frame(AVFrame * picture);

protected:
  std::string output_filename_;
  std::string opts_;
  AVFormatContext * octx = nullptr;
  AVStream * ostream = nullptr;
  AVCodecContext * codec_ctx = nullptr;
  AVFrame * input_frame = nullptr;
  AVFrame * output_frame = nullptr;
  enum AVPixelFormat input_pix_fmt = AV_PIX_FMT_NONE;
  uint8_t * aligned_input =  nullptr;
  size_t  aligned_input_size = 0;
  cv::Size frame_size;
  int64_t frames_written_ = 0;
  int64_t start_pts_ = 0;
  int64_t ppts_ = 0;
  bool header_written_ = false;

  AVPacket * encoded_pkt = nullptr;
  SwsContext * sws_ctx = nullptr;

};


/**
 * c_ffmpeg_encoder
 *
 * Based on code example
 *  <https://ffmpeg.org/doxygen/0.6/output-example_8c-source.html>
 */
class c_ffmpeg_encoder
{
public:
  typedef c_ffmpeg_encoder this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::function<bool(const AVPacket &, int index)> writefunc;

  c_ffmpeg_encoder();
  ~c_ffmpeg_encoder();

  const std::string & opts() const;

  bool create(const cv::Size & image_size,
      bool iscolor, const std::string & ffmpeg_opts = "");

  void close();

  bool is_open() const;

  bool encode(const cv::Mat & frame, const writefunc & writepkt);

protected:
  std::string _opts;
  AVCodecContext * codec_ctx = nullptr;
  AVFrame * input_frame = nullptr;
  AVFrame * output_frame = nullptr;
  enum AVPixelFormat input_pix_fmt = AV_PIX_FMT_NONE;
  uint8_t * aligned_input =  nullptr;
  AVPacket * encoded_pkt = nullptr;
  SwsContext * sws_ctx = nullptr;
  size_t  aligned_input_size = 0;
  cv::Size frame_size;
  int64_t pts = 0;
};


/**
 * c_ffmpeg_decoder
 *
 * Based on code example
 *  <https://ffmpeg.org/doxygen/0.6/output-example_8c-source.html>
 */
class c_ffmpeg_decoder
{
public:
  typedef c_ffmpeg_decoder this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  typedef std::function<bool(AVPacket &, int pktidx)> readfunc;
  typedef std::function<bool(const cv::Mat &, int frmidx)> writefunc;

  c_ffmpeg_decoder();
  ~c_ffmpeg_decoder();

  const std::string & opts() const;

  bool create(const std::string & ffmpeg_opts = "");

  void close();

  bool is_open() const;

  bool decode(const readfunc & readpkt, const writefunc & writeframe);

protected:
  std::string _opts;
  ff_const59 AVCodec * codec = nullptr;
  AVCodecContext *  cctx = nullptr;
  //AVStream *        istream = nullptr;
  //AVPacket *        avpacket = nullptr;
  AVFrame *         avframe = nullptr;
  AVFrame *         rgbpicture = nullptr;
  SwsContext *      swsctx = nullptr;
  cv::Size          scaledSize_;

//  struct cvframe {
//    cv::Mat image;
//    int64_t pts; // in stream time_base units
//  };
//  std::vector<cvframe> received_frames;
//  int64_t last_ts = 0;
//  double timescale_ = 0;
};

#endif // HAVE_FFMPEG

#endif /* __c_ffmpeg_file_h__ */
