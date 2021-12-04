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
#if HAVE_AVDEVICE
  #include <libavdevice/avdevice.h>
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

  ///@brief estimate video duration in seconds
  double duration() const;

  ///@brief estimate fps
  double fps() const;

  ///@brief get coded image size
  cv::Size coded_size() const;

  ///@brief get coded image width
  int coded_width() const;

  ///@brief get coded image height
  int coded_height() const;

  ///@brief estimate number of video frames in video stream
  int num_frames() const;

  ///@brief try seek to given frame
  bool seek_frame(int index);

  int64_t curpos() const;

  bool read(cv::Mat & outframe, double * outpts = nullptr);
  void close();

  ///@brief set output size
  void set_scaled_size(const cv::Size &size);

  ///@brief get scalled output size
  cv::Size scaled_size() const;

  ///@brief get scalled output width
  int scaled_width() const;

  ///@brief get scalled output height
  int scaled_height() const;


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
  AVCodec *         codec = nullptr;
  AVCodecContext *  cctx = nullptr;
  AVStream *        stream = nullptr;
  AVPacket *        avpacket = nullptr;
  AVFrame *         avframe = nullptr;
  AVFrame *         rgbpicture = nullptr;
  SwsContext *      swsctx = nullptr;
  cv::Size          scaledSize_;

  struct cvframe {
    cv::Mat image;
    double  pts;
  };
  std::vector<cvframe> received_frames;
  int64_t last_ts = 0;
};

#endif /* __c_ffmpeg_file_h__ */
