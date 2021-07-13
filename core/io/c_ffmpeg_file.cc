/*
 * c_ffmpeg_video_reader.cc
 *
 *  Created on: Feb 3, 2020
 *      Author: amyznikov
 */

#include "c_ffmpeg_file.h"
#include <mutex>
#include <core/debug.h>


typedef c_ffmpeg_reader::timeout_interrupt_callback
    timeout_interrupt_callback;


static std::mutex g_mtx;
static std::vector<std::string> g_supported_input_formats;

static int64_t ffmpeg_gettime_us()
{
  struct timespec tm;
  clock_gettime(CLOCK_REALTIME, &tm);
  return ((int64_t) tm.tv_sec * 1000000 + (int64_t) tm.tv_nsec / 1000 );
}

static const char * averr2str(int status)
{
  static thread_local char buff[AV_ERROR_MAX_STRING_SIZE];
  return av_make_error_string(buff, AV_ERROR_MAX_STRING_SIZE, status);
}

static int ffmpeg_timeout_interrupt_callback(void * arg)
{
  timeout_interrupt_callback * cb =
      (timeout_interrupt_callback * )arg;
  return ffmpeg_gettime_us() >= cb->end_time;
}

static void ffmpeg_set_timeout_interrupt_callback(timeout_interrupt_callback * cb, int64_t tmo_us)
{
  cb->icb.callback = ffmpeg_timeout_interrupt_callback;
  cb->end_time = ffmpeg_gettime_us() + tmo_us;
  cb->icb.opaque = cb;
}

static void av_log_callback(void *avcl, int level, const char *fmt, va_list arglist)
{
  if ( level <= av_log_get_level() ) {
    AVClass * avc = avcl ? *(AVClass **) avcl : nullptr;
    cf_plogv(CF_LOG_ERROR, avc ? avc->item_name(avcl) : "avc", "", 0, fmt, arglist);
  }
}

static int ffmpeg_parse_options(const std::string & options, bool remove_prefix, AVDictionary ** rv)
{
  AVDictionary * tmp = nullptr;
  AVDictionaryEntry * e = nullptr;

  int status = 0;

  const char * coptions = options.c_str();
  if ( !*coptions ) {
    goto end;
  }

  if ( (status = av_dict_parse_string(&tmp, coptions, " \t", " \t", AV_DICT_MULTIKEY)) || !tmp ) {
    CF_ERROR("av_dict_parse_string('%s') fails: %s", coptions, averr2str(status));
    goto end;
  }

  while ( (e = av_dict_get(tmp, "", e, AV_DICT_IGNORE_SUFFIX)) ) {

    const char * key = e->key;
    const char * value = e->value;
    const int flags = strcmp(key, "-map") == 0 ? AV_DICT_MULTIKEY : 0;

    if ( remove_prefix ) {
      if ( *e->key != '-' ) {
        CF_ERROR("Option '%s' not starting with '-'", e->key);
        status = AVERROR(EINVAL);
        break;
      }
      ++key;
    }

    if ( (status = av_dict_set(rv, key, value, flags)) < 0 ) {
      CF_ERROR("av_dict_set('%s'='%s') fails: %s", key, value, averr2str(status));
      break;
    }

    status = 0;
  }

end:

  av_dict_free(&tmp);

  return status;
}

static int ffmpeg_alloc_input_context(AVFormatContext **ic, AVIOInterruptCB * icb, AVDictionary ** options)
{
  AVInputFormat * fmt = nullptr;
  AVDictionaryEntry * e = nullptr;

  int status = 0;

  if ( !(*ic = avformat_alloc_context()) ) {
    status = AVERROR(ENOMEM);
    goto end;
  }

  if ( icb ) {
    (*ic)->interrupt_callback = *icb;
  }

  if ( options && *options ) {

    if ( (e = av_dict_get(*options, "f", e, 0)) ) {
      if ( (fmt = av_find_input_format(e->value)) ) {
        (*ic)->iformat = fmt;
      }
      else {
        status = AVERROR_DEMUXER_NOT_FOUND;
        goto end;
      }
    }

    while ( (e = av_dict_get(*options, "", e, AV_DICT_IGNORE_SUFFIX)) ) {
      if ( strcmp(e->key, "f") != 0 ) {
        av_opt_set(*ic, e->key, e->value, AV_OPT_SEARCH_CHILDREN);
      }
    }
  }

end:

  if ( status ) {
    avformat_close_input(ic);
  }

  return status;
}


static int ffmpeg_open_input(AVFormatContext **ic, const char * filename, AVIOInterruptCB * icb, AVDictionary ** options)
{
  int status;

  if ( (status = ffmpeg_alloc_input_context(ic, icb, options)) ) {
    CF_ERROR("[%s] ffmpeg_alloc_format_context() fails: %s", filename,
        averr2str(status));
    goto end;
  }

  (*ic)->flags |= AVFMT_FLAG_DISCARD_CORRUPT | AVFMT_FLAG_GENPTS; // | AVFMT_FLAG_NOBUFFER

  if ( (status = avformat_open_input(ic, filename, nullptr, options)) < 0 ) {
    if ( icb && icb->callback && icb->callback(icb->opaque) ) {
      status = AVERROR(ETIMEDOUT);
    }
    goto end;
  }

  if ( (status = avformat_find_stream_info(*ic, nullptr)) < 0 ) {
    CF_ERROR("[%s] avformat_find_stream_info() fails : %s", filename,
        averr2str(status));
    goto end;
  }


end:

  if ( status ) {
    avformat_close_input(ic);
  }

  return status;
}



static void ffmpeg_close_input(AVFormatContext ** ic)
{
  avformat_close_input(ic);
}

static void ensure_ffmpeg_initialized()
{
  std::lock_guard<std::mutex> lock(g_mtx);

  static bool already_initialized = false;

  if ( !already_initialized ) {

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    av_register_all();
#endif

    avdevice_register_all();
    avformat_network_init();

    av_log_set_level(AV_LOG_ERROR);
    av_log_set_callback(av_log_callback);


    const AVInputFormat * iformat = nullptr;
    void * opaque = nullptr;

    static const char delims[] = ", \t";
    char * tok;

    while ( (iformat = av_demuxer_iterate(&opaque)) ) {
      if ( iformat->name && iformat->extensions ) {

        char buf[strlen(iformat->extensions) + 1];

        tok = strtok(strcpy(buf, iformat->extensions), delims);
        while ( tok ) {
          g_supported_input_formats.emplace_back(tok);
          tok = strtok(NULL, delims);
        }
      }
    }

    already_initialized = true;
  }
}

const std::vector<std::string> & c_ffmpeg_reader::supported_input_formats()
{
  ensure_ffmpeg_initialized();
  return g_supported_input_formats;
}

c_ffmpeg_reader::~c_ffmpeg_reader()
{
  close();
}

void c_ffmpeg_reader::set_stream_name(const std::string & v)
{
  stream_name_ = v;
}

const std::string & c_ffmpeg_reader::stream_name() const
{
  return stream_name_;
}

void c_ffmpeg_reader::set_rcvtmo(int64_t v)
{
  rcvtmo_ = v;
}

int64_t c_ffmpeg_reader::rcvtmo() const
{
  return rcvtmo_;
}


void c_ffmpeg_reader::set_scaled_size(const cv::Size &size)
{
  scaledSize_ = size;
}

cv::Size c_ffmpeg_reader::coded_size() const
{
  return cctx ? cv::Size(cctx->coded_width, cctx->coded_height) : cv::Size(0, 0);
}

int c_ffmpeg_reader::coded_width() const
{
  return cctx ? cctx->coded_width : 0;
}

int c_ffmpeg_reader::coded_height() const
{
  return cctx ? cctx->coded_height : 0;
}

cv::Size c_ffmpeg_reader::scaled_size() const
{
  return scaledSize_;
}


int c_ffmpeg_reader::scaled_width() const
{
  return scaledSize_.width > 0 ? scaledSize_.width : coded_width();
}

int c_ffmpeg_reader::scaled_height() const
{
  return scaledSize_.height > 0 ? scaledSize_.height : coded_height();
}


bool c_ffmpeg_reader::is_open() const
{
  return ic != nullptr;
}

double c_ffmpeg_reader::duration() const
{
  double sec = 0;

  if ( stream ) {
    if ( stream->duration > 0 && stream->time_base.num > 0 && stream->time_base.den > 0 ) {
      sec = (double) stream->duration * stream->time_base.num / stream->time_base.den;
    }
    else {
      sec = (double) ic->duration / (double) AV_TIME_BASE;
    }
  }

  return sec;
}

double c_ffmpeg_reader::fps() const
{
  double fps = 0;

  if ( stream ) {

#if LIBAVCODEC_BUILD >= AV_VERSION_INT(54, 1, 0)
    fps = (double) stream->avg_frame_rate.num / (double) stream->avg_frame_rate.den;
#else
    fps = (double)stream->r_frame_rate.num / (double)stream->r_frame_rate.den;
#endif

    const double eps_zero = 0.000025;
    if ( fps < eps_zero && cctx ) {
      fps = (double) cctx->time_base.den / (double) cctx->time_base.num;
    }
  }

  return fps;
}

int c_ffmpeg_reader::num_frames() const
{
  if ( !stream ) {
    return 0;
  }

  int num_frames_estimated_from_duration = (int) floor(duration() * fps() + 0.5);

  CF_DEBUG("stream->nb_frames=%lld num_frames_estimated_from_duration=%d", (long long ) stream->nb_frames, num_frames_estimated_from_duration);
  if ( stream->nb_frames > 0 ) {
    return stream->nb_frames;
  }


  return num_frames_estimated_from_duration;
}

bool c_ffmpeg_reader::open(const std::string & url, const std::string & input_options)
{
  AVDictionary * opts = nullptr;
  AVDictionary * codec_opts = nullptr;
  int status = 0;

  video_stream_index = -1;

  ensure_ffmpeg_initialized();

  if ( (status = ffmpeg_parse_options(input_options, true, &opts)) ) {
    CF_ERROR("[%s] ffmpeg_parse_options() fails: %s", stream_name_.c_str(), averr2str(status));
    goto end;
  }

  if ( opts ) {
    av_dict_copy(&codec_opts, opts, 0);
  }

  ffmpeg_set_timeout_interrupt_callback(&tcb, rcvtmo_);
  if ( (status = ffmpeg_open_input(&ic, url.c_str(), &tcb.icb, &opts)) ) {
    CF_ERROR("[%s] ffmpeg_open_input() fails: %s", stream_name_.c_str(), averr2str(status));
    goto end;
  }

  video_stream_index = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
  if ( video_stream_index < 0 ) {
    CF_ERROR("[%s] av_find_best_stream(AVMEDIA_TYPE_VIDEO) fails: %d", stream_name_.c_str(), video_stream_index);
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  stream = ic->streams[video_stream_index];
  if ( !stream->codecpar ) {
    CF_ERROR("[%s] stream->codecpar is null", stream_name_.c_str());
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  if ( !(codec = avcodec_find_decoder(stream->codecpar->codec_id)) ) {
    CF_ERROR("[%s] avcodec_find_decoder(codec_id=%d) fails", stream_name_.c_str(), stream->codecpar->codec_id);
    status = AVERROR_DECODER_NOT_FOUND;
    goto end;
  }

  if ( !(cctx = avcodec_alloc_context3(codec)) ) {
    CF_ERROR("[%s] avcodec_alloc_context3(codec=%s) fails", stream_name_.c_str(), codec->name);
    status = AVERROR(ENOMEM);
    goto end;
  }

  if ( (status = avcodec_parameters_to_context(cctx, stream->codecpar)) < 0 ) {
    CF_ERROR("[%s] avcodec_parameters_to_context(codec=%s) fails: %s", stream_name_.c_str(), codec->name,
        averr2str(status));
    goto end;
  }

  if ( (status = avcodec_open2(cctx, codec, &codec_opts)) < 0 ) {
    CF_ERROR("[%s] avcodec_open2() fails for codec=%s : %s", stream_name_.c_str(), codec->name,
        averr2str(status));
    goto end;
  }

  CF_DEBUG("[%s] %d:%s %d/%d tbn %d/%d tbr", stream_name_.c_str(),
      video_stream_index,
      codec->name,
      stream->time_base.num, stream->time_base.den,
      cctx->time_base.num, cctx->time_base.den);

end:

  if ( codec_opts ) {
    av_dict_free(&codec_opts);
  }

  if ( status ) {
    close();
  }

  return status >= 0;
}

void c_ffmpeg_reader::close()
{
  if ( ic ) {
    ffmpeg_close_input(&ic);
  }

  av_packet_free(&avpacket);
  av_frame_free(&avframe);
  av_frame_free(&rgbpicture);

  //sws_freeContext(swsctx);
  swsctx = nullptr;
  video_stream_index = -1;
  last_ts = 0;
}


bool c_ffmpeg_reader::seek_frame(int frame_index)
{
  if ( !stream ) {
    return false;
  }

  const int64_t start_time = stream->start_time > 0 ? stream->start_time : 0;
  const int64_t timestamp = start_time + frame_index * ((int64_t) stream->time_base.den * stream->avg_frame_rate.den)
      / ((int64_t) stream->avg_frame_rate.num * stream->time_base.num);

  int status = av_seek_frame(ic, video_stream_index, timestamp, AVSEEK_FLAG_BACKWARD);
  if ( status < 0 ) {
    CF_ERROR("av_seek_frame(video_stream_index=%d frame_index=%d timestamp=%lld start_time=%lld stream->time_base=%d/%d) fails: "
        "status=%d (%s)",
        video_stream_index, frame_index, (long long )(timestamp), (long long )(start_time),
        stream->time_base.num, stream->time_base.den,
        status, averr2str(status));
  }

  return status >= 0;
}

int64_t c_ffmpeg_reader::curpos() const
{
  int64_t pos = 0;
  if ( stream ) {
    const double start_time = stream->start_time > 0 ? stream->start_time : 0;
    const double sec = (last_ts - start_time) * stream->time_base.num / stream->time_base.den;
    pos = (int64_t) (fps() * sec + 0.5);
  }
  return pos;
}



bool c_ffmpeg_reader::read(cv::Mat & outframe, double * outpts)
{
  int status;

  if ( !received_frames.empty() ) {
    outframe = std::move(received_frames.front().image);
    if ( outpts ) {
      *outpts = received_frames.front().pts;
    }
    received_frames.erase(received_frames.begin());
    return true;
  }

  if ( !avpacket && !(avpacket = av_packet_alloc()) ) {
    CF_ERROR("av_packet_alloc() fails - out of memory?");
    return false;
  }

  if ( !avframe && !(avframe = av_frame_alloc())) {
    CF_ERROR("av_frame_alloc() fails - out of memory?");
    return false;
  }

  if ( !rgbpicture ) {
    if ( !(rgbpicture = av_frame_alloc()) ) {
      CF_ERROR("av_frame_alloc() fails - out of memory?");
      return false;
    }

    rgbpicture->width = scaledSize_.empty() ? cctx->coded_width : scaledSize_.width;
    rgbpicture->height = scaledSize_.empty() ? cctx->coded_height : scaledSize_.height;
    rgbpicture->format = AV_PIX_FMT_BGR24;

    if ( (status = av_frame_get_buffer(rgbpicture, 32)) < 0 ) {
      CF_ERROR("av_frame_get_buffer() fails : %s", averr2str(status));
      return false;
    }
  }


  ffmpeg_set_timeout_interrupt_callback(&tcb, rcvtmo_);

  while ( 42 ) {

    if ( (status = av_read_frame(ic, avpacket)) == AVERROR(EAGAIN) ) {
      continue;
    }

    if ( status < 0 ) {
      CF_ERROR("av_read_frame() fails: %s", averr2str(status));
      break;
    }

    if ( avpacket->stream_index != video_stream_index ) {
      av_packet_unref(avpacket);
      continue;
    }

    if ( (status = avcodec_send_packet(cctx, avpacket)) < 0 ) {
      CF_ERROR("avcodec_send_packet() fails: %s", averr2str(status));
      if ( status == AVERROR_EOF || status == AVERROR(EINVAL) || status == AVERROR(ENOMEM) ) {
        CF_ERROR("break");
        break;
      }
      continue;
    }

    while ( (status = avcodec_receive_frame(cctx, avframe)) >= 0 ) {

      if ( !swsctx || cctx->coded_width != rgbpicture->width || cctx->coded_height != rgbpicture->height
          || cctx->pix_fmt != rgbpicture->format ) {

        swsctx = sws_getCachedContext(
            swsctx,
            cctx->coded_width,
            cctx->coded_height,
            cctx->pix_fmt,
            rgbpicture->width,
            rgbpicture->height,
            (AVPixelFormat)rgbpicture->format,
            SWS_FAST_BILINEAR,
            nullptr, nullptr, nullptr);

        if ( !swsctx ) {
          CF_ERROR("sws_getCachedContext() fails");
          return false;
        }
      }

      sws_scale(swsctx,
          avframe->data,
          avframe->linesize,
          0, cctx->coded_height,
          rgbpicture->data,
          rgbpicture->linesize);

      received_frames.emplace_back();

      cv::Mat(rgbpicture->height,
          rgbpicture->width,
          CV_MAKETYPE(CV_8U, 3),
          rgbpicture->data[0],
          rgbpicture->linesize[0]).copyTo(received_frames.back().image);

      last_ts = avframe->best_effort_timestamp;
      const double pts = avframe->best_effort_timestamp == AV_NOPTS_VALUE ? 0 : avframe->best_effort_timestamp;
      received_frames.back().pts = pts * stream->time_base.num / stream->time_base.den;
    }

    if ( status == AVERROR(EAGAIN) && received_frames.empty() ) {
      ffmpeg_set_timeout_interrupt_callback(&tcb, rcvtmo_);
      continue;
    }

    if ( received_frames.empty() ) {
      CF_ERROR("avcodec_receive_frame() fails: %s", averr2str(status));
    }

    break;
  }


  if ( !received_frames.empty() ) {
    outframe = std::move(received_frames.front().image);
    if ( outpts ) {
      *outpts = received_frames.front().pts;
    }
    received_frames.erase(received_frames.begin());
    return true;
  }

  return false;
}
