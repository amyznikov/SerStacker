/*
 * c_ffmpeg_video_reader.cc
 *
 *  Created on: Feb 3, 2020
 *      Author: amyznikov
 */

#include "c_ffmpeg_file.h"
#include <mutex>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>


extern "C" {

// as defined in libavformat/internal.h
typedef struct AVCodecTag {
  enum AVCodecID id;
  unsigned int tag;
} AVCodecTag;

}

typedef c_ffmpeg_reader::timeout_interrupt_callback
    timeout_interrupt_callback;


static std::mutex g_mtx;
static std::vector<std::string> g_supported_input_formats;
static std::vector<std::string> g_supported_output_formats;
static std::vector<std::string> g_supported_video_decoders;
static std::vector<std::string> g_supported_video_encoders;

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
  ff_const59 AVInputFormat * fmt = nullptr;
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

    if ( (e = av_dict_get(*options, "f", nullptr, 0)) ) {
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
    CF_DEBUG("avformat_open_input('%s') fails: %s", filename, averr2str(status));
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

static bool ffmpeg_setup_encoder_parameters(AVCodecContext * codec_ctx,
    int w, int h, double fps, AVPixelFormat pix_fmt)
{
  double bitrate = 0;
  double bitrate_scale = 1;

  int frame_rate, frame_rate_base;
  int64_t lbit_rate;

  bitrate = std::min(bitrate_scale * fps * w * h,
      (double) INT_MAX / 2);

  /* put sample parameters */
  lbit_rate = (int64_t) bitrate;
  lbit_rate += (bitrate / 2);
  lbit_rate = std::min(lbit_rate, (int64_t) INT_MAX);
  codec_ctx->bit_rate = lbit_rate;
  codec_ctx->bit_rate_tolerance = (int) lbit_rate;

  // took advice from
  // http://ffmpeg-users.933282.n4.nabble.com/warning-clipping-1-dct-coefficients-to-127-127-td934297.html
  codec_ctx->qmin = 3;
  /* resolution must be a multiple of two */
  codec_ctx->width = w;
  codec_ctx->height = h;

  /* time base: this is the fundamental unit of time (in seconds) in terms
   of which frame timestamps are represented. for fixed-fps content,
   timebase should be 1/framerate and timestamp increments should be
   identically 1. */
  frame_rate = (int) (fps + 0.5);
  frame_rate_base = 1;
  while ( fabs(((double) frame_rate / frame_rate_base) - fps) > 0.001 ) {
    frame_rate_base *= 10;
    frame_rate = (int) (fps * frame_rate_base + 0.5);
  }

  codec_ctx->time_base.num = frame_rate_base;
  codec_ctx->time_base.den = frame_rate;

  /* adjust time base for supported framerates */
  if ( codec_ctx->codec && codec_ctx->codec->supported_framerates ) {

    const AVRational * p =
        codec_ctx->codec->supported_framerates;

    const AVRational * best = nullptr;

    AVRational req = { frame_rate, frame_rate_base };
    AVRational best_error = { INT_MAX, 1 };

    for ( ; p->den != 0; p++ ) {
      AVRational error = av_sub_q(req, *p);
      if ( error.num < 0 ) {
        error.num *= -1;
      }
      if ( av_cmp_q(error, best_error) < 0 ) {
        best_error = error;
        best = p;
      }
    }

    if ( best ) {
      codec_ctx->time_base = *best;
    }
  }

  /* adjust pix_fmt if was not set */
  if ( pix_fmt != AV_PIX_FMT_NONE ) {
    codec_ctx->pix_fmt = pix_fmt;
  }
  else if ( codec_ctx->pix_fmt == AV_PIX_FMT_NONE && codec_ctx->codec ) {
    if ( codec_ctx->codec->pix_fmts ) {
      codec_ctx->pix_fmt =
          codec_ctx->codec->pix_fmts[0];
    }
  }


  return true;
}

/* Open the encoder.
 * */
static AVCodecContext * ffmpeg_open_encoder(AVCodecID codec_id,
    int w, int h, AVPixelFormat pix_fmt, double fps,
    int flags,
    int qscale,
    AVDictionary ** opts = nullptr)
{
  const AVCodec * codec = nullptr;
  AVCodecContext * codec_ctx =  nullptr;
  int status;
  bool fOk = false;

  if ( !(codec = avcodec_find_encoder(codec_id)) ) {
    CF_ERROR("avcodec_find_encoder() fails for codec_id=%d", codec_id);
    goto end;
  }

  if ( !(codec_ctx = avcodec_alloc_context3(codec)) ) {
    CF_ERROR("avcodec_alloc_context3() fails\n");
    goto end;
  }

  codec_ctx->flags |= flags; // AV_CODEC_FLAG_GLOBAL_HEADER;
  ffmpeg_setup_encoder_parameters(codec_ctx, w, h, fps, pix_fmt);

  if ( qscale >= 0 ) {
    // Based on new_output_stream() from /ffmpeg/fftools/ffmpeg_opt.c
    codec_ctx->flags |= AV_CODEC_FLAG_QSCALE;
    codec_ctx->global_quality = FF_QP2LAMBDA * qscale;
    CF_DEBUG("SET codec_ctx->global_quality=%d (qscale=%d)", codec_ctx->global_quality, qscale);
  }


  if ( (status = avcodec_open2(codec_ctx, codec, opts)) < 0 ) {
    CF_ERROR("avcodec_open2() fails: %s\n", averr2str(status));
    goto end;
  }

  fOk = true;
end:
  if ( !fOk ) {
    if ( codec_ctx ) {
      avcodec_close(codec_ctx);
      codec_ctx = nullptr;
    }
  }

  return codec_ctx;
}


/**
 * the following function is a modified version of code
 * found in ffmpeg-0.4.9-pre1/output_example.c
 */
static AVFrame * ffmpeg_alloc_frame(AVPixelFormat pix_fmt, int width, int height, bool alloc_buffer)
{
  AVFrame * picture;
  uint8_t * picture_buf = 0;
  int size;

#if LIBAVCODEC_BUILD >= (LIBAVCODEC_VERSION_MICRO >= 100 ? AV_VERSION_INT(55, 45, 101) : AV_VERSION_INT(55, 28, 1))
  if ( !(picture = av_frame_alloc()) ) {
    CF_ERROR("av_frame_alloc() fails");
    return nullptr;
  }
#else
  if ( !(output_frame = avcodec_alloc_frame()) ) {
    CF_ERROR("avcodec_alloc_frame() fails");
    return nullptr;
  }
#endif

  picture->format = pix_fmt;
  picture->width = width;
  picture->height = height;

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
  size = av_image_get_buffer_size(pix_fmt, width, height, 1);
#else
  size = avpicture_get_size(pix_fmt, width, height);
#endif

  if ( alloc_buffer ) {
    if ( !(picture_buf = (uint8_t *) av_malloc(size)) ) {
      CF_DEBUG("av_malloc(picture_buf, size=%d) fails", size);
      av_free(picture);
      return nullptr;
    }
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
    av_image_fill_arrays((picture)->data, (picture)->linesize, picture_buf, pix_fmt, width, height, 1);
#else
    avpicture_fill(output_frame, picture_buf, pix_fmt, width, height);
#endif
  }

  return picture;
}

static void ensure_ffmpeg_initialized()
{
  std::lock_guard<std::mutex> lock(g_mtx);

  static bool already_initialized = false;

  if ( !already_initialized ) {

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    av_register_all();
#endif

#if HAVE_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();

    av_log_set_level(AV_LOG_ERROR);
    av_log_set_callback(av_log_callback);


    const AVInputFormat * iformat = nullptr;
    const AVOutputFormat * oformat = nullptr;
    const AVCodecDescriptor *codec_desc = nullptr;

    void * opaque = nullptr;
    static const char delims[] = ", \t";
    char * tok;


#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 0, 0)
    while ( (iformat = av_iformat_next(iformat)) ) {
      if ( iformat->name ) {
        if ( !iformat->extensions ) {
          g_supported_input_formats.emplace_back(iformat->name);
        }
        else {

          char buf[strlen(iformat->extensions) + 1];

          tok = strtok(strcpy(buf, iformat->extensions), delims);
          while ( tok ) {
            g_supported_input_formats.emplace_back(tok);
            tok = strtok(NULL, delims);
          }
        }
      }
    }
#else
    opaque = nullptr;
    while ( (iformat = av_demuxer_iterate(&opaque)) ) {
      if ( iformat->name ) {
        if ( !iformat->extensions ) {
          g_supported_input_formats.emplace_back(iformat->name);
        }
        else {

          char buf[strlen(iformat->extensions) + 1];

          tok = strtok(strcpy(buf, iformat->extensions), delims);
          while ( tok ) {
            g_supported_input_formats.emplace_back(tok);
            tok = strtok(NULL, delims);
          }
        }
      }
    }
#endif

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 0, 0)
    while ( (oformat = av_oformat_next(oformat)) ) {
      if ( oformat->name ) {
        if ( !oformat->extensions ) {
          g_supported_output_formats.emplace_back(oformat->name);
        }
        else {
          char buf[strlen(oformat->extensions) + 1];

          tok = strtok(strcpy(buf, oformat->extensions), delims);
          while ( tok ) {
            g_supported_output_formats.emplace_back(tok);
            tok = strtok(NULL, delims);
          }
        }
      }
    }
#else
    opaque = nullptr;
    while ( (oformat = av_muxer_iterate(&opaque)) ) {
      if ( oformat->name ) {
        if ( !oformat->extensions ) {
          g_supported_output_formats.emplace_back(oformat->name);
        }
        else {
          char buf[strlen(oformat->extensions) + 1];

          tok = strtok(strcpy(buf, oformat->extensions), delims);
          while ( tok ) {
            g_supported_output_formats.emplace_back(tok);
            tok = strtok(NULL, delims);
          }
        }
      }
    }
#endif

    std::vector<const AVCodecDescriptor *> encoders;
    std::vector<const AVCodecDescriptor *> decoders;

    while ((codec_desc = avcodec_descriptor_next(codec_desc))) {
      if( codec_desc->type == AVMEDIA_TYPE_VIDEO ) {

        if( avcodec_find_encoder(codec_desc->id) ) {
          encoders.emplace_back(codec_desc);
        }

        if( avcodec_find_decoder(codec_desc->id) ) {
          decoders.emplace_back(codec_desc);
        }
      }
    }

    std::sort(encoders.begin(), encoders.end(),
        [](const AVCodecDescriptor * prev, const AVCodecDescriptor * next) {
          return strcasecmp(prev->name, next->name) < 0;
        });

    std::sort(decoders.begin(), decoders.end(),
        [](const AVCodecDescriptor * prev, const AVCodecDescriptor * next) {
          return strcasecmp(prev->name, next->name) < 0;
        });

    for ( const AVCodecDescriptor * desc: encoders ) {
      const std::string props =
          ssprintf("%s%s%s",
              (desc->props & AV_CODEC_PROP_INTRA_ONLY) ? "I" : ".",
              (desc->props & AV_CODEC_PROP_LOSSY) ? "L" : ".",
              (desc->props & AV_CODEC_PROP_LOSSLESS) ? "S" : ".");

      g_supported_video_encoders.emplace_back(
          ssprintf("%s %s %s",
              props.c_str(),
              desc->name,
              desc->long_name ? desc->long_name : ""));
    }

    for ( const AVCodecDescriptor * desc: decoders ) {
      const std::string props =
          ssprintf("%s%s%s",
              (desc->props & AV_CODEC_PROP_INTRA_ONLY) ? "I" : ".",
              (desc->props & AV_CODEC_PROP_LOSSY) ? "L" : ".",
              (desc->props & AV_CODEC_PROP_LOSSLESS) ? "S" : ".");

      g_supported_video_decoders.emplace_back(
          ssprintf("%s %s %s",
              props.c_str(),
              desc->name,
              desc->long_name ? desc->long_name : ""));
    }


    already_initialized = true;
  }
}


///////////////////////////////////////////////////////////////////////////////

const std::vector<std::string> & c_ffmpeg_reader::supported_input_formats()
{
  ensure_ffmpeg_initialized();
  return g_supported_input_formats;
}

const std::vector<std::string> & c_ffmpeg_reader::supported_decoders()
{
  ensure_ffmpeg_initialized();
  return g_supported_video_decoders;
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

cv::Size c_ffmpeg_reader::coded_size() const
{
  return cctx ? cv::Size(cctx->coded_width, cctx->coded_height) : cv::Size(0, 0);
}

void c_ffmpeg_reader::set_frame_size(const cv::Size &size)
{
  scaledSize_ = size;
}

cv::Size c_ffmpeg_reader::frame_size() const
{
  return scaledSize_.empty() ? coded_size() : scaledSize_;
}

bool c_ffmpeg_reader::is_open() const
{
  return ic != nullptr;
}

double c_ffmpeg_reader::duration() const
{
  double sec = 0;

  if ( istream ) {
    if ( istream->duration > 0 && istream->time_base.num > 0 && istream->time_base.den > 0 ) {
      sec = (double) istream->duration * istream->time_base.num / istream->time_base.den;
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

  if ( istream ) {

#if LIBAVCODEC_BUILD >= AV_VERSION_INT(54, 1, 0)
    fps = (double) istream->avg_frame_rate.num / (double) istream->avg_frame_rate.den;
#else
    fps = (double)istream->r_frame_rate.num / (double)istream->r_frame_rate.den;
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
  if ( !istream ) {
    return 0;
  }

  if ( istream->nb_frames > 0 ) {
    return istream->nb_frames;
  }

  int num_frames_estimated_from_duration = (int) floor(duration() * fps() + 0.5);

  //  CF_DEBUG("stream->nb_frames=%lld num_frames_estimated_from_duration=%d",
  //      (long long ) stream->nb_frames,
  //      num_frames_estimated_from_duration);

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
    CF_ERROR("[%s] ffmpeg_open_input('%s') fails: %s", stream_name_.c_str(), url.c_str(), averr2str(status));
    goto end;
  }

  video_stream_index = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
  if ( video_stream_index < 0 ) {
    CF_ERROR("[%s] av_find_best_stream(AVMEDIA_TYPE_VIDEO) fails: %d", stream_name_.c_str(), video_stream_index);
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  istream = ic->streams[video_stream_index];
  if ( !istream->codecpar ) {
    CF_ERROR("[%s] stream->codecpar is null", stream_name_.c_str());
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  if ( !(codec = avcodec_find_decoder(istream->codecpar->codec_id)) ) {
    CF_ERROR("[%s] avcodec_find_decoder(codec_id=%d) fails", stream_name_.c_str(), istream->codecpar->codec_id);
    status = AVERROR_DECODER_NOT_FOUND;
    goto end;
  }

  if ( !(cctx = avcodec_alloc_context3(codec)) ) {
    CF_ERROR("[%s] avcodec_alloc_context3(codec=%s) fails", stream_name_.c_str(), codec->name);
    status = AVERROR(ENOMEM);
    goto end;
  }

  if ( (status = avcodec_parameters_to_context(cctx, istream->codecpar)) < 0 ) {
    CF_ERROR("[%s] avcodec_parameters_to_context(codec=%s) fails: %s", stream_name_.c_str(), codec->name,
        averr2str(status));
    goto end;
  }

  if ( (status = avcodec_open2(cctx, codec, &codec_opts)) < 0 ) {
    CF_ERROR("[%s] avcodec_open2() fails for codec=%s : %s", stream_name_.c_str(), codec->name,
        averr2str(status));
    goto end;
  }

    CF_DEBUG("[%s] %d:%s %dx%d %d/%d tbn %d/%d tbr", url.c_str(),
        video_stream_index,
        codec->name,
        cctx->coded_width,
        cctx->coded_height,
        istream->time_base.num, istream->time_base.den,
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

  if ( swsctx ) {
    sws_freeContext(swsctx);
    swsctx = nullptr;
  }

  video_stream_index = -1;
  last_ts = 0;
}

const AVStream * c_ffmpeg_reader::stream() const
{
  return istream;
}


bool c_ffmpeg_reader::seek_frame(int frame_index)
{
  if ( !istream || istream->avg_frame_rate.num <= 0 || istream->time_base.num <= 0 ) {
    return false;
  }

  const int64_t start_time = istream->start_time > 0 ? istream->start_time : 0;
  const int64_t timestamp = start_time + frame_index * ((int64_t) istream->time_base.den * istream->avg_frame_rate.den)
      / ((int64_t) istream->avg_frame_rate.num * istream->time_base.num);

  int status = av_seek_frame(ic, video_stream_index, timestamp, AVSEEK_FLAG_BACKWARD);
  if ( status < 0 ) {
    CF_ERROR("av_seek_frame(video_stream_index=%d frame_index=%d timestamp=%lld start_time=%lld stream->time_base=%d/%d) fails: "
        "status=%d (%s)",
        video_stream_index, frame_index, (long long )(timestamp), (long long )(start_time),
        istream->time_base.num, istream->time_base.den,
        status, averr2str(status));
  }

  return status >= 0;
}

int64_t c_ffmpeg_reader::curpos() const
{
  int64_t pos = 0;
  if ( istream ) {
    const double start_time = istream->start_time > 0 ? istream->start_time : 0;
    pos = last_ts - start_time;
    //const double sec = (last_ts - start_time) * stream->time_base.num / stream->time_base.den;
    //pos = (int64_t) (fps() * sec + 0.5);
  }
  return pos;
}



bool c_ffmpeg_reader::read(cv::Mat & outframe, int64_t * outpts)
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

    status = avcodec_send_packet(cctx, avpacket);
    av_packet_unref(avpacket);

    if ( status < 0 ) {
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
      received_frames.back().pts = avframe->best_effort_timestamp;
      // CF_DEBUG("avframe->pts=%ld best_effort_timestamp=%ld", avframe->pts, avframe->best_effort_timestamp);
      //const double pts = avframe->best_effort_timestamp == AV_NOPTS_VALUE ? 0 : avframe->best_effort_timestamp;
      //received_frames.back().pts = pts * stream->time_base.num / stream->time_base.den;
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

///////////////////////////////////////////////////////////////////////////////

const std::vector<std::string> & c_ffmpeg_writer::supported_output_formats()
{
  ensure_ffmpeg_initialized();
  return g_supported_input_formats;
}

const std::vector<std::string> & c_ffmpeg_writer::supported_encoders()
{
  ensure_ffmpeg_initialized();
  return g_supported_video_encoders;
}

c_ffmpeg_writer::~c_ffmpeg_writer()
{
  close();
}

const AVPixelFormat * c_ffmpeg_writer::supported_codec_pix_formats(AVCodecID codec_id)
{
  ensure_ffmpeg_initialized();

  const AVCodec * codec =
      avcodec_find_encoder(
          codec_id);

  return codec ? codec->pix_fmts : nullptr;
}

///@brief open (create) video file
bool c_ffmpeg_writer::open(const std::string & output_filename,
    const cv::Size & image_size, bool is_color,
    const std::string & ffmpeg_opts)
{
  std::string output_directory;
  AVDictionary * opts = nullptr;

  ff_const59 AVOutputFormat * ofmt = nullptr;

  AVCodecID codec_id = AV_CODEC_ID_NONE;
  AVPixelFormat codec_pix_fmt = AV_PIX_FMT_NONE;
  double fps = -1;
  int qscale = -1;

  bool need_color_convert = false;

  int status = 0;
  bool fOk = false;

  ensure_ffmpeg_initialized();
  close();


  /*
   * Get output file name
   *  */
  if ( (output_filename_ = output_filename).empty() ) {
    CF_ERROR("No output file name spcified, can not create output file");
    return false;
  }

  /*
   * Select input pixel format
   * */
  input_pix_fmt =
      is_color ? AV_PIX_FMT_BGR24 :
          AV_PIX_FMT_GRAY8;


  /*
   * Like to OpenCV:
   *  we allow frames of odd width or height, but in this case we truncate
   *  the rightmost column/the bottom row. Probably, this should be handled more elegantly,
   *  but some internal functions inside FFMPEG swscale require even width/height.
   */
  frame_size.width = image_size.width & -2;
  frame_size.height = image_size.height & -2;

  if ( frame_size.width <= 0 || frame_size.height <= 0 ) {
    CF_ERROR("Invalid output frame size specified : %dx%d", frame_size.width, frame_size.height);
    goto end;
  }


  /*
   * Check if ffmpeg options are specified
   */
  if ( !ffmpeg_opts.empty() ) {

    if ( (status = ffmpeg_parse_options(ffmpeg_opts, true, &opts)) < 0 ) {
      CF_ERROR("ffmpeg_parse_options() fails");
      goto end;
    }

    if ( opts ) {

      AVDictionaryEntry * e;
      const int flags = AV_DICT_IGNORE_SUFFIX;

      /* check if output format specified */
      if ( (e = av_dict_get(opts, "f", nullptr, flags)) ) {
        if ( !(ofmt = av_guess_format(e->value, nullptr, nullptr)) ) {
          CF_ERROR("requested output format '%s; not found", e->value);
          goto end;
        }
      }

      /* check if codec specified */
      if ( (e = av_dict_get(opts, "c", nullptr, flags)) ) {
        const AVCodec * codec = avcodec_find_encoder_by_name(e->value);
        if ( codec ) {
          codec_id = codec->id;
        }
        else {
          CF_ERROR("requested encoder '%s; not found", e->value);
          goto end;
        }
      }
      if ( (e = av_dict_get(opts, "c:v", nullptr, flags)) ) {
        const AVCodec * codec = avcodec_find_encoder_by_name(e->value);
        if ( codec ) {
          codec_id = codec->id;
        }
        else {
          CF_ERROR("requested encoder '%s; not found", e->value);
          goto end;
        }
      }

      /* check if codec pixel format specified */
      if ( (e = av_dict_get(opts, "pix_fmt", nullptr, flags)) ) {
        codec_pix_fmt = av_get_pix_fmt(e->value);
        if ( codec_pix_fmt == AV_PIX_FMT_NONE ) {
          CF_ERROR("requested pixel format '%s; not found", e->value);
          goto end;
        }
      }

      /* check if fps (frame rate) specified */
      if ( (e = av_dict_get(opts, "r", nullptr, flags)) || (e = av_dict_get(opts, "fps", nullptr, flags)) ) {
        if ( sscanf(e->value, "%lf", &fps) != 1 || fps <= 0 ) {
          CF_ERROR("requested fps %s is invalid", e->value);
          goto end;
        }
      }

      /* check if global_quality (q, qscale) option specified */
      if ( (e = av_dict_get(opts, "q", nullptr, flags)) || (e = av_dict_get(opts, "qscale", nullptr, flags)) ||
          (e = av_dict_get(opts, "q:v", nullptr, flags)) ) {
        if ( sscanf(e->value, "%d", &qscale) != 1 ) {
          CF_ERROR("requested qscale %s is invalid", e->value);
          goto end;
        }
      }

    }
  }

  if ( fps < 0 ) { // try to make timestamps as precise as possible
    fps = 1000000;
  }

  /*
   * Select output format from the file name if was not specified before
   * */
  if ( ofmt ) {

    std::string suffix =
        get_file_suffix(
            output_filename_);

    if ( suffix.empty() && ofmt->extensions ) {

      std::vector<std::string> suffixes =
          strsplit(ofmt->extensions, " ,");

      if ( !suffixes.empty() ) {
        (output_filename_ += '.') += suffixes[0];
      }
    }
  }


  if ( !ofmt && !(ofmt = av_guess_format(nullptr, output_filename_.c_str(), nullptr)) ) {
    CF_ERROR("Could not deduce output format from file extension for '%s'\n",
        output_filename_.c_str());
    goto end;
  }

  output_directory =
      get_parent_directory(output_filename_);

  if ( !output_directory.empty() && !create_path(output_directory) ) {
    CF_ERROR("create_path('%s') fails: %s", output_directory.c_str(),
        strerror(errno));
    return false;
  }




  /*
   * Auto select codec if was not specified
   * */
  if ( codec_id == AV_CODEC_ID_NONE ) {
    /* try default video codec for given output format */
    if ( (codec_id = ofmt->video_codec) == AV_CODEC_ID_NONE ) {
      CF_ERROR("Can not automatically select appropriate video codec");
      goto end;
    }
  }
  else if ( ofmt->codec_tag ) {
    /* check if requested coded is supported by the selected output format */
    bool supported = false;
    for ( int i = 0; ofmt->codec_tag[i] && ofmt->codec_tag[i]->id != AV_CODEC_ID_NONE; ++i ) {
      if ( codec_id == ofmt->codec_tag[i]->id ) {
        supported = true;
        break;
      }
    }
    if ( !supported ) {
      CF_WARNING("WARNING: requested codec id=%d may be not supported by selected output format %s",
          codec_id, ofmt->name);
    }
  }


  /*
   * Open encoder
   * */

  codec_ctx =
      ffmpeg_open_encoder(codec_id,
          frame_size.width,
          frame_size.height,
          codec_pix_fmt,
          fps,
          AV_CODEC_FLAG_GLOBAL_HEADER, /*(ofmt->flags & AVFMT_GLOBALHEADER) ? AV_CODEC_FLAG_GLOBAL_HEADER : 0,*/
          qscale,
          &opts);

  if ( !codec_ctx ) {
    CF_ERROR("ffmpeg_open_encoder() fails");
    goto end;
  }


  /*
   * Setup output context and and new video stream.
   *  Based on ffmpeg/doc/examples/transcoding.c
   * */
  status = avformat_alloc_output_context2(&octx, ofmt, nullptr, output_filename_.c_str());
  if ( status < 0 || !octx ) {
    CF_ERROR("avformat_alloc_output_context2() fails : %s\n",averr2str(status));
    goto end;
  }

  if ( !(ostream = avformat_new_stream(octx, codec_ctx->codec)) ) {
    CF_ERROR("avformat_new_stream() fails\n");
    goto end;
  }

  status = avcodec_parameters_from_context(ostream->codecpar, codec_ctx);
  if ( status < 0 ) {
    CF_ERROR("avcodec_parameters_from_context() fails: %s\n", averr2str(status));
    goto end;
  }

  ostream->time_base = codec_ctx->time_base;

  /*
   * Allocate AVFrames
   * */

  output_frame =
      ffmpeg_alloc_frame(codec_ctx->pix_fmt,
          codec_ctx->width,
          codec_ctx->height,
          codec_ctx->pix_fmt != input_pix_fmt);

  if ( !output_frame ) {
    CF_ERROR("ffmpeg_alloc_frame(output_frame) fails");
    goto end;
  }

  if ( codec_ctx->pix_fmt != input_pix_fmt ) {

    input_frame =
        ffmpeg_alloc_frame(input_pix_fmt,
            codec_ctx->width,
            codec_ctx->height,
            false);

    if ( !input_frame ) {
      CF_ERROR("ffmpeg_alloc_frame(input_frame) fails");
      goto end;
    }
  }


  /**
   * Create output file, init muxer,
   * write output file header
   */

  if ( !(octx->oformat->flags & AVFMT_NOFILE) ) {
    if ( (status = avio_open(&octx->pb, output_filename_.c_str(), AVIO_FLAG_WRITE)) < 0 ) {
      CF_ERROR("avio_open('%s') fails : %s", output_filename_.c_str(), averr2str(status));
      goto end;
    }
  }


  if ( (status = avformat_write_header(octx, &opts)) < 0 ) {
    CF_ERROR("avformat_write_header() fails: %s\n", averr2str(status));
    goto end;
  }

  CF_DEBUG("c_ffmpeg_writer: pix_fmt=%d (%s) codec.time_base={%d/%d} stream.time_base={%d/%d}",
      codec_ctx->pix_fmt,
      av_get_pix_fmt_name(codec_ctx->pix_fmt),
      codec_ctx->time_base.num,
      codec_ctx->time_base.den,
      ostream->time_base.num,
      ostream->time_base.den );

  ////////////////////////////////////

  av_dict_free(&opts);
  header_written_ = true;
  fOk = true;

end:
  if ( !fOk ) {
    close();
  }

  return fOk;
}

const AVStream * c_ffmpeg_writer::stream() const
{
  return ostream;
}

///@brief return true if video file is open
bool c_ffmpeg_writer::is_open() const
{
  return octx != nullptr;
}


///@brief close current output file
void c_ffmpeg_writer::close()
{
  if ( octx ) {

    if ( header_written_  ) {

      /* flush encoder */
      int status = encode_and_send_frame(nullptr);
      while ( status == 0 ) {
        status = encode_and_send_frame(nullptr);
      }

      if ( status <= 0 && status != AVERROR_EOF ) {
        CF_ERROR("encode_and_send_frame(): status = %d (%s)",
            status, averr2str(status));
      }

      /* write trailer */
      if ( (status = av_write_trailer(octx)) < 0 ) {
        CF_ERROR("av_write_trailer() fails: status=%d (%s)",
            status, averr2str(status));
      }
    }

    /* close the output file */
    if ( octx->oformat && !(octx->oformat->flags & AVFMT_NOFILE) ) {
      avio_close(octx->pb);
    }

    /* free the context */
    avformat_free_context(octx);
    octx = nullptr;
  }

  if ( codec_ctx ) {
    avcodec_close(codec_ctx);
    avcodec_free_context(&codec_ctx);
  }

  if ( sws_ctx ) {
    sws_freeContext(sws_ctx);
    sws_ctx = nullptr;
  }

  if ( aligned_input ) {
    av_free(aligned_input);
    aligned_input = nullptr;
  }

  if ( input_frame ) {
    av_frame_free(&input_frame);
  }

  if ( output_frame ) {
    av_frame_free(&output_frame);
  }

  if (encoded_pkt) {
    av_packet_unref(encoded_pkt);
    encoded_pkt = nullptr;
  }

  start_pts_ = 0;
}

bool c_ffmpeg_writer::write(const cv::Mat image, int64_t pts)
{
  if ( !is_open() ) {
    CF_ERROR("c_ffmpeg_writer: output stream is not open");
    return false;
  }

  if ( !frames_written_ ) {
    start_pts_ = pts;
  }

  bool fOk =
      write_frame(image.ptr(),
          (int) image.step,
          image.cols,
          image.rows,
          image.channels(),
          0,
          pts - start_pts_);

  if ( !fOk ) {
    CF_ERROR("c_ffmpeg_writer: write_frame() fails");
  }

  return fOk;
}


int c_ffmpeg_writer::encode_and_send_frame(AVFrame * picture)
{
  if ( picture ) {

    picture->pts =
        av_rescale_q(picture->pts,
            ostream->time_base,
            codec_ctx->time_base);

    if ( codec_ctx->flags & AV_CODEC_FLAG_QSCALE ) {
      picture->quality =
          codec_ctx->global_quality;
    }
  }

  int status = avcodec_send_frame(codec_ctx, picture);
  while ( status >= 0 ) {

    if ( !encoded_pkt ) {
      encoded_pkt = av_packet_alloc();
    }

    if ( (status = avcodec_receive_packet(codec_ctx, encoded_pkt)) >= 0 ) {

      av_packet_rescale_ts(encoded_pkt, codec_ctx->time_base, ostream->time_base);

      encoded_pkt->stream_index = ostream->index;
      encoded_pkt->dts = frames_written_;

      if ( (status = av_write_frame(octx, encoded_pkt)) >= 0 ) {
        ++frames_written_;
      }

      continue;
    }

    if ( status == AVERROR(EAGAIN) ) {
      status = 0;
    }
    else if ( picture || status != AVERROR_EOF ) {
      CF_ERROR("avcodec_receive_packet() fails: %s",
          averr2str(status));
    }

    break;
  }

  return status;
}

bool c_ffmpeg_writer::write_frame(const uint8_t * data, int step, int width, int height, int cn, int origin, int64_t pts)
{
  // check parameters
  if ( input_pix_fmt == AV_PIX_FMT_BGR24 ) {
    if ( cn != 3 ) {
      CF_ERROR("Invalid inpput: cn = %d but 3 expected", cn);
      return false;
    }
  }
  else if ( input_pix_fmt == AV_PIX_FMT_GRAY8 ) {
    if ( cn != 1 ) {
      CF_ERROR("Invalid inpput: cn = %d but 1 expected", cn);
      return false;
    }
  }
  else {
    CF_ERROR("APP BUG: invalid input_pix_fmt=%d encountered", input_pix_fmt);
    return false;
  }

  if ( (width & -2) != frame_size.width || (height & -2) != frame_size.height || !data ) {
    CF_ERROR("Invalid input frame size: %dx%d", width, height);
    return false;
  }

  width = frame_size.width;
  height = frame_size.height;


  // FFmpeg contains SIMD optimizations which can sometimes read data past the supplied input buffer.
  // Related info: https://trac.ffmpeg.org/ticket/6763
  // 1. To ensure that doesn't happen, we pad the step to a multiple of 32
  // (that's the minimal alignment for which Valgrind doesn't raise any warnings).
  // 2. (dataend - SIMD_SIZE) and (dataend + SIMD_SIZE) is from the same 4k page
  const int CV_STEP_ALIGNMENT = 32;
  const size_t CV_SIMD_SIZE = 32;
  const size_t CV_PAGE_MASK = ~(4096 - 1);
  const uint8_t * const dataend = data + ((size_t)height * step);
  int status;


  if ( step % CV_STEP_ALIGNMENT != 0 ||
      (((size_t) dataend - CV_SIMD_SIZE) & CV_PAGE_MASK) != (((size_t) dataend + CV_SIMD_SIZE) & CV_PAGE_MASK) ) {

    const int aligned_step = (step + CV_STEP_ALIGNMENT - 1) & ~(CV_STEP_ALIGNMENT - 1);
    const size_t new_size = (aligned_step * height + CV_SIMD_SIZE);

    if ( !aligned_input || aligned_input_size < new_size ) {
      if ( aligned_input ) {
        av_freep(&aligned_input);
      }
      aligned_input_size = new_size;
      aligned_input = (unsigned char*) av_mallocz(aligned_input_size);
    }

    if ( origin == 1 )
      for ( int y = 0; y < height; y++ ) {
        memcpy(aligned_input + y * aligned_step, data + (height - 1 - y) * step, step);
      }
    else {
      for ( int y = 0; y < height; y++ ) {
        memcpy(aligned_input + y * aligned_step, data + y * step, step);
      }
    }

    data = aligned_input;
    step = aligned_step;
  }

  if ( codec_ctx->pix_fmt == input_pix_fmt ) {

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
    av_image_fill_arrays(output_frame->data, output_frame->linesize, data, input_pix_fmt, width, height, 1);
#else
    avpicture_fill((AVPicture*)output_frame, data, input_pix_fmt, width, height);
#endif

    output_frame->linesize[0] = step;
  }
  else {

    if ( !input_frame ) {
      CF_ERROR("APP BUG: input_picture is null");
      return false;
    }

    // let input_picture point to the raw data buffer of 'image'
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
    av_image_fill_arrays(input_frame->data, input_frame->linesize, (uint8_t *) data, input_pix_fmt, width, height, 1);
#else
    avpicture_fill((AVPicture*)input_frame, data, input_pix_fmt, width, height);
#endif

    input_frame->linesize[0] = step;

    if ( !sws_ctx ) {
      sws_ctx = sws_getContext(width,
          height,
          input_pix_fmt,
          codec_ctx->width,
          codec_ctx->height,
          codec_ctx->pix_fmt,
          SWS_BICUBIC,
          NULL, NULL, NULL);
      if ( !sws_ctx ) {
        CF_ERROR("sws_getContext() fails");
        return false;
      }
    }


    status =
        sws_scale(sws_ctx,
            input_frame->data,
            input_frame->linesize, 0,
            height,
            output_frame->data,
            output_frame->linesize);

    if ( status < 0 ) {
      CF_ERROR("sws_scale() fails : %s",
          averr2str(status));
      return false;
    }
  }

  output_frame->pts = pts;
  if ( (status = encode_and_send_frame(output_frame)) < 0 ) {
    CF_ERROR("ffmpeg_write_frame() fails: %s",
        averr2str(status));
  }

  return status >= 0 ;
}



