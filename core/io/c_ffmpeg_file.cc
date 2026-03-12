/*
 * c_ffmpeg_video_reader.cc
 *
 *  Created on: Feb 3, 2020
 *      Author: amyznikov
 */

#include "c_ffmpeg_file.h"

#if HAVE_FFMPEG

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

static void ensure_ffmpeg_initialized()
{
  static const bool dummy = []() {
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    av_register_all();
#endif

#if HAVE_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();

    av_log_set_level(AV_LOG_ERROR);
    av_log_set_callback(av_log_callback);
    return true;
  }();
}


// Initialization is protected by the C++11 standard (thread-safe static)
static const std::vector<std::string> g_supported_input_formats = []()
{
  std::vector<std::string> tmp;
  const AVInputFormat * iformat = nullptr;
  void * opaque = nullptr;
  const char delims[] = ", \t";

  ensure_ffmpeg_initialized();

  while ((iformat = av_demuxer_iterate(&opaque))) {
    if( iformat->name ) {
      if( !iformat->extensions ) {
        tmp.emplace_back(std::string(".") + iformat->name);
      }
      else {
        char buf[strlen(iformat->extensions) + 1];
        char * tok = strtok(strcpy(buf, iformat->extensions), delims);
        while ( tok ) {
          tmp.emplace_back(std::string(".") + tok);
          tok = strtok(nullptr, delims);
        }
      }
    }
  }

  // for unclear reason the 'ogv' suffix is missing in archlinux version of ffmpeg
  tmp.emplace_back(".ogv");
  std::sort(tmp.begin(), tmp.end());
  tmp.erase(std::unique(tmp.begin(), tmp.end()), tmp.end());
  return tmp;
}();

static const std::vector<std::string> g_supported_output_formats = []()
{
  std::vector<std::string> tmp;
  const AVOutputFormat * oformat = nullptr;
  void * opaque = nullptr;
  const char delims[] = ", \t";

  ensure_ffmpeg_initialized();

  while ( (oformat = av_muxer_iterate(&opaque)) ) {
    if ( oformat->name ) {
      if ( !oformat->extensions ) {
        tmp.emplace_back(oformat->name);
      }
      else {
        char buf[strlen(oformat->extensions) + 1];
        char * tok = strtok(strcpy(buf, oformat->extensions), delims);
        while ( tok ) {
          tmp.emplace_back(tok);
          tok = strtok(NULL, delims);
        }
      }
    }
  }

  std::sort(tmp.begin(), tmp.end());
  tmp.erase(std::unique(tmp.begin(), tmp.end()), tmp.end());
  return tmp;
}();


static const std::vector<std::string> g_supported_video_decoders = []()
{
  std::vector<std::string> tmp;
  std::vector<const AVCodecDescriptor*> decodecs;
  const AVCodecDescriptor * desc = nullptr;

  ensure_ffmpeg_initialized();

  while ((desc = avcodec_descriptor_next(desc))) {
    if( desc->type == AVMEDIA_TYPE_VIDEO && avcodec_find_decoder(desc->id) ) {
      decodecs.push_back(desc);
    }
  }

  std::sort(decodecs.begin(), decodecs.end(),
      [](const AVCodecDescriptor * a, const AVCodecDescriptor * b) {
        return strcasecmp(a->name, b->name) < 0;
      });

  tmp.reserve(decodecs.size());

  for( const auto * d : decodecs ) {
    char props[4];
    props[0] = (d->props & AV_CODEC_PROP_INTRA_ONLY) ? 'I' : '.';
    props[1] = (d->props & AV_CODEC_PROP_LOSSY) ? 'L' : '.';
    props[2] = (d->props & AV_CODEC_PROP_LOSSLESS) ? 'S' : '.';
    props[3] = '\0';

    tmp.emplace_back(ssprintf("%s %-12s %s",
        props,
        d->name,
        d->long_name ? d->long_name : ""));
  }

  return tmp;
}();


static const std::vector<std::string> g_supported_video_encoders = []()
{
  std::vector<std::string> tmp;
  std::vector<const AVCodecDescriptor *> encoders;
  const AVCodecDescriptor * desc = nullptr;

  ensure_ffmpeg_initialized();

  while ((desc = avcodec_descriptor_next(desc))) {
    if( desc->type == AVMEDIA_TYPE_VIDEO && avcodec_find_encoder(desc->id) ) {
      encoders.emplace_back(desc);
    }
  }

  std::sort(encoders.begin(), encoders.end(),
      [](const AVCodecDescriptor * prev, const AVCodecDescriptor * next) {
        return strcasecmp(prev->name, next->name) < 0;
      });

  for ( const AVCodecDescriptor * desc: encoders ) {
    const std::string props =
        ssprintf("%s%s%s",
            (desc->props & AV_CODEC_PROP_INTRA_ONLY) ? "I" : ".",
            (desc->props & AV_CODEC_PROP_LOSSY) ? "L" : ".",
            (desc->props & AV_CODEC_PROP_LOSSLESS) ? "S" : ".");

    tmp.emplace_back(
        ssprintf("%s %s %s",
            props.c_str(),
            desc->name,
            desc->long_name ? desc->long_name : ""));
  }

  return tmp;
}();

static const std::vector<std::string> g_supported_pix_fmts = []()
{
  // From opt_common.c :
  // int show_pix_fmts(void *optctx, const char *opt, const char *arg)
  //        printf("Pixel formats:\n"
  //               "I.... = Supported Input  format for conversion\n"
  //               ".O... = Supported Output format for conversion\n"
  //               "..H.. = Hardware accelerated format\n"
  //               "...P. = Paletted format\n"
  //               "....B = Bitstream format\n"
  //               "FLAGS NAME            NB_COMPONENTS BITS_PER_PIXEL BIT_DEPTHS\n"
  //               "-----\n");

  //    #if !CONFIG_SWSCALE
  //    #   define sws_isSupportedInput(x)  0
  //    #   define sws_isSupportedOutput(x) 0
  //    #endif

  std::vector<std::string> tmp;

  const AVPixFmtDescriptor *desc = nullptr;

  ensure_ffmpeg_initialized();

  while ((desc = av_pix_fmt_desc_next(desc))) {

    enum AVPixelFormat av_unused pix_fmt =
        av_pix_fmt_desc_get_id(desc);

    std::string s =
        ssprintf("%c%c%c%c%c %-16s       %d            %3d      %d",
            sws_isSupportedInput(pix_fmt) ? 'I' : '.',
            sws_isSupportedOutput(pix_fmt) ? 'O' : '.',
            desc->flags & AV_PIX_FMT_FLAG_HWACCEL ? 'H' : '.',
            desc->flags & AV_PIX_FMT_FLAG_PAL ? 'P' : '.',
            desc->flags & AV_PIX_FMT_FLAG_BITSTREAM ? 'B' : '.',
            desc->name,
            desc->nb_components,
            av_get_bits_per_pixel(desc),
            desc->comp[0].depth);

    for( unsigned i = 1; i < desc->nb_components; i++ ) {
      s += ssprintf("-%d", desc->comp[i].depth);
    }

    tmp.emplace_back(s);
  }

  std::sort(tmp.begin(), tmp.end());
  return tmp;
}();


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
  const AVInputFormat * fmt = nullptr;
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



static bool ffmpeg_setup_encoder_parameters(AVCodecContext * codec_ctx,
    int w, int h, double fps, AVPixelFormat pix_fmt)
{
  codec_ctx->width = w;
  codec_ctx->height = h;

  if (fps > 0) {
    // In FFmpeg 7.0 for video: time_base = 1/FPS, framerate = FPS
    codec_ctx->framerate = av_d2q(fps, 1000000);
    codec_ctx->time_base = av_inv_q(codec_ctx->framerate);
  }

  if( codec_ctx->codec ) {

    const AVRational * framerates = nullptr;

    int status = avcodec_get_supported_config(codec_ctx, nullptr,
        AV_CODEC_CONFIG_FRAME_RATE, 0,
        (const void**) &framerates, nullptr);

    if( status >= 0 && framerates ) {

      AVRational req = codec_ctx->framerate;
      const AVRational * best = nullptr;
      double best_diff = 1e10;

      for( const AVRational * p = framerates; p->den != 0; p++ ) {
        double diff = fabs(av_q2d(req) - av_q2d(*p));
        if( diff < best_diff ) {
          best_diff = diff;
          best = p;
        }
      }

      if( best ) {
        codec_ctx->framerate = *best;
        codec_ctx->time_base = av_inv_q(*best);
      }
    }
  }

  // Check: if the quality is NOT set and the bitrate is NOT set yet (via opts)
  if (!(codec_ctx->flags & AV_CODEC_FLAG_QSCALE) && codec_ctx->bit_rate == 0) {
    // A factor of 0.1 - 0.15 usually gives acceptable quality for h264
    int64_t calculated_br = (int64_t)(fps * w * h * 0.15);
    // Limit it to some reasonable (for example, no more than 50 Mbps for regular video)
    codec_ctx->bit_rate = std::min(calculated_br, (int64_t)50000000);
    // bit_rate_tolerance is often ignored in FFmpeg 7.0, but is useful for older codecs
    codec_ctx->bit_rate_tolerance = (int)codec_ctx->bit_rate;
  }


  if( pix_fmt != AV_PIX_FMT_NONE ) {
    codec_ctx->pix_fmt = pix_fmt;
  }
  else if( codec_ctx->codec ) {
    const enum AVPixelFormat * pix_fmts = nullptr;
    int status = avcodec_get_supported_config(codec_ctx, nullptr,
        AV_CODEC_CONFIG_PIX_FORMAT, 0,
        (const void**) &pix_fmts, nullptr);

    if( status >= 0 && pix_fmts && pix_fmts[0] != AV_PIX_FMT_NONE ) {
      codec_ctx->pix_fmt = pix_fmts[0];
    }
  }

  codec_ctx->qmin = 3;

  return true;
}

///* Open the encoder.
static AVCodecContext * ffmpeg_open_encoder(AVCodecID codec_id,
    int w, int h, AVPixelFormat pix_fmt, double fps,
    int flags,
    int qscale,
    AVDictionary ** opts = nullptr)
{
  const AVCodec * codec = nullptr;
  AVCodecContext * codec_ctx = nullptr;
  int status;

  if ( !(codec = avcodec_find_encoder(codec_id)) ) {
    CF_ERROR("avcodec_find_encoder() fails for codec_id=%d", codec_id);
    return nullptr;
  }

  if ( !(codec_ctx = avcodec_alloc_context3(codec)) ) {
    CF_ERROR("avcodec_alloc_context3() fails");
    return nullptr;
  }

  // Flags (Global Header is needed for MP4)
  codec_ctx->flags |= flags;
  if ( qscale >= 0 ) {
    codec_ctx->flags |= AV_CODEC_FLAG_QSCALE;
    codec_ctx->global_quality = FF_QP2LAMBDA * qscale;
  }

  ffmpeg_setup_encoder_parameters(codec_ctx, w, h, fps, pix_fmt);

  // Open the codec, avcodec_open2 can modify the opts dictionary, removing found options.
  if ( (status = avcodec_open2(codec_ctx, codec, opts)) < 0 ) {
    CF_ERROR("avcodec_open2() fails: %s", averr2str(status));
    avcodec_free_context(&codec_ctx);
    return nullptr;
  }

  return codec_ctx;
}

/**
 * the following function is a modified version of code
 * found in ffmpeg-0.4.9-pre1/output_example.c
 */
static AVFrame * ffmpeg_alloc_frame(AVPixelFormat pix_fmt, int width, int height, bool alloc_buffer)
{
  AVFrame * picture = av_frame_alloc();
  if (!picture) {
    CF_ERROR("av_frame_alloc() fails");
    return nullptr;
  }

  picture->format = pix_fmt;
  picture->width  = width;
  picture->height = height;

  if ( alloc_buffer ) {
    // The second number (alignment) is 0, which means auto-selection (usually 32 or 64 bytes).
    // This replaces av_malloc + av_image_fill_arrays
    int status = av_frame_get_buffer(picture, 0);
    if (status < 0) {
      CF_ERROR("av_frame_get_buffer() fails: %s", averr2str(status));
      av_frame_free(&picture);
      return nullptr;
    }
  }

  return picture;
}

///////////////////////////////////////////////////////////////////////////////

const std::vector<std::string>& c_ffmpeg_reader::supported_input_formats()
{
  return g_supported_input_formats;
}

const std::vector<std::string>& c_ffmpeg_reader::supported_decoders()
{
  return g_supported_video_decoders;
}

const std::vector<std::string> & c_ffmpeg_reader::supported_pixel_formats()
{
  return g_supported_pix_fmts;
}

c_ffmpeg_reader::~c_ffmpeg_reader()
{
  close();
}

void c_ffmpeg_reader::set_stream_name(const std::string & v)
{
  _stream_name = v;
}

const std::string & c_ffmpeg_reader::stream_name() const
{
  return _stream_name;
}

void c_ffmpeg_reader::set_rcvtmo(int64_t v)
{
  _rcvtmo = v;
}

int64_t c_ffmpeg_reader::rcvtmo() const
{
  return _rcvtmo;
}

cv::Size c_ffmpeg_reader::coded_size() const
{
  return _cctx ? cv::Size(_cctx->coded_width, _cctx->coded_height) : cv::Size(0, 0);
}

void c_ffmpeg_reader::set_frame_size(const cv::Size &size)
{
  _scaledSize = size;
}

cv::Size c_ffmpeg_reader::frame_size() const
{
  return _scaledSize.empty() ? coded_size() : _scaledSize;
}

bool c_ffmpeg_reader::is_open() const
{
  return _ic != nullptr;
}

double c_ffmpeg_reader::duration() const
{
  if( !_ic ) {
    return 0;
  }

  // The duration in the container is stored in microseconds (AV_TIME_BASE)
  if( _ic->duration != AV_NOPTS_VALUE ) {
    return (double) _ic->duration / AV_TIME_BASE;
  }

  // If there is no container, we look at the duration of the stream itself
  if( _istream && _istream->duration != AV_NOPTS_VALUE ) {
    return _istream->duration * av_q2d(_istream->time_base);
  }

  return 0;
}

double c_ffmpeg_reader::fps() const
{
  if( !_istream ) {
    return 0;
  }

  // avg_frame_rate is the standard for FFmpeg 7.x
  if( _istream->avg_frame_rate.den > 0 && _istream->avg_frame_rate.num > 0 ) {
    return av_q2d(_istream->avg_frame_rate);
  }

  // Fallback to r_frame_rate if average is not defined
  if( _istream->r_frame_rate.den > 0 && _istream->r_frame_rate.num > 0 ) {
    return av_q2d(_istream->r_frame_rate);
  }

  return 25.0; // Default if nothing is known at all
}

int c_ffmpeg_reader::num_frames() const
{
  if( !_istream ) {
    return 0;
  }

  // Try to take a direct value from the stream
  if( _istream->nb_frames > 0 ) {
    return (int) _istream->nb_frames;
  }

  // If the stream doesn't know, we ask the container (sometimes MKV writes this in the header)
  if( _ic && _ic->nb_streams > _video_stream_index && _ic->streams[_video_stream_index]->nb_frames > 0 ) {
    return (int) _ic->streams[_video_stream_index]->nb_frames;
  }

  // Fallback: calculated based on duration (for VFR this will only be an estimate!)
  const double d = duration();
  if( d > 0 ) {
    return (int) (d * fps());
  }

  return 0;
}

bool c_ffmpeg_reader::open(const std::string & url, const std::string & input_options)
{
  AVDictionary * opts = nullptr;
  AVDictionary * codec_opts = nullptr;
  int status = 0;

  _video_stream_index = -1;
  _last_ts = -1; // AV_NOPTS_VALUE; // Use the standard FFmpeg constant ?
  _frames_read = 0;

  ensure_ffmpeg_initialized();

  if ( (status = ffmpeg_parse_options(input_options, true, &opts)) ) {
    CF_ERROR("[%s] ffmpeg_parse_options() fails: %s", _stream_name.c_str(), averr2str(status));
    goto end;
  }

  if ( opts ) {
    av_dict_copy(&codec_opts, opts, 0);
  }

  ffmpeg_set_timeout_interrupt_callback(&_tcb, _rcvtmo);
  if ( (status = ffmpeg_open_input(&_ic, url.c_str(), &_tcb.icb, &opts)) ) {
    CF_ERROR("[%s] ffmpeg_open_input('%s') fails: %s", _stream_name.c_str(), url.c_str(), averr2str(status));
    goto end;
  }

  _video_stream_index = av_find_best_stream(_ic, AVMEDIA_TYPE_VIDEO, -1, -1, &_codec, 0);
  if ( _video_stream_index < 0 ) {
    CF_ERROR("[%s] av_find_best_stream(AVMEDIA_TYPE_VIDEO) fails: %d", _stream_name.c_str(), _video_stream_index);
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  _istream = _ic->streams[_video_stream_index];
  if ( !_istream->codecpar ) {
    CF_ERROR("[%s] stream->codecpar is null", _stream_name.c_str());
    status = AVERROR_STREAM_NOT_FOUND;
    goto end;
  }

  if ( !_codec && !(_codec = avcodec_find_decoder(_istream->codecpar->codec_id)) ) {
    CF_ERROR("[%s] avcodec_find_decoder(codec_id=%d) fails", _stream_name.c_str(), _istream->codecpar->codec_id);
    status = AVERROR_DECODER_NOT_FOUND;
    goto end;
  }

  if ( !(_cctx = avcodec_alloc_context3(_codec)) ) {
    CF_ERROR("[%s] avcodec_alloc_context3(codec=%s) fails", _stream_name.c_str(), _codec->name);
    status = AVERROR(ENOMEM);
    goto end;
  }

  if ( (status = avcodec_parameters_to_context(_cctx, _istream->codecpar)) < 0 ) {
    CF_ERROR("[%s] avcodec_parameters_to_context(codec=%s) fails: %s", _stream_name.c_str(), _codec->name,
        averr2str(status));
    goto end;
  }

  // FFmpeg 7.0+: Set thread safety (optional, but may be useful)
  _cctx->thread_count = 0; // automatic selection of the number of threads
  _cctx->thread_type = FF_THREAD_SLICE;

  if ( (status = avcodec_open2(_cctx, _codec, &codec_opts)) < 0 ) {
    CF_ERROR("[%s] avcodec_open2() fails for codec=%s : %s", _stream_name.c_str(), _codec->name,
        averr2str(status));
    goto end;
  }

  if( !(_avpacket = av_packet_alloc()) ) {
    CF_ERROR("av_packet_alloc() fails - out of memory?");
    status = AVERROR(ENOMEM);
    goto end;
  }

  if ( !(_avframe = av_frame_alloc())) {
    CF_ERROR("av_frame_alloc() fails - out of memory?");
    status = AVERROR(ENOMEM);
    goto end;
  }

  _timescale = (_istream->time_base.num > 0 ? (double) _istream->time_base.den / _istream->time_base.num : 1);

  CF_DEBUG("[%s] %d:%s %dx%d %d/%d tbn _istream->nb_frames=%lld _ic->nb_streams=%lld", url.c_str(),
      _video_stream_index,
      _codec ? _codec->name : "null",
      _cctx ? _cctx->coded_width : 0,
      _cctx ? _cctx->coded_height : 0,
      _istream ?  _istream->time_base.num : 0,
      _istream? _istream->time_base.den : 0,
      _istream ? _istream->nb_frames : 0LL,
      _ic ? _ic->nb_streams : 0LL);

end:
  if ( codec_opts ) {
    av_dict_free(&codec_opts);
  }

  if ( status < 0 ) {
    close();
  }

  return status >= 0;
}

void c_ffmpeg_reader::close()
{
  if (_cctx) {
    avcodec_free_context(&_cctx);
  }

  if ( _ic ) {
    avformat_close_input(&_ic);
  }

  av_packet_free(&_avpacket);
  av_frame_free(&_avframe);
  av_frame_free(&_rgbpicture);

  if ( _swsctx ) {
    sws_freeContext(_swsctx);
    _swsctx = nullptr;
  }

  _received_frames.clear();

  _cctx = nullptr;
  _ic = nullptr;
  _istream = nullptr;
  _codec = nullptr;
  _video_stream_index = -1;
  _last_ts = -1;
  _frames_read = 0;
}

const AVStream * c_ffmpeg_reader::stream() const
{
  return _istream;
}

bool c_ffmpeg_reader::seek_frame(int64_t pts)
{
  if ( !_istream || _istream->avg_frame_rate.num <= 0 || _istream->time_base.num <= 0 ) {
    return false;
  }

  // Clear the queue of already decoded frames
  if( !_received_frames.empty() && pts >= _received_frames.front().pts
      && pts <= _received_frames.back().pts ) {

    while (!_received_frames.empty() && pts > _received_frames.front().pts) {
      _received_frames.pop_front();
    }
    while (!_received_frames.empty() && pts < _received_frames.back().pts) {
      _received_frames.pop_back();
    }

    if( !_received_frames.empty() ) {
      return true;
    }
  }

  _received_frames.clear();

  const int64_t start_time = _istream->start_time > 0 ? _istream->start_time : 0;
  const int64_t den = ((int64_t) _istream->avg_frame_rate.num * _istream->time_base.num);
  const int64_t timestamp = start_time + pts * ((int64_t) _istream->time_base.den * _istream->avg_frame_rate.den) / std::max(den, int64_t(1));
  int status = av_seek_frame(_ic, _video_stream_index, timestamp, AVSEEK_FLAG_BACKWARD);
  if ( status >= 0 ) {
    _last_ts = timestamp;
    avcodec_flush_buffers(_cctx);
  }
  else  {
    CF_ERROR("av_seek_frame(video_stream_index=%d frame_index=%d timestamp=%lld start_time=%lld stream->time_base=%d/%d) fails: "
        "status=%d (%s)",
        _video_stream_index, pts, (long long )(timestamp), (long long )(start_time),
        _istream->time_base.num, _istream->time_base.den,
        status, averr2str(status));
  }

  return status >= 0;
}

int64_t c_ffmpeg_reader::curpos() const
{
  if( !_received_frames.empty() ) {
    return _received_frames.front().pts;
  }

  int64_t pos = 0;
  if ( _istream ) {
    const double start_time = _istream->start_time > 0 ? _istream->start_time : 0;
    pos = _last_ts - start_time + 1;
  }
  return pos;
}

bool c_ffmpeg_reader::read(cv::Mat & outframe, double * out_pts)
{
  int status;

  if( !_received_frames.empty() ) {
    auto & f = _received_frames.front();
    outframe = std::move(f.image);
    if( out_pts ) {
      *out_pts = _timescale * _received_frames.front().pts;
    }
    _received_frames.pop_front();
    return true;
  }

  // Initializing the buffer for conversion (RGB/BGR)
  if( !_rgbpicture || _rgbpicture->width != (_scaledSize.empty() ? _cctx->width : _scaledSize.width) ) {

    if( _rgbpicture ) {
      av_frame_free(&_rgbpicture);
    }

    if( !(_rgbpicture = av_frame_alloc()) ) {
      CF_ERROR("av_frame_alloc() fails - out of memory?");
      return false;
    }

    _rgbpicture->width = _scaledSize.empty() ? _cctx->width : _scaledSize.width;
    _rgbpicture->height = _scaledSize.empty() ? _cctx->height : _scaledSize.height;
    _rgbpicture->format = AV_PIX_FMT_BGR24;

    if( (status = av_frame_get_buffer(_rgbpicture, 32)) < 0 ) {
      CF_ERROR("av_frame_get_buffer() fails : %s", averr2str(status));
      return false;
    }
  }

  ffmpeg_set_timeout_interrupt_callback(&_tcb, _rcvtmo);

  // Loop to receive all available frames (the decoder can output several at a time)
  const auto receive_frames =
      [this]() {
        int status;
        while ( (status = avcodec_receive_frame(_cctx, _avframe)) >= 0 ) {

          // Update the scaling context when the stream parameters change
          _swsctx = sws_getCachedContext(_swsctx,
              _avframe->width,   // Take it from the frame, not from the codec context!
              _avframe->height,
              (AVPixelFormat) _avframe->format,
              _rgbpicture->width,
              _rgbpicture->height,
              (AVPixelFormat) _rgbpicture->format,
              SWS_FAST_BILINEAR,
              nullptr, nullptr, nullptr);

          if( !_swsctx ) {
            CF_ERROR("sws_getCachedContext() fails");
            status = AVERROR(ENOMEM);
            break;
          }

          sws_scale(_swsctx, _avframe->data, _avframe->linesize, 0, _avframe->height,
              _rgbpicture->data, _rgbpicture->linesize);

          _received_frames.emplace_back();
          cv::Mat(_rgbpicture->height, _rgbpicture->width, CV_8UC3,
              _rgbpicture->data[0], _rgbpicture->linesize[0]).copyTo(_received_frames.back().image);

          // PTS
          const int64_t pts = (_avframe->best_effort_timestamp != AV_NOPTS_VALUE) ? _avframe->best_effort_timestamp : _avframe->pts;
          _received_frames.back().pts = _last_ts = pts;

          av_frame_unref(_avframe);
        }

        return status;
    };

  while ( 42 ) {

    if ( (status = av_read_frame(_ic, _avpacket)) == AVERROR(EAGAIN) ) {
      continue;
    }

    if ( status < 0 ) {
      if( status != AVERROR_EOF ) {
        CF_ERROR("av_read_frame() fails: %s", averr2str(status));
      }
      else {
        avcodec_send_packet(_cctx, nullptr);
        if ( (status = receive_frames()) < 0 && status != AVERROR_EOF ) {
          CF_ERROR("receive_frames() fails: %s", averr2str(status));
        }
      }
      break;
    }

    if ( _avpacket->stream_index != _video_stream_index ) {
      av_packet_unref(_avpacket);
      continue;
    }

    status = avcodec_send_packet(_cctx, _avpacket);
    av_packet_unref(_avpacket);

    if ( status < 0 ) {
      CF_ERROR("avcodec_send_packet() fails: %s", averr2str(status));
      if ( status == AVERROR_EOF || status == AVERROR(EINVAL) || status == AVERROR(ENOMEM) ) {
        CF_ERROR("break");
        break;
      }
      continue;
    }

    status = receive_frames();

    if ( status == AVERROR(EAGAIN) && _received_frames.empty() ) {
      ffmpeg_set_timeout_interrupt_callback(&_tcb, _rcvtmo);
      continue;
    }

    if ( _received_frames.empty() ) {
      CF_ERROR("avcodec_receive_frame() fails: %s", averr2str(status));
    }

    break;
  }

  if ( !_received_frames.empty() ) {
    outframe = std::move(_received_frames.front().image);
    if ( out_pts ) {
      *out_pts = _timescale * _received_frames.front().pts;
    }
    _received_frames.erase(_received_frames.begin());
    return true;
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////

const std::vector<std::string> & c_ffmpeg_writer::supported_output_formats()
{
  return g_supported_output_formats;
}

const std::vector<std::string> & c_ffmpeg_writer::supported_encoders()
{
  return g_supported_video_encoders;
}

const std::vector<std::string> & c_ffmpeg_writer::supported_pixel_formats()
{
  ensure_ffmpeg_initialized();
  return g_supported_pix_fmts;
}

c_ffmpeg_writer::~c_ffmpeg_writer()
{
  close();
}

const AVPixelFormat * c_ffmpeg_writer::supported_codec_pix_formats(AVCodecID codec_id)
{
  ensure_ffmpeg_initialized();

  const AVPixelFormat * pix_fmts = nullptr;

  const AVCodec * codec = avcodec_find_encoder(codec_id);
  if (codec) {
    avcodec_get_supported_config(nullptr,
        codec,
        AV_CODEC_CONFIG_PIX_FORMAT,
        0,
        (const void**)&pix_fmts,
        nullptr);
  }

  return pix_fmts;
}

///@brief open (create) video file
bool c_ffmpeg_writer::open(const std::string & output_filename,
    const cv::Size & image_size, bool is_color,
    const std::string & ffmpeg_opts)
{
  std::string output_directory;
  AVDictionary * opts = nullptr;

  const AVOutputFormat * ofmt = nullptr;

  AVCodecID codec_id = AV_CODEC_ID_NONE;
  AVPixelFormat codec_pix_fmt = AV_PIX_FMT_NONE;
  double fps = -1;
  int qscale = -1;

  bool need_color_convert = false;

  int status = 0;
  bool fOk = false;

  ensure_ffmpeg_initialized();
  close();

  _frames_written = 0;
  _start_pts = 0;


  /*
   * Get output file name
   *  */
  _opts = ffmpeg_opts;
  if ( (_output_filename = output_filename).empty() ) {
    CF_ERROR("No output file name specified, can not create output file");
    return false;
  }

  /*
   * Select input pixel format
   * */
  _input_pix_fmt =
      is_color ? AV_PIX_FMT_BGR24 :
          AV_PIX_FMT_GRAY8;


  /*
   * Like to OpenCV:
   *  we allow frames of odd width or height, but in this case we truncate
   *  the rightmost column/the bottom row. Probably, this should be handled more elegantly,
   *  but some internal functions inside FFMPEG swscale require even width/height.
   */
  _frame_size.width = image_size.width & -2;
  _frame_size.height = image_size.height & -2;

  if ( _frame_size.width <= 0 || _frame_size.height <= 0 ) {
    CF_ERROR("Invalid output frame size specified : %dx%d", _frame_size.width, _frame_size.height);
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

    std::string suffix = get_file_suffix(_output_filename);

    if ( suffix.empty() && ofmt->extensions ) {

      std::vector<std::string> suffixes =
          strsplit(ofmt->extensions, " ,");

      if ( !suffixes.empty() ) {
        (_output_filename += '.') += suffixes[0];
      }
    }
  }


  if ( !ofmt && !(ofmt = av_guess_format(nullptr, _output_filename.c_str(), nullptr)) ) {
    CF_ERROR("Could not deduce output format from file extension for '%s'\n",
        _output_filename.c_str());
    goto end;
  }

  output_directory = get_parent_directory(_output_filename);
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

  _codec_ctx =
      ffmpeg_open_encoder(codec_id,
          _frame_size.width,
          _frame_size.height,
          codec_pix_fmt,
          fps,
          (ofmt->flags & AVFMT_GLOBALHEADER) ? AV_CODEC_FLAG_GLOBAL_HEADER : 0,
          qscale,
          &opts);

  if ( !_codec_ctx ) {
    CF_ERROR("ffmpeg_open_encoder() fails");
    goto end;
  }


  /*
   * Setup output context and and new video stream.
   *  Based on ffmpeg/doc/examples/transcoding.c
   * */
  status = avformat_alloc_output_context2(&_octx, ofmt, nullptr, _output_filename.c_str());
  if ( status < 0 || !_octx ) {
    CF_ERROR("avformat_alloc_output_context2() fails : %s\n",averr2str(status));
    goto end;
  }

  /*
  * Create a new video stream.
  * In FFmpeg 7.0, the second argument is ALWAYS nullptr.
  */
  if ( !(_ostream = avformat_new_stream(_octx, nullptr/*_codec_ctx->codec*/)) ) {
    CF_ERROR("avformat_new_stream() fails\n");
    goto end;
  }

  status = avcodec_parameters_from_context(_ostream->codecpar, _codec_ctx);
  if ( status < 0 ) {
    CF_ERROR("avcodec_parameters_from_context() fails: %s\n", averr2str(status));
    goto end;
  }

  /*
  * Synchronizing time bases.
  * We've already set the time_base in the encoder (e.g., 1/25).
  * Additionally some muxers require setting avg_frame_rate on the stream
  */
  _ostream->time_base = _codec_ctx->time_base;
  _ostream->avg_frame_rate = _codec_ctx->framerate;


  /*
   * Allocate AVFrames
   * */

  _output_frame =
      ffmpeg_alloc_frame(_codec_ctx->pix_fmt,
          _codec_ctx->width,
          _codec_ctx->height,
          _codec_ctx->pix_fmt != _input_pix_fmt);

  if ( !_output_frame ) {
    CF_ERROR("ffmpeg_alloc_frame(output_frame) fails");
    goto end;
  }

  if ( _codec_ctx->pix_fmt != _input_pix_fmt ) {

    _input_frame =
        ffmpeg_alloc_frame(_input_pix_fmt,
            _codec_ctx->width,
            _codec_ctx->height,
            false);

    if ( !_input_frame ) {
      CF_ERROR("ffmpeg_alloc_frame(input_frame) fails");
      goto end;
    }
  }

  if( !(_encoded_pkt = av_packet_alloc()) ) {
    CF_ERROR("av_packet_alloc(_encoded_pkt) fails - out of memory?");
    status = AVERROR(ENOMEM);
    goto end;
  }

  /**
   * Create output file, init muxer,
   * write output file header
   */

  if ( !(_octx->oformat->flags & AVFMT_NOFILE) ) {
    if ( (status = avio_open(&_octx->pb, _output_filename.c_str(), AVIO_FLAG_WRITE)) < 0 ) {
      CF_ERROR("avio_open('%s') fails : %s", _output_filename.c_str(), averr2str(status));
      goto end;
    }
  }


  if ( (status = avformat_write_header(_octx, &opts)) < 0 ) {
    CF_ERROR("avformat_write_header() fails: %s\n", averr2str(status));
    goto end;
  }


  CF_DEBUG("c_ffmpeg_writer: pix_fmt=%d (%s) codec.time_base={%d/%d} stream.time_base={%d/%d}",
      _codec_ctx->pix_fmt,
      av_get_pix_fmt_name(_codec_ctx->pix_fmt),
      _codec_ctx->time_base.num,
      _codec_ctx->time_base.den,
      _ostream->time_base.num,
      _ostream->time_base.den );

  ////////////////////////////////////

  av_dict_free(&opts);
  _header_written = true;
  fOk = true;

end:
  if ( !fOk ) {
    close();
  }

  return fOk;
}

const AVStream * c_ffmpeg_writer::stream() const
{
  return _ostream;
}

///@brief return true if video file is open
bool c_ffmpeg_writer::is_open() const
{
  return _octx != nullptr;
}


///@brief close current output file
void c_ffmpeg_writer::close()
{
  if ( _octx ) {

    if ( _header_written  ) {

      /* flush encoder */
      int status = encode_and_send_frame(nullptr);
      //  while ( status == 0 ) {
      //    status = encode_and_send_frame(nullptr);
      //  }

      if ( status <= 0 && status != AVERROR_EOF ) {
        CF_ERROR("encode_and_send_frame(): status = %d (%s)",
            status, averr2str(status));
      }

      /* write trailer */
      if ( (status = av_write_trailer(_octx)) < 0 ) {
        CF_ERROR("av_write_trailer() fails: status=%d (%s)",
            status, averr2str(status));
      }
    }

    /* close the output file */
    if ( _octx->oformat && !(_octx->oformat->flags & AVFMT_NOFILE) ) {
      avio_close(_octx->pb);
    }
  }

  if (_codec_ctx) {
    avcodec_free_context(&_codec_ctx);
  }

  /* free the context */
  if ( _octx ) {
    avformat_free_context(_octx);
    _octx = nullptr;
  }

  if ( _output_frame ) {
    av_frame_free(&_output_frame);
  }

  if ( _input_frame ) {
    av_frame_free(&_input_frame);
  }

  if (_encoded_pkt) {
    av_packet_free(&_encoded_pkt);
  }

  if ( _swsctx ) {
    sws_freeContext(_swsctx);
    _swsctx = nullptr;
  }

  _start_pts = 0;
}


bool c_ffmpeg_writer::write(const cv::Mat & image, int64_t pts)
{
  if ( !is_open() ) {
    CF_ERROR("c_ffmpeg_writer: output stream is not open");
    return false;
  }

  if ( image.empty() ) {
    CF_ERROR("image to write is empty");
    return false;
  }

  const int cn = image.channels();
  if (_input_pix_fmt == AV_PIX_FMT_BGR24 && cn != 3) {
    CF_ERROR("Invalid input: Mat has %d channels, but 3 (BGR24) expected", cn);
    return false;
  }

  if (_input_pix_fmt == AV_PIX_FMT_GRAY8 && cn != 1) {
    CF_ERROR("Invalid input: Mat has %d channels, but 1 (GRAY8) expected", cn);
    return false;
  }

  cv::Mat F;
  if( image.cols == _frame_size.width && image.rows == _frame_size.height ) {
    F = image;
  }
  else if( image.cols >= _frame_size.width && image.rows >= _frame_size.height ) {
    F = image(cv::Rect(0, 0, _frame_size.width, _frame_size.height));
  }
  else {
    CF_ERROR("Invalid input size: %dx%d, but %dx%d expected",
        image.cols, image.rows, _frame_size.width, _frame_size.height);
    return false;
  }


  if( !_frames_written ) {
    _start_pts = pts;
    _ppts = _start_pts - 1;
  }

  if( (pts -= _start_pts) <= _ppts ) {
    pts = _ppts + 1;
  }

  _ppts = pts;

  _swsctx = sws_getCachedContext(_swsctx,
      F.cols, F.rows,
      _input_pix_fmt,
      _codec_ctx->width,
      _codec_ctx->height
      , _codec_ctx->pix_fmt,
      SWS_BILINEAR,
      nullptr, nullptr, nullptr);

  if (!_swsctx) {
    CF_ERROR("ERROR: sws_getCachedContext() fails");
    return false;
  }

  const uint8_t* src_slice[] = { F.data, nullptr, nullptr, nullptr };
  int src_stride[] = { (int)F.step, 0, 0, 0 };

  sws_scale(_swsctx, src_slice, src_stride, 0, F.rows,
      _output_frame->data, _output_frame->linesize);

  // Setting the PTS
  // IMPORTANT: The PTS must be in _codec_ctx->time_base units
  _output_frame->pts = pts;

  return encode_and_send_frame(_output_frame) >= 0;
}

const std::string & c_ffmpeg_writer::filename() const
{
  return _output_filename;
}

const std::string & c_ffmpeg_writer::opts() const
{
  return _opts;
}

int c_ffmpeg_writer::encode_and_send_frame(AVFrame * picture)
{
  int status;

  // Send the frame to the codec (nullptr means "flush" at the end of the file)
  if( (status = avcodec_send_frame(_codec_ctx, picture)) < 0 ) {
    if( picture != nullptr || status != AVERROR_EOF ) {
      CF_ERROR("avcodec_send_frame(picture=%p) fails: %s", picture, averr2str(status));
    }
    return status;
  }

  while (status >= 0) {

    // Try to get a packet of compressed data
    status = avcodec_receive_packet(_codec_ctx, _encoded_pkt);
    if( status == AVERROR(EAGAIN) || status == AVERROR_EOF ) {
      return 0; // The codec wants more frames or the data is out
    }
    else if( status < 0 ) {
      CF_ERROR("avcodec_receive_packet() fails: %s", averr2str(status));
      return status;
    }

    // CRITICAL for FFmpeg 7.0: Scaling PTS from the codec base to the stream base
    // Bases may differ (for example, 1/25 and 1/1000)
    av_packet_rescale_ts(_encoded_pkt, _codec_ctx->time_base, _ostream->time_base);
    _encoded_pkt->stream_index = _ostream->index;

    // Write the packet to the container
    status = av_interleaved_write_frame(_octx, _encoded_pkt);

    // We must free the packet, since receive_packet allocates its internals
    av_packet_unref(_encoded_pkt);

    if( status < 0 ) {
      CF_ERROR("av_interleaved_write_frame() fails: %s", averr2str(status));
      return status;
    }
    _frames_written++;
  }
  return 0;
}

///////////////////////////////////////////////////////////////////////////////
#if 0
c_ffmpeg_encoder::c_ffmpeg_encoder()
{

}

c_ffmpeg_encoder::~c_ffmpeg_encoder()
{
  close();
}

const std::string & c_ffmpeg_encoder::opts() const
{
  return _opts;
}

bool c_ffmpeg_encoder::is_open() const
{
  return _codec_ctx != nullptr;
}

bool c_ffmpeg_encoder::create(const cv::Size & image_size, bool is_color, const std::string & ffmpeg_opts)
{
  std::string output_directory;
  AVDictionary * opts = nullptr;

  AVCodecID codec_id = AV_CODEC_ID_NONE;
  AVPixelFormat codec_pix_fmt = AV_PIX_FMT_NONE;
  double fps = -1;
  int qscale = -1;

  bool need_color_convert = false;

  int status = 0;
  bool fOk = false;

  ensure_ffmpeg_initialized();

  /*
   * Get output file name
   *  */
  _opts = ffmpeg_opts;
  /*
   * Select input pixel format
   * */
  _input_pix_fmt =
      is_color ? AV_PIX_FMT_BGR24 :
          AV_PIX_FMT_GRAY8;


  /*
   * Like to OpenCV:
   *  we allow frames of odd width or height, but in this case we truncate
   *  the rightmost column/the bottom row. Probably, this should be handled more elegantly,
   *  but some internal functions inside FFMPEG swscale require even width/height.
   */
  _frame_size.width = image_size.width & -2;
  _frame_size.height = image_size.height & -2;

  if ( _frame_size.width <= 0 || _frame_size.height <= 0 ) {
    CF_ERROR("Invalid output frame size specified : %dx%d",
        _frame_size.width, _frame_size.height);
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
   * Auto select codec if was not specified
   * */
  if ( codec_id == AV_CODEC_ID_NONE ) {
    codec_id = AV_CODEC_ID_HUFFYUV;
  }

  /*
   * Open encoder
   * */

  _codec_ctx =
      ffmpeg_open_encoder(codec_id,
          _frame_size.width,
          _frame_size.height,
          codec_pix_fmt,
          fps,
          AV_CODEC_FLAG_GLOBAL_HEADER, /*(ofmt->flags & AVFMT_GLOBALHEADER) ? AV_CODEC_FLAG_GLOBAL_HEADER : 0,*/
          qscale,
          &opts);

  if ( !_codec_ctx ) {
    CF_ERROR("ffmpeg_open_encoder() fails");
    goto end;
  }

  /*
   * Allocate AVFrames
   * */

  _output_frame =
      ffmpeg_alloc_frame(_codec_ctx->pix_fmt,
          _codec_ctx->width,
          _codec_ctx->height,
          _codec_ctx->pix_fmt != _input_pix_fmt);

  if ( !_output_frame ) {
    CF_ERROR("ffmpeg_alloc_frame(output_frame) fails");
    goto end;
  }

  if ( _codec_ctx->pix_fmt != _input_pix_fmt ) {

    _input_frame =
        ffmpeg_alloc_frame(_input_pix_fmt,
            _codec_ctx->width,
            _codec_ctx->height,
            false);

    if ( !_input_frame ) {
      CF_ERROR("ffmpeg_alloc_frame(input_frame) fails");
      goto end;
    }
  }

  av_dict_free(&opts);
  fOk = true;

end:
  if ( !fOk ) {
    close();
  }

  return fOk;

}

void c_ffmpeg_encoder::close()
{
  if ( _codec_ctx ) {
#if (USE_AVCODEC_CLOSE)
    avcodec_close(_codec_ctx);
#endif
    avcodec_free_context(&_codec_ctx);
  }

  if ( _swsctx ) {
    sws_freeContext(_swsctx);
    _swsctx = nullptr;
  }

  if ( _aligned_input ) {
    av_free(_aligned_input);
    _aligned_input = nullptr;
  }

  if ( _input_frame ) {
    av_frame_free(&_input_frame);
  }

  if ( _output_frame ) {
    av_frame_free(&_output_frame);
  }

  if (_encoded_pkt) {
    av_packet_unref(_encoded_pkt);
    _encoded_pkt = nullptr;
  }
}


bool c_ffmpeg_encoder::encode(const cv::Mat & image, const writefunc & writepkt)
{
  const uint8_t * data =
      image.ptr();

  const int cn =
      image.channels();

  const int origin =
      0;

  int step =
      (int) image.step;

  int width =
      image.cols;

  int height =
      image.rows;


  // check parameters
  if ( _input_pix_fmt == AV_PIX_FMT_BGR24 ) {
    if ( cn != 3 ) {
      CF_ERROR("Invalid inpput: cn = %d but 3 expected", cn);
      return false;
    }
  }
  else if ( _input_pix_fmt == AV_PIX_FMT_GRAY8 ) {
    if ( cn != 1 ) {
      CF_ERROR("Invalid inpput: cn = %d but 1 expected", cn);
      return false;
    }
  }
  else {
    CF_ERROR("APP BUG: invalid input_pix_fmt=%d encountered", _input_pix_fmt);
    return false;
  }

  if ( (width & -2) != _frame_size.width || (height & -2) != _frame_size.height || !data ) {
    CF_ERROR("Invalid input frame size: %dx%d. Expected %dx%d", width, height,
        _frame_size.width, _frame_size.height);
    return false;
  }

  width = _frame_size.width;
  height = _frame_size.height;


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

    if ( !_aligned_input || _aligned_input_size < new_size ) {
      if ( _aligned_input ) {
        av_freep(&_aligned_input);
      }
      _aligned_input_size = new_size;
      _aligned_input = (unsigned char*) av_mallocz(_aligned_input_size);
    }

    if ( origin == 1 )
      for ( int y = 0; y < height; y++ ) {
        memcpy(_aligned_input + y * aligned_step, data + (height - 1 - y) * step, step);
      }
    else {
      for ( int y = 0; y < height; y++ ) {
        memcpy(_aligned_input + y * aligned_step, data + y * step, step);
      }
    }

    data = _aligned_input;
    step = aligned_step;
  }

  if ( _codec_ctx->pix_fmt == _input_pix_fmt ) {

#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
    av_image_fill_arrays(_output_frame->data, _output_frame->linesize, data, _input_pix_fmt, width, height, 1);
#else
    avpicture_fill((AVPicture*)_output_frame, data, _input_pix_fmt, width, height);
#endif

    _output_frame->linesize[0] = step;
  }
  else {

    if ( !_input_frame ) {
      CF_ERROR("APP BUG: input_picture is null");
      return false;
    }

    // let input_picture point to the raw data buffer of 'image'
#if LIBAVUTIL_BUILD >= (LIBAVUTIL_VERSION_MICRO >= 100  ? AV_VERSION_INT(51, 63, 100) : AV_VERSION_INT(54, 6, 0))
    av_image_fill_arrays(_input_frame->data, _input_frame->linesize, (uint8_t *) data, _input_pix_fmt, width, height, 1);
#else
    avpicture_fill((AVPicture*)_input_frame, data, _input_pix_fmt, width, height);
#endif

    _input_frame->linesize[0] = step;

    if ( !_swsctx ) {
      _swsctx = sws_getContext(width,
          height,
          _input_pix_fmt,
          _codec_ctx->width,
          _codec_ctx->height,
          _codec_ctx->pix_fmt,
          SWS_AREA,
          NULL, NULL, NULL);
      if ( !_swsctx ) {
        CF_ERROR("sws_getContext() fails");
        return false;
      }
    }


    status =
        sws_scale(_swsctx,
            _input_frame->data,
            _input_frame->linesize, 0,
            height,
            _output_frame->data,
            _output_frame->linesize);

    if ( status < 0 ) {
      CF_ERROR("sws_scale() fails : %s",
          averr2str(status));
      return false;
    }
  }

  _output_frame->pts = pts ++;

  if ( _codec_ctx->flags & AV_CODEC_FLAG_QSCALE ) {
    _output_frame->quality =
        _codec_ctx->global_quality;
  }

  if ( !_encoded_pkt ) {
    _encoded_pkt = av_packet_alloc();
  }

  int index = 0;
  status = avcodec_send_frame(_codec_ctx, _output_frame);
  while ( status >= 0 ) {

    if ( (status = avcodec_receive_packet(_codec_ctx, _encoded_pkt)) >= 0 ) {

      errno = 0;
      if( !writepkt(*_encoded_pkt, index++) ) {
        status = errno ? AVERROR(errno) : AVERROR(EIO);
        break;
      }

      continue;
    }

    if ( status == AVERROR(EAGAIN) ) {

    //      while ((status = avcodec_send_frame(codec_ctx, nullptr)) == AVERROR(EAGAIN)) {
    //        CF_DEBUG("FLUSH: status=%d (%s)", status, averr2str(status));
    //      }

      status = 0;
    }
    else if ( _output_frame || status != AVERROR_EOF ) {
      CF_ERROR("avcodec_receive_packet() fails: %s",
          averr2str(status));
    }

    break;
  }

  return status >= 0 ;
}


///////////////////////////////////////////////////////////////////////////////

c_ffmpeg_decoder::c_ffmpeg_decoder()
{
}

c_ffmpeg_decoder::~c_ffmpeg_decoder()
{
  close();
}


const std::string & c_ffmpeg_decoder::opts() const
{
  return _opts;
}

bool c_ffmpeg_decoder::is_open() const
{
  return cctx != nullptr;
}

void c_ffmpeg_decoder::close()
{

}

bool c_ffmpeg_decoder::create(const std::string & ffmpeg_opts)
{
  AVDictionary * opts = nullptr;
  AVCodecID codec_id = AV_CODEC_ID_HUFFYUV;

  int status = 0;

  ensure_ffmpeg_initialized();

  if ( (status = ffmpeg_parse_options(ffmpeg_opts, true, &opts)) ) {
    CF_ERROR("[%s] ffmpeg_parse_options() fails", averr2str(status));
    goto end;
  }

  if ( opts ) {

    AVDictionaryEntry * e;
    const int flags = AV_DICT_IGNORE_SUFFIX;

    /* check if codec specified */
    if ( (e = av_dict_get(opts, "c", nullptr, flags)) ) {
      const AVCodec * codec = avcodec_find_decoder_by_name(e->value);
      if ( codec ) {
        codec_id = codec->id;
      }
      else {
        CF_ERROR("requested decoder '%s; not found", e->value);
        goto end;
      }
    }
    if ( (e = av_dict_get(opts, "c:v", nullptr, flags)) ) {
      const AVCodec * codec = avcodec_find_decoder_by_name(e->value);
      if ( codec ) {
        codec_id = codec->id;
      }
      else {
        CF_ERROR("requested encoder '%s; not found", e->value);
        goto end;
      }
    }
  }


  if ( !(codec = avcodec_find_decoder(codec_id)) ) {
    CF_ERROR("avcodec_find_decoder(codec_id=%d) fails", codec_id);
    status = AVERROR_DECODER_NOT_FOUND;
    goto end;
  }

  if ( !(cctx = avcodec_alloc_context3(codec)) ) {
    CF_ERROR("avcodec_alloc_context3(codec=%s) fails", codec->name);
    status = AVERROR(ENOMEM);
    goto end;
  }

//  if ( (status = avcodec_parameters_to_context(cctx, istream->codecpar)) < 0 ) {
//    CF_ERROR("[%s] avcodec_parameters_to_context(codec=%s) fails: %s", stream_name_.c_str(), codec->name,
//        averr2str(status));
//    goto end;
//  }

  CF_DEBUG("cctx: width=%d height=%d", cctx->width, cctx->height);

  if ( (status = avcodec_open2(cctx, codec, &opts)) < 0 ) {
    CF_ERROR("avcodec_open2() fails for codec=%s : %s", codec->name,
        averr2str(status));
    goto end;
  }

end:

  if ( opts ) {
    av_dict_free(&opts);
  }

  if ( status ) {
    close();
  }

  return status >= 0;
}

bool c_ffmpeg_decoder::decode(const readfunc & readpkt, const writefunc & writeframe)
{
  class AVPacketPtr
  {
  public:
    AVPacketPtr() :
      pkt(av_packet_alloc())
    {
    }
    ~AVPacketPtr()
    {
      av_packet_free(&pkt);
    }

    operator AVPacket*()
    {
      return pkt;
    }

    operator const AVPacket*() const
    {
      return pkt;
    }

  protected:
    AVPacket * pkt;
  };

  AVPacketPtr pkt;

  int pktidx = 0;
  int frmidx = 0;
  int status;


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

  while ( readpkt(*pkt, pktidx++) ) {

    if( (status = avcodec_send_packet(cctx, pkt)) < 0 ) {
      CF_ERROR("avcodec_send_packet() fails: %s", averr2str(status));
      return false;
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

      const cv::Mat frame(rgbpicture->height,
          rgbpicture->width,
          CV_MAKETYPE(CV_8U, 3),
          rgbpicture->data[0],
          rgbpicture->linesize[0]);

      if ( !writeframe(frame, frmidx++) ) {
        break;
      }
    }

    if ( status < 0 && status != AVERROR(EAGAIN)  ) {
      CF_ERROR("avcodec_receive_frame() fails: %s", averr2str(status));
      return false;
    }
  }

  return true;
}
#endif



#endif // HAVE_FFMPEG
