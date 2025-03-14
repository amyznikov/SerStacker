/*
 * c_output_frame_writer.cc
 *
 *  Created on: Mar 26, 2023
 *      Author: amyznikov
 */

#include "c_output_frame_writer.h"
#include <core/io/c_input_source.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
#include <core/debug.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
c_output_frame_writer_options::c_output_frame_writer_options() :
    ffmpeg_opts(c_output_frame_writer::default_ffmpeg_opts()),
    output_pixel_depth(PIXEL_DEPTH_NO_CHANGE),
    save_frame_mapping(false)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::string c_output_frame_writer::default_ffmpeg_opts_ =
    "-r 30 -c huffyuv -f avi";
    // "-r 10 -c rawvideo -pix_fmt rgb24";

c_output_frame_writer::c_output_frame_writer()
{
}


c_output_frame_writer::~c_output_frame_writer()
{
  close();
}


bool c_output_frame_writer::create_output_frame(const cv::Mat & src, const cv::Mat & src_mask,
    cv::Mat & dst, cv::Mat & dst_mask,
    const c_image_processor::sptr & processor,
    PIXEL_DEPTH ddepth)
{
  cv::Mat tmp, tmp_mask;

  if( !processor || processor->empty() ) {
    tmp = src;
    if( src_mask.type() == CV_8UC1 && src_mask.size() == src.size() ) {
      tmp_mask = src_mask;
    }
  }
  else {

    src.copyTo(tmp);

    if( src_mask.type() == CV_8UC1 && src_mask.size() == src.size() ) {
      src_mask.copyTo(tmp_mask);
    }


    if( !processor->process(tmp, tmp_mask) ) {
      CF_ERROR("c_output_frame_writer: processor->process() fails");
      return false;
    }
  }

  if( ddepth != PIXEL_DEPTH_NO_CHANGE && ddepth != tmp.depth() ) {

    // CF_DEBUG("CONVERSION HERE");

    double scale = 1;
    double offset = 0;

    if( !get_scale_offset(tmp.depth(), (int) ddepth, &scale, &offset) ) {
      CF_ERROR("c_output_frame_writer: get_scale_offset() fails");
      return false;
    }

    tmp.convertTo(tmp, ddepth, scale, offset);
  }

  if( tmp.data != src.data ) {
    dst = std::move(tmp);
  }
  else {
    tmp.copyTo(dst);
  }

  if( tmp_mask.data != src_mask.data ) {
    dst_mask = std::move(tmp_mask);
  }
  else {
    tmp_mask.copyTo(dst_mask);
  }

  return true;
}

const std::string & c_output_frame_writer::filename() const
{
  return output_file_name;
}

const std::string & c_output_frame_writer::ffmpeg_opts() const
{
  return ffmpeg_opts_;
}

void c_output_frame_writer::set_default_ffmpeg_opts(const std::string & opts)
{
  default_ffmpeg_opts_ = opts;
}

const std::string & c_output_frame_writer::default_ffmpeg_opts()
{
  return default_ffmpeg_opts_;
}

bool c_output_frame_writer::is_open() const
{
  switch (output_type) {
    case output_type_images:
    case output_type_ser:
    case output_type_video:
      return !output_file_name.empty();
  }
  return false;
}

bool c_output_frame_writer::open(const std::string & filename, const std::string & ffmpeg_opts, bool write_frame_mapping)
{
  return open(filename, ffmpeg_opts,
      c_image_processor::sptr(),
      PIXEL_DEPTH_NO_CHANGE,
      write_frame_mapping);
}

bool c_output_frame_writer::open(const std::string & filename,
    const std::string & ffmpeg_opts,
    const c_image_processor::sptr & output_image_processor,
    PIXEL_DEPTH output_pixel_depth,
    bool write_frame_mapping)
{
  output_file_name = filename;
  ffmpeg_opts_ = ffmpeg_opts;
  output_type = output_type_unknown;
  output_image_processor_ = output_image_processor;
  output_pixel_depth_ = output_pixel_depth;
  current_frame_index = 0;
  pts = 0;

  if ( filename.empty() ) {
    CF_ERROR("c_output_frame_writer: No output file name specified");
    return false;
  }

  const std::string suffix =
      get_file_suffix(filename);

  if ( suffix.empty() ) {
    CF_ERROR("c_output_frame_writer: Can not suggest output type from empty file suffix");
    return false;
  }


  static std::vector<std::string> ser_suffixes;
  static std::vector<std::string> ffmpeg_suffixes;
  static std::vector<std::string> image_suffixes;

  if ( ser_suffixes.empty() ) {
    ser_suffixes.emplace_back(".ser");
  }

  if ( ffmpeg_suffixes.empty() ) {

    const std::vector<std::string> & ffmpeg_formats =
        c_ffmpeg_writer::supported_output_formats();

    ffmpeg_suffixes.reserve(ffmpeg_formats.size());

    for (const std::string & fmt : ffmpeg_formats ) {
      // ffmpeg_suffixes.emplace_back("." + fmt);
      ffmpeg_suffixes.emplace_back(fmt);
    }
  }

  if( image_suffixes.empty() ) {
    image_suffixes = std::vector<std::string>( {
        ".tif", ".tiff",
        ".png",
        ".exr",
        ".hdr", ".pic",
        ".jpg", ".jpeg", ".jp2",
        ".bmp", ".dib",
        ".ppm", ".pgm",
        ".webp",
        ".flo",
        ".pbm", ".pgm", ".ppm", ".pxm", ".pnm",  // Portable image format
        ".sr", ".ras",      // Sun rasters
        ".pfm",
    });
  }

  static const auto contains =
      [](const std::vector<std::string> & suffixes, const std::string & suffix) -> bool {

        const char * csuffix = suffix.c_str();
        for ( const std::string & s : suffixes ) {
          if ( strcasecmp(csuffix, s.c_str()) == 0 ) {
            return true;
          }
        }
        return false;
      };


  output_type = output_type_unknown;

  if( contains(ser_suffixes, suffix) ) {
    output_type = output_type_ser;
  }
  else if( contains(ffmpeg_suffixes, suffix) ) {
    output_type = output_type_video;
  }
  else if( contains(image_suffixes, suffix) ) {
    output_type = output_type_images;
  }
  else {
    CF_ERROR("c_output_frame_writer: Can not suggest output type from file suffix '%s'",
        suffix.c_str());
    return false;
  }

  if( !create_path(get_parent_directory(filename)) ) {
    CF_ERROR("c_output_frame_writer: create_path('%s') fails: %s",
        filename.c_str(),
        strerror(errno));
    return false;
  }

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  if( write_frame_mapping ) {

    std::string mapfilename =
        ssprintf("%s.map.txt", filename.c_str());

    if( !(frame_mapping_fp = fopen(mapfilename.c_str(), "w")) ) {
      CF_ERROR("fopen('%s') fails : %s", mapfilename.c_str(), strerror(errno));
    }
    else {
      fprintf(frame_mapping_fp, "seqidx\tfrmidx\n");
    }
  }

  return true;
}

bool c_output_frame_writer::write(cv::InputArray currenFrame, cv::InputArray currentMask,
    bool with_alpha_mask, int seqindex)
{
  cv::Mat output_frame, output_mask;

  switch (output_type) {
    case output_type_video: {

      bool fOk = true;
          create_output_frame(currenFrame.getMat(), currentMask.getMat(),
              output_frame, output_mask,
              output_image_processor_,
              PIXEL_DEPTH_8U);

      if( !fOk ) {
        CF_ERROR("create_output_frame() fails for '%s'",
            output_file_name.c_str());
        return false;
      }


      if( !output_mask.empty() ) {
        output_frame.setTo(0, ~output_mask);
      }

      if( !ffmpeg.is_open() ) {

        const std::string opts =
            ffmpeg_opts_.empty() ? default_ffmpeg_opts_ :
                ffmpeg_opts_;

        fOk =
            ffmpeg.open(filename(), output_frame.size(),
                output_frame.channels() > 1,
                opts);

        if( !fOk ) {
          CF_ERROR("Can not write video file '%s'",
              filename().c_str());
          return false;
        }
      }

      if( !ffmpeg.write(output_frame, pts++) ) {
        CF_ERROR("ffmpeg.write() fails");
        return false;
      }

      break;
    }

    case output_type_ser: {

      bool fOk =
          create_output_frame(currenFrame.getMat(), currentMask.getMat(),
              output_frame, output_mask,
              output_image_processor_,
              output_pixel_depth_);

      if( !fOk ) {
        CF_ERROR("create_output_frame() fails for '%s'",
            output_file_name.c_str());
        return false;
      }

      if( !output_mask.empty() ) {
        output_frame.setTo(0, ~output_mask);
      }

      if( !ser.is_open() ) {

        fOk =
            ser.create(filename(), output_frame.cols, output_frame.rows,
                output_frame.channels() > 1 ? COLORID_BGR : COLORID_MONO,
                c_ser_file::bits_per_plane(output_frame.depth()));

        if( !fOk ) {
          CF_ERROR("Can not create SER file '%s'", filename().c_str());
          return false;
        }
      }

      if( !ser.write(output_frame) ) {
        CF_ERROR("ser.write() fails");
        return false;
      }

      break;
    }

    case output_type_images: {

      bool fOk =
          create_output_frame(currenFrame.getMat(), currentMask.getMat(),
              output_frame, output_mask,
              output_image_processor_,
              output_pixel_depth_);

      if( !fOk ) {
        CF_ERROR("create_output_frame() fails for '%s'",
            output_file_name.c_str());
        return false;
      }

      std::string fname =
          output_file_name;

      const std::string suffix =
          get_file_suffix(fname);

      set_file_suffix(fname, ssprintf("-%06d%s",
          current_frame_index,
          suffix.c_str()));

      if( with_alpha_mask ) {
        if( !save_image(output_frame, output_mask, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }
      else {
        if( !output_mask.empty() ) {
          output_frame.setTo(0, ~output_mask);
        }
        if( !save_image(output_frame, fname) ) {
          CF_ERROR("save_image('%s) fails", fname.c_str());
          return false;
        }
      }

      break;
    }

    default:
      CF_ERROR("ERROR: Output video file is not open");
      return false;
  }

  if( frame_mapping_fp ) {
    fprintf(frame_mapping_fp, "%5d\t%d\n",
        seqindex, current_frame_index);
  }

  current_input_sequence_index = seqindex;
  ++current_frame_index;
  return true;
}

void c_output_frame_writer::close()
{
  ffmpeg.close();
  ser.close();
  tmp.release();

  if( frame_mapping_fp ) {
    fclose(frame_mapping_fp);
    frame_mapping_fp = nullptr;
  }

  output_type = output_type_unknown;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_output_text_writer::c_output_text_writer()
{
}

c_output_text_writer::~c_output_text_writer()
{
  close();
}

const std::string & c_output_text_writer::filename() const
{
  return filename_;
}

bool c_output_text_writer::open(const std::string & filename)
{
  close();

  this->filename_ =
      filename;

  if( !(fp_ = fopen(filename_.c_str(), "w")) ) {
    CF_ERROR("fopen('%s') fails: %s", filename_.c_str(), strerror(errno));
    return false;
  }

  return true;
}

bool c_output_text_writer::vprintf(const char * format, va_list arglist)
{
  if( !fp_ ) {
    CF_ERROR("c_output_text_writer: file '%s' is not open",
        filename_.c_str());
    errno = EBADF;
    return false;
  }

  if( vfprintf(fp_, format, arglist) < 0 ) {
    CF_ERROR("c_output_text_writer: vfprintf into '%s' fails: %s",
        filename_.c_str(), strerror(errno));
    return false;
  }

  return true;
}

bool c_output_text_writer::printf(const char * format, ...)
{
  va_list arglist;
  int n;

  va_start(arglist, format);
  n = vprintf(format, arglist);
  va_end(arglist);

  return n >= 0;
}

bool c_output_text_writer::is_open() const
{
  return fp_ != nullptr;
}

void c_output_text_writer::close()
{
  if ( fp_ ) {
    fclose(fp_);
    fp_ = nullptr;
  }
}

void c_output_text_writer::flush()
{
  if ( fp_ ) {
    fflush(fp_);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool load_settings(c_config_setting settings, c_output_frame_writer_options * opts)
{
  LOAD_OPTION(settings, *opts, output_filename);
  LOAD_OPTION(settings, *opts, ffmpeg_opts);
  LOAD_IMAGE_PROCESSOR(settings, *opts, output_image_processor);
  LOAD_OPTION(settings, *opts, output_pixel_depth);
  LOAD_OPTION(settings, *opts, save_frame_mapping);
  return true;
}

bool save_settings(c_config_setting settings, const c_output_frame_writer_options & opts)
{
  SAVE_OPTION(settings, opts, output_filename);
  SAVE_OPTION(settings, opts, ffmpeg_opts);
  SAVE_IMAGE_PROCESSOR(settings, opts, output_image_processor);
  SAVE_OPTION(settings, opts, output_pixel_depth);
  SAVE_OPTION(settings, opts, save_frame_mapping);
  return true;
}
