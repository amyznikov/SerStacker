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

std::string c_output_frame_writer::_default_ffmpeg_opts =
    "-r 30 -c huffyuv -f avi";
    // "-r 10 -c rawvideo -pix_fmt rgb24";

c_output_frame_writer::c_output_frame_writer()
{
}


c_output_frame_writer::~c_output_frame_writer()
{
  close();
}

const std::string & c_output_frame_writer::filename() const
{
  return output_file_name;
}

const char * c_output_frame_writer::cfilename() const
{
  return output_file_name.c_str();
}

const std::string & c_output_frame_writer::ffmpeg_opts() const
{
  return _ffmpeg_opts;
}

void c_output_frame_writer::set_default_ffmpeg_opts(const std::string & opts)
{
  _default_ffmpeg_opts = opts;
}

const std::string & c_output_frame_writer::default_ffmpeg_opts()
{
  return _default_ffmpeg_opts;
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
  _ffmpeg_opts = ffmpeg_opts;
  output_type = output_type_unknown;
  _output_image_processor = output_image_processor;
  _output_pixel_depth = output_pixel_depth;
  current_frame_index = 0;
  pts = 0;

  if ( filename.empty() ) {
    CF_ERROR("c_output_frame_writer: No output file name specified");
    return false;
  }

  const std::string suffix = get_file_suffix(filename);
  if ( suffix.empty() ) {
    CF_ERROR("c_output_frame_writer: Can not suggest output type from empty file suffix");
    return false;
  }


  static const std::vector<std::string> ser_suffixes = {".ser"};
  static std::vector<std::string> ffmpeg_suffixes = c_ffmpeg_writer::supported_output_formats();

  static std::vector<std::string> image_suffixes = {
      ".fits", ".fit", ".fts",
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
  };

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
  else if( contains(image_suffixes, suffix) ) {
    output_type = output_type_images;
  }
  else if( contains(ffmpeg_suffixes, suffix) ) {
    output_type = output_type_video;
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
    const std::string mapfilename = ssprintf("%s.map.txt", filename.c_str());
    if( !(frame_mapping_fp = fopen(mapfilename.c_str(), "w")) ) {
      CF_ERROR("fopen('%s') fails : %s", mapfilename.c_str(), strerror(errno));
    }
    else {
      fprintf(frame_mapping_fp, "seqidx\tfrmidx\n");
    }
  }

  return true;
}

bool c_output_frame_writer::create_output_frame(cv::InputArray image, cv::InputArray mask,
    cv::Mat & out_image, cv::Mat & out_mask,
    const c_image_processor::sptr & processor,
    PIXEL_DEPTH ddepth)
{
  if ( !processor ) {
    out_image = image.getMat();
    out_mask = mask.getMat();
  }
  else {
    image.copyTo(out_image);
    mask.copyTo(out_mask);
    if ( !processor->process(out_image, out_mask) ) {
      CF_ERROR("processor->process() fails");
      return false;
    }
  }

  if( ddepth != PIXEL_DEPTH_NO_CHANGE && ddepth != out_image.depth() ) {
    double scale = 1, offset = 0;
    if( !getScaleOffset(image.depth(), ddepth, &scale, &offset) ) {
      CF_ERROR("c_output_frame_writer: get_scale_offset() fails");
      return false;
    }
    image.getMat().convertTo(out_image, ddepth, scale, offset);
  }

  return true;
}

bool c_output_frame_writer::write(cv::InputArray image, cv::InputArray mask, int seqindex, enum COLORID colorid)
{
  cv::Mat out_image, out_mask;

  switch (output_type) {
    case output_type_ser: {

      if( !create_output_frame(image, mask, out_image, out_mask, _output_image_processor, _output_pixel_depth) ) {
        CF_ERROR("output_type_ser: create_output_frame() fails");
        return false;
      }

      if( !ser.is_open() ) {

        if ( colorid == COLORID_UNKNOWN ) {
          switch(out_image.channels()) {
            case 1: colorid = COLORID_MONO; break;
            case 2: colorid = COLORID_OPTFLOW; break;
            case 3: colorid = COLORID_BGR; break;
            case 4: colorid = COLORID_BGRA; break;
          }
        }

        bool fOk =
            ser.create(filename(), out_image.cols, out_image.rows, colorid,
                c_ser_file::bits_per_plane(out_image.depth()),
                out_mask.empty() ? -1 : out_mask.depth());

        if( !fOk ) {
          CF_ERROR("Can not create SER file '%s'", filename().c_str());
          return false;
        }
      }

      if( !ser.write(out_image, out_mask) ) {
        CF_ERROR("ser.write() fails");
        return false;
      }

      break;
    }

    case output_type_video: {

      if( !create_output_frame(image, mask, out_image, out_mask, _output_image_processor, PIXEL_DEPTH_8U) ) {
        CF_ERROR("output_type_video: create_output_frame() fails");
        return false;
      }


      if( !ffmpeg.is_open() ) {

        const std::string opts =
            _ffmpeg_opts.empty() ? _default_ffmpeg_opts :
                _ffmpeg_opts;

        if( !ffmpeg.open(filename(), out_image.size(), out_image.channels() > 1, opts) ) {
          CF_ERROR("Can not write video file '%s'", filename().c_str());
          return false;
        }
      }

      if( !ffmpeg.write(out_image, pts++) ) {
        CF_ERROR("ffmpeg.write() fails");
        return false;
      }

      break;
    }

    case output_type_images: {

      if( !create_output_frame(image, mask, out_image, out_mask, _output_image_processor, _output_pixel_depth) ) {
        CF_ERROR("output_type_images: create_output_frame() fails");
        return false;
      }

      std::string fname = output_file_name;
      const std::string suffix = get_file_suffix(fname);

      set_file_suffix(fname, ssprintf("-%06d%s",
          seqindex >= 0 ? seqindex : current_frame_index,
          suffix.c_str()));

      if( !save_image(out_image, out_mask, fname, colorid) ) {
        CF_ERROR("save_image('%s) fails", fname.c_str());
        return false;
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
  return _filename;
}

bool c_output_text_writer::open(const std::string & filename)
{
  close();

  this->_filename =
      filename;

  if( !(_fp = fopen(_filename.c_str(), "w")) ) {
    CF_ERROR("fopen('%s') fails: %s", _filename.c_str(), strerror(errno));
    return false;
  }

  return true;
}

bool c_output_text_writer::vprintf(const char * format, va_list arglist)
{
  if( !_fp ) {
    CF_ERROR("c_output_text_writer: file '%s' is not open",
        _filename.c_str());
    errno = EBADF;
    return false;
  }

  if( vfprintf(_fp, format, arglist) < 0 ) {
    CF_ERROR("c_output_text_writer: vfprintf into '%s' fails: %s",
        _filename.c_str(), strerror(errno));
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
  return _fp != nullptr;
}

void c_output_text_writer::close()
{
  if ( _fp ) {
    fclose(_fp);
    _fp = nullptr;
  }
}

void c_output_text_writer::flush()
{
  if ( _fp ) {
    fflush(_fp);
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
