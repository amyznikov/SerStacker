/*
 * c_video_input_source.cc
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#include "c_video_input_source.h"
#include "c_video_frame.h"
#include <core/io/load_image.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <mutex>
#include <core/debug.h>


c_video_input_source::c_video_input_source(enum source_type type, const std::string & filename) :
  base(type, filename)
{
}

bool c_video_input_source::read(c_data_frame::sptr & output_frame)
{
  if( !is_open() ) {
    errno = EBADF;
    return false;
  }

  c_video_frame * f =
      dynamic_cast<c_video_frame*>(output_frame.get());

  if( !f ) {
    output_frame.reset(f = new c_video_frame());
  }

  if( !((base*) this)->read(f->image, &f->colorid, &f->bpc) ) {
    CF_ERROR("input_source_->read() fails");
    return false;
  }

  if( f->colorid != COLORID_OPTFLOW && (f->image.channels() == 4 || f->image.channels() == 2) ) {
    if( !splitbgra(f->image, f->image, &f->mask) ) {
      CF_WARNING("c_video_input_source: splitbgra() fails for colorid=%s image.channels=%d",
          toString(f->colorid), f->image.channels());
    }
  }

  if( (f->has_color_matrix = has_color_matrix()) ) {
    f->color_matrix = color_matrix();
  }

//  if ( filterBadPixels_ && badPixelsVariationThreshold_ > 0 ) {
//
//    if( !is_bayer_pattern(f->colorid) ) {
//      median_filter_hot_pixels(f->image, badPixelsVariationThreshold_, false);
//    }
//    else if( !extract_bayer_planes(f->image, f->image, f->colorid) ) {
//      CF_ERROR("ERROR: extract_bayer_planes() fails");
//    }
//    else {
//      median_filter_hot_pixels(f->image, badPixelsVariationThreshold_, true);
//      if( !nninterpolation(f->image, f->image, f->colorid) ) {
//        CF_ERROR("nninterpolation() fails");
//      }
//    }
//  }
//  else if( is_bayer_pattern(f->colorid) ) {
//    debayer(f->image, f->image, f->colorid,
//        debayerAlgorithm_);
//  }


  return true;
}


c_ser_input_source::c_ser_input_source(const std::string & filename) :
    base(c_input_source::SER, filename)
{
}

c_ser_input_source::sptr c_ser_input_source::create(const std::string & filename)
{
  sptr obj(new this_class(filename));
  if ( obj->ser_.open(filename) ) {
    obj->size_ = obj->ser_.num_frames();
    obj->ser_.close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_ser_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".ser"
  };

  return suffixes_;
}

bool c_ser_input_source::open()
{
  return ser_.open(filename_);
}

void c_ser_input_source::close()
{
  ser_.close();
}

bool c_ser_input_source::seek(int pos)
{
  return ser_.seek(pos);
}

int c_ser_input_source::curpos()
{
  return ser_.curpos();
}

bool c_ser_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( ser_.read(output_frame) ) {

    if ( output_colorid ) {
      *output_colorid = ser_.color_id();
    }

    if ( output_bpc ) {
      *output_bpc = ser_.bits_per_plane();
    }

    return true;
  }

  return false;
}

bool c_ser_input_source::is_open() const
{
  return ser_.is_open();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if HAVE_CFITSIO

c_fits_input_source::c_fits_input_source(const std::string & filename) :
    base(c_input_source::FITS, filename)
{
}

c_fits_input_source::sptr c_fits_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_fits_input_source::sptr obj(new c_fits_input_source(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_fits_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".fits",
      ".fit",
      ".fts"
  };

  return suffixes_;
}

bool c_fits_input_source::open()
{
  if ( !fits_.open(filename_) ) {
    return false;
  }
  curpos_ = 0;
  return true;
}

void c_fits_input_source::close()
{
  fits_.close();
  curpos_ = -1;
}

bool c_fits_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_fits_input_source::curpos()
{
  return curpos_;
}

bool c_fits_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 || !fits_.read(output_frame) ) {
    return false;
  }

  ++curpos_;

  if ( output_colorid ) {
    *output_colorid = suggest_colorid(
        output_frame.channels());
  }

  if ( output_bpc ) {
    *output_bpc = suggest_bbp(
        output_frame.depth());
  }

  return true;
}

bool c_fits_input_source::is_open() const
{
  return curpos_ >= 0;
}

#endif // HAVE_CFITSIO

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_movie_input_source::c_movie_input_source(const std::string & filename) :
    base(c_input_source::MOVIE, filename)
{
}

c_movie_input_source::sptr c_movie_input_source::create(const std::string & filename)
{
  sptr obj(new this_class(filename));
  if ( obj->ffmpeg_.open(filename) ) {
    obj->size_ = obj->ffmpeg_.num_frames();
    obj->ffmpeg_.close();
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_movie_input_source::suffixes()
{
  static std::vector<std::string> suffixes_;

  if ( suffixes_.empty() ) {

    const std::vector<std::string> & ffmpeg_formats =
        c_ffmpeg_reader::supported_input_formats();

    suffixes_.reserve(ffmpeg_formats.size());
    for (const std::string & fmt : ffmpeg_formats ) {
      suffixes_.emplace_back("." + fmt);
    }
  }

  return suffixes_;
}


bool c_movie_input_source::open()
{
  return ffmpeg_.open(filename_);
}

void c_movie_input_source::close()
{
  return ffmpeg_.close();
}

bool c_movie_input_source::seek(int pos)
{
  return ffmpeg_.seek_frame(pos);
}

int c_movie_input_source::curpos()
{
  return ffmpeg_.curpos();
}

bool c_movie_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( ffmpeg_.read(output_frame) ) {

    if ( output_colorid ) {
      *output_colorid = suggest_colorid(
          output_frame.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bbp(
          output_frame.depth());
    }

    return true;
  }
  return false;
}

bool c_movie_input_source::is_open() const
{
  return ffmpeg_.is_open();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_regular_image_input_source::c_regular_image_input_source(const std::string & filename) :
    base(c_input_source::REGULAR_IMAGE, filename)
{
}

c_regular_image_input_source::sptr c_regular_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    sptr obj(new this_class(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_regular_image_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
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

  return suffixes_;
}

bool c_regular_image_input_source::open()
{
  if ( file_readable(filename_) && !is_directory(filename_) ) {
    curpos_ = 0;
    return true;
  }
  return false;
}


void c_regular_image_input_source::close()
{
  curpos_ = -1;
}


bool c_regular_image_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_regular_image_input_source::curpos()
{
  return curpos_;
}

bool c_regular_image_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 ) {
    return false;
  }

  const std::string suffix =
      get_file_suffix(filename_);

  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {

    if ( !(output_frame = cv::readOpticalFlow(filename_)).data ) {
      return false;
    }

    if ( output_colorid ) {
      *output_colorid = COLORID_OPTFLOW;
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bbp(
          output_frame.depth());
    }

  }
  else {

    if ( !load_image(filename_, output_frame) ) {
      return false;
    }

    // CF_DEBUG("output_frame.channels()=%d", output_frame.channels());


    if ( output_colorid ) {
      *output_colorid = suggest_colorid(
          output_frame.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bbp(
          output_frame.depth());
    }

  }

  ++curpos_;

  return true;
}

bool c_regular_image_input_source::is_open() const
{
  return curpos_ >= 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if HAVE_LIBRAW
c_raw_image_input_source::c_raw_image_input_source(const std::string & filename) :
    base(c_input_source::RAW_IMAGE, filename)
{
}

c_raw_image_input_source::sptr c_raw_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_raw_image_input_source::sptr obj(new c_raw_image_input_source(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string> & c_raw_image_input_source::suffixes()
{
  static const std::vector<std::string> suffixes_ = {
      ".nef",
      ".cr2"
  };

  return suffixes_;
}

bool c_raw_image_input_source::open()
{
  if ( file_readable(filename_) && !is_directory(filename_) ) {
    curpos_ = 0;
    return true;
  }
  return false;
}

void c_raw_image_input_source::close()
{
  curpos_ = -1;
}

bool c_raw_image_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_raw_image_input_source::curpos()
{
  return curpos_;
}

bool c_raw_image_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 || !raw_.read(filename_, output_frame, output_colorid, output_bpc) ) {
    return false;
  }

  ++curpos_;

  if ( (this->has_color_matrix_ = raw_.has_color_matrix()) ) {
    this->color_matrix_ = raw_.color_matrix();
  }

  return true;
}

bool c_raw_image_input_source::is_open() const
{
  return curpos_ >= 0;
}

#endif // HAVE_LIBRAW

