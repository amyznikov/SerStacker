/*
 * c_input_source.cc
 *
 *  Created on: Jan 11, 2021
 *      Author: amyznikov
 */

#include "c_input_source.h"
#include "load_image.h"
#include <core/readdir.h>
#include <mutex>
#include <core/debug.h>

static inline enum COLORID suggest_colorid(int cn)
{
  switch ( cn ) {
  case 1 :
    return COLORID_MONO;
  case 3 :
    return COLORID_BGR;
  case 4 :
    return COLORID_BGRA;
  }
  return COLORID_UNKNOWN;
}

static inline int suggest_bbp(int ddepth)
{
  switch ( ddepth ) {
  case CV_8U :
    return 8;
  case CV_8S :
    return 8;
  case CV_16U :
    return 16;
  case CV_16S :
    return 16;
  case CV_32S :
    return 32;
  }
  return 0;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_input_source::c_input_source(enum source_type type, const std::string & filename)
  : type_(type), filename_(filename)
{
}

c_input_source::ptr c_input_source::create(source_type type, const std::string & filename)
{
  c_input_source::ptr obj;

  if ( !filename.empty() ) {

    switch ( type ) {
    case c_input_source::SER :
      obj = c_ser_input_source::create(filename);
      break;
#if HAVE_CFITSIO
    case c_input_source::FITS :
      obj = c_fits_input_source::create(filename);
      break;
#endif // HAVE_CFITSIO
    case c_input_source::MOVIE :
      obj = c_movie_input_source::create(filename);
      break;
    case c_input_source::REGULAR_IMAGE :
      obj = c_regular_image_input_source::create(filename);
      break;
#if HAVE_LIBRAW
    case c_input_source::RAW_IMAGE :
      obj = c_raw_image_input_source::create(filename);
      break;
#endif // HAVE_LIBRAW
    default :
      CF_ERROR("c_input_source: invalid source type = %d requested", type);
      break;
    }

  }
  return obj;
}

enum c_input_source::source_type c_input_source::suggest_source_type(
    const std::string & filename)
{
  enum source_type type = c_input_source::UNKNOWN;

  const std::string suffix = get_file_suffix(filename);
  if ( !suffix.empty() ) {

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

    if ( contains(c_ser_input_source::suffixes(), suffix) ) {
      type = c_input_source::SER;
    }
#if HAVE_CFITSIO
    else if ( contains(c_fits_input_source::suffixes(), suffix) ) {
      type = c_input_source::FITS;
    }
#endif // HAVE_CFITSIO
    else if ( contains(c_movie_input_source::suffixes(), suffix) ) {
      type = c_input_source::MOVIE;
    }
    else if ( contains(c_regular_image_input_source::suffixes(), suffix) ) {
      type = c_input_source::REGULAR_IMAGE;
    }
#if HAVE_LIBRAW
    else if ( contains(c_raw_image_input_source::suffixes(), suffix) ) {
      type = c_input_source::RAW_IMAGE;
    }
#endif // HAVE_LIBRAW
  }

  return type;
}

c_input_source::ptr c_input_source::create(const std::string & filename)
{
  enum source_type type =
      suggest_source_type(filename);

  if( type != c_input_source::UNKNOWN ) {

    c_input_source::ptr source =
        c_input_source::create(type, filename);

    if( source ) {
      source->load_badframes();
    }

    return source;
  }

  return nullptr;
}

const std::vector<uint> & c_input_source::badframes() const
{
  return badframes_;
}

bool c_input_source::is_badframe(uint index) const
{
  if( !badframes_.empty() ) {

    const std::vector<uint>::const_iterator pos =
        std::lower_bound(badframes_.begin(), badframes_.end(), index);
    if( pos != badframes_.end() && *pos == index ) {
      return true;
    }
  }

  return false;
}

void c_input_source::set_badframe(uint index, bool is_bad)
{
  const std::vector<uint>::const_iterator pos =
      std::lower_bound(badframes_.begin(), badframes_.end(), index);

  if( is_bad ) {
    if( pos == badframes_.end() || *pos != index ) {
      badframes_.insert(pos, index);
    }
  }
  else if( pos != badframes_.end() ) {
    while (!badframes_.empty() && *pos == index) {
      badframes_.erase(pos);
    }
  }
}

void c_input_source::set_badframes(const std::vector<uint> & indexes)
{
  badframes_ = indexes;
  std::sort(badframes_.begin(), badframes_.end());
}

static std::string gen_badframes_file_name(const std::string & input_source_fname)
{
  return input_source_fname + ".badframes.txt";
}

const std::vector<uint> & c_input_source::load_badframes(const std::string & fname)
{
  std::string badframes_file_name;

  if ( !fname.empty() ) {
    badframes_file_name =
        gen_badframes_file_name(fname);
  }
  else {
    badframes_file_name =
        gen_badframes_file_name(this->filename_);
  }

  if ( !badframes_file_name.empty() ) {

    FILE * fp =
        fopen(badframes_file_name.c_str(), "r");

    if ( fp ) {

      badframes_.clear();

      uint index;
      while ( fscanf(fp,"%u", &index) == 1 ) {
        badframes_.emplace_back(index);
      }

      fclose(fp);

      std::sort(badframes_.begin(), badframes_.end());
    }
  }

  return badframes_;
}

void c_input_source::save_badframes(const std::string & fname) const
{
  std::string badframes_file_name;

  if( !fname.empty() ) {
    badframes_file_name =
        gen_badframes_file_name(fname);
  }
  else {
    badframes_file_name =
        gen_badframes_file_name(this->filename_);
  }

  if( !badframes_file_name.empty() ) {

    if( badframes_.empty() ) {
      unlink(badframes_file_name.c_str());
    }
    else {

      FILE *fp =
          fopen(badframes_file_name.c_str(), "w");

      if( !fp ) {
        CF_ERROR("fopen('%s') fails: %s",
            badframes_file_name.c_str(),
            strerror(errno));
      }
      else {

        for( uint i = 0, n = badframes_.size(); i < n; ++i ) {
          fprintf(fp, "%u\n", badframes_[i]);
        }

        fclose(fp);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_ser_input_source::c_ser_input_source(const std::string & filename)
  : base(c_input_source::SER, filename)
{
}

c_ser_input_source::ptr c_ser_input_source::create(const std::string & filename)
{
  c_ser_input_source::ptr obj(new c_ser_input_source(filename));
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if HAVE_CFITSIO

c_fits_input_source::c_fits_input_source(const std::string & filename)
  : base(c_input_source::FITS, filename)
{
}

c_fits_input_source::ptr c_fits_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_fits_input_source::ptr obj(new c_fits_input_source(filename));
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
#endif // HAVE_CFITSIO

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_movie_input_source::c_movie_input_source(const std::string & filename)
  : base(c_input_source::MOVIE, filename)
{
}

c_movie_input_source::ptr c_movie_input_source::create(const std::string & filename)
{
  c_movie_input_source::ptr obj(new c_movie_input_source(filename));
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_regular_image_input_source::c_regular_image_input_source(const std::string & filename)
  : base(c_input_source::REGULAR_IMAGE, filename)
{
}

c_regular_image_input_source::ptr c_regular_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_regular_image_input_source::ptr obj(new c_regular_image_input_source(filename));
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
      ".jpg", ".jpeg",
      ".bmp", ".dib",
      ".ppm", ".pgm",
      ".webp",
      ".flo",
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if HAVE_LIBRAW
c_raw_image_input_source::c_raw_image_input_source(const std::string & filename)
  : base(c_input_source::RAW_IMAGE, filename)
{
}

c_raw_image_input_source::ptr c_raw_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_raw_image_input_source::ptr obj(new c_raw_image_input_source(filename));
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

#endif // HAVE_LIBRAW

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
