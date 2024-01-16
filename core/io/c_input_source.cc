/*
 * c_input_source.cc
 *
 *  Created on: Jan 11, 2021
 *      Author: amyznikov
 */

#include "c_input_source.h"
//#include "load_image.h"
#include "video/c_video_input_source.h"
#include "vlo/c_vlo_input_source.h"
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <mutex>
#include <core/debug.h>


template<>
const c_enum_member* members_of<c_input_source::OUTPUT_TYPE>()
{
  static const c_enum_member members[] = {
      { c_input_source::OUTPUT_TYPE_IMAGE, "IMAGE", "" },
      { c_input_source::OUTPUT_TYPE_CLOUD3D, "Cloud3D", "" },
      { c_input_source::OUTPUT_TYPE_IMAGE },
  };

  return members;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



c_input_source::c_input_source(enum source_type type, const std::string & filename)
  : type_(type), filename_(filename)
{
}


enum COLORID c_input_source::suggest_colorid(int cn)
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

int c_input_source::suggest_bbp(int ddepth)
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
    else if ( contains(c_vlo_input_source::suffixes(), suffix) ) {
      type = c_input_source::VLO;
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

c_input_source::sptr c_input_source::create(source_type type, const std::string & filename)
{
  c_input_source::sptr obj;

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
    case c_input_source::VLO :
      obj = c_vlo_input_source::create(filename);
      break;

    default :
      CF_ERROR("c_input_source: invalid source type = %d requested", type);
      break;
    }

  }
  return obj;
}

c_input_source::sptr c_input_source::create(const std::string & filename)
{
  enum source_type type =
      suggest_source_type(filename);

  if( type != c_input_source::UNKNOWN ) {

    c_input_source::sptr source =
        c_input_source::create(type, filename);

    if( source ) {
      source->load_badframes();
    }

    return source;
  }

  return nullptr;
}


c_input_source::sptr c_input_source::open(const std::string & filename)
{
  c_input_source::sptr obj = this_class::create(filename);
  if( obj && !obj->is_open() && !obj->open() ) {
    CF_ERROR("obj->open(filename='%s') fails", filename.c_str());
    obj.reset();
  }

  return obj;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
