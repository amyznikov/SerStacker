/*
 * c_input_source.cc
 *
 *  Created on: Jan 11, 2021
 *      Author: amyznikov
 */

#include "c_input_source.h"
#include "image/c_ffmpeg_input_source.h"
#include "image/c_fits_input_source.h"
#include "image/c_image_input_source.h"
#include "image/c_raw_image_input_source.h"
#include "image/c_regular_image_input_source.h"
#include "image/c_ser_input_source.h"
#include "hdl/c_hdl_input_source.h"
#include "sply/c_sply_input_source.h"
#include "text/c_textfile_input_source.h"
#include "ply/c_ply_input_source.h"
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <mutex>
#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



c_input_source::c_input_source(const std::string & filename) :
    filename_(filename)
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

int c_input_source::suggest_bpp(int ddepth)
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

c_input_source::sptr c_input_source::create(const std::string & filename)
{
  const std::string suffix =
      get_file_suffix(filename);

  if( !suffix.empty() ) {

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


    c_input_source::sptr obj;

#if have_ser_input_source
    if( contains(c_ser_input_source::suffixes(), suffix) && (obj = c_ser_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_vlo_input_source
    if( contains(c_vlo_input_source::suffixes(), suffix) && (obj = c_vlo_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_hdl_input_source
    if( contains(c_hdl_input_source::suffixes(), suffix) && (obj = c_hdl_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_sply_input_source
    if( contains(c_sply_input_source::suffixes(), suffix) && (obj = c_sply_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_textfile_input_source
    if( contains(c_textfile_input_source::suffixes(), suffix) && (obj = c_textfile_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_ply_input_source
    if( contains(c_ply_input_source::suffixes(), suffix) && (obj = c_ply_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_fits_input_source
    if( contains(c_fits_input_source::suffixes(), suffix) && (obj = c_fits_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_ffmpeg_input_source
    if( contains(c_ffmpeg_input_source::suffixes(), suffix) && (obj = c_ffmpeg_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_regular_image_input_source
    if( contains(c_regular_image_input_source::suffixes(), suffix) && (obj = c_regular_image_input_source::create(filename)) ) {
      goto end;
    }
#endif

#if have_raw_image_input_source
    if( contains(c_raw_image_input_source::suffixes(), suffix) && (obj = c_raw_image_input_source::create(filename)) ) {
      goto end;
    }
#endif

end:
    if ( obj ) {
      obj->load_badframes();
    }

    return obj;
  }

  return nullptr;
}


c_input_source::sptr c_input_source::open(const std::string & filename)
{
  //CF_DEBUG("c_input_source::open(filename='%s')", filename.c_str());

  c_input_source::sptr obj =
      create(filename);

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

  // CF_DEBUG("badframes_file_name='%s'", badframes_file_name.c_str());

  if ( !badframes_file_name.empty() ) {

    FILE * fp =
        fopen(badframes_file_name.c_str(), "r");

    if ( fp ) {

      badframes_.clear();

      char line[256] = "";

      while (fgets(line, 255, fp)) {

        int index1 = -1, index2 = -1;

        const int n =
            sscanf(line, "%d-%d", &index1, &index2);

        // CF_DEBUG("line '%s' n=%d index1=%d, index2=%d", line, n, index1, index2);

        if( n == 1 ) {
          badframes_.emplace_back(index1);
        }
        else if( n == 2 ) {
          for( uint i = index1; i <= index2; ++i ) {
            badframes_.emplace_back(i);
          }
        }
      }

      fclose(fp);

      std::sort(badframes_.begin(),
          badframes_.end());

      const auto pos =
          std::unique(badframes_.begin(),
              badframes_.end());

      if ( pos != badframes_.end() ) {
        badframes_.erase(pos, badframes_.end());
      }

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

        for( uint32_t i1 = 0, n = badframes_.size(); i1 < n; ++i1 ) {

          uint32_t i2 = i1;
          while (i2 + 1 < n && badframes_[i2 + 1] == badframes_[i2] + 1) {
            ++i2;
          }

          if( i2 == i1 ) {
            fprintf(fp, "%u\n", badframes_[i1]);
          }
          else {
            fprintf(fp, "%u-%u\n", badframes_[i1], badframes_[i2]);
            i1 = i2;
          }
        }

        fclose(fp);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
