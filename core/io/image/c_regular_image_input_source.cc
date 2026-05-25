/*
 * c_regular_image_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_regular_image_input_source.h"
#include <core/io/load_image.h>
#include <core/readdir.h>


#if defined(_WIN32) || defined(_WIN64)
# define timegm(x) _mkgmtime(x)
#endif


c_regular_image_input_source::c_regular_image_input_source(const std::string & filename) :
    base(/*c_input_source::REGULAR_IMAGE, */filename)
{
}

c_regular_image_input_source::sptr c_regular_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    sptr obj(new this_class(filename));
    obj->_size = 1;
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
  if ( file_readable(_filename) && !is_directory(_filename) ) {
    _curpos = 0;
    return true;
  }
  return false;
}


void c_regular_image_input_source::close()
{
  _curpos = -1;
}


bool c_regular_image_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  _curpos = pos;
  return true;
}

int c_regular_image_input_source::curpos()
{
  return _curpos;
}

bool c_regular_image_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( _curpos != 0 ) {
    return false;
  }

  const std::string suffix = get_file_suffix(_filename);
  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {

    if ( !(output_frame = cv::readOpticalFlow(_filename)).data ) {
      return false;
    }

    if ( output_colorid ) {
      *output_colorid = COLORID_OPTFLOW;
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bpp(
          output_frame.depth());
    }

  }
  else {

    if ( !load_image(_filename, output_frame) ) {
      return false;
    }

    // CF_DEBUG("output_frame.channels()=%d", output_frame.channels());


    if ( output_colorid ) {
      *output_colorid = suggest_colorid(
          output_frame.channels());
    }

    if ( output_bpc ) {
      *output_bpc = suggest_bpp(
          output_frame.depth());
    }

  }

  // Try to parse file name for encoded time stamp
  if ( true ) {
    _has_last_ts = false;

    bool timestamp_found = false;


    if ( !timestamp_found ) {
      // JUP1.20230815_010015_GMT.32F.tiff
      const std::string marker = "_GMT";
      const size_t timestamp_length = 15;
      const size_t marker_pos = _filename.find(marker);

      if (marker_pos != std::string::npos && marker_pos >= timestamp_length) {
        timestamp_found = true;

        const size_t start_pos = marker_pos - timestamp_length;
        const std::string timestamp = _filename.substr(start_pos, timestamp_length);

        int year = 0, month = 0, day = 0;
        int hour = 0, minute = 0, second = 0;
        int parsed = sscanf(timestamp.c_str(), "%4d%2d%2d_%2d%2d%2d", &year, &month, &day, &hour, &minute, &second);

        if( parsed == 6 ) {
          std::tm t = {};
          t.tm_year = year - 1900; // Years in tm are counted from 1900
          t.tm_mon = month - 1;    // Months are counted from 0 (January = 0)
          t.tm_mday = day;
          t.tm_hour = hour;
          t.tm_min = minute;
          t.tm_sec = second;
          t.tm_isdst = -1;

          _last_ts = 1000.0 * timegm(&t);
          _has_last_ts = true;
        }
      }
    }

    if ( !timestamp_found ) {
      //2021-09-16-2110_8-CapObj-32F.tiff
      const std::string marker = "-CapObj";
      const size_t timestamp_length = 17;
      const size_t marker_pos = _filename.find(marker);

      if (marker_pos != std::string::npos && marker_pos >= timestamp_length) {
        timestamp_found = true;

        const size_t start_pos = marker_pos - timestamp_length;
        const std::string timestamp = _filename.substr(start_pos, timestamp_length);

        int year = 0, month = 0, day = 0;
        int hour = 0, minute = 0, sfraq = 0;
        int parsed = sscanf(timestamp.c_str(), "%4d-%2d-%2d-%2d%2d_%1d", &year, &month, &day, &hour, &minute, &sfraq);

        if( parsed == 6 ) {
          std::tm t = {};
          t.tm_year = year - 1900; // Years in tm are counted from 1900
          t.tm_mon = month - 1;    // Months are counted from 0 (January = 0)
          t.tm_mday = day;
          t.tm_hour = hour;
          t.tm_min = minute;
          t.tm_sec = (int)(6.0 * sfraq);
          t.tm_isdst = -1;

          _last_ts = 1000.0 * timegm(&t);
          _has_last_ts = true;
        }
      }
    }
  }

  ++_curpos;

  return true;
}

bool c_regular_image_input_source::is_open() const
{
  return _curpos >= 0;
}

