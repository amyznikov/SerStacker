/*
 * save_image.cc
 *
 *  Created on: Nov 16, 2019
 *      Author: amyznikov
 */

#include "save_image.h"
#include <tiff.h>
#include <tiffio.h>
#include <core/readdir.h>
#include <core/debug.h>

#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) ((a)<<16 | (b)<<8 | (c))
#endif

#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

template<class T>
static void write_tiff_image(const cv::Mat & image, int sampleformat, TIFF * tiff)
{
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, (int)(sizeof(T) * 8));
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, sampleformat);

  for ( int y = 0; y < image.rows; ++y ) {
    TIFFWriteScanline(tiff, (void*) (image.ptr<const T>(y)), y, 0);
  }
}

static bool write_tiff(cv::InputArray src, const std::string & filename)
{
  cv::Mat image;
  TIFF * tiff;
  int photometric_tag = 0;

  if ( filename.empty() ) {
    CF_FATAL("Empty file name for tiff image to write");
    return false;
  }

  if ( src.depth() < CV_8U || src.depth() > CV_64F ) {
    CF_FATAL("Unsupported image depth=%d in write_tiff()", src.depth());
    return false;
  }

  switch ( src.channels() ) {
  case 1 :
    photometric_tag = PHOTOMETRIC_MINISBLACK;
    image = src.getMat();
    break;

  case 2 :
    photometric_tag = PHOTOMETRIC_MINISBLACK;
    image = src.getMat();
    break;

  case 3 :
    photometric_tag = PHOTOMETRIC_RGB;
    cv::cvtColor(src, image, cv::COLOR_BGR2RGB);
    break;

  case 4 :
    photometric_tag = PHOTOMETRIC_RGB;
    cv::cvtColor(src, image, cv::COLOR_BGRA2RGBA);
    break;

  default:
    CF_ERROR("Unsupported number of channels in write_tiff(): %d", src.channels());
    return false;
  }

  if ( !(tiff = TIFFOpen(filename.c_str(), "w")) ) {
    CF_FATAL("TIFFOpen(%s) fails", filename.c_str());
    return false;
  }

  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, photometric_tag);
  TIFFSetField(tiff, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, (uint64_t) (image.cols));  // width of the image
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, (uint64_t) (image.rows));  // height of the image
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, image.channels());   // number of channels per pixel
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tiff, TIFFTAG_PAGENUMBER, 0, 1);
  if ( image.channels() == 4 || image.channels() == 2 ) {
    static const uint16_t extras[] = {EXTRASAMPLE_ASSOCALPHA};
    TIFFSetField(tiff, TIFFTAG_EXTRASAMPLES, 1, extras);
  }


  switch (src.depth()) {
    case CV_8U:
      write_tiff_image<uint8_t>(image, SAMPLEFORMAT_UINT, tiff);
      break;
    case CV_8S :
      write_tiff_image<int8_t>(image, SAMPLEFORMAT_INT, tiff);
      break;
    case CV_16U:
      write_tiff_image<uint16_t>(image, SAMPLEFORMAT_UINT, tiff);
      break;
    case CV_16S:
      write_tiff_image<int16_t>(image, SAMPLEFORMAT_INT, tiff);
      break;
    case CV_32S:
      write_tiff_image<int32_t>(image, SAMPLEFORMAT_INT, tiff);
      break;
    case CV_32F:
      write_tiff_image<float>(image, SAMPLEFORMAT_IEEEFP, tiff);
      break;
    case CV_64F:
      write_tiff_image<double>(image, SAMPLEFORMAT_IEEEFP, tiff);
      break;
  }

  if ( !TIFFWriteDirectory(tiff) ) {
    CF_FATAL("TIFFWriteDirectory(%s) fails", filename.c_str());
  }

  TIFFClose(tiff);

  return true;
}



static bool write_image(const std::string & filename, cv::InputArray src,
    const std::vector<int>& _params = std::vector<int>())
{
  std::vector<int> params = _params;

  const std::string output_path = get_parent_directory(filename);
  if ( !output_path.empty() && !create_path(output_path) ) {
    CF_ERROR("create_path('%s') fails: %s", output_path.c_str(), strerror(errno));
    return false;
  }

  const std::string output_suffix = get_file_suffix(filename);
  if ( strcasecmp(output_suffix.c_str(), ".tiff") == 0 || strcasecmp(output_suffix.c_str(), ".tif") == 0 ) {
    if ( write_tiff(src, filename) ) {
      return true;
    }
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(3,4,2) )
      params.emplace_back(cv::IMWRITE_TIFF_COMPRESSION);
      params.emplace_back(1);
#else
      CF_ERROR("OpenCV is too old to support tiff");
#endif
  }

  if ( !cv::imwrite(filename, src, params) ) {
    CF_ERROR("cv::imwrite('%s') fails", filename.c_str());
    return false;
  }

  return true;
}




bool save_image(cv::InputArray _image, const std::string & fname,
    const std::vector<int>& params, int pixtype)
{
  cv::Mat image, converted;
  const cv::Mat * imgptr;
  std::string dirname;


  image = _image.getMat();
  if ( image.empty() ) {
    CF_CRITICAL("empty image specified to save as '%s'", fname.c_str());
    return false;
  }

  if ( pixtype == -1 ) {
    pixtype = image.type();
  }

  if ( CV_MAT_DEPTH(pixtype) == CV_MAT_DEPTH(image.type()) ) {
    imgptr = &image;
  }
  else {
    image.convertTo(converted, pixtype);
    imgptr = &converted;
  }

  if ( !(dirname = get_parent_directory(fname)).empty() ) {
    if ( !create_path(dirname) ) {
      CF_CRITICAL("WARNING: create_path(%s) fails: %s", dirname.c_str(), strerror(errno));
    }
  }

  if ( !write_image(fname, *imgptr, params) ) {
    CF_CRITICAL("write_image(%s) fails", fname.c_str());
    return false;
  }

  return true;
}


