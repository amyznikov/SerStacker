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


// LZW is contr-productive with floating point data, growing file size
// LZMA is not supported by ImageJ
static int g_default_tiff_compression = COMPRESSION_NONE;

void set_default_tiff_compression(int compression)
{
  g_default_tiff_compression = compression;
}

int default_tiff_compression()
{
  return g_default_tiff_compression;
}


template<class T>
static void write_tiff_image(const cv::Mat & image, int sampleformat, TIFF * tiff)
{
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, (int)(sizeof(T) * 8));
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT, sampleformat);

  for ( int y = 0; y < image.rows; ++y ) {
    TIFFWriteScanline(tiff, (void*) (image.ptr<const T>(y)), y, 0);
  }
}

static bool write_tiff(cv::InputArray src, const std::string & filename, const std::vector<int> & _params)
{
  cv::Mat image;
  TIFF * tiff;

  int photometric_tag = 0;
  int compression = g_default_tiff_compression;

  if ( !_params.empty() ) {
    for ( uint i = 0, n = _params.size(); i < n; i += 2 ) {
      if ( _params[i] == cv::ImwriteFlags::IMWRITE_TIFF_COMPRESSION ) {
        compression = _params[i + 1];
        break;
      }
    }
  }

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

  // CF_DEBUG("SET TIFFTAG_COMPRESSION= %d", compression);

  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, photometric_tag);
  TIFFSetField(tiff, TIFFTAG_COMPRESSION, compression);

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



// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_image, const cv::Mat & input_alpha_mask, cv::Mat & output_image)
{
  const int cn = input_image.channels();
  if ( cn != 1 && cn != 3 ) {
    return false;
  }

  if ( input_alpha_mask.empty() || input_alpha_mask.type() != CV_8UC1 || input_alpha_mask.size() != input_image.size() ) {
    return false;
  }


  cv::Mat alpha;

  switch ( input_image.depth() ) {
  case CV_8U :
    alpha = input_alpha_mask;
    break;
  case CV_8S :
    alpha = input_alpha_mask;
    break;
  case CV_16U :
    input_alpha_mask.convertTo(alpha, input_image.depth(), UINT16_MAX / UINT8_MAX);
    break;
  case CV_16S :
    input_alpha_mask.convertTo(alpha, input_image.depth(), INT16_MAX / (double) UINT8_MAX);
    break;
  case CV_32S :
    input_alpha_mask.convertTo(alpha, input_image.depth(), INT32_MAX / (double) UINT8_MAX);
    break;
  case CV_32F :
    input_alpha_mask.convertTo(alpha, input_image.depth(), 1.0 / UINT8_MAX);
    break;
  case CV_64F :
    input_alpha_mask.convertTo(alpha, input_image.depth(), 1.0 / UINT8_MAX);
    break;
  }

  if ( cn == 1 ) {

    cv::Mat src[2] = { input_image, alpha };
    cv::merge(src, 2, output_image);
  }
  else { // if ( cn == 3 )

    cv::Mat bgra;

    cv::Mat & dst = (output_image.data == input_image.data ||
        output_image.data == input_alpha_mask.data) ?
        bgra : output_image;

    cv::Mat src[2] = { input_image, alpha };

    static constexpr int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };

    dst.create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), input_image.channels() + 1));

    cv::mixChannels(src, 2, &dst, 1, from_to, 4);

    if ( dst.data != output_image.data ) {
      output_image = std::move(dst);
    }

  }


  return true;
}

static bool write_image(const std::string & filename, cv::InputArray image,
    const std::vector<int> & _params)
{
  std::vector<int> params = _params;

  const std::string output_path = get_parent_directory(filename);
  if ( !output_path.empty() && !create_path(output_path) ) {
    CF_ERROR("create_path('%s') fails: %s", output_path.c_str(), strerror(errno));
    return false;
  }

  const std::string output_suffix = get_file_suffix(filename);

  if ( strcasecmp(output_suffix.c_str(), ".flo") == 0 ) {

    if ( image.channels() != 2 ) {
      CF_ERROR("Invalid argument: optical flow image must have 2 channels");
      return false;
    }

    if ( !cv::writeOpticalFlow(filename, image) ) {
      CF_ERROR("cv::writeOpticalFlow('%s') fails", filename.c_str());
      return false;
    }

    return  true;
  }

  if ( strcasecmp(output_suffix.c_str(), ".tiff") == 0 || strcasecmp(output_suffix.c_str(), ".tif") == 0 ) {
    if ( write_tiff(image, filename, params) ) {
      return true;
    }

#if ( CV_VERSION_CURRRENT < CV_VERSION_INT(3,4,2) )
    CF_ERROR("CRITICAL WARNING: This OpenCV version is too old to support tiff");
#else
    params.emplace_back(cv::IMWRITE_TIFF_COMPRESSION);
    params.emplace_back(1);
#endif
  }

  if ( !cv::imwrite(filename, image, params) ) {
    CF_ERROR("cv::imwrite('%s') fails", filename.c_str());
    return false;
  }

  return true;
}




bool save_image(cv::InputArray image, const std::string & fname, const std::vector<int> & params)
{
  std::string dirname;

  if ( image.empty() ) {
    CF_CRITICAL("empty image specified to save as '%s'", fname.c_str());
    return false;
  }

  if ( !(dirname = get_parent_directory(fname)).empty() ) {
    if ( !create_path(dirname) ) {
      CF_CRITICAL("WARNING: create_path(%s) fails: %s", dirname.c_str(), strerror(errno));
    }
  }

  if ( !write_image(fname, image, params) ) {
    CF_CRITICAL("write_image(%s) fails", fname.c_str());
    return false;
  }

  return true;
}

bool save_image(cv::InputArray _image, cv::InputArray _mask, const std::string & fname,
    const std::vector<int> & params)
{
  cv::Mat image_to_write;

  if( _mask.empty() ) {
    image_to_write = _image.getMat();
  }
  else if( !mergebgra(_image.getMat(), _mask.getMat(), image_to_write) ) {
    CF_WARNING("mergebgra() fails, saving image with NO alpha mask");
    image_to_write = _image.getMat();
  }

  return save_image(image_to_write, fname, params);
}

