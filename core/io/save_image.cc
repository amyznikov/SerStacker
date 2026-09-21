/*
 * save_image.cc
 *
 *  Created on: Nov 16, 2019
 *      Author: amyznikov
 */

#include "save_image.h"
#include <tiff.h>
#include <tiffio.h>
#include <core/proc/pixtype.h>
#include <core/io/c_fits_file.h>
#include <core/readdir.h>
#include <core/ssprintf.h>
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

static bool get_data_range_for_pixel_depth(int ddepth, double * minval, double * maxval)
{
  switch ( ddepth ) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}

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
  TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, static_cast<uint16_t>(sizeof(T) * 8));
  TIFFSetField(tiff, TIFFTAG_SAMPLEFORMAT,  static_cast<uint16_t>(sampleformat));
  for ( int y = 0; y < image.rows; ++y ) {
    TIFFWriteScanline(tiff, (void*) (image.ptr<const T>(y)), y, 0);
  }
}

static bool write_tiff(cv::InputArray image, cv::InputArray mask, const std::string & filename,
    const std::vector<int> & _params, enum COLORID colorid)
{
  if ( filename.empty() ) {
    CF_FATAL("Empty file name for tiff image to write");
    return false;
  }

  if ( image.empty() ) {
    CF_FATAL("Empty src image in write_tiff()");
    return false;
  }

  if ( image.depth() < CV_8U || image.depth() > CV_64F ) {
    CF_FATAL("Unsupported image depth=%d in write_tiff()", image.depth());
    return false;
  }

  cv::Mat src = image.getMat();
  cv::Mat image_to_write;
  int mask_depth = -1;
  double mask_scale = 1;
  bool has_mask = !mask.empty();
  std::string description;

  if( !has_mask ) {
    if( src.channels() == 3 ) {
      cv::cvtColor(src, image_to_write, cv::COLOR_BGR2RGB);
    }
    else {
      image_to_write = src;
    }
  }
  else {
    const cv::Mat srcm = mask.getMat();
    cv::Mat prepared_mask;
    double max_weight = 1.0;

    cv::minMaxLoc(srcm, nullptr, &max_weight);
    if ( std::abs(max_weight) > std::numeric_limits<float>::min() ) {
      mask_scale = getMaxValForPixelDepth(src.depth()) * getMaxValForPixelDepth(srcm.depth()) / max_weight;
    }

    mask_depth = srcm.depth();
    srcm.convertTo(prepared_mask, src.depth(), mask_scale);

    if( src.channels() == 1 ) {
      // Monochrome image -> 1 channel + mask
      const cv::Mat mono_mask_planes[2] = { src, prepared_mask };
      cv::merge(mono_mask_planes, 2, image_to_write);
    }
    else if( src.channels() == 3 ) {
      // BGR -> RGB + Mask
      cv::Mat planes[3];
      cv::split(src, planes);

      const cv::Mat rgb_mask_planes[4] = {
          planes[2],      // Red
          planes[1],      // Green
          planes[0],      // Blue
          prepared_mask   // Mask
          };

      cv::merge(rgb_mask_planes, 4, image_to_write);
    }
    else {
      CF_ERROR("Not supported number of channels %d in input image ", src.channels());
      return false;
    }
  }

  // Add photometric tag
  const int photometric_tag = (image_to_write.channels() >= 3) ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK;
  int compression = g_default_tiff_compression;
  if ( !_params.empty() ) {
    for ( uint i = 0, n = _params.size(); i < n; i += 2 ) {
      if ( _params[i] == cv::ImwriteFlags::IMWRITE_TIFF_COMPRESSION ) {
        compression = _params[i + 1];
        break;
      }
    }
  }

  TIFF * tiff = TIFFOpen(filename.c_str(), "w");
  if( !tiff ) {
    CF_FATAL("TIFFOpen(%s) fails", filename.c_str());
    return false;
  }

  TIFFSetField(tiff, TIFFTAG_PHOTOMETRIC, static_cast<uint16_t>(photometric_tag));
  TIFFSetField(tiff, TIFFTAG_COMPRESSION, static_cast<uint16_t>(compression));
  TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH,  static_cast<uint32_t>(image_to_write.cols));
  TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(image_to_write.rows));
  TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, static_cast<uint16_t>(image_to_write.channels()));
  TIFFSetField(tiff, TIFFTAG_PLANARCONFIG, static_cast<uint16_t>(PLANARCONFIG_CONTIG));
  TIFFSetField(tiff, TIFFTAG_PAGENUMBER, static_cast<uint16_t>(0), static_cast<uint16_t>(1));

  const char * c_str_color = toCString(colorid);
  if ( c_str_color && *c_str_color ) {
    description += ssprintf("COLORTYP: %s\n", c_str_color);
  }
  if( has_mask ) {
    description += ssprintf("AstroTIFF_maskScale: %g\n", mask_scale);
    description += ssprintf("AstroTIFF_maskDepth: %d\n", mask_depth);
    static const uint16_t extras[] = { EXTRASAMPLE_ASSOCALPHA };
    TIFFSetField(tiff, TIFFTAG_EXTRASAMPLES, (uint16_t)1, extras);
  }

  if ( !description.empty() ) {
    TIFFSetField(tiff, TIFFTAG_IMAGEDESCRIPTION, description.c_str());
  }

  CF_DEBUG("\nfilename=%s\n"
      "image_to_write.depth()=%d",
      filename.c_str(),
      image_to_write.depth());

  switch (image_to_write.depth()) {
    case CV_8U:  write_tiff_image<uint8_t>(image_to_write, SAMPLEFORMAT_UINT, tiff);   break;
    case CV_8S:  write_tiff_image<int8_t>(image_to_write, SAMPLEFORMAT_INT, tiff);     break;
    case CV_16U: write_tiff_image<uint16_t>(image_to_write, SAMPLEFORMAT_UINT, tiff); break;
    case CV_16S: write_tiff_image<int16_t>(image_to_write, SAMPLEFORMAT_INT, tiff);   break;
    case CV_32S: write_tiff_image<int32_t>(image_to_write, SAMPLEFORMAT_INT, tiff);   break;
    case CV_32F: write_tiff_image<float>(image_to_write, SAMPLEFORMAT_IEEEFP, tiff);  break;
    case CV_64F: write_tiff_image<double>(image_to_write, SAMPLEFORMAT_IEEEFP, tiff); break;
  }

  if ( !TIFFWriteDirectory(tiff) ) {
    CF_FATAL("TIFFWriteDirectory(%s) fails", filename.c_str());
  }

  TIFFClose(tiff);
  return true;
}



static bool write_image(const std::string & filename, cv::InputArray image, cv::InputArray mask,
    const std::vector<int> & _params, enum COLORID colorid)
{
  if ( image.empty() ) {
    CF_CRITICAL("empty image specified to save as '%s'", filename.c_str());
    return false;
  }

  const std::string output_path = get_parent_directory(filename);
  if ( !output_path.empty() && !create_path(output_path) ) {
    CF_ERROR("create_path('%s') fails: %s", output_path.c_str(), strerror(errno));
    return false;
  }

  if( colorid == COLORID_UNKNOWN ) {
    switch (image.channels()) {
      case 1: colorid = COLORID_MONO; break;
      case 2: colorid = COLORID_OPTFLOW; break;
      case 3: colorid = COLORID_BGR; break;
      case 4: colorid = COLORID_BGRA; break;
    }
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

#if HAVE_CFITSIO
  if( strcasecmp(output_suffix.c_str(), ".fits") == 0 || strcasecmp(output_suffix.c_str(), ".fit") == 0
      || strcasecmp(output_suffix.c_str(), ".fts") == 0 ) {

    c_fits_writer fits;

    if( !fits.write(filename, image, colorid, mask) ) {
      CF_ERROR("c_fits_writer::write('%s') fails", filename.c_str());
      return false;
    }

    return true;
  }
#endif // HAVE_CFITSIO

  if ( strcasecmp(output_suffix.c_str(), ".tiff") == 0 || strcasecmp(output_suffix.c_str(), ".tif") == 0 ) {
    if ( write_tiff(image, mask, filename, _params, colorid) ) {
      return true;
    }

#if ( CV_VERSION_CURRRENT < CV_VERSION_INT(3,4,2) )
    CF_ERROR("CRITICAL WARNING: This OpenCV version is too old to support tiff");
#else
    std::vector<int> params = _params;
    params.emplace_back(cv::IMWRITE_TIFF_COMPRESSION);
    params.emplace_back(1);
#endif
  }

  // FIXME: temporary hack how to write image + mask using OpenCV
  if ( mask.empty() ) {
    if ( !cv::imwrite(filename, image, _params) ) {
      CF_ERROR("cv::imwrite('%s') fails", filename.c_str());
      return false;
    }
  }
  else {
    cv::Mat bgra;
    if ( !mergebgra(image.getMat(), mask.getMat(), bgra) ) {
      CF_ERROR("mergebgra('%s') fails", filename.c_str());
      return false;
    }
    if ( !cv::imwrite(filename, bgra, _params) ) {
      CF_ERROR("cv::imwrite('%s') fails", filename.c_str());
      return false;
    }
  }

  return true;
}

bool save_image(cv::InputArray image, cv::InputArray mask, const std::string & fname,
    enum COLORID colorid, const std::vector<int> & params)
{
  if ( !write_image(fname, image, mask, params, colorid) ) {
    CF_CRITICAL("write_image(%s) fails", fname.c_str());
    return false;
  }

  return true;
}


// Merge BGR and mask to to BGRA
bool mergebgra(const cv::Mat & input_image, const cv::Mat & input_mask, cv::Mat & output_image)
{
  const int cn = input_image.channels();
  if ( cn != 1 && cn != 3 ) {
    CF_ERROR("Invalid number of image channels: %d. Must be 1 or 3", cn);
    return false;
  }

  if ( input_mask.empty() ) {
    CF_ERROR("No alpha mask specified");
    return false;
  }

  if ( input_mask.channels() != 1 ) {
    CF_ERROR("Invalid number of channels in alpha mask %d. Must be 1",
        input_mask.channels());
    return false;
  }

  if ( input_mask.size() != input_image.size() ) {
    CF_ERROR("Image and mask sizes not match. image: %dx%d mask:%dx%d",
        input_image.cols, input_image.rows,
        input_mask.cols, input_mask.rows);
    return false;
  }


  cv::Mat alpha;

  if ( input_image.depth() == input_mask.depth() ) {
    alpha = input_mask;
  }
  else {

    double image_minval, image_maxval;
    get_data_range_for_pixel_depth(input_image.depth(),
        &image_minval, &image_maxval);

    double mask_minval, mask_maxval;
    get_data_range_for_pixel_depth(input_mask.depth(),
        &mask_minval, &mask_maxval);

    input_mask.convertTo(alpha, input_image.depth(), image_maxval / mask_maxval);
  }

  if ( cn == 1 ) {

    cv::Mat src[2] = { input_image, alpha };
    cv::merge(src, 2, output_image);
  }
  else { // if ( cn == 3 )

    cv::Mat bgra;

    cv::Mat & dst = (output_image.data == input_image.data ||
        output_image.data == input_mask.data) ?
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
