/*
 * load_image.cc
 *
 *  Created on: Jul 19, 2021
 *      Author: amyznikov
 *
 *   http://www.libtiff.org/libtiff.html
 *   http://www.libtiff.org/man/TIFFGetField.3t.html
 *
 */


#include "load_image.h"
#include <tiff.h>
#include <tiffio.h>
#include <core/io/c_fits_file.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>


namespace {

static constexpr int MAX_IMAGE_SIZE = 524288; // 2^19

enum {
  TIFF_LOAD_ASSOCALPHA,
  TIFF_LOAD_UNASSALPHA,
  TIFF_LOAD_CHANNEL
}; // DefaultExtra

/* is_non_conformant_tiff assumes TIFFTAG_EXTRASAMPLES was not set */
static bool is_non_conformant_tiff(uint16_t photomet, uint16_t spp)
{
  switch ( photomet )
  {
  case PHOTOMETRIC_RGB :
    case PHOTOMETRIC_YCBCR :
    case PHOTOMETRIC_CIELAB :
    case PHOTOMETRIC_ICCLAB :
    case PHOTOMETRIC_ITULAB :
    case PHOTOMETRIC_LOGLUV :
    return (spp > 3 || (spp == 2 && photomet != PHOTOMETRIC_RGB));

  case PHOTOMETRIC_SEPARATED :
    return (spp > 4);
  }

  return (spp > 1);
}

///* get_extra_channels_count returns number of channels excluding
// * alpha and color channels
// */
//static uint16_t get_extra_channels_count(uint16_t photomet, uint16_t spp, bool alpha)
//{
//  switch ( photomet )
//  {
//  case PHOTOMETRIC_RGB :
//  case PHOTOMETRIC_YCBCR :
//  case PHOTOMETRIC_CIELAB :
//  case PHOTOMETRIC_ICCLAB :
//  case PHOTOMETRIC_ITULAB :
//  case PHOTOMETRIC_LOGLUV :
//    return (spp >= 3) ? (spp - 3 - (alpha ? 1 : 0)) : (spp - 1 - (alpha ? 1 : 0));
//
//  case PHOTOMETRIC_SEPARATED :
//    return (spp - 4 - (alpha ? 1 : 0));
//  }
//
//  return spp - 1 - (alpha ? 1 : 0);
//}

static int cvMatDepth(uint32_t tiff_sample_format, uint32_t tiff_bps)
{
  switch ( tiff_bps ) {
  case 1 :
    case 2 :
    case 4 :
    case 8 :
    switch ( tiff_sample_format ) {
    case SAMPLEFORMAT_INT :
      return CV_8S;
    case SAMPLEFORMAT_UINT :
      return CV_8U;
    }
    break;

  case 16 :
    switch ( tiff_sample_format ) {
    case SAMPLEFORMAT_INT :
      return CV_16S;
    case SAMPLEFORMAT_UINT :
      return CV_16U;
    }
    break;

  case 32 :
    switch ( tiff_sample_format ) {
    case SAMPLEFORMAT_INT :
      case SAMPLEFORMAT_UINT :
      return CV_32S;
    case SAMPLEFORMAT_IEEEFP :
      return CV_32F;
    }
    break;

  case 64 :
    switch ( tiff_sample_format ) {
    case SAMPLEFORMAT_IEEEFP :
      return CV_64F;
    }
    break;
  }

  CF_ERROR("Unsupported combination for bps %u and sample format %u",
      tiff_bps, tiff_sample_format);

  return -1;
}

static bool load_tiff_image(const std::string & filename,
    cv::OutputArray output_image, cv::OutputArray output_mask,
    enum COLORID * output_colorid)
{
  if ( filename.empty() ) {
    CF_ERROR("No input file name specified");
    return false;
  }

  if ( output_colorid ) {
    *output_colorid = COLORID_UNKNOWN;
  }

  const char * cfilename = filename.c_str();
  TIFF * tif = nullptr;
  uint32_t IMAGE_WIDTH = 0;
  uint32_t IMAGE_HEIGHT = 0;
  uint16_t BITS_PER_SAMPLE = 0;
  uint16_t SAMPLE_FORMAT = 0;
  uint16_t SAMPLES_PER_PIXEL = 0;
  uint16_t PHOTOMETRIC = 0;
  uint16_t PLANAR_CONFIG = 0;
  uint16_t EXTRA = 0;
  uint16_t * EXTRA_TYPES = nullptr;
  bool alpha = false;
  bool worst_case = false;
  int default_extra = TIFF_LOAD_UNASSALPHA;

  // custom application metadata
  double max_weight = 1.0;
  enum COLORID local_colorid = COLORID_UNKNOWN;
  bool is_analog_mask = false;

  struct c_tiff_auto_close {
    TIFF * & tif;
    c_tiff_auto_close(TIFF * & _tif) : tif(_tif) {}
    ~c_tiff_auto_close() { if ( tif ) { TIFFClose(tif); tif = nullptr; } }
  } auto_close(tif);

  if ( !(tif = TIFFOpen(cfilename, "r")) ) {
    CF_ERROR("TIFFOpen('%s') fails", cfilename);
    return false;
  }

  if ( !TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &IMAGE_WIDTH) ||
       !TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &IMAGE_HEIGHT) ) {
    CF_ERROR("Could not get image dimensions from '%s'", cfilename);
    return false;
  }

  TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &BITS_PER_SAMPLE);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &SAMPLE_FORMAT);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &SAMPLES_PER_PIXEL);
  TIFFGetField(tif, TIFFTAG_EXTRASAMPLES, &EXTRA, &EXTRA_TYPES);
  TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &PLANAR_CONFIG);

  if ( BITS_PER_SAMPLE > 64 || IMAGE_WIDTH > MAX_IMAGE_SIZE || IMAGE_HEIGHT > MAX_IMAGE_SIZE ) {
    CF_ERROR("Invalid or corrupt image metadata in '%s'", cfilename);
    return false;
  }

  if ( BITS_PER_SAMPLE < 8 || (BITS_PER_SAMPLE > 8 && BITS_PER_SAMPLE != 16 && BITS_PER_SAMPLE != 32 && BITS_PER_SAMPLE != 64) ) {
    // Indexed or non-standard format -> go into RGBA
    worst_case = true;
  }

  if ( !TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &PHOTOMETRIC) ) {
    uint16_t compression;
    if ( TIFFGetField(tif, TIFFTAG_COMPRESSION, &compression) &&
        (compression == COMPRESSION_CCITTFAX3 || compression == COMPRESSION_CCITTFAX4 ||
         compression == COMPRESSION_CCITTRLE || compression == COMPRESSION_CCITTRLEW) ) {
      PHOTOMETRIC = PHOTOMETRIC_MINISWHITE;
    }
    else {
      PHOTOMETRIC = PHOTOMETRIC_MINISBLACK;
    }
  }

  // Basic definition of Alpha presence according to LibTIFF standards
  if ( EXTRA > 0 && (EXTRA_TYPES[0] == EXTRASAMPLE_ASSOCALPHA || EXTRA_TYPES[0] == EXTRASAMPLE_UNASSALPHA) ) {
    alpha = true;
  }
  else if ( EXTRA > 0 && EXTRA_TYPES[0] == EXTRASAMPLE_UNSPECIFIED ) {
    alpha = (default_extra != 3); // old logic
  }
  else if ( EXTRA == 0 && is_non_conformant_tiff(PHOTOMETRIC, SAMPLES_PER_PIXEL) ) {
    alpha = (default_extra != 3);
  }

  // Parse the text description if exists
  const char* description = nullptr;
  if ( TIFFGetField(tif, TIFFTAG_IMAGEDESCRIPTION, &description) == 1 && description != nullptr ) {
    if( sscanf(description, "AstroTIFF_MaxWeight: %lf", &max_weight) == 1 ) {
      is_analog_mask = true;
    }

    const char* color_pos = strstr(description, "COLORTYP: ");
    if ( color_pos != nullptr ) {
      fromString(color_pos + 10, &local_colorid);
    }
  }

//  CF_DEBUG("\nworst_case=%d\n"
//      "BITS_PER_SAMPLE=%u\n"
//      "IMAGE_WIDTH=%u IMAGE_HEIGHT=%u\n"
//      "BITS_PER_SAMPLE = %u SAMPLE_FORMAT=%u SAMPLES_PER_PIXEL=%u\n"
//      "PHOTOMETRIC=%u\n"
//      "EXTRA=%u\n"
//      "alpha=%d\n"
//      "max_weight=%g\n"
//      ,worst_case,
//      BITS_PER_SAMPLE,
//      IMAGE_WIDTH, IMAGE_HEIGHT,
//      BITS_PER_SAMPLE, SAMPLE_FORMAT, SAMPLES_PER_PIXEL,
//      PHOTOMETRIC,
//      EXTRA,
//      alpha,
//      max_weight);


  cv::Mat raw_loaded_image;

  if ( worst_case ) {
    // WORST CASE (Old RGBA buffer)
    // Convert RGBA to the standard OpenCV format (supporting BGR/BGRA)

    raw_loaded_image.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);
    if ( !TIFFReadRGBAImage(tif, IMAGE_WIDTH, IMAGE_HEIGHT, (uint32_t*)raw_loaded_image.data, 0) ) {
      CF_ERROR("TIFFReadRGBAImage() fails");
      return false;
    }

    if ( alpha ) {
      cv::cvtColor(raw_loaded_image, raw_loaded_image, cv::COLOR_RGBA2BGRA);
    }
    else {
      cv::cvtColor(raw_loaded_image, raw_loaded_image, cv::COLOR_RGBA2BGR);
    }
  }
  else {
    // STANDARD ROW/TILE READING
    if ( PLANAR_CONFIG != PLANARCONFIG_CONTIG ) {
      CF_ERROR("Sorry, non-planar tiff images are not yet supported");
      return false;
    }

    uint32_t tile_width = TIFFIsTiled(tif) ? 0 : IMAGE_WIDTH;
    uint32_t tile_height = TIFFIsTiled(tif) ? 0 : 1;
    if ( TIFFIsTiled(tif) ) {
      TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tile_width);
      TIFFGetField(tif, TIFFTAG_TILELENGTH, &tile_height);
      if ( !tile_width || !tile_height || tile_width > IMAGE_WIDTH || tile_height > IMAGE_HEIGHT ) {
        CF_ERROR("Invalid tile size : width=%u height=%u", tile_width, tile_height);
        return false;
      }
    }

    const int ddepth = cvMatDepth(SAMPLE_FORMAT, BITS_PER_SAMPLE);
    if ( ddepth < 0 ) {
      CF_ERROR("cvMatDepth() fails for format=%u, bits=%u", SAMPLE_FORMAT, BITS_PER_SAMPLE);
      return false;
    }

    raw_loaded_image.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_MAKETYPE(ddepth, SAMPLES_PER_PIXEL));

    if ( !TIFFIsTiled(tif) ) {
      for ( uint32_t y = 0; y < IMAGE_HEIGHT; ++y ) {
        if ( TIFFReadScanline(tif, raw_loaded_image.ptr(y), y, 0) < 0 ) {
          CF_ERROR("TIFFReadScanline(y=%d) fails", y);
          return false;
        }
      }
    }
    else {
      cv::Mat tile(tile_height, tile_width, raw_loaded_image.type());
      for ( uint32_t y = 0; y < IMAGE_HEIGHT; ) {
        const uint32_t h = y + tile_height <= IMAGE_HEIGHT ? tile_height : IMAGE_HEIGHT - y;
        for ( uint32_t x = 0; x < IMAGE_WIDTH; ) {
          if ( TIFFReadTile(tif, tile.data, x, y, 0, 0) < 0 ) {
            CF_ERROR("TIFFReadTile(x=%d y=%d) fails", x, y);
            return false;
          }
          const uint32_t w = x + tile_width <= IMAGE_WIDTH ? tile_width : IMAGE_WIDTH - x;
          tile(cv::Rect(0, 0, w, h)).copyTo(raw_loaded_image(cv::Rect(x, y, w, h)));
          x += w;
        }
        y += h;
      }
    }
  }

  cv::Mat final_image, final_mask;
  const int total_channels = raw_loaded_image.channels();
  const int ddepth = raw_loaded_image.depth();
  const cv::Size img_size = raw_loaded_image.size();

//  CF_DEBUG("\nraw_loaded_image=%dx%d channels=%d depth=%d\n",
//      raw_loaded_image.cols, raw_loaded_image.rows, raw_loaded_image.channels(), raw_loaded_image.depth());

  if ( total_channels == 1 ) { // Mono
    final_image = raw_loaded_image;
  }
  else if ( total_channels == 2 ) { // Mono + Mask
    final_image.create(img_size, CV_MAKETYPE(ddepth, 1));
    final_mask.create(img_size, CV_MAKETYPE(ddepth, 1));

    const cv::Mat src[] = { raw_loaded_image };
    cv::Mat dst[] = { final_image, final_mask };

    const int from_to[] = { 0, 0,  1, 0 };
    cv::mixChannels(src, 1, dst, 2, from_to, 2);
  }
  else if ( total_channels == 3 ) { // RGB -> BGR
    // Most fast way
    cv::cvtColor(raw_loaded_image, final_image, cv::COLOR_RGB2BGR);
  }
  else if ( total_channels == 4 ) { // RGBA -> BGR + MASK


    final_image.create(img_size, CV_MAKETYPE(ddepth, 3));
    final_mask.create(img_size, CV_MAKETYPE(ddepth, 1));

    const cv::Mat src[] = { raw_loaded_image };
    cv::Mat dst[] = { final_image, final_mask };

    const int from_to[] = { 0, 2,  1, 1,  2, 0,  3, 3 };
    cv::mixChannels(src, 1, dst, 2, from_to, 4);

//    CF_DEBUG("\nRGBA -> BGR + MASK: \n"
//        "final_image: %dx%d channels=%d depth=%d\n"
//        "final_mask : %dx%d channels=%d depth=%d\n",
//        final_image.cols, final_image.rows, final_image.channels(), final_image.depth(),
//        final_mask.cols, final_mask.rows, final_mask.channels(), final_mask.depth());

  }
  else {
    CF_ERROR("Unsupported number of channels in TIFF: %d", total_channels);
    return false;
  }

  if ( output_colorid ) {
    if( local_colorid == COLORID_UNKNOWN ) {
      switch (final_image.channels()) {
        case 1:
          local_colorid = COLORID_MONO;
          break;
        case 3:
          local_colorid = COLORID_BGR;
          break;
      }
    }

    *output_colorid = local_colorid;
  }

  if ( output_image.needed() ) {
    output_image.assign(final_image);
  }

//  CF_DEBUG("\noutput_mask.needed()=%d final_mask: %dx%d\n", output_mask.needed(), final_mask.cols, final_mask.rows);
  if( output_mask.needed() ) {
    if( final_mask.empty() ) {
      output_mask.release();
    }
    else if( is_analog_mask ) {
      if( std::abs(max_weight - 1.0) < std::numeric_limits<float>::epsilon() ) {
        output_mask.move(final_mask);
      }
      else {
        cv::multiply(final_mask, max_weight, output_mask);
      }
    }
    else if( final_mask.depth() != CV_8U ) {
      cv::compare(final_mask, 0, output_mask, cv::CMP_NE);
    }
    else {
      output_mask.move(final_mask);
    }
  }

  return true;
}

}



// Split BGRA to BGR and mask
bool splitbgra(const cv::Mat & input_image, cv::Mat & output_image, cv::Mat * output_alpha_mask)
{
  const int cn = input_image.channels();

  if ( cn == 2 ) {

    cv::Mat dst[2];

    cv::split(input_image, dst);

    output_image = std::move(dst[0]);

    if ( output_alpha_mask ) {
      cv::compare(dst[1], 0, *output_alpha_mask, cv::CMP_GT);
    }

    return true;
  }

  if ( cn == 4 ) {

    cv::Mat dst[2];

    dst[0].create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), cn - 1));

    dst[1].create(input_image.size(),
        CV_MAKETYPE(input_image.depth(), 1));

    static const int from_to[] = { 0, 0, 1, 1, 2, 2, 3, 3 };

    cv::mixChannels(&input_image, 1, dst, 2, from_to, 4);

    output_image = std::move(dst[0]);

    if ( output_alpha_mask ) {
      cv::compare(dst[1], 0, *output_alpha_mask, cv::CMP_GT);
    }

    return true;
  }

  return false;
}



//bool load_image(cv::OutputArray dst, const std::string & filename)
//{
//  const std::string suffix = get_file_suffix(filename);
//
//  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {
//    return (dst = cv::readOpticalFlow(filename)).data != nullptr;
//  }
//
//  if ( strcasecmp(suffix.c_str(), ".tif") == 0 || strcasecmp(suffix.c_str(), ".tiff") == 0  ) {
//    if ( load_tiff_image(dst, filename) ) {
//      // CF_DEBUG("[%s] loaded with load_tiff_image()", filename.c_str());
//      return true;
//    }
//  }
//
//  return (dst = cv::imread(filename, cv::IMREAD_UNCHANGED)).data != nullptr;
//}

bool load_image(const std::string & filename, cv::OutputArray output_image, cv::OutputArray output_mask,
    enum COLORID * output_colorid)
{
  const std::string suffix = get_file_suffix(filename);
  const char * csuffix = suffix.c_str();

  if( strcasecmp(csuffix, ".flo") == 0 ) {
    if( output_colorid ) {
      *output_colorid = COLORID_OPTFLOW;
    }
    if( output_mask.needed() ) {
      output_mask.release();
    }
    if( output_image.needed() ) {
      output_image.assign(cv::readOpticalFlow(filename));
      if( output_image.empty() ) {
        return false;
      }
    }
    return true;
  }

#if HAVE_CFITSIO
  if( strcasecmp(csuffix, ".fits") == 0 || strcasecmp(csuffix, ".fit") == 0
      || strcasecmp(csuffix, ".fts") == 0 ) {

    c_fits_reader fits;

    if ( !fits.open(filename) ) {
      CF_ERROR("fits.open('%s') fails", filename.c_str());
      return false;
    }

    if ( !fits.read(output_image, -1, output_mask) ) {
      CF_ERROR("fits.read('%s') fails", filename.c_str());
      return false;
    }

    if ( output_colorid )  {
      *output_colorid = fits.colorid();
    }

    return true;
  }
#endif // HAVE_CFITSIO

  if ( strcasecmp(csuffix, ".tif") == 0 || strcasecmp(csuffix, ".tiff") == 0 ) {
//    CF_DEBUG("call load_tiff_image('%s') ", filename.c_str());
    if ( load_tiff_image(filename, output_image, output_mask, output_colorid) ) {
//      CF_DEBUG("load_tiff_image: image=%dx%d channels=%d depth=%d mask=%dx%d channels=%d depth=%d",
//          output_image.cols(), output_image.rows(), output_image.channels(), output_image.depth(),
//          output_mask.cols(), output_mask.rows(), output_mask.channels(), output_mask.depth());
      return true;
    }
//    CF_DEBUG("load_tiff_image('%s') fails, fallback to opencv ", filename.c_str());
  }


  // Try fall back to OpenCV loader

  cv::Mat m = cv::imread(filename, cv::IMREAD_UNCHANGED);
  if ( m.empty() ) {
    CF_ERROR("cv::imread('%s') fails", filename.c_str());
    return false;
  }

  cv::Mat final_image, final_mask;
  const int total_channels = m.channels();
  const int ddepth = m.depth();
  const cv::Size img_size = m.size();

  if ( total_channels == 1 ) { // Mono
    final_image = m;
  }
  else if ( total_channels == 2 ) { // Mono + Mask
    final_image.create(img_size, CV_MAKETYPE(ddepth, 1));
    final_mask.create(img_size, CV_MAKETYPE(ddepth, 1));

    const cv::Mat src[] = { m };
    cv::Mat dst[] = { final_image, final_mask };

    const int from_to[] = { 0, 0,  1, 0 };
    cv::mixChannels(src, 1, dst, 2, from_to, 2);
  }
  else if ( total_channels == 3 ) { // RGB -> BGR
    // Most fast way
    cv::cvtColor(m, final_image, cv::COLOR_RGB2BGR);
  }
  else if ( total_channels == 4 ) { // RGBA -> BGR + MASK
    final_image.create(img_size, CV_MAKETYPE(ddepth, 3));
    final_mask.create(img_size, CV_MAKETYPE(ddepth, 1));

    const cv::Mat src[] = { m };
    cv::Mat dst[] = { final_image, final_mask };

    const int from_to[] = { 0, 2,  1, 1,  2, 0,  3, 3 };
    cv::mixChannels(src, 1, dst, 2, from_to, 4);
  }
  else {
    CF_ERROR("Unsupported number of channels in TIFF: %d", total_channels);
    return false;
  }

  if( output_colorid ) {
    switch (final_image.channels()) {
      case 1:
        *output_colorid = COLORID_MONO;
        break;
      case 3:
        *output_colorid = COLORID_BGR;
        break;
    }
  }

  if ( output_image.needed() ) {
    output_image.assign(final_image);
  }

  if ( output_mask.needed() ) {
    output_mask.assign(final_mask);
  }

  return true;
}
