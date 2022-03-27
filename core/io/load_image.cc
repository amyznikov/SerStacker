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

/* get_extra_channels_count returns number of channels excluding
 * alpha and color channels
 */
static uint16_t get_extra_channels_count(uint16_t photomet, uint16_t spp, bool alpha)
{
  switch ( photomet )
  {
  case PHOTOMETRIC_RGB :
  case PHOTOMETRIC_YCBCR :
  case PHOTOMETRIC_CIELAB :
  case PHOTOMETRIC_ICCLAB :
  case PHOTOMETRIC_ITULAB :
  case PHOTOMETRIC_LOGLUV :
    return (spp >= 3) ? (spp - 3 - (alpha ? 1 : 0)) : (spp - 1 - (alpha ? 1 : 0));

  case PHOTOMETRIC_SEPARATED :
    return (spp - 4 - (alpha ? 1 : 0));
  }

  return spp - 1 - (alpha ? 1 : 0);
}

static int cvMatDepth(uint tiff_sample_format, uint tiff_bps)
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

static bool load_tiff_image (cv::Mat & image, const std::string & filename)
{
  CF_DEBUG("ENTER");
  image.release();

  if ( filename.empty() ) {
    CF_ERROR("No input file name specified");
    return false;
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

  //GList *images_list = NULL;
  int default_extra = TIFF_LOAD_UNASSALPHA;

  struct c_tiff_auto_close {
    TIFF * & tif;
    c_tiff_auto_close(TIFF * & _tif) :
        tif(_tif)
    {
    }
    ~c_tiff_auto_close()
    {
      if ( tif ) {
        TIFFClose(tif), tif = nullptr;
      }
    }
  } auto_close(tif);


  if ( !(tif = TIFFOpen(cfilename, "r")) ) {
    CF_ERROR("TIFFOpen('%s') fails", cfilename);
    return false;
  }

  if ( !TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &IMAGE_WIDTH) ) {
    CF_ERROR("Could not get image width from '%s'", cfilename);
    return false;
  }

  if ( !TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &IMAGE_HEIGHT) ) {
    CF_ERROR("Could not get image length from '%s'", cfilename);
    return false;
  }

  TIFFGetFieldDefaulted(tif, TIFFTAG_BITSPERSAMPLE, &BITS_PER_SAMPLE);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLEFORMAT, &SAMPLE_FORMAT);
  TIFFGetFieldDefaulted(tif, TIFFTAG_SAMPLESPERPIXEL, &SAMPLES_PER_PIXEL);
  TIFFGetField(tif, TIFFTAG_EXTRASAMPLES, &EXTRA, &EXTRA_TYPES);
  TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &PLANAR_CONFIG);

//  CF_DEBUG("IMAGE SIZE        : %u x %u", IMAGE_WIDTH, IMAGE_HEIGHT);
//  CF_DEBUG("BITS_PER_SAMPLE   : %u", BITS_PER_SAMPLE);
//  CF_DEBUG("SAMPLES_PER_PIXEL : %u", SAMPLES_PER_PIXEL);
//  CF_DEBUG("SAMPLE_FORMAT     : %u", SAMPLE_FORMAT);
//  CF_DEBUG("PLANAR_CONFIG     : %u", PLANAR_CONFIG);
//  CF_DEBUG("EXTRA             : %u EXTRA_TYPES = %p", EXTRA, EXTRA_TYPES);


  if ( BITS_PER_SAMPLE > 64 ) {
    CF_ERROR("Suspicious bit depth: %d for page 0. Image may be corrupt.", BITS_PER_SAMPLE);
    return false;
  }

  if ( IMAGE_WIDTH > MAX_IMAGE_SIZE || IMAGE_HEIGHT > MAX_IMAGE_SIZE ) {
    CF_ERROR("Invalid image dimensions (%u x %u). Image may be corrupt.", IMAGE_WIDTH, IMAGE_HEIGHT);
    return false;
  }

  if ( BITS_PER_SAMPLE < 8 ) {
    worst_case = true; /* I don't want to play with indexed formats, => RGBA */
  }
  else if ( BITS_PER_SAMPLE > 8 && BITS_PER_SAMPLE != 16 && BITS_PER_SAMPLE != 32 && BITS_PER_SAMPLE != 64 ) {
    worst_case = true; /* Wrong sample width => RGBA */
  }

  if ( TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &PHOTOMETRIC) ) {
    // CF_DEBUG("PHOTOMETRIC  : %u", PHOTOMETRIC);
  }
  else {
    uint16_t compression;

    if ( TIFFGetField(tif, TIFFTAG_COMPRESSION, &compression) &&
        (compression == COMPRESSION_CCITTFAX3 ||
            compression == COMPRESSION_CCITTFAX4 ||
            compression == COMPRESSION_CCITTRLE ||
            compression == COMPRESSION_CCITTRLEW) ) {

      PHOTOMETRIC = PHOTOMETRIC_MINISWHITE;
      CF_WARNING("Could not get photometric from '%s'. Image is CCITT compressed, assuming min-is-white", cfilename);
    }
    else {
      /* old AppleScan software misses out the photometric tag
       * (and incidentally assumes min-is-white, but xv
       * assumes min-is-black, so we follow xv's lead.  It's
       * not much hardship to invert the image later).
       */
      PHOTOMETRIC = PHOTOMETRIC_MINISBLACK;
      CF_WARNING("Could not get photometric from '%s'. Assuming min-is-black", cfilename);
    }
  }

  /* test if the extrasample represents an associated alpha channel... */
  if ( EXTRA > 0 && (EXTRA_TYPES[0] == EXTRASAMPLE_ASSOCALPHA) ) {
    alpha = true;
  }
  else if ( EXTRA > 0 && (EXTRA_TYPES[0] == EXTRASAMPLE_UNASSALPHA) ) {
    alpha = true;
  }
  else if ( EXTRA > 0 && (EXTRA_TYPES[0] == EXTRASAMPLE_UNSPECIFIED) ) {
    /* In non-interactive mode, we assume unassociated alpha if unspecified.
     * We don't output messages in interactive mode as the user
     * has already the ability to choose through a dialog. */
    CF_WARNING("Alpha channel type not defined for %s. Assuming alpha is not premultiplied", cfilename);

    switch ( default_extra ) {
    case TIFF_LOAD_ASSOCALPHA :
      alpha = true;
      break;
    case TIFF_LOAD_UNASSALPHA :
      alpha = true;
      break;
    default : /* GIMP_TIFF_LOAD_CHANNEL */
      alpha = false;
      break;
    }
  }
  else { /* EXTRA == 0 */

    if ( is_non_conformant_tiff(PHOTOMETRIC, SAMPLES_PER_PIXEL) ) {
      CF_WARNING("Image '%s' does not conform to the TIFF specification:\n"
          "ExtraSamples field is not set while extra channels are present.\n "
          "Assuming the first extra channel is non-premultiplied alpha.",
          cfilename);

      switch ( default_extra ) {
      case TIFF_LOAD_ASSOCALPHA :
        alpha = true;
        //save_transp_pixels = false;
        break;
      case TIFF_LOAD_UNASSALPHA :
        alpha = true;
        //save_transp_pixels = true;
        break;
      default : /* TIFF_LOAD_CHANNEL */
        alpha = false;
        break;
      }
    }
    else {
      alpha = false;
    }
  }

  if ( (EXTRA = get_extra_channels_count(PHOTOMETRIC, SAMPLES_PER_PIXEL, alpha)) > 0 ) {
    CF_WARNING("%u additional extra channels are not handled", EXTRA);
  }

  CF_DEBUG("[%s] worst_case=%d", cfilename, worst_case);

  if ( worst_case ) {

    image.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);

    if ( !TIFFReadRGBAImage(tif, IMAGE_WIDTH, IMAGE_HEIGHT, (uint32_t*) image.data, 0) ) {
      CF_ERROR("TIFFReadRGBAImage() fails");
      return false;
    }

    if ( alpha ) {
      cv::cvtColor(image, image, cv::COLOR_RGBA2BGRA);
    }
    else {
      cv::cvtColor(image, image, cv::COLOR_RGBA2BGR);
    }

  }
  else {


    uint32_t tile_width = 0;
    uint32_t tile_height = 0;

    if ( TIFFIsTiled(tif) ) {
      TIFFGetField(tif, TIFFTAG_TILEWIDTH, &tile_width);
      TIFFGetField(tif, TIFFTAG_TILELENGTH, &tile_height);

      if ( !tile_width || !tile_height || tile_width > IMAGE_WIDTH || tile_height > IMAGE_HEIGHT ) {
        CF_ERROR("Invalid tile size : width=%u height=%u", tile_width, tile_height);
        return false;
      }
    }
    else {
      tile_width = IMAGE_WIDTH;
      tile_height = 1;
    }

    const int ddepth = cvMatDepth(SAMPLE_FORMAT, BITS_PER_SAMPLE);
    if ( ddepth < 0 ) {
      CF_ERROR("cvMatDepth(SAMPLE_FORMAT=%u, BITS_PER_SAMPLE=%u) fails", SAMPLE_FORMAT, BITS_PER_SAMPLE);
      return false;
    }

    const int channels = SAMPLES_PER_PIXEL;

    if ( PLANAR_CONFIG == PLANARCONFIG_CONTIG ) {
      // CF_DEBUG("PLANAR_CONFIG == PLANARCONFIG_CONTIG");

      // load_contiguous

      image.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_MAKETYPE(ddepth, channels));

      if ( !TIFFIsTiled(tif) ) {

        for ( uint y = 0; y < IMAGE_HEIGHT; ++y ) {
          if ( TIFFReadScanline(tif, image.ptr<void>(y), y, 0) < 0 ) {
            CF_ERROR("TIFFReadScanline(y=%d) fails", y);
            return false;
          }
        }

      }
      else {

        cv::Mat tile;
        uint w, h;
        tile.create(tile_height, tile_width, image.type());

        for ( uint y = 0; y < IMAGE_HEIGHT; ) {

          h = y + tile_height <= IMAGE_HEIGHT ? tile_height : IMAGE_HEIGHT - y;

          for ( uint x = 0; x < IMAGE_WIDTH;  ) {

            if ( TIFFReadTile(tif, tile.data, x, y, 0, 0) < 0 ) {
              CF_ERROR("TIFFReadTile(x=%d y=%d) fails", x, y);
              return false;
            }

            w = x + tile_width <= IMAGE_WIDTH ? tile_width : IMAGE_WIDTH - x;

            tile(cv::Rect(0,0,w,h)).copyTo(image(cv::Rect(x, y, w, h)));

            x += w;
          }

          y += h;
        }

      }

      switch ( image.channels() ) {
      case 3 :
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        break;
      case 4 :
        cv::cvtColor(image, image, cv::COLOR_RGBA2BGRA);
        break;
      }
    }
    else {
      // load_separate
      CF_ERROR("Sorry, non-planar tiff images are not yet supported");
      return false;
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

bool load_image(const std::string & filename, cv::OutputArray output_image, cv::OutputArray output_mask)
{
  const std::string suffix = get_file_suffix(filename);

  if ( strcasecmp(suffix.c_str(), ".flo") == 0 ) {

    if ( output_mask.needed() ) {
      output_mask.release();
    }

    if ( output_image.needed() ) {

      cv::Mat m =
          cv::readOpticalFlow(filename);
      output_image.move(m);

      if ( output_image.empty() ) {
        return false;
      }
    }

    return true;
  }

  if ( !output_mask.needed() ) {
    if ( output_image.needed() ) {

      if ( strcasecmp(suffix.c_str(), ".tif") == 0 || strcasecmp(suffix.c_str(), ".tiff") == 0 ) {
        if ( !load_tiff_image(output_image.getMatRef(), filename) ) {
          return false;
        }
      }
      else {

        cv::Mat m = cv::imread(filename,
            cv::IMREAD_UNCHANGED);
        output_image.move(m);

        if ( output_image.empty() ) {
          return false;
        }
      }
    }

    return true;
  }


  cv::Mat tmp_image, tmp_mask;

  if ( strcasecmp(suffix.c_str(), ".tif") == 0 || strcasecmp(suffix.c_str(), ".tiff") == 0 ) {
    if ( !load_tiff_image(tmp_image, filename) ) {
      return false;
    }
  }
  else {
    if ( !(tmp_image = cv::imread(filename, cv::IMREAD_UNCHANGED)).data ) {
      return false;
    }
  }

  if ( tmp_image.channels() == 4 || tmp_image.channels() == 2 ) { // assume the last channel is mask
    splitbgra(tmp_image, tmp_image, &tmp_mask);
  }

  if ( output_image.needed() ) {
    output_image.move(tmp_image);
  }
  if ( output_mask.needed() ) {
    output_mask.move(tmp_mask);
  }

  return true;
}
