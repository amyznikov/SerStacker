/*
 * c_fits_file.cc
 *
 *  Created on: Jan 9, 2021
 *      Author: amyznikov
 *
 *  TODO: https://root.cern/doc/v610/TFITS_8cxx_source.html
 */

#include "c_fits_file.h"

#if HAVE_CFITSIO

#include <core/proc/run-loop.h>
#include <core/debug.h>

static const char * fits_errmsg(int status)
{
  static thread_local char errmsg[128];
  fits_get_errstatus(status, errmsg);
  return errmsg;
}

static int bitpix2ddepth(int bitpix)
{
  switch ( bitpix ) {
  case BYTE_IMG :  // 8
    return CV_8U;
  case SBYTE_IMG :  //    10
    return CV_8S;
  case SHORT_IMG :  //    16
    return CV_16S;
  case USHORT_IMG :  //    20
    return CV_16U;
  case LONG_IMG :  //    32
    return CV_32S;
  case ULONG_IMG :  //    40
    return CV_32S;
  case FLOAT_IMG :  //  -32
    return CV_32F;
  case DOUBLE_IMG :  //  -64
    return CV_64F;

    // case LONGLONG_IMG :  // 64
    // case ULONGLONG_IMG :  //    80
  default :
    break;
  }

  return -1;
}

static bool cvdepth2fits(int depth, int * bitpix, int * datatype)
{
  switch (depth) {
    case CV_8U:
      *bitpix = BYTE_IMG;
      *datatype = TBYTE;
      break;
    case CV_8S:
      *bitpix = SHORT_IMG;
      *datatype = TSBYTE;
      break;
    case CV_16U:
      *bitpix = USHORT_IMG;
      *datatype = TUSHORT;
      break; // cfitsio will automatically add BZERO=32768
    case CV_16S:
      *bitpix = SHORT_IMG;
      *datatype = TSHORT;
      break;
    case CV_32S:
      *bitpix = LONG_IMG;
      *datatype = TINT;
      break;
    case CV_32F:
      *bitpix = FLOAT_IMG;
      *datatype = TFLOAT;
      break;
    case CV_64F:
      *bitpix = DOUBLE_IMG;
      *datatype = TDOUBLE;
      break;
    default:
      return false;
  }

  return true;
}

static inline int cvdepth2bitpix(int depth)
{
  int bitpix = -1, datatype = -1;
  cvdepth2fits(depth, &bitpix, &datatype);
  return bitpix;
}

static inline int cvdepth2datatype(int depth)
{
  int bitpix = -1, datatype = -1;
  cvdepth2fits(depth, &bitpix, &datatype);
  return datatype;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief default d'tor closes fits file if was opened */
c_fits_file::~c_fits_file()
{
  close();
}

/** @brief return cfitsio status code of last operation */
int c_fits_file::status() const
{
  return _status;
}

/** @brief return cfitsio error message associated with status code of last operation */
const char * c_fits_file::statusmsg() const
{
  return fits_errmsg(_status);
}

/** @brief close fits file */
void c_fits_file::close()
{
  if ( fp ) {
    int status = 0; /* don't alter this->status_ value */
    fits_close_file(fp, &status);
    fp = nullptr;
  }
}

/** @brief Read-Only access to fits header row by row */
const std::vector<c_fits_file::FKEY> & c_fits_file::header() const
{
  return _header;
}

/** @brief total number of HDUs in fits file */
int c_fits_file::num_hdus() const
{
  return _num_hdus;
}

/** @brief current bitpix of loaded data */
int c_fits_file::bitpix() const
{
  return _bitpix;
}

/** @brief suggested cv::Mat depth for current bitpix of loaded data  */
int c_fits_file::ddepth() const
{
  return bitpix2ddepth(_bitpix);
}

/** @brief suggested cv::Mat channels count for current naxes */
int c_fits_file::channels() const
{
  if ( _naxes.size() == 3 && _naxes[2] == 3 ) {
    return 3;
  }
  if ( _naxes.size() == 2 ) {
    return 1;
  }

  /* not supported combination */
  return 0;
}

/** @brief return colorid for current image */
enum COLORID c_fits_file::colorid() const
{
  return _colorid;
}

/** @brief suggested cv::Mat size for current naxes */
cv::Size c_fits_file::size() const
{
  if ( _naxes.size() >= 2 ) {
    return cv::Size(_naxes[0], _naxes[1]);
  }
  return cv::Size(-1, -1);
}


/** @brief number of dimensions of the image */
int c_fits_file::naxis() const
{
  return _naxes.size();
}

/** @brief current bzero of loaded data  */
double c_fits_file::bzero() const
{
  return _bzero;
}

/** @brief size of each dimension */
const std::vector<long> & c_fits_file::naxes() const
{
  return _naxes;
}

/** @brief size of image dimension specified by index */
long c_fits_file::naxes(int index) const
{
  return _naxes[index];
}

const char* c_fits_file::colorid2fits(enum COLORID colorid)
{
  switch (colorid) {
    case COLORID_BAYER_RGGB:
      return "RGGB";
    case COLORID_BAYER_GRBG:
      return "GRBG";
    case COLORID_BAYER_GBRG:
      return "GBRG";
    case COLORID_BAYER_BGGR:
      return "BGGR";
    case COLORID_BAYER_CYYM:
      return "CYYM";
    case COLORID_BAYER_YCMY:
      return "YCMY";
    case COLORID_BAYER_YMCY:
      return "YMCY";
    case COLORID_BAYER_MYYC:
      return "MYYC";

    case COLORID_MONO:
      return "MONO";
    case COLORID_RGB:
      return "RGB";
    case COLORID_BGR:
      return "BGR";
    case COLORID_BGRA:
      return "BGRA";

    case COLORID_OPTFLOW:
      return "OPTFLOW";

    default:
      break;
  }

  return nullptr;
}

enum COLORID c_fits_file::fits2colorid(const char * cname)
{
  if( cname && *cname ) {

    constexpr static struct S {
      const char * pattern;
      enum COLORID colorid;
    } known_colorids[] = {
        { "RGGB", COLORID_BAYER_RGGB },
        { "GRBG", COLORID_BAYER_GRBG },
        { "GBRG", COLORID_BAYER_GBRG },
        { "BGGR", COLORID_BAYER_BGGR },
        { "CYYM", COLORID_BAYER_CYYM },
        { "YCMY", COLORID_BAYER_YCMY },
        { "YMCY", COLORID_BAYER_YMCY },
        { "MYYC", COLORID_BAYER_MYYC },

        { "MONO", COLORID_MONO },
        { "GRAY", COLORID_MONO },
        { "GREY", COLORID_MONO },
        { "RAW", COLORID_MONO },
        { "RGB", COLORID_RGB },
        { "BGR", COLORID_BGR },
        { "BGRA", COLORID_BGRA },

        { "OPTFLOW", COLORID_OPTFLOW },
    };

    for( uint i = 0; i < sizeof(known_colorids) / sizeof(known_colorids[0]); ++i ) {
      if( strcasecmp(cname, known_colorids[i].pattern) == 0 ) {
        return known_colorids[i].colorid;
      }
    }
  }

  return COLORID_UNKNOWN;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

c_fits_reader::c_fits_reader(const std::string & filename)
{
  open(filename);
}


bool c_fits_reader::is_open() const
{
  return fp != nullptr;
}

bool c_fits_reader::open(const std::string & filename)
{
  int numkeys = 0;
  int naxis = 0;
  char bayerpat[FLEN_VALUE] = "";
  char colortyp[FLEN_VALUE] = "";

  close();

  _header.clear();
  _naxes.clear();
  _num_hdus = 0;
  _bitpix = 0;
  _bzero = 0;
  _bscale = 1.0;
  _colorid = COLORID_UNKNOWN;

  /*
   * Open fits file
   */
  fits_open_diskfile(&fp, filename.c_str(), READONLY, &(_status = 0));
  if( !fp || _status ) {
    CF_ERROR("'%s': fits_open_file() fails: %s", filename.c_str(), fits_errmsg(_status));
    return false;
  }

  /*
   * Read FITS header row by row
   */
  if( fits_get_hdrspace(fp, &numkeys, NULL, &(_status = 0)) ) {
    CF_ERROR("'%s': fits_get_hdrspace() fails: %s", filename.c_str(), fits_errmsg(_status));
    goto end;
  }

  _header.reserve(numkeys);

  for( int keynum = 1; keynum <= numkeys; ++keynum ) {
    FKEY c;
    char keyname[256] = "";
    char value[256] = "";
    char comment[256] = "";

    if( fits_read_keyn(fp, keynum, keyname, value, comment, &(_status = 0)) ) {
      CF_ERROR("'%s': fits_read_keyn(keynum=%d) fails: %s", filename.c_str(), keynum, fits_errmsg(_status));
      goto end;
    }

    c.name = keyname; // Исправлено: в вашей структуре это c.keyname, а не c.name
    c.value = value;
    c.comment = comment;
    _header.emplace_back(c);
  }

  /*
   * Save num HDUs
   */
  if( fits_get_num_hdus(fp, &_num_hdus, &(_status = 0)) ) {
    CF_ERROR("'%s': fits_get_num_hdus() fails: %s", filename.c_str(), fits_errmsg(_status));
    goto end;
  }

  /*
   * Try to load image format from primary HDU
   */
  if( fits_get_img_type(fp, &_bitpix, &(_status = 0)) ) {
    CF_ERROR("'%s': fits_get_img_type() fails: %s", filename.c_str(), fits_errmsg(_status));
    goto end;
  }

  if( fits_read_key(fp, TDOUBLE, "BZERO", &_bzero, nullptr, &(_status = 0)) ) {
    if( _status == KEY_NO_EXIST ) {
      _status = 0;
      _bzero = 0.0;
      if( _bitpix == SHORT_IMG ) {
        _bitpix = USHORT_IMG;
      }
    }
    else {
      CF_ERROR("'%s': fits_read_key(BZERO) fails: %s", filename.c_str(), fits_errmsg(_status));
      goto end;
    }
  }
  else if( _bitpix == SHORT_IMG && _bzero != 0 ) {
    _bitpix = USHORT_IMG;
  }
  else if( _bitpix == LONG_IMG && _bzero != 0.0 ) {
    _bitpix = ULONG_IMG;
  }

  if( fits_read_key(fp, TDOUBLE, "BSCALE", &_bscale, nullptr, &(_status = 0)) ) {
    if( _status == KEY_NO_EXIST ) {
      _status = 0;
      _bscale = 1.0;
    }
    else {
      CF_ERROR("'%s': fits_read_key(BSCALE) fails: %s", filename.c_str(), fits_errmsg(_status));
      goto end;
    }
  }

  /*
   * Get image dimensions
   */
  if( fits_get_img_dim(fp, &naxis, &(_status = 0)) ) {
    CF_ERROR("'%s': fits_get_img_dim() fails: %s", filename.c_str(), fits_errmsg(_status));
    goto end;
  }

  _naxes.resize(naxis, 0);
  if( fits_get_img_size(fp, naxis, _naxes.data(), &(_status = 0)) ) {
    CF_ERROR("'%s': fits_get_img_size() fails: %s", filename.c_str(), fits_errmsg(_status));
    goto end;
  }

  /*
   * Get COLORID
   */
  if( _naxes.size() == 2 ) {
    fits_read_key(fp, TSTRING, "BAYERPAT", bayerpat, nullptr, &(_status = 0));
    if( _status == KEY_NO_EXIST ) {
      _status = 0;
    }

    fits_read_key(fp, TSTRING, "COLORTYP", colortyp, nullptr, &(_status = 0));
    if( _status == KEY_NO_EXIST ) {
      _status = 0;
    }

    if( *bayerpat ) {
      _colorid = fits2colorid(bayerpat);
    }
    else if( *colortyp ) {
      _colorid = fits2colorid(colortyp);
    }
  }

  if( _colorid == COLORID_UNKNOWN ) {
    switch (channels()) {
      case 1:
        _colorid = COLORID_MONO;
        break;
      case 3:
        _colorid = COLORID_BGR;
        break;
    }
  }

end:
  if( _status ) {
    close();
  }

  return _status == 0;
}

bool c_fits_reader::read(cv::OutputArray outImage, int ddepth,
    cv::OutputArray outWeights /*= cv::noArray()*/)
{
  if ( !is_open() ) {
    return false;
  }

  if ( _naxes.size() < 1 || _naxes.size()  > 3 ) {
    CF_ERROR("Unsupported FITS image with naxes_=%zu", _naxes.size());
    return false;
  }

  if ( _naxes.size() == 3 && _naxes[2] != 3 ) {
    CF_ERROR("Unsupported FITS image with %ld channels", _naxes[2]);
    return false;
  }

  if ( outImage.fixedType() ) {
    ddepth = outImage.depth();
  }
  else if ( ddepth < 0 && (ddepth = bitpix2ddepth(_bitpix)) < 0 ) {
    CF_ERROR("Unsupported FITS image with bitpix=%d", _bitpix);
    return false;
  }

  const int cn = channels();
  if ( cn < 1  ) {
    CF_ERROR("Unsupported FITS image with %d channels", cn);
    return false;
  }

  outImage.create(size(), CV_MAKETYPE(ddepth, cn));
  if ( !outImage.isContinuous() ) {
    outImage.release();
    outImage.create(size(), CV_MAKETYPE(ddepth, cn));
    return false;
  }

  cv::Mat & dst = outImage.getMatRef();

  /* Coordinate in each dimension of the first pixel to be read */
  const int datatype = cvdepth2datatype(ddepth);
  long orig[3] = { 1L, 1L, 1L };
  int zero = 0;

  uint nbdata = _naxes[0] * _naxes[1];
  if ( _naxes.size() > 2 ) {
    nbdata *= _naxes[2];
  }

  if ( cn == 1 ) {
    if ( fits_read_pix(fp, datatype, orig, nbdata, &zero, dst.data, &zero, &(_status = 0)) ) {
      CF_ERROR("fits_read_pix() fails: %s", fits_errmsg(_status));
      return false;
    }
  }
  else if ( cn == 3 ) {
    const int w = _naxes[0];
    const int h = _naxes[1];
    cv::Mat tmp(3 * h, w, ddepth);

    if( fits_read_pix(fp, datatype, orig, nbdata, &zero, tmp.data, &zero, &(_status = 0)) ) {
      CF_ERROR("fits_read_pix() fails: %s", fits_errmsg(_status));
      return false;
    }

    const cv::Mat planes[3] = {
      tmp.rowRange(2 * h, 3 * h), // Blue
      tmp.rowRange(h, 2 * h),     // Green
      tmp.rowRange(0, h)          // Red
    };

    cv::merge(planes, 3, dst);
  }
  else {
    CF_ERROR("Unsupported FITS image with %d channels", cn);
    return false;
  }

  // Check if there is a second HDU layer physically in the file
  if ( outWeights.needed() ) {

    bool haveWeights = false;

    if ( _num_hdus > 1 ) {
      // Go to the second HDU (in cfitsio indexing starts with 1, i.e. 1 is Primary, 2 is Extension)

      int hdu_type = 0;
      fits_movabs_hdu(fp, 2, &hdu_type, &(_status = 0));
      if ( _status == 0 && hdu_type == IMAGE_HDU ) {

        // Matrix size and depth for weights/masks based on wbitpix (always singke channel)

        int wbitpix = 0;
        long wnaxes[2] = { 0, 0 };
        int wnaxis = 0;

        fits_get_img_param(fp, 2, &wbitpix, &wnaxis, wnaxes, &(_status = 0));
        if ( _status == 0 && wnaxis == 2 && wnaxes[0] == _naxes[0] && wnaxes[1] == _naxes[1] ) {

          const int wddepth = bitpix2ddepth(wbitpix);
          if ( wddepth >= 0 ) {

            outWeights.create(size(), CV_MAKETYPE(wddepth, 1));
            if ( !outWeights.isContinuous() ) {
              outWeights.release();
              outWeights.create(size(), CV_MAKETYPE(wddepth, 1));
            }

            cv::Mat & dst_w = outWeights.getMatRef();

            int w_datatype = cvdepth2datatype(wddepth);
            long w_orig[2] = { 1L, 1L };
            long w_nbdata = wnaxes[0] * wnaxes[1];

            fits_read_pix(fp, w_datatype, w_orig, w_nbdata, &zero, dst_w.data, &zero, &(_status = 0));
          }
        }
      }

      if ( _status == 0 ) {
        haveWeights = true;
      }
      else {
        // Suppress the error so as not to break the successful reading of the main image
        CF_ERROR("Error reading weights extension layer: %s", fits_errmsg(_status));
        _status = 0;
      }

      // Return the cfitsio pointer back to the Primary HDU (layer 1)
      int back_hdu_type = 0;
      fits_movabs_hdu(fp, 1, &back_hdu_type, &(_status = 0));
    }

    if ( !haveWeights ) {
      outWeights.release();
    }
  }

  return true;
}

bool c_fits_reader::read(const std::string & filename,
    cv::OutputArray output_image,
    enum COLORID * output_colorid,
    int ddepth,
    cv::OutputArray outWeights /*= cv::noArray()*/)
{
  c_fits_reader fits(filename);
  if ( output_colorid ) {
    *output_colorid = fits.colorid();
  }

  return fits.read(output_image, ddepth, outWeights);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool c_fits_writer::write(const std::string & filename, cv::InputArray image, enum COLORID colorid,
    cv::InputArray weights /* = cv::noArray() */)
{
  if (filename.empty() || image.empty()) {
    CF_ERROR("Empty filename or image array in c_fits_writer::write");
    return false;
  }

  if( !weights.empty() ) {
    if( weights.channels() != 1 ) {
      CF_ERROR("c_fits_writer: Not supported weights.channels()=%d, "
          "single channel image expected", weights.channels());
      return false;
    }

    if( weights.size() != image.size() ) {
      CF_ERROR("c_fits_writer: image (%dx%d) and mask (weigts) sizes %dx%d nit match",
          image.cols(), image.rows(), weights.cols(), weights.rows());
      return false;
    }
  }


  _status = 0;

  // Create a file with overwriting via "!"
  const std::string fits_path = "!" + filename;
  if (fits_create_file(&fp, fits_path.c_str(), &_status)) {
    CF_ERROR("fits_create_file('%s') fails: %s", filename.c_str(), fits_errmsg(_status));
    return false;
  }

  const cv::Mat src = image.getMat();

  // BITPIX based on image.depth()
  int bitpix = BYTE_IMG, datatype = TBYTE;
  if ( !cvdepth2fits(src.depth(), &bitpix, &datatype) ) {
    CF_ERROR("Unsupported cv::Mat depth=%d", src.depth());
    close();
    return false;
  }

  // Frame dimension for HDU 0
  const int cn = src.channels();
  int naxis = 2;
  long naxes_img[3] = {0};
  naxes_img[0] = src.cols;
  naxes_img[1] = src.rows;

  if (colorid == COLORID_RGB || colorid == COLORID_BGR || cn == 3) {
    // 3 color planes
    naxis = 3;
    naxes_img[2] = 3;
  }

  if (fits_create_img(fp, bitpix, naxis, naxes_img, &_status)) {
    CF_ERROR("fits_create_img(HDU 0) fails: %s", fits_errmsg(_status));
    close();
    return false;
  }

  // Standard color metadata
  const char* bayer_str = colorid2fits(colorid);
  if (bayer_str != nullptr) {
    fits_write_key(fp, TSTRING, "BAYERPAT", (void*)bayer_str,
        "Bayer color pattern", &_status);
  }

  // CUSTOM user fields form _header
  for( const auto & key : _header ) {
    if( !key.name.empty() ) {
      // Try to determine the type (if it's a number/flag, we can extend the logic,
      // but writing it as a TSTRING is the safest and most universal option for custom fields)
      fits_write_key(fp, TSTRING, key.name.c_str(), (void*) key.value.c_str(), key.comment.c_str(), &_status);
      if( _status ) {
        CF_ERROR("fits_write_key() fails: %s", fits_errmsg(_status));
        close();
        return false;
      }
    }
  }

  // Pixel-by-pixel recording of the main frame
  long fpixel[3] = {1, 1, 1};
  long nelements = naxes_img[0] * naxes_img[1];

  if (naxis != 3) {
    // Monochrome or raw Bayer (single-channel)
    fits_write_pix(fp, datatype, fpixel, nelements, (void*)src.data, &_status);
    if( _status ) {
      CF_ERROR("fits_write_pix() fails: %s", fits_errmsg(_status));
      close();
      return false;
    }
  }
  else {
    // Planar channel separation for FITS
    std::vector<cv::Mat> planes;
    cv::split(src, planes);

    // Maintain correct order of RGB/BGR planes
    int r_idx = (colorid == COLORID_BGR) ? 2 : 0;
    int g_idx = 1;
    int b_idx = (colorid == COLORID_BGR) ? 0 : 2;

    // Write planes (cfitsio will automatically convert from `datatype` to the target `bitpix`)
    fpixel[2] = 1; // Layer 1 (R)
    fits_write_pix(fp, datatype, fpixel, nelements, (void*)planes[r_idx].data, &_status);
    fpixel[2] = 2; // Layer 2 (G)
    fits_write_pix(fp, datatype, fpixel, nelements, (void*)planes[g_idx].data, &_status);
    fpixel[2] = 3; // Layer 3 (B)
    fits_write_pix(fp, datatype, fpixel, nelements, (void*)planes[b_idx].data, &_status);
  }

  if( _status ) {
    CF_ERROR("fits_write_pix() fails: %s", fits_errmsg(_status));
    close();
    return false;
  }

  // Write the weight map to HDU 1 - Extension, if it was passed
  if ( !weights.empty() ) {

    int wbitpix = BYTE_IMG, wdatatype = TBYTE;
    if ( !cvdepth2fits(weights.depth(), &wbitpix, &wdatatype) ) {
      CF_ERROR("Unsupported weights.depth() =%d", weights.depth());
      close();
      return false;
    }

    const cv::Mat wsrc = weights.getMat();
    long naxes_weight[] = { wsrc.cols, wsrc.rows };

    // HDU 1 with dynamic data type
    if (fits_create_img(fp, wbitpix, 2, naxes_weight, &_status)) {
      CF_ERROR("fits_create_img(HDU 1) failed: %s", fits_errmsg(_status));
      close();
      return false;
    }

    // Extension name (MASK or WEIGHTS)
    const char * ext_name = weights.depth() == CV_8U ? "MASK" : "WEIGHTS";
    fits_write_key(fp, TSTRING, "EXTNAME", (void*)ext_name, "Extension Layer Name", &_status);

    // Raw matrix data
    long ext_fpixel[] = {1, 1};
    long ext_nelements = wsrc.cols * wsrc.rows;
    fits_write_pix(fp, wdatatype, ext_fpixel, ext_nelements, (void*)wsrc.data, &_status);
    if (_status) {
      CF_ERROR("Error writing extension pixels: %s", fits_errmsg(_status));
      close();
      return false;
    }
  }

  close();
  return true;
}

#endif // HAVE_CFITSIO
