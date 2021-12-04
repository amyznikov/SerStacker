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

#include <tbb/tbb.h>
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
//
//static int bitpix2fits(int bitpix)
//{
//  switch ( bitpix ) {
//  case BYTE_IMG :  // 8
//    return TBYTE;
//  case SBYTE_IMG :  //    10
//    return TSBYTE;
//  case SHORT_IMG :  //    16
//    return TSHORT;
//  case USHORT_IMG :  //    20
//    return TUSHORT;
//  case LONG_IMG :  //    32
//    return TINT;
//  case ULONG_IMG :  //    40
//    return TINT;
//  case FLOAT_IMG :  //  -32
//    return TFLOAT;
//  case DOUBLE_IMG :  //  -64
//    return TDOUBLE;
//  case LONGLONG_IMG :  // 64
//    return TLONGLONG;
//  case ULONGLONG_IMG :  //    80
//    return TULONGLONG;
//  default :
//    break;
//  }
//
//  return false;
//}

static int ddepth2fits(int ddepth)
{
  switch ( ddepth ) {
  case CV_8U:
    return TBYTE;
  case CV_8S:
    return TSBYTE;
  case CV_16U:
    return TUSHORT;
  case CV_16S:
    return TSHORT;
  case CV_32S:
    return TLONG;
  case CV_32F:
    return TFLOAT;
  case CV_64F:
    return TDOUBLE;
  }
  return -1;
}


/** @brief The dst must be continuous and properly preallocated before this call */
template<class T>
static void fits2bgr_(const cv::Mat & src, cv::Mat & dst)
{
  cv::Mat_<cv::Vec<T, 3>> dest = dst;

  using tbb_range = tbb::blocked_range<int>;

  tbb::parallel_for(tbb_range(0, src.rows, 256),
      [&src, &dest](const tbb_range & range) {

        const T * rplane = (const T*)src.data;
        const T * gplane = rplane + src.cols * src.rows;
        const T * bplane = gplane + src.cols * src.rows;

        for ( int y = range.begin(), ny = range.end(); y < ny; ++y ) {
          for ( int x = 0, nx = src.cols; x < nx; ++x ) {

            cv::Vec<T,3> & v = dest[y][x];

            v[0] = bplane[y * nx + x];
            v[1] = gplane[y * nx + x];
            v[2] = rplane[y * nx + x];
          }
        }
      });

}

/** @brief The dst must be continuous and properly preallocated before this call */
static void fits2bgr(const cv::Mat & src, cv::Mat & dst)
{
  switch (src.type()) {
    case CV_8UC3:
      fits2bgr_<uint8_t>(src, dst);
      break;
    case CV_8SC3:
      fits2bgr_<int8_t>(src, dst);
      break;
    case CV_16UC3:
      fits2bgr_<uint16_t>(src, dst);
      break;
    case CV_16SC3:
      fits2bgr_<int16_t>(src, dst);
      break;
    case CV_32SC3:
      fits2bgr_<int32_t>(src, dst);
      break;
    case CV_32FC3:
      fits2bgr_<float>(src, dst);
      break;
    case CV_64FC3:
      fits2bgr_<double>(src, dst);
      break;
    default:
      break;
  }
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
  return status_;
}

/** @brief return cfitsio error message associated with status code of last operation */
const char * c_fits_file::statusmsg() const
{
  return fits_errmsg(status_);
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
  return header_;
}

/** @brief total number of HDUs in fits file */
int c_fits_file::num_hdus() const
{
  return num_hdus_;
}

/** @brief current bitpix of loaded data */
int c_fits_file::bitpix() const
{
  return bitpix_;
}

/** @brief suggested cv::Mat depth for current bitpix of loaded data  */
int c_fits_file::ddepth() const
{
  return bitpix2ddepth(bitpix_);
}

/** @brief suggested cv::Mat channels count for current naxes */
int c_fits_file::channels() const
{
  if ( naxes_.size() == 3 && naxes_[2] == 3 ) {
    return 3;
  }
  if ( naxes_.size() == 2 ) {
    return 1;
  }

  /* not supported combination */
  return 0;
}

/** @brief return colorid for current image */
enum COLORID c_fits_file::colorid() const
{
  return colorid_;
}


/** @brief suggested cv::Mat size for current naxes */
cv::Size c_fits_file::size() const
{
  if ( naxes_.size() >= 2 ) {
    return cv::Size(naxes_[0], naxes_[1]);
  }
  return cv::Size(-1, -1);
}


/** @brief number of dimensions of the image */
int c_fits_file::naxis() const
{
  return naxes_.size();
}

/** @brief current bzero of loaded data  */
double c_fits_file::bzero() const
{
  return bzero_;
}

/** @brief size of each dimension */
const std::vector<long> & c_fits_file::naxes() const
{
  return naxes_;
}

/** @brief size of image dimension specified by index */
long c_fits_file::naxes(int index) const
{
  return naxes_[index];
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
  int nfound = 0;
  int naxis = 0;
  char bayerpat[FLEN_VALUE] = "";

  close();

  header_.clear();
  naxes_.clear();
  num_hdus_ = 0;
  bitpix_ = 0;
  bzero_ = 0;
  bscale_ = 1.0;
  colorid_ = COLORID_UNKNOWN;

  /*
   * Open fits file
   */

  fits_open_diskfile(&fp, filename.c_str(), READONLY, &(status_ = 0));
  if ( !fp || status_ ) {
    CF_ERROR("'%s':  fits_open_file() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    return false;
  }

  /*
   * Read FITS header row by row
   */
  if ( fits_get_hdrspace(fp, &numkeys, NULL, &(status_ = 0)) ) {
    CF_ERROR("'%s': fits_get_hdrspace() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    goto end;
  }

  header_.reserve(numkeys);

  for ( int keynum = 1; keynum <= numkeys; ++keynum ) {

    FKEY c;
    char keyname[256];
    char value[256];
    char comment[256];

    if ( fits_read_keyn(fp, keynum, keyname, value, comment, &(status_ = 0)) ) {
      CF_ERROR("'%s': fits_read_keyn(keynum=%d) fails: %s", filename.c_str(),
          keynum, fits_errmsg(status_));
      goto end;
    }

    c.keyname = keyname;
    c.value = value;
    c.comment = comment;
    header_.emplace_back(c);
  }

  /*
   * Save num HDUs
   */
  if ( fits_get_num_hdus(fp, &num_hdus_, &(status_ = 0)) ) {
    CF_ERROR("'%s': fits_get_num_hdus() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    goto end;
  }

  /*
   * Try to load image format from primary HDU.
   * https://heasarc.gsfc.nasa.gov/docs/software/fitsio/quick/node9.html
   */
  if ( fits_get_img_type(fp, &bitpix_, &(status_ = 0)) ) {
    CF_ERROR("'%s': fits_get_img_type() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    goto end;
  }

  if ( fits_read_key(fp, TDOUBLE, "BZERO", &bzero_, nullptr, &(status_ = 0)) ) {
    /* Some software just put unsigned 16-bit data in the file and don't set the BZERO keyword */
    if ( status_ == KEY_NO_EXIST && bitpix_ == SHORT_IMG ) {
      bitpix_ = USHORT_IMG;
    }
    else {
      CF_ERROR("'%s': fits_read_key(BZERO) fails: %s", filename.c_str(),
          fits_errmsg(status_));
      goto end;
    }
  }
  else if ( bitpix_ == SHORT_IMG && bzero_ != 0 ) {
    bitpix_ = USHORT_IMG;
  }
  else if ( bitpix_ == LONG_IMG && bzero_ != 0.0 ) {
    bitpix_ = ULONG_IMG;
  }

  if ( fits_read_key(fp, TDOUBLE, "BSCALE", &bscale_, nullptr, &(status_ = 0)) ) {
    bscale_ = 1.0;
  }

  if ( fits_get_img_dim(fp, &naxis, &(status_ = 0)) ) {
    CF_ERROR("'%s': fits_get_img_dim() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    goto end;
  }

  naxes_.resize(naxis, 0);
  if ( fits_get_img_size(fp, naxis, naxes_.data(), &(status_ = 0)) ) {
    CF_ERROR("'%s': fits_get_img_size() fails: %s", filename.c_str(),
        fits_errmsg(status_));
    goto end;
  }

  if ( naxes_.size() == 2 ) { /* ignore BAYERPAT key for non-single channel images */
    fits_read_key(fp, TSTRING, "BAYERPAT", bayerpat, nullptr, &(status_ = 0));
  }

  if ( *bayerpat ) {

    constexpr static struct S {
      const char * pattern;
      enum COLORID colorid;
    } known_bayer_patters[] = {
        { "RGGB", COLORID_BAYER_RGGB },
        { "GRBG", COLORID_BAYER_GRBG },
        { "GBRG", COLORID_BAYER_GBRG },
        { "BGGR", COLORID_BAYER_BGGR },
        { "CYYM", COLORID_BAYER_CYYM },
        { "YCMY", COLORID_BAYER_YCMY },
        { "YMCY", COLORID_BAYER_YMCY },
        { "MYYC", COLORID_BAYER_MYYC },
    };

    for ( uint i = 0; i < sizeof(known_bayer_patters) / sizeof(known_bayer_patters[0]); ++i ) {
      if ( strcasecmp(bayerpat, known_bayer_patters[i].pattern) == 0 ) {
        colorid_ = known_bayer_patters[i].colorid;
        break;
      }
    }
  }

  if ( colorid_ == COLORID_UNKNOWN ) {
    switch ( channels() ) {
    case 1 :
      colorid_ = COLORID_MONO;
      break;
    case 3 :  // rgb -> bgr conversion will done in read() by fits2bgr()
      colorid_ = COLORID_BGR;
      break;
    }
  }

end:

  if ( status_ ) {
    close();
  }

  return status_ == 0;
}

bool c_fits_reader::read(cv::OutputArray output_image, int ddepth)
{
  if ( !is_open() ) {
    return false;
  }

  if ( naxes_.size() < 1 || naxes_.size()  > 3 ) {
    CF_ERROR("Unsupported FITS image with naxes_=%zu", naxes_.size());
    return false;
  }

  if ( naxes_.size() == 3 && naxes_[2] != 3 ) {
    CF_ERROR("Unsupported FITS image with %ld channels", naxes_[2]);
    return false;
  }

  if ( output_image.fixedType() ) {
    ddepth = output_image.depth();
  }
  else if ( ddepth < 0 && (ddepth = bitpix2ddepth(bitpix_)) < 0 ) {
    CF_ERROR("Unsupported FITS image with bitpix=%d", bitpix_);
    return false;
  }


  const int cn = channels();
  if ( cn < 1  ) {
    CF_ERROR("Unsupported FITS image with %d channels", cn);
    return false;
  }

  output_image.create(size(), CV_MAKETYPE(ddepth, cn));
  cv::Mat & dst = output_image.getMatRef();
  if ( !dst.isContinuous() ) {
    CF_ERROR("c_fits_reader: destination image must be continuous");
    return false;
  }


  /* Coordinate in each dimension of the first pixel to be read */
  long orig[3] = { 1L, 1L, 1L };
  int zero = 0;

  uint nbdata = naxes_[0] * naxes_[1];
  if ( naxes_.size() > 2 ) {
    nbdata *= naxes_[2];
  }


  if ( cn == 1 ) {

    if ( fits_read_pix(fp, ddepth2fits(ddepth), orig, nbdata, &zero, dst.data, &zero, &(status_ = 0)) ) {
      CF_ERROR("fits_read_pix() fails: %s", fits_errmsg(status_));
      return false;
    }
  }

  else if ( cn == 3 ) {

    cv::Mat tmp(size(), CV_MAKETYPE(ddepth, cn));

    if ( fits_read_pix(fp, ddepth2fits(ddepth), orig, nbdata, &zero, tmp.data, &zero, &(status_ = 0)) ) {
      CF_ERROR("fits_read_pix() fails: %s", fits_errmsg(status_));
      return false;
    }

    fits2bgr(tmp, dst);
  }

  else {
    CF_ERROR("Unsupported FITS image with %d channels", cn);
    return false;
  }

  return true;
}

bool c_fits_reader::read(const std::string & filename,
    cv::OutputArray output_image,
    enum COLORID * output_colorid,
    int ddepth)
{
  c_fits_reader fits(filename);

  if ( output_colorid ) {
    *output_colorid =
        fits.colorid();
  }

  return fits.read(output_image, ddepth);
}

#endif // HAVE_CFITSIO
