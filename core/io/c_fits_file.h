/*
 * c_fits_file.h
 *
 *  Created on: Jan 9, 2021
 *      Author: amyznikov
 */

#ifndef __c_fits_file_h__
#define __c_fits_file_h__

#include "debayer.h" // for COLORID

#if HAVE_CFITSIO

#include <fitsio2.h>

/** @brief Base class for fits reader and writer */
class c_fits_file
{
public:

  /** @brief stub for fits key/value pair */
  struct FKEY {
    std::string name;
    std::string value;
    std::string comment;
  };

  /** @brief Read-Only access to fits header row by row */
  const std::vector<FKEY> & header() const;

  /** @brief total number of HDUs in fits file */
  int num_hdus() const;

  /** @brief current bitpix of loaded data */
  int bitpix() const;

  /** @brief number of dimensions of the image */
  int naxis() const;

  /** @brief current bzero of loaded data  */
  double bzero() const;

  /** @brief size of each dimension */
  const std::vector<long> & naxes() const;

  /** @brief size of image dimension specified by index */
  long naxes(int index) const;

  /** @brief suggested cv::Mat depth for current bitpix of loaded data  */
  int ddepth() const;

  /** @brief suggested cv::Mat channels count for current naxes */
  int channels() const;

  /** @brief return colorid for current image */
  enum COLORID colorid() const;

  /** @brief suggested cv::Mat size for current naxes */
  cv::Size size() const;

  /** @brief return cfitsio status code of last operation */
  int status() const;

  /** @brief return cfitsio error message associated with status code of last operation */
  const char * statusmsg() const;

  /** @brief close fits file */
  void close();

  /** @brief default d'tor closes fits file if was opened */
  ~c_fits_file();

  static const char * colorid2fits(enum COLORID ciolorid);
  static enum COLORID fits2colorid(const char * cname);

protected:
  c_fits_file() = default;

  fitsfile * fp = nullptr;

  std::vector<FKEY> _header;
  std::vector<long> _naxes;
  enum COLORID _colorid = COLORID_UNKNOWN;
  double _bzero = 0;
  double _bscale = 1;
  int _bitpix = 0;
  int _num_hdus = 0;
  int _status = 0;
};

/** @brief Very basic FITS image reader from primary HDU */
class c_fits_reader :
    public c_fits_file
{
public:
  c_fits_reader() = default;

  c_fits_reader(const std::string & filename);

  bool open(const std::string & filename);

  bool is_open() const;

  bool read(cv::OutputArray outImage,int ddepth = -1, cv::OutputArray outWeights = cv::noArray());

  static bool read(const std::string & filename,
      cv::OutputArray output_image,
      enum COLORID  * output_colorid = nullptr,
      int ddepth = -1,
      cv::OutputArray outWeights = cv::noArray());

};

/** @brief FITS image writer, had no time to implement yet */
class c_fits_writer :
    public c_fits_file
{
public:
  c_fits_writer() = default;

  /** @brief Clear the current prepared header */
  void clear_header() {
    _header.clear();
  }

  /** @brief Add a text field to the title */
  void add_header_key(const std::string& key, const std::string& value, const std::string& comment = "")
  {
    _header.push_back({key, value, comment});
  }

   bool write(const std::string & filename, cv::InputArray image, enum COLORID colorid,
       cv::InputArray weights = cv::noArray());

//protected:
//   bool write_header_keys();
};

#endif // HAVE_CFITSIO

#endif /* __c_fits_file_h__ */
