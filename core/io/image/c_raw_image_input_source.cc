/*
 * c_raw_image_input_source.cc
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#include "c_raw_image_input_source.h"
#include <core/readdir.h>


#if HAVE_LIBRAW
c_raw_image_input_source::c_raw_image_input_source(const std::string & filename) :
    base(/*c_input_source::RAW_IMAGE, */filename)
{
}

c_raw_image_input_source::sptr c_raw_image_input_source::create(const std::string & filename)
{
  if ( file_exists(filename) && !is_directory(filename) ) {
    c_raw_image_input_source::sptr obj(new c_raw_image_input_source(filename));
    obj->size_ = 1;
    return obj;
  }
  return nullptr;
}

const std::vector<std::string>& c_raw_image_input_source::suffixes()
{
  // https://api.kde.org/libkdcraw/html/rawfiles_8h_source.html

  static const std::vector<std::string> suffixes_ = {
      ".bay",      // Casio Digital Camera Raw File Format.
      ".bmq",  // NuCore Raw Image File.
      ".cr2",  // Canon Digital Camera RAW Image Format version 2.0. These images are based on the TIFF image standard.
      ".crw",  // Canon Digital Camera RAW Image Format version 1.0.
      ".cs1",  // Capture Shop Raw Image File.
      ".dc2",  // Kodak DC25 Digital Camera File.
      ".dcr", // Kodak Digital Camera Raw Image Format for these models: Kodak DSC Pro SLR/c, Kodak DSC Pro SLR/n, Kodak DSC Pro 14N, Kodak DSC PRO 14nx.
      ".dng", // Adobe Digital Negative: DNG is publicly available archival format for the raw files generated by digital cameras. By addressing the lack of an open standard for the raw files created by individual camera models, DNG helps ensure that photographers will be able to access their files in the future.
      ".erf",  // Epson Digital Camera Raw Image Format.
      ".fff",  // Imacon Digital Camera Raw Image Format.
      ".hdr",  // Leaf Raw Image File.
      ".k25",  // Kodak DC25 Digital Camera Raw Image Format.
      ".kdc",  // Kodak Digital Camera Raw Image Format.
      ".mdc",  // Minolta RD175 Digital Camera Raw Image Format.
      ".mos",  // Mamiya Digital Camera Raw Image Format.
      ".mrw",  // Minolta Dimage Digital Camera Raw Image Format.
      ".nef",  // Nikon Digital Camera Raw Image Format.
      ".orf",  // Olympus Digital Camera Raw Image Format.
      ".pef",  // Pentax Digital Camera Raw Image Format.
      ".pxn",  // Logitech Digital Camera Raw Image Format.
      ".raf",  // Fuji Digital Camera Raw Image Format.
      ".raw",  // Panasonic Digital Camera Image Format.
      ".rdc",  // Digital Foto Maker Raw Image File.
      ".sr2",  // Sony Digital Camera Raw Image Format.
      ".srf",  // Sony Digital Camera Raw Image Format for DSC-F828 8 megapixel digital camera or Sony DSC-R1
      ".x3f",  // Sigma Digital Camera Raw Image Format for devices based on Foveon X3 direct image sensor.
      ".arw",  // Sony Digital Camera Raw Image Format for Alpha devices.

      // NOTE: VERSION 2
      ".3fr",// Hasselblad Digital Camera Raw Image Format.
      ".cine",  // Phantom Software Raw Image File.
      ".ia",   // Sinar Raw Image File.
      ".kc2",  // Kodak DCS200 Digital Camera Raw Image Format.
      ".mef",  // Mamiya Digital Camera Raw Image Format.
      ".nrw",  // Nikon Digital Camera Raw Image Format.
      ".qtk",  // Apple Quicktake 100/150 Digital Camera Raw Image Format.
      ".rw2",  // Panasonic LX3 Digital Camera Raw Image Format.
      ".sti",  // Sinar Capture Shop Raw Image File.

      // NOTE: VERSION 3

      ".rwl",// Leica Digital Camera Raw Image Format.

      // NOTE: VERSION 4

      ".srw",// Samsung Raw Image Format.

      // NOTE: VERSION 5
      ".drf",// Kodak Digital Camera Raw Image Format.
      ".dsc",  // Kodak Digital Camera Raw Image Format.
      ".ptx",  // Pentax Digital Camera Raw Image Format.
      ".cap",  // Phase One Digital Camera Raw Image Format.
      ".iiq",  // Phase One Digital Camera Raw Image Format.
      ".rwz"  // Rawzor Digital Camera Raw Image Format.#endif
      };

  return suffixes_;
}

bool c_raw_image_input_source::open()
{
  if ( file_readable(filename_) && !is_directory(filename_) ) {
    curpos_ = 0;
    return true;
  }
  return false;
}

void c_raw_image_input_source::close()
{
  curpos_ = -1;
}

bool c_raw_image_input_source::seek(int pos)
{
  if ( pos != 0 ) {
    return false;
  }
  curpos_ = pos;
  return true;
}

int c_raw_image_input_source::curpos()
{
  return curpos_;
}

bool c_raw_image_input_source::read(cv::Mat & output_frame,
    enum COLORID * output_colorid,
    int * output_bpc)
{
  if ( curpos_ != 0 || !raw_.read(filename_, output_frame, output_colorid, output_bpc) ) {
    return false;
  }

  ++curpos_;

  if ( (this->has_color_matrix_ = raw_.has_color_matrix()) ) {
    this->color_matrix_ = raw_.color_matrix();
  }

  return true;
}

bool c_raw_image_input_source::is_open() const
{
  return curpos_ >= 0;
}

#endif // HAVE_LIBRAW
