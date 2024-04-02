/*
 * c_fits_input_source.h
 *
 *  Created on: Apr 2, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_fits_input_source_h__
#define __c_fits_input_source_h__

#if HAVE_CFITSIO

#include "c_image_input_source.h"
#include <core/io/c_fits_file.h>

#define have_fits_input_source  1

class c_fits_input_source :
    public c_image_input_source
{
public:
  typedef c_fits_input_source this_class;
  typedef c_image_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_fits_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

  bool is_open() const override;

protected:
  c_fits_reader fits_;
  int curpos_ = -1;
};
#endif // HAVE_CFITSIO

#endif /* __c_fits_input_source_h__ */
