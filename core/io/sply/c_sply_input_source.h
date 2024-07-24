/*
 * c_sply_input_source.h
 *
 *  Created on: Jul 24, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_sply_input_source_h__
#define __c_sply_input_source_h__

#include <core/io/c_input_source.h>
#include <core/io/c_sply_file.h>

#define have_sply_input_source 1

class c_sply_input_source :
    public c_input_source
{
public:
  c_sply_input_source();

  typedef c_sply_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_sply_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  bool is_open() const override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(c_data_frame::sptr & output_frame) override;

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_sply_reader _sply;

};

#endif /* __c_sply_input_source_h__ */
