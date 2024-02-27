/*
 * c_hdl_input_source.h
 *
 *  Created on: Feb 25, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_hdl_input_source_h__
#define __c_hdl_input_source_h__

#include <core/io/c_input_source.h>
#include "c_hdl_frame_reader.h"

#if HAVE_PCAP

class c_hdl_input_source :
    public c_input_source
{
public:
  typedef c_hdl_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_hdl_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  bool is_open() const override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(c_data_frame::sptr & output_frame) override;

  c_hdl_frame::sptr read();

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

protected:
  c_hdl_offline_pcap_reader reader_;
};

#endif // HAVE_PCAP

#endif /* __c_hdl_input_source_h__ */
