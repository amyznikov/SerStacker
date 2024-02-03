/*
 * c_vlo_input_source.h
 *
 *  Created on: Dec 15, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_input_source_h__
#define __c_vlo_input_source_h__

#include <core/io/c_input_source.h>
#include "c_vlo_frame.h"

class c_vlo_input_source :
    public c_input_source
{
public:
  typedef c_vlo_input_source this_class;
  typedef c_input_source base;
  typedef std::shared_ptr<this_class> sptr;

  c_vlo_input_source(const std::string & filename);

  static sptr create(const std::string & filename);

  static const std::vector<std::string> & suffixes();

  bool open() override;

  bool is_open() const override;

  void close() override;

  bool seek(int pos) override;

  int curpos() override;

  bool read(c_data_frame::sptr & output_frame) override;

  bool read(c_vlo_scan * scan);

  bool read(cv::Mat & output_frame,
      enum COLORID * output_colorid,
      int * output_bpc) override;

//  bool read_cloud3d(cv::OutputArray points,
//      cv::OutputArray colors);


  void set_read_channel(VLO_DATA_CHANNEL v);
  VLO_DATA_CHANNEL read_channel() const;

  // c_vlo_processing_options * processing_options();

  VLO_VERSION version() const;


protected:
  c_vlo_reader vlo_;
  c_vlo_scan current_scan_;

  VLO_DATA_CHANNEL read_channel_ =
      VLO_DATA_CHANNEL_AMBIENT;
};


#endif /* __c_vlo_input_source_h__ */
