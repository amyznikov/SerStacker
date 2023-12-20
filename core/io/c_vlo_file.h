/*
 * c_vlo_file.h
 *
 *  Created on: Oct 21, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_vlo_file_h__
#define __c_vlo_file_h__

#include "c_ifhd_file.h"
#include "vlo/c_vlo_scan.h"

struct c_vlo_processing_options
{
  struct {
    bool enabled = false;
    double saturation_level = 122; // for 'peak' intensity measure

    // -10 cm for makrolon,
    // +40 cm for FIR20
    double doubled_distanse_systematic_correction = 0; // [cm]
    double doubled_distanse_depth_tolerance = 100; // [cm]

  } ghost_filter;

  struct {
    bool enabled = false;
    double low_intensity_level = 1800; // for 'area' intensity measure
    double u = 100;
    double v = 0.0074;
  } low_intensity_filter;

};

class c_vlo_file
{
public:
  typedef c_vlo_file this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_file();
  c_vlo_file(const std::string & filename);

  const std::string & filename() const;
  VLO_VERSION version() const;

  static cv::Mat get_thumbnail_image(const c_vlo_scan & scan);

//  static cv::Mat get_image(const c_vlo_scan & scan, VLO_DATA_CHANNEL channel,
//      cv::InputArray exclude_mask = cv::noArray(),
//      const c_vlo_processing_options * opts = nullptr);

//  static bool get_cloud3d(const c_vlo_scan & scan, VLO_DATA_CHANNEL intensity_channel,
//      cv::OutputArray points, cv::OutputArray colors,
//      cv::InputArray exclude_mask = cv::noArray(),
//      const c_vlo_processing_options * opts = nullptr);

//  static bool get_clouds3d(const c_vlo_scan & scan,
//      cv::Mat3f clouds[3],
//      const c_vlo_processing_options * opts = nullptr);

protected:
  std::string filename_;
  VLO_VERSION version_ = VLO_VERSION_UNKNOWN;
};

class c_vlo_reader :
    public c_vlo_file
{
public:
  typedef c_vlo_reader this_class;
  typedef c_vlo_file base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_vlo_reader();
  c_vlo_reader(const std::string & filename);
  ~c_vlo_reader();

  //void set_apply_ghost_filter(bool v);
  //bool apply_ghost_filter() const;

  c_vlo_processing_options * processing_options();


  bool open(const std::string & filename = "");
  void close();
  bool is_open() const;
  bool seek(int32_t frame_index);
  int32_t curpos();

  /// @brief get frame size in bytes
  ssize_t frame_size() const;

  /// @brief get number of frames in this file
  ssize_t num_frames() const;

  bool read(c_vlo_scan * scan);
//  bool read(cv::Mat * image, VLO_DATA_CHANNEL channel);
//  bool read_cloud3d(cv::OutputArray points, cv::OutputArray colors, VLO_DATA_CHANNEL colors_channel);

protected:
  template<class ScanType> std::enable_if_t<(c_vlo_scan_type_traits<ScanType>::VERSION > 0),
    bool> read(ScanType * scan);

protected:
  c_file_handle fd_;
  c_ifhd_reader ifhd_;
  ssize_t num_frames_ = -1;
  ssize_t frame_size_ = -1;
  c_vlo_processing_options processing_options_;

  //bool apply_ghost_filter_  = false;
};



#endif /* __c_vlo_file_h__ */
